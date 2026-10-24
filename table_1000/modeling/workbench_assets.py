"""Electronics-workbench props, each one complete rigid body with semantic parts.

Dimensions are engineering estimates in metres from the scene 000101 reference
photo. Origins sit at the bottom centre of the resting pose; +X is the longest
horizontal side, +Z up, -Y the preview front. No branding or text is modelled.
"""

import math

import bmesh
import bpy

from table_1000.modeling.asset_builders import asset, body, box, cylinder, material, mesh, tube
from table_1000.modeling.pen_holder_assets import outline
from table_1000.modeling.small_props_assets import (
    component, ellipsoid, finish_whole, rounded_box,
)


# --------------------------------------------------------------------------- geometry helpers

def convex(points):
    """Vertices and faces of the convex hull of ``points`` (near-coplanar faces merged)."""
    bm = bmesh.new()
    seen = set()
    for p in points:
        key = tuple(round(c, 7) for c in p)
        if key not in seen:
            seen.add(key)
            bm.verts.new(key)
    result = bmesh.ops.convex_hull(bm, input=list(bm.verts))
    drop = list({v for v in result["geom_interior"] + result["geom_unused"] if isinstance(v, bmesh.types.BMVert)})
    bmesh.ops.delete(bm, geom=drop, context="VERTS")
    bmesh.ops.dissolve_limit(bm, angle_limit=0.0005, verts=list(bm.verts), edges=list(bm.edges))
    bm.verts.index_update()
    vertices = [tuple(v.co) for v in bm.verts]
    faces = [tuple(v.index for v in f.verts) for f in bm.faces]
    bm.free()
    return vertices, faces


def hull_collider(name, points, parent):
    vertices, faces = convex(points)
    return mesh("Collision_" + name, vertices, faces, parent, shape="CONVEX_HULL")


def hull(name, points, surface, parent, collision=True, smooth=False):
    """Convex visual solid from points, with the identical convex collider."""
    vertices, faces = convex(points)
    obj = mesh(name, vertices, faces, parent, surface)
    if smooth:
        for face in obj.data.polygons:
            face.use_smooth = True
    if collision:
        mesh("Collision_" + name, vertices, faces, parent, shape="CONVEX_HULL")
    return obj


def box_collider(name, dimensions, position, parent):
    vertices = [(position[0] + x * dimensions[0] / 2, position[1] + y * dimensions[1] / 2,
                 position[2] + z * dimensions[2] / 2)
                for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return mesh("Collision_" + name, vertices, faces, parent, shape="BOX")


def ring(center_axial, radius, axis, n, circumscribed=False, vertex_down=False):
    """Polygon ring with a flat side (or, with vertex_down, a vertex) at -Z (-Y for a Z axis)."""
    r = radius / math.cos(math.pi / n) if circumscribed else radius
    pts = []
    for k in range(n):
        a = -math.pi / 2 + (k + (0 if vertex_down else 0.5)) * math.tau / n
        u, v = r * math.cos(a), r * math.sin(a)
        if axis == "X":
            pts.append((center_axial, u, v))
        elif axis == "Y":
            pts.append((u, center_axial, v))
        else:
            pts.append((u, v, center_axial))
    return pts


def shifted(points, offset):
    return [(x + offset[0], y + offset[1], z + offset[2]) for x, y, z in points]


def slab(name, x0, x1, y0, y1, z_of_x, height, surface, parent, collision=False):
    """Box whose bottom follows the (planar) surface z(x); used on sloped tops."""
    pts = [(x, y, z_of_x(x) + dz) for x in (x0, x1) for y in (y0, y1) for dz in (0, height)]
    return hull(name, pts, surface, parent, collision=collision)


def rod(name, p0, p1, w0, w1, h0, h1, zc, surface, parent, collision=True):
    """Flat-bottomed octagonal bar between XY points p0 and p1, centred at height zc."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    lx, ly = -dy / length, dx / length
    pts = []
    for (px, py), w, h in ((p0, w0, h0), (p1, w1, h1)):
        for k in range(8):
            a = (k + 0.5) * math.tau / 8
            s = w / 2 * math.cos(a) / math.cos(math.pi / 8)
            z = h / 2 * math.sin(a) / math.sin(3 * math.pi / 8)
            pts.append((px + lx * s, py + ly * s, zc + z))
    return hull(name, pts, surface, parent, collision=collision, smooth=True)


def lathe(name, rings, surface, parent, segments=32, collision_segments=16, collision=True):
    """Z-axis solid of revolution; colliders are hulls of each convex profile run."""
    n = segments
    vertices = []
    for z, r in rings:
        vertices.extend((r * math.cos(i * math.tau / n), r * math.sin(i * math.tau / n), z)
                        for i in range(n))
    faces = [tuple(range(n - 1, -1, -1)), tuple((len(rings) - 1) * n + i for i in range(n))]
    for k in range(len(rings) - 1):
        a, b = k * n, (k + 1) * n
        faces.extend((a + i, a + (i + 1) % n, b + (i + 1) % n, b + i) for i in range(n))
    obj = mesh(name, vertices, faces, parent, surface)
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    if collision:
        slopes = [(r1 - r0) / (z1 - z0) for (z0, r0), (z1, r1) in zip(rings, rings[1:])]
        breaks = [0] + [i for i in range(1, len(slopes)) if slopes[i] > slopes[i - 1] + 1e-9]
        breaks.append(len(rings) - 1)
        # PhysX simplifies hulls above 64 vertices; long runs are split into
        # overlapping chunks of at most 64 // collision_segments rings.
        per = max(2, 64 // collision_segments)
        chunks = []
        for first, last in zip(breaks, breaks[1:]):
            while last - first + 1 > per:
                chunks.append((first, first + per - 1))
                first += per - 1
            chunks.append((first, last))
        for k, (first, last) in enumerate(chunks):
            pts = []
            for z, r in rings[first:last + 1]:
                pts.extend(ring(z, r, "Z", collision_segments, circumscribed=True))
            hull_collider(f"{name} run {k}", pts, parent)
    return obj


def x_solid(name, rings, z_axis, surface, parent, segments=16, collision_sides=None):
    """X-axis solid of revolution resting on a polygon vertex line at z_axis - r_max.

    Colliders are hulls of each convex profile run. In the Isaac CPU settling
    check a face-down polygon rolled half a segment while vertex-down rings
    stayed put, so round props use vertex-down rings (as cylinder() does).
    """
    n = segments
    def section(x, r):
        return [(x, r * math.cos(-math.pi / 2 + i * math.tau / n),
                 z_axis + r * math.sin(-math.pi / 2 + i * math.tau / n)) for i in range(n)]
    vertices = [v for x, r in rings for v in section(x, r)]
    faces = [tuple(range(n - 1, -1, -1)), tuple((len(rings) - 1) * n + i for i in range(n))]
    for k in range(len(rings) - 1):
        a, b = k * n, (k + 1) * n
        faces.extend((a + i, a + (i + 1) % n, b + (i + 1) % n, b + i) for i in range(n))
    obj = mesh(name, vertices, faces, parent, surface)
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    slopes = [(r1 - r0) / (x1 - x0) for (x0, r0), (x1, r1) in zip(rings, rings[1:])]
    breaks = [0] + [i for i in range(1, len(slopes)) if slopes[i] > slopes[i - 1] + 1e-9]
    breaks.append(len(rings) - 1)
    for k, (first, last) in enumerate(zip(breaks, breaks[1:])):
        if collision_sides:
            # Flat-sided handle collider (face down, apothem = profile radius).
            pts = []
            for x, r in rings[first:last + 1]:
                pts.extend(shifted(ring(x, r, "X", collision_sides, circumscribed=True), (0, 0, z_axis)))
        else:
            pts = vertices[first * n:(last + 1) * n]
        hull_collider(f"{name} run {k}", pts, parent)
    return obj


def ring_sectors(name, r_in, r_out, z0, z1, parent, n=10):
    """Open-centred annulus as n convex prisms (outer chords circumscribe r_out)."""
    ro = r_out / math.cos(math.pi / n)
    for k in range(n):
        a0, a1 = k * math.tau / n, (k + 1) * math.tau / n
        pts = [(r * math.cos(a), r * math.sin(a), z) for r in (r_in, ro) for a in (a0, a1)
               for z in (z0, z1)]
        hull_collider(f"{name} sector {k:02d}", pts, parent)


def open_container(whole, p, surface):
    """Floor plus separate wall strips (and dividers) so every cavity stays open."""
    w, d, h, t, f, r = (p[k] for k in ("width", "depth", "height", "wall", "floor", "radius"))
    outer, inner = outline(w, d, r), outline(w - 2 * t, d - 2 * t, r - t)
    floor = component(whole, "floor")
    hull("Floor plate", [(x, y, z) for x, y in outer for z in (0, f)], surface, floor)
    walls = component(whole, "walls")
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        pts = [(x, y, z) for x, y in (outer[i], outer[j], inner[i], inner[j]) for z in (f, h)]
        hull(f"Wall strip {i:02d}", pts, surface, walls)
    if p.get("rim"):
        rim_t = p["rim"]
        rim = component(whole, "rim")
        rim_outer = outline(w + 2 * rim_t, d + 2 * rim_t, r + rim_t)
        for i in range(n):
            j = (i + 1) % n
            pts = [(x, y, z) for x, y in (rim_outer[i], rim_outer[j], outer[i], outer[j])
                   for z in (h - 0.004, h)]
            hull(f"Rim strip {i:02d}", pts, surface, rim)
    columns, rows = p.get("columns", 1), p.get("rows", 1)
    dt, dh = p.get("divider", t), p.get("divider_height", h)
    iw, idp = w - 2 * t, d - 2 * t
    cell = (iw - (columns - 1) * dt) / columns
    for c in range(1, columns):
        x = -iw / 2 + c * cell + (c - 0.5) * dt
        node = component(whole, f"divider_x{c}")
        box(f"Cross divider {c}", (dt, idp, dh - f), (x, 0, (dh + f) / 2), surface, node)
    cell_y = (idp - (rows - 1) * dt) / rows
    for c in range(1, rows):
        y = -idp / 2 + c * cell_y + (c - 0.5) * dt
        node = component(whole, f"divider_y{c}")
        box(f"Long divider {c}", (iw, dt, dh - f), (0, y, (dh + f) / 2), surface, node)


# --------------------------------------------------------------------------- profiles

PROFILES = {
    "multimeter": {
        "bench": {"size": (0.185, 0.092, 0.052), "lip": 0.0055, "rim_h": 0.006, "radius": 0.009,
                  "body_color": (0.95, 0.60, 0.015), "face_color": (0.10, 0.11, 0.12),
                  "lcd": (0.050, 0.064), "lcd_x": 0.050, "dial_r": 0.021, "dial_x": -0.016,
                  "dial_ring": None, "buttons": 6, "jacks": ("red", "black", "red", "red"),
                  "jack_x": -0.070, "jack_pitch": 0.019},
        "pocket": {"size": (0.135, 0.072, 0.032), "lip": 0.0045, "rim_h": 0.004, "radius": 0.008,
                   "body_color": (0.95, 0.66, 0.02), "face_color": (0.018, 0.020, 0.022),
                   "lcd": (0.034, 0.048), "lcd_x": 0.037, "dial_r": 0.0155, "dial_x": -0.010,
                   "dial_ring": (0.95, 0.66, 0.02), "buttons": 3, "jacks": ("black", "red", "red"),
                   "jack_x": -0.050, "jack_pitch": 0.018},
    },
    "pliers": {
        "needle_nose": {"tip_x": 0.080, "joint": (0.002, 0.024), "pivot_w": 0.017,
                        "jaw": (0.022, 0.0068, 0.0015, 0.0090, 0.0045),
                        "neck": [(0.004, 0.0028), (-0.020, 0.0080)], "neck_w": 0.0072,
                        "handle": [(-0.018, 0.0078), (-0.052, 0.0160), (-0.077, 0.0188)],
                        "handle_w": (0.0105, 0.0122, 0.0130), "butt_x": -0.080,
                        "thickness": 0.012, "grip_colors": ("red", "red")},
        "diagonal": {"tip_x": 0.0625, "joint": (-0.002, 0.022), "pivot_w": 0.024,
                     "jaw": (0.014, 0.0110, 0.0120, 0.0090, 0.0070),
                     "neck": [(-0.002, 0.0040), (-0.016, 0.0075)], "neck_w": 0.0080,
                     "handle": [(-0.014, 0.0075), (-0.038, 0.0135), (-0.058, 0.0155)],
                     "handle_w": (0.0110, 0.0120, 0.0125), "butt_x": -0.0625,
                     "thickness": 0.011, "grip_colors": ("red", "blue")},
    },
    "solder_spool": {"default": {"flange_r": 0.0275, "flange_t": 0.002, "height": 0.032,
                                 "wire_r": 0.022, "core_r": 0.0095, "hole_r": 0.0060}},
    "drink_can": {"default": {"rings": [(0, 0.0245), (0.0025, 0.0300), (0.008, 0.0330),
                                        (0.100, 0.0330), (0.1085, 0.0290), (0.112, 0.0272),
                                        (0.115, 0.0272)],
                              "band": (0.030, 0.086)}},
    "water_bottle": {"default": {"rings": [(0, 0.0260), (0.003, 0.0315), (0.008, 0.0325),
                                           (0.058, 0.0325), (0.066, 0.0300), (0.074, 0.0294),
                                           (0.082, 0.0300), (0.090, 0.0325), (0.140, 0.0325),
                                           (0.160, 0.0270), (0.172, 0.0180), (0.178, 0.0135),
                                           (0.186, 0.0135)],
                                 "label": (0.094, 0.136), "collar": (0.180, 0.0025, 0.0172),
                                 "cap": (0.186, 0.205, 0.0158)}},
    "calculator": {"default": {"size": (0.160, 0.080), "z_front": 0.0120, "z_back": 0.0165,
                               "key_h": 0.0015, "rows": 8, "cols": 5}},
    "computer_mouse": {"default": {"stations": [(-0.0560, 0.016, 0.008), (-0.0545, 0.034, 0.016),
                                                (-0.0505, 0.046, 0.024), (-0.0420, 0.055, 0.032),
                                                (-0.0280, 0.060, 0.037), (-0.0120, 0.061, 0.038),
                                                (0.0050, 0.060, 0.0365), (0.0220, 0.057, 0.0325),
                                                (0.0370, 0.052, 0.026), (0.0480, 0.044, 0.019),
                                                (0.0540, 0.032, 0.013), (0.0560, 0.018, 0.009)],
                                   "wheel_x": 0.026}},
    "circuit_board": {
        "dev_board": {"size": (0.085, 0.056, 0.0016), "standoff": 0.003, "foot_r": 0.0028,
                      "foot_inset": 0.0035, "color": (0.02, 0.13, 0.50), "usb": True,
                      "chips": [("MCU", (0.012, 0.012, 0.0014), (0.006, 0.002)),
                                ("Driver", (0.008, 0.005, 0.0016), (-0.016, 0.012)),
                                ("Regulator", (0.0065, 0.0035, 0.0018), (-0.026, -0.012)),
                                ("Flash", (0.005, 0.005, 0.0011), (0.022, -0.010))],
                      "metal_parts": [("Crystal", (0.006, 0.0025, 0.0012), (-0.004, -0.012)),
                                      ("Reset button", (0.006, 0.006, 0.0025), (0.030, 0.012))],
                      "button": (0.030, 0.012), "led": (0.034, -0.002),
                      "header_h": 0.0085, "pin_h": 0.0025,
                      "headers": [("front", 0.004, -0.025, 20), ("back", 0.004, 0.025, 20)]},
        "breakout": {"size": (0.050, 0.025, 0.0016), "standoff": 0.002, "foot_r": 0.0018,
                     "foot_inset": 0.0025, "color": (0.03, 0.30, 0.10), "usb": False,
                     "chips": [("Sensor", (0.007, 0.007, 0.0012), (0.004, -0.002))],
                     "metal_parts": [], "led": (0.016, -0.006),
                     "header_h": 0.0025, "pin_h": 0.0030,
                     "headers": [("pins", -0.008, 0.009, 8)]},
    },
    "soldering_iron": {"default": {
        "grip_r": 0.011,
        "handle": [(-0.110, 0.0035), (-0.100, 0.0045), (-0.090, 0.0080), (-0.082, 0.0095),
                   (-0.062, 0.0098)],
        "grip": [(-0.0625, 0.0106), (-0.010, 0.0106), (0.000, 0.0096)],
        "guard": [(-0.0005, 0.0080), (0.004, 0.0110), (0.008, 0.0110), (0.012, 0.0060)],
        "barrel": [(0.0115, 0.0042), (0.070, 0.0035), (0.085, 0.0030)],
        "tip": [(0.0845, 0.0020), (0.100, 0.0014), (0.110, 0.0004)]}},
    "tip_cleaner": {"default": {
        "rings": [(0.0, 0.0375), (0.007, 0.0375), (0.011, 0.0330), (0.042, 0.0310),
                  (0.045, 0.0290), (0.045, 0.0240), (0.030, 0.0225), (0.0295, 0.0001)],
        "outer_rings": 5,
        "wool": [(0.0, 0.0, 0.037, 0.040, 0.038, 0.016), (0.008, -0.006, 0.040, 0.020, 0.018, 0.012),
                 (-0.009, 0.005, 0.040, 0.022, 0.020, 0.011), (0.003, 0.010, 0.039, 0.018, 0.016, 0.012),
                 (-0.004, -0.010, 0.039, 0.018, 0.016, 0.011)]}},
    "battery": {"default": {"radius": 0.00725, "length": 0.0505, "split_x": 0.011}},
    "tool_tray": {"default": {"width": 0.330, "depth": 0.220, "height": 0.060, "wall": 0.004,
                              "floor": 0.004, "radius": 0.014, "color": (0.30, 0.32, 0.33)}},
    "parts_box": {"default": {"width": 0.210, "depth": 0.130, "height": 0.038, "wall": 0.003,
                              "floor": 0.003, "radius": 0.005, "columns": 3, "rows": 2,
                              "divider": 0.003, "divider_height": 0.036,
                              "color": (0.18, 0.40, 0.80)}},
    "tool_cup": {"default": {"radius": 0.0425, "height": 0.120, "wall": 0.003, "floor": 0.004,
                             "segments": 20, "strands": 18}},
    "precision_screwdriver": {
        "black_blue": {"radius": 0.0070, "cap_color": (0.015, 0.015, 0.017),
                       "handle_color": (0.015, 0.015, 0.017), "stripe_color": (0.05, 0.25, 0.75),
                       "cap": [(-0.080, 0.0050), (-0.078, 0.0062), (-0.073, 0.0062)],
                       "handle": [(-0.0735, 0.0068), (-0.066, 0.0070), (-0.020, 0.0070),
                                  (-0.008, 0.0058), (-0.002, 0.0040), (0.000, 0.0030)],
                       "shaft": [(-0.0005, 0.0015), (0.074, 0.0015), (0.080, 0.0005)]},
        "red": {"radius": 0.0080, "cap_color": (0.02, 0.02, 0.022),
                "handle_color": (0.62, 0.03, 0.03), "stripe_color": (0.40, 0.015, 0.02),
                "cap": [(-0.0875, 0.0058), (-0.0855, 0.0070), (-0.0800, 0.0070)],
                "handle": [(-0.0805, 0.0078), (-0.072, 0.0080), (-0.022, 0.0080),
                           (-0.010, 0.0066), (-0.003, 0.0045), (0.000, 0.0034)],
                "shaft": [(-0.0005, 0.0018), (0.081, 0.0018), (0.0875, 0.0006)]}},
    "wire_stripper": {"default": {"tip_x": 0.085, "joint": (0.000, 0.024), "pivot_w": 0.020,
                                  "jaw": (0.020, 0.0085, 0.0100, 0.0110, 0.0085), "notches": 5,
                                  "neck": [(0.002, 0.0035), (-0.020, 0.0085)], "neck_w": 0.0080,
                                  "handle": [(-0.018, 0.0082), (-0.052, 0.0165), (-0.080, 0.0185)],
                                  "handle_w": (0.0110, 0.0125, 0.0135), "butt_x": -0.085,
                                  "thickness": 0.014, "grip_colors": ("yellow", "yellow"),
                                  "end_color": "black"}},
    "electrical_tape": {"default": {"outer_r": 0.029, "inner_r": 0.019, "core_inner_r": 0.0175,
                                    "width": 0.019}},
    "tweezers": {"default": {"length": 0.120, "blade_t": 0.0011, "hinge_h": 0.0060,
                             "path": [(-0.060, 0.0015, 0.0060), (-0.040, 0.0030, 0.0058),
                                      (0.010, 0.0044, 0.0045), (0.045, 0.0025, 0.0030),
                                      (0.060, 0.0008, 0.0016)]}},
    "power_adapter": {"default": {"size": (0.110, 0.048, 0.030), "stub": 0.012}},
    "lab_notebook": {"default": {"size": (0.240, 0.180, 0.012), "gutter": 0.008, "loops": 34}},
    "test_probe": {
        "red": {"color": (0.70, 0.03, 0.03), "guard_r": 0.0070,
                "boot": [(-0.070, 0.0030), (-0.064, 0.0042), (-0.060, 0.0045)],
                "handle": [(-0.0602, 0.0045), (-0.010, 0.0045), (0.020, 0.0045)],
                "guard": [(0.0195, 0.0050), (0.022, 0.0070), (0.026, 0.0070), (0.028, 0.0045)],
                "nose": [(0.0275, 0.0040), (0.040, 0.0030), (0.046, 0.0020)],
                "tip": [(0.0455, 0.0010), (0.066, 0.0010), (0.070, 0.0002)]},
        "black": {"color": (0.02, 0.02, 0.022), "guard_r": 0.0070,
                  "boot": [(-0.070, 0.0030), (-0.064, 0.0042), (-0.060, 0.0045)],
                  "handle": [(-0.0602, 0.0045), (-0.010, 0.0045), (0.020, 0.0045)],
                  "guard": [(0.0195, 0.0050), (0.022, 0.0070), (0.026, 0.0070), (0.028, 0.0045)],
                  "nose": [(0.0275, 0.0040), (0.040, 0.0030), (0.046, 0.0020)],
                  "tip": [(0.0455, 0.0010), (0.066, 0.0010), (0.070, 0.0002)]}},
}


# --------------------------------------------------------------------------- builders

def multimeter(whole, p):
    L, W, H = p["size"]
    lip, rim_h = p["lip"], p["rim_h"]
    yellow = material("Yellow rubber holster", p["body_color"], 0.55)
    face = material("Meter face plate", p["face_color"], 0.40)
    lcd = material("Grey LCD window", (0.42, 0.45, 0.40), 0.18)
    bezel = material("Dark LCD bezel", (0.03, 0.035, 0.04), 0.30)
    knob = material("Black rotary dial", (0.012, 0.012, 0.014), 0.35)
    grey = material("Grey push buttons", (0.55, 0.56, 0.57), 0.40)
    red = material("Red jack", (0.65, 0.02, 0.02), 0.30)
    black = material("Black jack", (0.015, 0.015, 0.015), 0.30)
    base_h = H - rim_h
    holster = component(whole, "holster")
    rounded_box("Holster body", (L, W, base_h), (0, 0, base_h / 2), yellow, holster,
                radius=p["radius"])
    # Raised rubber lip around the face (four strips keep the face recess visible).
    for name, dims, pos in (("front", (L - 2 * p["radius"], lip, rim_h), (0, -W / 2 + lip / 2)),
                            ("back", (L - 2 * p["radius"], lip, rim_h), (0, W / 2 - lip / 2)),
                            ("left", (lip, W - 2 * lip, rim_h), (-L / 2 + lip / 2 + 0.002, 0)),
                            ("right", (lip, W - 2 * lip, rim_h), (L / 2 - lip / 2 - 0.002, 0))):
        box(f"Holster lip {name}", dims, (*pos, base_h + rim_h / 2), yellow, holster, bevel=0.0012)
    top = base_h + 0.0025
    panel = component(whole, "face")
    box("Face plate", (L - 2 * lip - 0.006, W - 2 * lip, 0.0025),
        (0, 0, base_h + 0.00125), face, panel, bevel=0.0008)
    display = component(whole, "display")
    lw, ly = p["lcd"]
    box("LCD bezel", (lw + 0.008, ly + 0.008, 0.0006), (p["lcd_x"], 0, top + 0.0003),
        bezel, display, collision=False)
    box("LCD window", (lw, ly, 0.0004), (p["lcd_x"], 0, top + 0.0008), lcd, display,
        collision=False)
    buttons = component(whole, "buttons")
    count = p["buttons"]
    bx = p["lcd_x"] - lw / 2 - 0.009
    pitch = (W - 2 * lip - 0.012) / count
    for i in range(count):
        y = -(count - 1) / 2 * pitch + i * pitch
        box(f"Button {i}", (0.006, pitch * 0.72, 0.0012), (bx, y, top + 0.0006),
            yellow if i in (0, count - 1) else grey, buttons, bevel=0.0004, collision=False)
    dial = component(whole, "dial")
    r = p["dial_r"]
    if p["dial_ring"]:
        ring_mat = material("Dial ring", p["dial_ring"], 0.45)
        cylinder("Dial ring", r + 0.003, 0.0006, (p["dial_x"], 0, top + 0.0003), ring_mat, dial,
                 axis="Z", segments=32, collision=False)
    knob_h = min(0.0045, H - top - 0.0001)
    cylinder("Dial knob", r, knob_h, (p["dial_x"], 0, top + knob_h / 2), knob, dial,
             axis="Z", segments=32)
    box("Dial pointer", (r * 1.7, 0.006, 0.003), (p["dial_x"], 0, top + knob_h + 0.0005 - 0.0015),
        knob, dial, bevel=0.0008, collision=False)
    box("Dial mark", (0.0035, 0.0012, 0.0003), (p["dial_x"] + r * 0.7, 0, top + knob_h + 0.00015),
        material("White dial mark", (0.9, 0.9, 0.9), 0.4), dial, collision=False)
    jacks = component(whole, "jacks")
    n = len(p["jacks"])
    for i, colour in enumerate(p["jacks"]):
        y = -(n - 1) / 2 * p["jack_pitch"] + i * p["jack_pitch"]
        cylinder(f"Jack collar {i}", 0.0042, 0.0014, (p["jack_x"], y, top + 0.0007),
                 red if colour == "red" else black, jacks, axis="Z", segments=16,
                 collision=False)
        cylinder(f"Jack hole {i}", 0.0019, 0.0002, (p["jack_x"], y, top + 0.0015), black, jacks,
                 axis="Z", segments=12, collision=False)


def pliers(whole, p):
    steel = material("Polished steel", (0.56, 0.58, 0.60), 0.28, metallic=0.9)
    dark = material("Dark steel joint", (0.25, 0.26, 0.27), 0.35, metallic=0.85)
    colours = {"red": material("Red dipped grip", (0.62, 0.025, 0.03), 0.42),
               "blue": material("Blue grip insert", (0.03, 0.12, 0.55), 0.40),
               "yellow": material("Yellow grip", (0.92, 0.62, 0.02), 0.45),
               "black": material("Black grip end", (0.015, 0.015, 0.017), 0.45)}
    t = p["thickness"]
    zc = t / 2
    jx, jw0, jw1, jh0, jh1 = p["jaw"]
    head = component(whole, "jaws")
    for sign in (-1, 1):
        rod(f"Jaw {'left' if sign < 0 else 'right'}", (jx, sign * jw0 / 2), (p["tip_x"], sign * jw1 / 4),
            jw0, jw1 / 2, jh0, jh1, zc, steel, head)
    for k in range(p.get("notches", 0)):
        x = jx + 0.012 + k * 0.006
        h = jh0 + (jh1 - jh0) * (x - jx) / (p["tip_x"] - jx)
        box(f"Stripping notch {k}", (0.0012, jw0 * 1.5, 0.0004), (x, 0, zc + h / 2 - 0.0001),
            dark, head, collision=False)
    pivot = component(whole, "pivot")
    x0, x1 = p["joint"]
    rod("Pivot block", (x0, 0), (x1, 0), p["pivot_w"], p["pivot_w"] * 0.9, t * 0.86, t * 0.84,
        zc, steel, pivot)
    cylinder("Rivet", p["pivot_w"] * 0.26, 0.0006, ((x0 + x1) / 2, 0, zc + t * 0.43 + 0.0002),
             dark, pivot, axis="Z", segments=16, collision=False)
    necks = component(whole, "necks")
    (nx0, ny0), (nx1, ny1) = p["neck"]
    for sign in (-1, 1):
        rod(f"Neck {sign}", (nx0, sign * ny0), (nx1, sign * ny1), p["neck_w"], p["neck_w"],
            t * 0.70, t * 0.72, zc, steel, necks)
    for side, sign in (("left", -1), ("right", 1)):
        grip = component(whole, f"handle_{side}")
        pts = p["handle"]
        colour = colours[p["grip_colors"][0 if sign < 0 else 1]]
        for k in range(len(pts) - 1):
            (ax, ay), (bx, by) = pts[k], pts[k + 1]
            rod(f"Grip {side} {k}", (ax, sign * ay), (bx, sign * by), p["handle_w"][k],
                p["handle_w"][k + 1], t * (0.92 if k == 0 else 1.0), t, zc, colour, grip)
        ex, ey = pts[-1]
        end = colours[p["end_color"]] if "end_color" in p else colour
        rod(f"Grip {side} end", (ex, sign * ey), (p["butt_x"], sign * (ey + 0.0004)),
            p["handle_w"][-1], p["handle_w"][-1] * 0.7, t, t * 0.75, zc, end, grip)


def solder_spool(whole, p):
    black = material("Black reel plastic", (0.02, 0.02, 0.022), 0.40)
    solder = material("Silver solder wire", (0.70, 0.71, 0.72), 0.30, metallic=0.85)
    fr, ft, h, hole = p["flange_r"], p["flange_t"], p["height"], p["hole_r"]
    reel = component(whole, "reel")
    for name, z in (("bottom", ft / 2), ("top", h - ft / 2)):
        tube(f"Flange {name}", fr, hole, ft, (0, 0, z), black, reel, axis="Z", segments=40,
             collision=False)
        ring_sectors(f"Flange {name}", hole, fr, z - ft / 2, z + ft / 2, reel)
    tube("Reel core", p["core_r"], hole, h - 2 * ft, (0, 0, h / 2), black, reel, axis="Z",
         segments=24, collision=False)
    wire = component(whole, "wire")
    tube("Wound solder", p["wire_r"], p["core_r"], h - 2 * ft - 0.0002, (0, 0, h / 2), solder,
         wire, axis="Z", segments=40, collision=False)
    ring_sectors("Wound solder", hole, p["wire_r"], ft, h - ft, wire)
    # A few winding grooves on the coil read as wrapped wire from the side.
    groove = material("Solder groove", (0.42, 0.43, 0.44), 0.4, metallic=0.8)
    for i in range(1, 6):
        z = ft + i * (h - 2 * ft) / 6
        tube(f"Winding groove {i}", p["wire_r"] + 0.0002, p["wire_r"] - 0.0003, 0.0004,
             (0, 0, z), groove, wire, axis="Z", segments=40, collision=False)


def drink_can(whole, p):
    alu = material("Brushed aluminium", (0.74, 0.75, 0.77), 0.25, metallic=0.95)
    red = material("Red can print", (0.70, 0.03, 0.04), 0.30, metallic=0.3)
    dark = material("Dark red stripe", (0.30, 0.01, 0.02), 0.35, metallic=0.2)
    shell = component(whole, "can")
    lathe("Can body", p["rings"], alu, shell, segments=40)
    band = component(whole, "print_band")
    z0, z1 = p["band"]
    r = 0.0330
    tube("Red band", r + 0.0001, r, z1 - z0, (0, 0, (z0 + z1) / 2), red, band, axis="Z",
         segments=40, collision=False)
    tube("Dark stripe", r + 0.00015, r + 0.00005, 0.004, (0, 0, z1 - 0.010), dark, band,
         axis="Z", segments=40, collision=False)
    top = p["rings"][-1][0]
    lid = component(whole, "lid")
    cylinder("Lid panel", 0.0250, 0.0004, (0, 0, top - 0.0002 + 0.0002), alu, lid, axis="Z",
             segments=32, collision=False)
    box("Pull tab", (0.020, 0.010, 0.0006), (0.006, 0, top + 0.0003),
        material("Tab metal", (0.62, 0.63, 0.65), 0.3, metallic=0.9), lid, collision=False)


def water_bottle(whole, p):
    pet = material("Clear PET with water", (0.70, 0.84, 0.92), 0.06, transmission=0.92)
    pet.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.33
    label = material("Blue and white label", (0.08, 0.30, 0.70), 0.45)
    white = material("White label stripe", (0.88, 0.90, 0.92), 0.5)
    cap_mat = material("Blue bottle cap", (0.03, 0.18, 0.62), 0.35)
    shell = component(whole, "bottle")
    lathe("Bottle body", p["rings"], pet, shell, segments=36)
    band = component(whole, "label")
    z0, z1 = p["label"]
    tube("Label band", 0.0326, 0.0325, z1 - z0, (0, 0, (z0 + z1) / 2), label, band, axis="Z",
         segments=36, collision=False)
    tube("Label stripe", 0.03265, 0.03258, 0.010, (0, 0, (z0 + z1) / 2), white, band, axis="Z",
         segments=36, collision=False)
    collar = component(whole, "neck_ring")
    cz, ch, cr = p["collar"]
    cylinder("Neck ring", cr, ch, (0, 0, cz + ch / 2), pet, collar, axis="Z", segments=24)
    cap = component(whole, "cap")
    z0, z1, r = p["cap"]
    cylinder("Cap", r, z1 - z0, (0, 0, (z0 + z1) / 2), cap_mat, cap, axis="Z", segments=24)
    for i in range(18):
        a = i * math.tau / 18
        box(f"Cap rib {i}", (0.0008, 0.0012, (z1 - z0) * 0.8),
            (r * math.cos(a), r * math.sin(a), (z0 + z1) / 2 - 0.0005), cap_mat, cap,
            collision=False).rotation_euler.z = a


def calculator(whole, p):
    L, W = p["size"]
    zf, zb, kh = p["z_front"], p["z_back"], p["key_h"]
    grey = material("Silver grey calculator body", (0.52, 0.54, 0.56), 0.35, metallic=0.2)
    face = material("Dark grey key panel", (0.16, 0.17, 0.18), 0.40)
    lcd = material("Grey-green LCD", (0.50, 0.53, 0.46), 0.20)
    keys = material("Black keys", (0.03, 0.03, 0.035), 0.38)
    light = material("Light grey keys", (0.70, 0.71, 0.72), 0.40)
    orange = material("Orange function keys", (0.85, 0.35, 0.05), 0.40)

    def top(x):
        return zf + (x + L / 2) / L * (zb - zf)

    shell = component(whole, "body")
    c = 0.004
    pts = []
    for x in (-L / 2, L / 2):
        xi = x - math.copysign(c, x)
        for y in (-W / 2, W / 2):
            yi = y - math.copysign(c, y)
            pts += [(x, yi, 0.0015), (xi, y, 0.0015), (x, yi, top(x) - 0.002), (xi, y, top(x) - 0.002),
                    (xi, yi, top(x)), (xi * 0.998, yi, 0), (xi, yi * 0.97, 0)]
    vertices, faces = convex(pts)
    mesh("Calculator shell", vertices, faces, shell, grey)
    # Collider covers the shell and the key caps that rise above it.
    hull_collider("Calculator shell and keys",
                  pts + [(x, y, top(x) + kh) for x in (-L / 2 + 0.006, L / 2 - 0.006)
                         for y in (-W / 2 + 0.006, W / 2 - 0.006)], shell)
    display = component(whole, "display")
    dx0, dx1 = L / 2 - 0.048, L / 2 - 0.010
    slab("Display bezel", dx0, dx1, -W / 2 + 0.008, W / 2 - 0.008, top, 0.0004, face, display)
    slab("LCD", dx0 + 0.005, dx1 - 0.005, -W / 2 + 0.013, W / 2 - 0.013,
         lambda x: top(x) + 0.0004, 0.0003, lcd, display)
    keypad = component(whole, "keys")
    slab("Key panel", -L / 2 + 0.006, dx0 - 0.004, -W / 2 + 0.005, W / 2 - 0.005, top, 0.0003,
         face, keypad)
    rows, cols = p["rows"], p["cols"]
    x0, x1 = -L / 2 + 0.010, dx0 - 0.008
    pitch_x, pitch_y = (x1 - x0) / rows, (W - 0.016) / cols
    for r in range(rows):
        for k in range(cols):
            cx = x1 - (r + 0.5) * pitch_x
            cy = -(W - 0.016) / 2 + (k + 0.5) * pitch_y
            small = r < 3
            sx = pitch_x * (0.55 if small else 0.68)
            sy = pitch_y * (0.70 if small else 0.76)
            surf = light if small else keys
            if r == 3 and k >= 3:
                surf = orange
            slab(f"Key {r}_{k}", cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2,
                 lambda x: top(x) + 0.0003, kh - 0.0003, surf, keypad)


def computer_mouse(whole, p):
    shell_mat = material("Silver mouse shell", (0.55, 0.57, 0.60), 0.30, metallic=0.35)
    button_mat = material("Dark grey buttons", (0.12, 0.13, 0.15), 0.32)
    rubber = material("Black wheel", (0.01, 0.01, 0.012), 0.5)
    shell = component(whole, "shell")
    na, zs, inset = 12, 0.005, 0.003
    stations = p["stations"]
    rings = []
    for x, w, h in stations:
        pts = [(x, w / 2 - inset, 0.0)]
        for k in range(na + 1):
            a = k * math.pi / na
            pts.append((x, w / 2 * math.cos(a), zs + (h - zs) * math.sin(a) ** 0.8))
        pts.append((x, -(w / 2 - inset), 0.0))
        rings.append(pts)
    m = len(rings[0])
    vertices = [v for r in rings for v in r]
    faces = [tuple(range(m - 1, -1, -1)), tuple((len(rings) - 1) * m + i for i in range(m))]
    for s in range(len(rings) - 1):
        a, b = s * m, (s + 1) * m
        faces.extend((a + i, a + (i + 1) % m, b + (i + 1) % m, b + i) for i in range(m))
    obj = mesh("Mouse shell", vertices, faces, shell, shell_mat)
    obj.data.materials.append(button_mat)
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
        c = face.center
        if c.x > 0.004 and c.z > 0.006 and len(face.vertices) == 4:
            face.material_index = 1
    front = [v for v in vertices if v[0] >= stations[5][0]]
    back = [v for v in vertices if v[0] <= stations[6][0]]
    hull_collider("Mouse front", front, shell)
    hull_collider("Mouse back", back, shell)
    wheel = component(whole, "wheel")
    wx = p["wheel_x"]
    # Surface height at the wheel from the station table.
    for (xa, _, ha), (xb, _, hb) in zip(stations, stations[1:]):
        if xa <= wx <= xb:
            hz = ha + (hb - ha) * (wx - xa) / (xb - xa)
    # Dark seam between the two buttons, following the top centre line.
    seam = [(x, y, zs + (h - zs) + dz) for x, _, h in stations if x > 0.03
            for y in (-0.0006, 0.0006) for dz in (-0.0008, 0.00025)]
    seam += [(wx + 0.007, y, hz + dz) for y in (-0.0006, 0.0006) for dz in (-0.0008, 0.00025)]
    hull("Button seam", seam, rubber, wheel, collision=False)
    cylinder("Scroll wheel", 0.0055, 0.0065, (wx, 0, hz - 0.0025), rubber, wheel, axis="Y",
             segments=20)


def circuit_board(whole, p):
    L, W, T = p["size"]
    s = p["standoff"]
    pcb = material("Solder mask", p["color"], 0.35)
    nylon = material("Black nylon standoff", (0.02, 0.02, 0.02), 0.5)
    chip = material("Black IC package", (0.02, 0.02, 0.025), 0.35)
    silver = material("Tinned metal", (0.68, 0.69, 0.70), 0.25, metallic=0.9)
    gold = material("Gold pins", (0.80, 0.62, 0.20), 0.25, metallic=0.9)
    white = material("White LED", (0.92, 0.92, 0.88), 0.3)
    board = component(whole, "pcb")
    top = s + T
    box("Printed circuit board", (L, W, T), (0, 0, s + T / 2), pcb, board, bevel=0.0003)
    inset, foot_r = p["foot_inset"], p["foot_r"]
    corners = [(x, y) for x in (-L / 2 + inset, L / 2 - inset) for y in (-W / 2 + inset, W / 2 - inset)]
    for i, (x, y) in enumerate(corners):
        cylinder(f"Mounting ring {i}", foot_r, 0.0002, (x, y, top + 0.0001), silver, board,
                 axis="Z", segments=16, collision=False)
    feet = component(whole, "standoffs")
    for i, (x, y) in enumerate(corners):
        cylinder(f"Standoff {i}", foot_r, s, (x, y, s / 2), nylon, feet, axis="Z", segments=12)
    chips = component(whole, "chips")
    for name, dims, pos in p["chips"]:
        box(f"Chip {name}", dims, (pos[0], pos[1], top + dims[2] / 2), chip, chips, bevel=0.0002)
    for name, dims, pos in p.get("metal_parts", []):
        box(name, dims, (pos[0], pos[1], top + dims[2] / 2), silver, chips)
    if "button" in p:
        bx, by = p["button"]
        cylinder("Button cap", 0.0017, 0.0006, (bx, by, top + 0.0028), chip, chips, axis="Z",
                 segments=12, collision=False)
    lx, ly = p["led"]
    box("Status LED", (0.002, 0.0012, 0.0008), (lx, ly, top + 0.0004), white, chips,
        collision=False)
    if p.get("usb"):
        usb = component(whole, "usb")
        box("USB connector", (0.0085, 0.0076, 0.0030), (-L / 2 + 0.0032, 0, top + 0.0015), silver,
            usb, bevel=0.0003)
    headers = component(whole, "headers")
    hh, pin_h = p["header_h"], p["pin_h"]
    for side, x, y, pins in p["headers"]:
        hl = pins * 0.00254
        box(f"Header {side}", (hl, 0.0025, hh), (x, y, top + hh / 2), chip, headers,
            collision=False)
        box_collider(f"Header {side}", (hl, 0.0025, hh + pin_h), (x, y, top + (hh + pin_h) / 2),
                     headers)
        for k in range(pins):
            px = x - hl / 2 + 0.00127 + k * 0.00254
            box(f"Pin {side} {k}", (0.0006, 0.0006, pin_h), (px, y, top + hh + pin_h / 2), gold,
                headers, collision=False)


def soldering_iron(whole, p):
    black = material("Black handle plastic", (0.015, 0.016, 0.018), 0.40)
    blue = material("Blue rubber grip", (0.04, 0.16, 0.55), 0.55)
    steel = material("Stainless barrel", (0.62, 0.64, 0.66), 0.25, metallic=0.9)
    tip_mat = material("Tinned tip", (0.40, 0.40, 0.41), 0.3, metallic=0.9)
    z = p["grip_r"]
    for name, rings, surf in (("handle", p["handle"], black), ("grip", p["grip"], blue),
                              ("guard", p["guard"], black), ("barrel", p["barrel"], steel),
                              ("tip", p["tip"], tip_mat)):
        node = component(whole, name)
        x_solid(f"Iron {name}", rings, z, surf, node)
    grip = bpy.data.objects["Iron grip"].parent
    for i in range(6):
        x = p["grip"][0][0] + 0.010 + i * 0.008
        cylinder(f"Grip ridge {i}", p["grip_r"], 0.0025, (x, 0, z), black, grip, segments=16)


def tip_cleaner(whole, p):
    black = material("Black cleaner cup", (0.02, 0.02, 0.022), 0.45)
    brass = material("Brass wool", (0.72, 0.52, 0.18), 0.55, metallic=0.85)
    cup = component(whole, "cup")
    lathe("Cleaner cup", p["rings"], black, cup, segments=40, collision=False)
    pts = []
    for z, r in p["rings"][:p["outer_rings"]]:
        pts.extend(ring(z, r, "Z", 12, circumscribed=True))
    hull_collider("Cleaner cup", pts, cup)
    wool = component(whole, "brass_wool")
    for i, (x, y, z, sx, sy, sz) in enumerate(p["wool"]):
        ellipsoid(f"Wool lump {i}", (sx, sy, sz), (x, y, z), brass, wool, collision=False)
    top = p["rings"][p["outer_rings"] - 1][0]
    hull_collider("Brass wool", [(x, y, z) for x in (-0.012, 0.012) for y in (-0.012, 0.012)
                                 for z in (top - 0.010, max(z + sz / 2 for _, _, z, _, _, sz in p["wool"]))],
                  wool)


def battery(whole, p):
    r, L, sx = p["radius"], p["length"], p["split_x"]
    black = material("Black battery wrap", (0.02, 0.02, 0.022), 0.30)
    copper = material("Copper battery wrap", (0.72, 0.38, 0.12), 0.30, metallic=0.5)
    metal = material("Nickel end cap", (0.70, 0.71, 0.72), 0.22, metallic=0.95)
    x_neg, x_pos = -L / 2, L / 2
    body_end = x_pos - 0.0012
    wrap = component(whole, "wrap")
    x0 = x_neg + 0.0004
    cylinder("Black wrap", r, sx - x0, ((x0 + sx) / 2, 0, r), black, wrap, segments=24,
             collision=False)
    cylinder("Copper wrap", r, body_end - sx, ((sx + body_end) / 2, 0, r), copper, wrap,
             segments=24, collision=False)
    hull_collider("Cell body", shifted(ring(x_neg, r, "X", 16, vertex_down=True)
                                       + ring(body_end, r, "X", 16, vertex_down=True), (0, 0, r)),
                  wrap)
    ends = component(whole, "terminals")
    cylinder("Negative end", r * 0.86, 0.0004, (x_neg + 0.0002, 0, r), metal, ends, segments=24,
             collision=False)
    cylinder("Positive shoulder", r * 0.7, 0.0003, (body_end + 0.00015, 0, r), metal, ends,
             segments=24, collision=False)
    cylinder("Positive nub", 0.0027, x_pos - body_end, ((x_pos + body_end) / 2, 0, r), metal, ends,
             segments=16)


def container(whole, p):
    clear = p.get("columns", 1) > 1
    surface = material("Translucent blue plastic" if clear else "Grey moulded plastic",
                       p["color"], 0.18 if clear else 0.55, transmission=0.65 if clear else 0)
    if clear:
        surface.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.49
    open_container(whole, p, surface)


def tool_cup(whole, p):
    r, h, t, f = p["radius"], p["height"], p["wall"], p["floor"]
    mesh_mat = material("Black wire mesh", (0.035, 0.037, 0.04), 0.45, metallic=0.6)
    rim_mat = material("Black steel rim", (0.01, 0.01, 0.012), 0.30, metallic=0.7)
    floor = component(whole, "floor")
    cylinder("Cup floor", r, f, (0, 0, f / 2), rim_mat, floor, axis="Z", segments=p["segments"])
    wall = component(whole, "wall")
    tube("Mesh wall", r, r - t, h - f, (0, 0, (h + f) / 2), mesh_mat, wall, axis="Z",
         segments=p["segments"])
    rims = component(whole, "rims")
    for name, z in (("top", h - 0.003), ("bottom", f + 0.003)):
        tube(f"Rim {name}", r + 0.0004, r - t - 0.0002, 0.006, (0, 0, z), rim_mat, rims,
             axis="Z", segments=40, collision=False)
    # Raised diamond-mesh strands suggest the perforated wall.
    for k in range(p["strands"]):
        a = k * math.tau / p["strands"]
        for lean in (-1, 1):
            obj = box(f"Mesh strand {k} {lean}", (0.0012, 0.0006, h - f - 0.012),
                      (r * math.cos(a), r * math.sin(a), (h + f) / 2), rim_mat, rims,
                      collision=False)
            obj.rotation_euler = (0, lean * 0.2, a + math.pi / 2)


def precision_screwdriver(whole, p):
    cap_mat = material("Swivel cap", p["cap_color"], 0.30)
    handle_mat = material("Fluted handle", p["handle_color"], 0.35)
    stripe = material("Flute stripes", p["stripe_color"], 0.35)
    steel = material("Chrome shaft", (0.62, 0.64, 0.66), 0.22, metallic=0.92)
    z = p["radius"]
    for name, rings, surf in (("cap", p["cap"], cap_mat), ("handle", p["handle"], handle_mat),
                              ("shaft", p["shaft"], steel)):
        node = component(whole, name)
        x_solid(f"Driver {name}", rings, z, surf, node,
                collision_sides=8 if name != "shaft" else None)
    flutes = bpy.data.objects["Driver handle"].parent
    x0, x1 = p["handle"][0][0] + 0.004, p["handle"][-3][0] - 0.002
    for i in range(6):
        a = i * math.tau / 6
        rr = p["radius"] - 0.0011
        cylinder(f"Flute stripe {i}", 0.0012, x1 - x0, ((x0 + x1) / 2, rr * math.cos(a),
                 z + rr * math.sin(a)), stripe, flutes, segments=8, collision=False)


def electrical_tape(whole, p):
    black = material("Black PVC tape", (0.012, 0.012, 0.014), 0.25)
    core_mat = material("Cardboard core", (0.62, 0.50, 0.34), 0.7)
    ro, ri, rc, w = p["outer_r"], p["inner_r"], p["core_inner_r"], p["width"]
    tape = component(whole, "tape")
    tube("Tape roll", ro, ri, w, (0, 0, w / 2), black, tape, axis="Z", segments=40,
         collision=False)
    ring_sectors("Tape roll", rc, ro, 0, w, tape, n=12)
    core = component(whole, "core")
    tube("Paper core", ri, rc, w, (0, 0, w / 2), core_mat, core, axis="Z", segments=32,
         collision=False)


def tweezers(whole, p):
    steel = material("Stainless tweezers", (0.66, 0.68, 0.70), 0.25, metallic=0.92)
    grip = material("Grip serration", (0.42, 0.43, 0.45), 0.4, metallic=0.85)
    t = p["blade_t"]
    pts = p["path"]
    for side, sign in (("left", -1), ("right", 1)):
        leg = component(whole, f"leg_{side}")
        for k in range(len(pts) - 1):
            (ax, ay, ah), (bx, by, bh) = pts[k], pts[k + 1]
            rod(f"Leg {side} {k}", (ax, sign * ay), (bx, sign * by), t, t, ah, bh,
                0, steel, leg)
    # Lift each leg segment so its lower edge rests on the table.
    for obj in list(whole.children_recursive):
        if obj.type == "MESH" and obj.data.vertices:
            zmin = min(v.co.z for v in obj.data.vertices)
            for v in obj.data.vertices:
                v.co.z -= zmin
    hinge = component(whole, "hinge")
    x0 = pts[0][0]
    box("Hinge spacer", (0.010, 2 * pts[0][1] + t, p["hinge_h"]),
        (x0 + 0.005, 0, p["hinge_h"] / 2), steel, hinge, bevel=0.0003)
    for side, sign in (("left", -1), ("right", 1)):
        (gx0, gy0, _), (gx1, gy1, _) = pts[1], pts[2]
        for i in range(8):
            u = 0.2 + i * 0.08
            x = gx0 + (gx1 - gx0) * u
            y = sign * (gy0 + (gy1 - gy0) * u + t / 2 + 0.00005)
            box(f"Serration {side} {i}", (0.0008, 0.0001, 0.0028), (x, y, 0.0024), grip, hinge,
                collision=False)


def power_adapter(whole, p):
    L, W, H = p["size"]
    black = material("Black adapter shell", (0.02, 0.02, 0.022), 0.35)
    label = material("Grey rating label", (0.42, 0.43, 0.45), 0.5)
    socket = material("Socket recess", (0.003, 0.003, 0.003), 0.6)
    shell = component(whole, "shell")
    rounded_box("Adapter shell", (L, W, H), (0, 0, H / 2), black, shell, radius=0.006)
    sticker = component(whole, "label")
    box("Rating label", (0.060, 0.030, 0.0003), (-0.010, 0, H + 0.00015), label, sticker,
        collision=False)
    box("Seam", (L - 0.004, 0.0006, 0.0006), (0, -W / 2 - 0.0001, H * 0.45), socket, sticker,
        collision=False)
    inlet = component(whole, "inlet")
    box("Mains inlet", (0.0006, 0.020, 0.010), (-L / 2 - 0.00025, 0, H / 2), socket, inlet,
        collision=False)
    stub = component(whole, "cable_stub")
    cylinder("Strain relief", 0.0055, 0.012, (L / 2 + 0.006, 0, H / 2), black, stub,
             radius_end=0.0035, segments=16)
    cylinder("Cable stub", 0.0028, p["stub"], (L / 2 + 0.012 + p["stub"] / 2, 0, H / 2), black,
             stub, segments=12)


def lab_notebook(whole, p):
    L, W, H = p["size"]
    cover = material("Grey-green cover", (0.30, 0.38, 0.33), 0.6)
    paper = material("Page block", (0.90, 0.89, 0.84), 0.75)
    wire = material("Spiral wire", (0.60, 0.62, 0.64), 0.25, metallic=0.9)
    gutter = p["gutter"]
    book = component(whole, "covers")
    bw = W - gutter
    yc = -gutter / 2
    box("Back cover", (L, bw, 0.0010), (0, yc, 0.0005), cover, book, collision=False)
    box("Front cover", (L, bw, 0.0010), (0, yc, H - 0.0005), cover, book, collision=False)
    box_collider("Book block", (L, bw, H), (0, yc, H / 2), book)
    pages = component(whole, "pages")
    box("Pages", (L - 0.003, bw - 0.003, H - 0.002), (0, yc - 0.0015, H / 2), paper, pages,
        collision=False)
    spiral = component(whole, "spiral")
    r = H / 2 - 0.0002
    yl = W / 2 - gutter / 2 - 0.001
    n = p["loops"]
    for i in range(n):
        x = -L / 2 + 0.008 + i * (L - 0.016) / (n - 1)
        tube(f"Loop {i:02d}", r, r - 0.0008, 0.0011, (x, yl, H / 2), wire, spiral, axis="X",
             segments=12, collision=False)
    box_collider("Spiral", (L - 0.010, gutter + 0.002, H - 0.0004),
                 (0, W / 2 - gutter / 2 - 0.001, H / 2), spiral)


def test_probe(whole, p):
    body_mat = material("Probe plastic", p["color"], 0.35)
    steel = material("Probe tip steel", (0.66, 0.68, 0.70), 0.22, metallic=0.92)
    z = p["guard_r"]
    for name, rings, surf in (("boot", p["boot"], body_mat), ("handle", p["handle"], body_mat),
                              ("guard", p["guard"], body_mat), ("nose", p["nose"], body_mat),
                              ("tip", p["tip"], steel)):
        node = component(whole, name)
        x_solid(f"Probe {name}", rings, z, surf, node)
    grip = bpy.data.objects["Probe handle"].parent
    ridge = material("Probe grip rings", tuple(c * 0.6 for c in p["color"]), 0.5)
    for i in range(5):
        x = p["handle"][0][0] + 0.030 + i * 0.008
        cylinder(f"Grip ring {i}", p["handle"][1][1] + 0.0002, 0.0018, (x, 0, z), ridge, grip,
                 segments=16, collision=False)


BUILDERS = {"multimeter": multimeter, "pliers": pliers, "solder_spool": solder_spool,
            "drink_can": drink_can, "water_bottle": water_bottle, "calculator": calculator,
            "computer_mouse": computer_mouse, "circuit_board": circuit_board, "battery": battery,
            "soldering_iron": soldering_iron, "tip_cleaner": tip_cleaner,
            "tool_cup": tool_cup, "precision_screwdriver": precision_screwdriver,
            "wire_stripper": pliers, "electrical_tape": electrical_tape, "tweezers": tweezers,
            "power_adapter": power_adapter, "lab_notebook": lab_notebook, "test_probe": test_probe,
            "tool_tray": container, "parts_box": container}


def build_workbench_asset(category, variant="default"):
    root = asset(category, "bottom centre of the resting pose",
                 "+X longest horizontal side; +Y width; +Z up", "-Y is the preview front")
    whole = body(category + ".body", root)
    BUILDERS[category](whole, PROFILES[category][variant])
    finish_whole(whole)
