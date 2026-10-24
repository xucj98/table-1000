"""Soldering-corner props for the electronics workbench (scene 000101), one rigid body each.

Dimensions are engineering estimates in metres from the scene 000101 reference
photo. Origins sit at the bottom centre of the resting pose; +X is the longest
horizontal side, +Z up, -Y the preview front. No branding or text is modelled.
"""

import math
import random

import bmesh
import bpy

from table_1000.modeling.asset_builders import asset, body, box, cylinder, material, mesh, tube
from table_1000.modeling.small_props_assets import component, finish_whole, rounded_box
from table_1000.modeling.workbench_assets import (
    box_collider, convex, lathe, ring_sectors, x_solid,
)


def convex_triangles(points):
    """Raw triangulated convex hull. Merged n-gons on tilted shapes are not exactly planar
    in float32 and fail the convexity check, so colliders skip the face merge."""
    bm = bmesh.new()
    for p in {tuple(round(c, 7) for c in p) for p in points}:
        bm.verts.new(p)
    result = bmesh.ops.convex_hull(bm, input=list(bm.verts))
    drop = list({v for v in result["geom_interior"] + result["geom_unused"]
                 if isinstance(v, bmesh.types.BMVert)})
    bmesh.ops.delete(bm, geom=drop, context="VERTS")
    bmesh.ops.triangulate(bm, faces=list(bm.faces))
    bm.verts.index_update()
    vertices = [tuple(v.co) for v in bm.verts]
    faces = [tuple(v.index for v in f.verts) for f in bm.faces]
    bm.free()
    return vertices, faces


def hull_collider(name, points, parent):
    vertices, faces = convex_triangles(points)
    return mesh("Collision_" + name, vertices, faces, parent, shape="CONVEX_HULL")


def hull(name, points, surface, parent, collision=True, smooth=False):
    """Convex visual solid from points, with the matching triangulated convex collider."""
    vertices, faces = convex(points)
    obj = mesh(name, vertices, faces, parent, surface)
    if smooth:
        for face in obj.data.polygons:
            face.use_smooth = True
    if collision:
        hull_collider(name, points, parent)
    return obj


# --------------------------------------------------------------------------- vector helpers

def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def unit(a):
    n = math.sqrt(dot(a, a))
    return tuple(x / n for x in a)


def frame(origin, ex, ey, ez):
    """Map local (x, y, z) into the frame with the given origin and axes."""
    def f(p):
        return tuple(origin[i] + p[0] * ex[i] + p[1] * ey[i] + p[2] * ez[i] for i in range(3))
    return f


def perpendicular(t):
    helper = (0, 0, 1) if abs(t[2]) < 0.9 else (1, 0, 0)
    return unit(cross(t, helper))


def disc(center, axis, radius, n, phase=0.5):
    """Polygon ring around ``axis`` (vertex count n)."""
    a = unit(axis)
    e1 = perpendicular(a)
    e2 = cross(a, e1)
    return [add(center, add(scale(e1, radius * math.cos((k + phase) * math.tau / n)),
                            scale(e2, radius * math.sin((k + phase) * math.tau / n))))
            for k in range(n)]


def smooth_quads(obj):
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4


def ocyl(name, center, axis, radius, depth, surface, parent, radius_end=None, segments=16,
         collision=True):
    """Cylinder (or cone frustum) of any orientation; convex, so one hull collider."""
    a = unit(axis)
    r1 = radius if radius_end is None else radius_end
    pts = (disc(add(center, scale(a, -depth / 2)), a, radius, segments)
           + disc(add(center, scale(a, depth / 2)), a, r1, segments))
    n = segments
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(k, (k + 1) % n, n + (k + 1) % n, n + k) for k in range(n)]
    obj = mesh(name, pts, faces, parent, surface)
    smooth_quads(obj)
    if collision:
        # Explicit prism (unrounded points): a hull of a tilted disc is not exactly planar.
        mesh("Collision_" + name, pts, faces, parent, shape="CONVEX_HULL")
    return obj


def obox(name, f, dims, center, surface, parent, collision=True):
    """Box given in the local frame ``f`` (centre and dimensions in that frame)."""
    pts = [f((center[0] + x * dims[0] / 2, center[1] + y * dims[1] / 2, center[2] + z * dims[2] / 2))
           for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    return hull(name, pts, surface, parent, collision=collision)


def spline(points, samples=6):
    """Catmull-Rom curve through ``points``."""
    pts = [points[0]] + list(points) + [points[-1]]
    out = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
        for k in range(samples):
            t = k / samples
            out.append(tuple(0.5 * (2 * p1[j] + (-p0[j] + p2[j]) * t
                                    + (2 * p0[j] - 5 * p1[j] + 4 * p2[j] - p3[j]) * t * t
                                    + (-p0[j] + 3 * p1[j] - 3 * p2[j] + p3[j]) * t ** 3)
                             for j in range(3)))
    out.append(tuple(points[-1]))
    return out


def path_frames(path):
    """Parallel-transport normals along a polyline."""
    tangents = []
    for i in range(len(path)):
        a, b = path[max(i - 1, 0)], path[min(i + 1, len(path) - 1)]
        tangents.append(unit(tuple(y - x for x, y in zip(a, b))))
    normals = [perpendicular(tangents[0])]
    for t in tangents[1:]:
        n = normals[-1]
        n = unit(tuple(n[j] - dot(n, t) * t[j] for j in range(3)))
        normals.append(n)
    return tangents, normals


def sweep(name, path, radius, surface, parent, sides=8):
    """Smooth closed tube along ``path`` (visual only)."""
    tangents, normals = path_frames(path)
    vertices = []
    for p, t, n in zip(path, tangents, normals):
        b = cross(t, n)
        vertices.extend(add(p, add(scale(n, radius * math.cos(k * math.tau / sides)),
                                   scale(b, radius * math.sin(k * math.tau / sides))))
                        for k in range(sides))
    m = len(path)
    faces = [tuple(range(sides - 1, -1, -1)), tuple((m - 1) * sides + k for k in range(sides))]
    for i in range(m - 1):
        a, c = i * sides, (i + 1) * sides
        faces.extend((a + k, a + (k + 1) % sides, c + (k + 1) % sides, c + k) for k in range(sides))
    obj = mesh(name, vertices, faces, parent, surface)
    smooth_quads(obj)
    return obj


def wire_colliders(name, path, radius, parent, sides=6):
    """One convex prism per polyline segment (circumscribing the wire)."""
    r = radius / math.cos(math.pi / sides)
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        t = unit(tuple(y - x for x, y in zip(a, b)))
        hull_collider(f"{name} segment {i:02d}", disc(a, t, r, sides) + disc(b, t, r, sides), parent)


def wire(name, controls, radius, surface, parent, samples=6, sides=8):
    """Visual smooth wire through ``controls``; colliders follow the control polyline."""
    path = [(x, y, max(z, radius)) for x, y, z in spline(controls, samples)]
    sweep(name, path, radius, surface, parent, sides)
    wire_colliders(name, controls, radius, parent)


def chamfer_section(f, x, w, z0, z1, c):
    """Chamfered rectangle (8 points) at local x, width w, from z0 to z1."""
    c = min(c, w / 2 - 1e-5, (z1 - z0) / 2 - 1e-5)
    pts = [(x, -w / 2 + c, z0), (x, w / 2 - c, z0), (x, w / 2, z0 + c), (x, w / 2, z1 - c),
           (x, w / 2 - c, z1), (x, -w / 2 + c, z1), (x, -w / 2, z1 - c), (x, -w / 2, z0 + c)]
    return [f(p) for p in pts]


def alligator(prefix, f, boot_mat, steel, parent, size=1.0, boot=True):
    """Alligator clip in local frame ``f``: rear (wire end) at x=0, jaw tip at +x, base at z=0."""
    k = size
    if boot:
        hull(f"{prefix} boot", chamfer_section(f, 0.0, 0.0050 * k, 0.0004 * k, 0.0052 * k, 0.0012 * k)
             + chamfer_section(f, 0.006 * k, 0.0072 * k, 0.0, 0.0068 * k, 0.0018 * k)
             + chamfer_section(f, 0.025 * k, 0.0088 * k, 0.0, 0.0092 * k, 0.0022 * k),
             boot_mat, parent, smooth=True)
        jaw0 = 0.024 * k
    else:
        hull(f"{prefix} body", chamfer_section(f, 0.0, 0.0040 * k, 0.0006 * k, 0.0040 * k, 0.0008 * k)
             + chamfer_section(f, 0.024 * k, 0.0060 * k, 0.0, 0.0085 * k, 0.0010 * k),
             steel, parent)
        jaw0 = 0.022 * k
    tip = 0.043 * k
    hull(f"{prefix} lower jaw", chamfer_section(f, jaw0, 0.0050 * k, 0.0, 0.0024 * k, 0.0006 * k)
         + chamfer_section(f, tip, 0.0026 * k, 0.0, 0.0018 * k, 0.0005 * k), steel, parent)
    hull(f"{prefix} upper jaw", chamfer_section(f, jaw0, 0.0050 * k, 0.0060 * k, 0.0086 * k, 0.0006 * k)
         + chamfer_section(f, tip, 0.0026 * k, 0.0020 * k, 0.0038 * k, 0.0005 * k), steel, parent)
    dark = material(f"{prefix} teeth", (0.30, 0.31, 0.32), 0.4, metallic=0.8)
    for i in range(4):
        x = jaw0 + 0.004 * k + i * 0.0038 * k
        hull(f"{prefix} tooth {i}", [f((x + dx, y, z)) for dx in (-0.0007 * k, 0.0007 * k)
                                     for y in (-0.0018 * k, 0.0018 * k)
                                     for z in (0.0022 * k, 0.0029 * k)], dark, parent, collision=False)


def bumpy_slab(name, x0, x1, y0, y1, z0, z1, amplitude, surface, parent, nx=16, ny=8, seed=0):
    """Closed slab with a randomly bumpy top (brass wool, scouring pad)."""
    rng = random.Random(seed)
    top = []
    for j in range(ny + 1):
        for i in range(nx + 1):
            edge = i in (0, nx) or j in (0, ny)
            dz = 0 if edge else rng.uniform(-amplitude, amplitude)
            top.append((x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny, z1 + dz - (0.3 * amplitude if edge else 0)))
    count = len(top)
    bottom = [(x, y, z0) for x, y, _ in top]
    vertices = top + bottom
    faces = []
    w = nx + 1
    for j in range(ny):
        for i in range(nx):
            a = j * w + i
            faces.append((a, a + 1, a + w + 1, a + w))
            faces.append((count + a, count + a + w, count + a + w + 1, count + a + 1))
    border = ([i for i in range(nx)] + [j * w + nx for j in range(ny)]
              + [ny * w + i for i in range(nx, 0, -1)] + [j * w for j in range(ny, 0, -1)])
    for k, a in enumerate(border):
        b = border[(k + 1) % len(border)]
        faces.append((b, a, count + a, count + b))
    obj = mesh(name, vertices, faces, parent, surface)
    for face in obj.data.polygons:
        face.use_smooth = True
    return obj


# --------------------------------------------------------------------------- profiles

PROFILES = {
    "soldering_station": {"default": {"size": (0.140, 0.120, 0.090), "lid_z": 0.074,
                                      "front": [(0.0, -0.060), (0.010, -0.060), (0.074, -0.0465),
                                                (0.084, -0.0445), (0.090, -0.040)]}},
    "iron_holder": {"default": {"base": (0.110, 0.075), "plate_h": 0.006, "rim_h": 0.016,
                                "wall": 0.004, "pocket_back_y": 0.001, "tilt_deg": 37.0,
                                "coil_low": (-0.038, 0.019, 0.034), "coil_len": 0.085,
                                "r_low": 0.007, "r_mouth": 0.015, "coil_wall": 0.003,
                                "sectors": 12, "wire_r": 0.0013, "turns": 10}},
    "flux_tin": {"default": {"radius": 0.0175, "height": 0.015, "lid_h": 0.0055}},
    "desolder_wick": {"default": {"radius": 0.020, "height": 0.012, "flange": 0.0014,
                                  "braid_r": 0.0150, "hub_r": 0.0085, "hole_r": 0.0050}},
    "helping_hands": {"default": {"base": (0.095, 0.060, 0.018), "rod_y": 0.012, "top_z": 0.150}},
    "heat_sink": {"default": {"size": (0.060, 0.050, 0.040), "base_h": 0.005, "fins": 14,
                              "fin_t": 0.0013}},
    "screwdriver_set": {"default": {"size": (0.150, 0.110, 0.025), "drivers": 6, "pitch": 0.0165,
                                    "header": 0.036}},
    "scope_probe": {"default": {"axis_z": 0.0085}},
    "clip_lead": {
        "red": {"color": (0.70, 0.03, 0.03),
                "a": ((-0.052, -0.016), math.pi + 0.10), "b": ((0.052, -0.008), -0.25),
                "path": [(-0.040, -0.018), (-0.024, -0.010), (-0.014, 0.010), (0.000, 0.026),
                         (0.018, 0.030), (0.032, 0.020), (0.038, 0.004), (0.044, -0.006)]},
        "black": {"color": (0.025, 0.025, 0.028),
                  "a": ((-0.050, 0.014), math.pi - 0.20), "b": ((0.054, 0.004), 0.15),
                  "path": [(-0.038, 0.016), (-0.022, 0.012), (-0.010, -0.006), (0.004, -0.024),
                           (0.022, -0.026), (0.034, -0.014), (0.040, -0.002), (0.046, 0.003)]}},
}


# --------------------------------------------------------------------------- builders

def soldering_station(whole, p):
    L, W, H = p["size"]
    grey = material("Dark grey station case", (0.11, 0.12, 0.14), 0.45)
    blue = material("Blue station cover", (0.025, 0.06, 0.20), 0.40)
    lcd = material("Dark display window", (0.05, 0.07, 0.07), 0.12)
    digits = material("Pale LCD digits", (0.16, 0.22, 0.19), 0.25)
    knob_mat = material("Blue knob", (0.06, 0.22, 0.65), 0.35)
    black = material("Black socket", (0.015, 0.015, 0.017), 0.4)
    metal = material("Socket ring", (0.55, 0.56, 0.58), 0.3, metallic=0.85)
    vent = material("Vent slot", (0.02, 0.02, 0.025), 0.6)
    front = p["front"]
    back = W / 2
    c = 0.004

    def profile_points(z0, z1):
        pts = []
        for z, y in front:
            if z0 - 1e-9 <= z <= z1 + 1e-9:
                pts.append((y, z))
        # interpolate cut heights
        for zc in (z0, z1):
            for (za, ya), (zb, yb) in zip(front, front[1:]):
                if za <= zc <= zb and zb > za:
                    pts.append((ya + (yb - ya) * (zc - za) / (zb - za), zc))
        pts += [(back, z0), (back, z1)]
        out = []
        for y, z in pts:
            for x in (-L / 2, L / 2):
                out += [(x - math.copysign(c, x), y, z)]
                out += [(x, y + (c if y < 0 else -c), z)]
        return out

    case = component(whole, "case")
    hull("Lower case", profile_points(0.0, p["lid_z"]), grey, case)
    cover = component(whole, "cover")
    hull("Top cover", profile_points(p["lid_z"], H), blue, cover)
    # Side vents.
    for side in (-1, 1):
        for i in range(5):
            box(f"Vent {side} {i}", (0.0006, 0.040, 0.0025), (side * (L / 2 + 0.0002), 0.012,
                0.024 + i * 0.008), vent, case, collision=False)
    # Sloped front panel frame.
    (za, ya), (zb, yb) = front[1], front[2]
    origin = (0, ya, za)
    ev = unit((0, yb - ya, zb - za))
    en = unit((0, -(zb - za), yb - ya))
    f = frame(origin, (1, 0, 0), ev, en)
    panel = component(whole, "display")
    obox("Display bezel", f, (0.058, 0.030, 0.0012), (-0.012, 0.040, 0.0004), black, panel,
         collision=False)
    obox("Display window", f, (0.050, 0.022, 0.0012), (-0.012, 0.040, 0.0008), lcd, panel,
         collision=False)
    for i in range(3):
        obox(f"Digit {i}", f, (0.008, 0.013, 0.0004), (-0.026 + i * 0.011, 0.040, 0.0014), digits,
             panel, collision=False)
    controls = component(whole, "controls")
    for i, u in enumerate((-0.054, 0.021)):
        obox(f"Blue button {i}", f, (0.007, 0.010, 0.004), (u, 0.040, 0.0015), knob_mat, controls,
             collision=False)
    knob_c = f((0.047, 0.036, 0.007))
    ocyl("Temperature knob", knob_c, en, 0.0115, 0.014, knob_mat, controls, segments=24)
    ocyl("Knob cap", f((0.047, 0.036, 0.0142)), en, 0.0095, 0.0006, material(
        "Knob face", (0.04, 0.16, 0.50), 0.3), controls, segments=24, collision=False)
    obox("Knob pointer", f, (0.0015, 0.008, 0.0006), (0.047, 0.040, 0.0146), material(
        "White pointer", (0.9, 0.9, 0.9), 0.4), controls, collision=False)
    socket = component(whole, "cord_socket")
    ocyl("Cord socket ring", f((-0.040, 0.012, 0.0025)), en, 0.0085, 0.005, metal, socket,
         segments=20)
    ocyl("Cord socket", f((-0.040, 0.012, 0.0052)), en, 0.0065, 0.0012, black, socket,
         segments=20, collision=False)
    obox("Power switch", f, (0.012, 0.008, 0.003), (0.047, 0.010, 0.001), black, socket,
         collision=False)


def coil_geometry(p):
    t = math.radians(p["tilt_deg"])
    u = (math.cos(t), 0.0, math.sin(t))
    e1 = (0.0, 1.0, 0.0)
    e2 = (-math.sin(t), 0.0, math.cos(t))
    c = p["coil_low"]

    def point(s, r, a):
        return add(add(c, scale(u, s)), add(scale(e1, r * math.cos(a)), scale(e2, r * math.sin(a))))

    def r_in(s):
        return p["r_low"] + (p["r_mouth"] - p["r_low"]) * s / p["coil_len"]
    return u, point, r_in


def iron_holder(whole, p):
    bx, by = p["base"]
    ph, rh, w = p["plate_h"], p["rim_h"], p["wall"]
    cast = material("Black cast base", (0.025, 0.025, 0.028), 0.55, metallic=0.3)
    tray = material("Steel tray", (0.48, 0.49, 0.50), 0.35, metallic=0.85)
    brass = material("Brass wool", (0.72, 0.53, 0.17), 0.6, metallic=0.8)
    chrome = material("Chrome spring", (0.70, 0.72, 0.75), 0.18, metallic=0.95)
    base = component(whole, "base")
    rounded_box("Base plate", (bx, by, ph), (0, 0, ph / 2), cast, base, radius=0.0025)
    yb = p["pocket_back_y"]
    block_d = by / 2 - yb
    box("Coil mount block", (bx, block_d, rh - ph), (0, yb + block_d / 2, (ph + rh) / 2), cast,
        base, bevel=0.0015)
    pocket = component(whole, "pocket")
    y0 = -by / 2
    box("Pocket front wall", (bx, w, rh - ph), (0, y0 + w / 2, (ph + rh) / 2), tray, pocket,
        bevel=0.0008)
    for side in (-1, 1):
        box(f"Pocket side wall {side}", (w, yb - y0 - w, rh - ph),
            (side * (bx / 2 - w / 2), (y0 + w + yb) / 2, (ph + rh) / 2), tray, pocket, bevel=0.0008)
    box("Pocket floor", (bx - 2 * w, yb - y0 - w, 0.0004), (0, (y0 + w + yb) / 2, ph + 0.0002),
        tray, pocket, collision=False)
    wool = component(whole, "brass_wool")
    wx0, wx1, wy0, wy1 = -bx / 2 + w, bx / 2 - w, y0 + w, yb
    bumpy_slab("Brass wool pad", wx0 + 0.0005, wx1 - 0.0005, wy0 + 0.0005, wy1 - 0.0005, ph + 0.0004,
               rh - 0.003, 0.0018, brass, wool, nx=22, ny=8, seed=7)
    box_collider("Brass wool", (wx1 - wx0, wy1 - wy0, rh - 0.005 - ph), (0, (wy0 + wy1) / 2,
                 (ph + rh - 0.005) / 2), wool)
    # Spiral holder: open funnel of convex sectors closed by a cup at the low end.
    u, point, r_in = coil_geometry(p)
    L, cw, n = p["coil_len"], p["coil_wall"], p["sectors"]
    coil = component(whole, "coil")
    for k in range(n):
        a0, a1 = k * math.tau / n, (k + 1) * math.tau / n
        ro0, ro1 = (r_in(0) + cw) / math.cos(math.pi / n), (r_in(L) + cw) / math.cos(math.pi / n)
        pts = [point(0, r_in(0), a) for a in (a0, a1)] + [point(L, r_in(L), a) for a in (a0, a1)]
        pts += [point(0, ro0, a) for a in (a0, a1)] + [point(L, ro1, a) for a in (a0, a1)]
        hull_collider(f"Coil sector {k:02d}", pts, coil)
    # Visual helix wound on the funnel wall.
    wr = p["wire_r"]
    steps = p["turns"] * 20
    helix = [point(L * i / steps, r_in(L * i / steps) + wr + 0.0002, i * math.tau / 20)
             for i in range(steps + 1)]
    sweep("Coil spring", helix, wr, chrome, coil, sides=6)
    # Closing rings at both ends of the spring.
    for name, s in (("mouth", L - wr), ("low", wr)):
        ring = [point(s, r_in(s) + wr + 0.0002, a * math.tau / 32) for a in range(33)]
        sweep(f"Coil {name} ring", ring, wr, chrome, coil, sides=6)
    cup = component(whole, "end_cup")
    ocyl("End cup", point(-0.0015, 0, 0), u, r_in(0) + cw, 0.003, chrome, cup, segments=16)
    # Bracket posts from the mount block to the coil underside.
    bracket = component(whole, "bracket")
    for i, s in enumerate((0.012, 0.050)):
        under = point(s, r_in(s) + cw, -math.pi / 2)
        top = under[2] + 0.002
        box(f"Bracket post {i}", (0.008, 0.006, top - rh + 0.002), (under[0], under[1],
            (top + rh - 0.002) / 2), chrome, bracket, bevel=0.0008)


def flux_tin(whole, p):
    r, h, lh = p["radius"], p["height"], p["lid_h"]
    silver = material("Tin plate", (0.72, 0.73, 0.75), 0.25, metallic=0.95)
    white = material("White lid print", (0.88, 0.88, 0.86), 0.45)
    grey = material("Grey lid print", (0.40, 0.42, 0.45), 0.4)
    can = component(whole, "can")
    zl = h - lh
    lathe("Tin body", [(0.0, r - 0.0008), (0.0008, r), (zl + 0.0005, r)], silver, can,
          segments=40)
    lid = component(whole, "lid")
    lathe("Tin lid", [(zl, r + 0.0004), (h - 0.0008, r + 0.0004), (h, r - 0.0004)], silver, lid,
          segments=40)
    tube("Lid band", r + 0.0005, r + 0.0003, lh - 0.0022, (0, 0, zl + (lh - 0.0008) / 2), white, lid,
         axis="Z", segments=40, collision=False)
    tube("Lid band stripe", r + 0.00055, r + 0.0004, 0.0012, (0, 0, zl + (lh - 0.0008) / 2), grey,
         lid, axis="Z", segments=40, collision=False)
    cylinder("Lid label", r - 0.003, 0.0003, (0, 0, h + 0.00005), white, lid, axis="Z",
             segments=40, collision=False)
    tube("Lid label ring", r - 0.0055, r - 0.0075, 0.0003, (0, 0, h + 0.0001), grey, lid, axis="Z",
         segments=40, collision=False)


def desolder_wick(whole, p):
    r, h, ft = p["radius"], p["height"], p["flange"]
    white = material("White bobbin", (0.90, 0.91, 0.90), 0.45)
    green = material("Green bobbin print", (0.02, 0.30, 0.12), 0.40)
    copper = material("Copper braid", (0.78, 0.42, 0.18), 0.45, metallic=0.85)
    dark = material("Braid weave", (0.50, 0.25, 0.10), 0.55, metallic=0.7)
    hole = p["hole_r"]
    bobbin = component(whole, "bobbin")
    for name, z in (("bottom", ft / 2), ("top", h - ft / 2)):
        tube(f"Flange {name}", r, hole, ft, (0, 0, z), white, bobbin, axis="Z", segments=40,
             collision=False)
    tube("Hub", p["hub_r"], hole, h - 2 * ft, (0, 0, h / 2), green, bobbin, axis="Z", segments=24,
         collision=False)
    ring_sectors("Bobbin", hole, r, 0, h, bobbin, n=10)
    tube("Top label", r - 0.0012, p["hub_r"] + 0.0015, 0.0003, (0, 0, h + 0.00015), green, bobbin,
         axis="Z", segments=40, collision=False)
    tube("Top label window", r - 0.0045, r - 0.0075, 0.0003, (0, 0, h + 0.0003), white, bobbin,
         axis="Z", segments=40, collision=False)
    braid = component(whole, "braid")
    tube("Wound braid", p["braid_r"], p["hub_r"], h - 2 * ft - 0.0001, (0, 0, h / 2), copper,
         braid, axis="Z", segments=40, collision=False)
    for i in range(1, 5):
        z = ft + i * (h - 2 * ft) / 5
        tube(f"Braid layer {i}", p["braid_r"] + 0.0002, p["braid_r"] - 0.0003, 0.0004, (0, 0, z),
             dark, braid, axis="Z", segments=40, collision=False)
    # Short loose tail of braid on the rim.
    box("Braid tail", (0.010, 0.0025, 0.0006), (p["braid_r"] + 0.003, 0, h + 0.0003), copper,
        braid, collision=False).rotation_euler.z = 0.4


def helping_hands(whole, p):
    bx, by, bh = p["base"]
    cast = material("Black cast iron", (0.03, 0.03, 0.035), 0.6, metallic=0.4)
    chrome = material("Chrome rods", (0.72, 0.74, 0.77), 0.18, metallic=0.95)
    glass = material("Magnifier glass", (0.80, 0.86, 0.88), 0.0, transmission=0.85)
    rim = material("Lens rim", (0.06, 0.06, 0.07), 0.35, metallic=0.5)
    ry, tz = p["rod_y"], p["top_z"]
    base = component(whole, "base")
    c = 0.006
    pts = []
    for x in (-bx / 2, bx / 2):
        for y in (-by / 2, by / 2):
            xi, yi = x - math.copysign(c, x), y - math.copysign(c, y)
            pts += [(x, yi, 0), (xi, y, 0), (x, yi, bh - c), (xi, y, bh - c),
                    (x - math.copysign(c + 0.004, x), y - math.copysign(c + 0.004, y), bh)]
    hull("Cast base", pts, cast, base)
    cylinder("Rod socket", 0.008, 0.008, (0, ry, bh + 0.004), cast, base, axis="Z", segments=16)
    stand = component(whole, "stand")
    cylinder("Upright rod", 0.0035, tz - bh - 0.004, (0, ry, (tz + bh + 0.004) / 2), chrome, stand,
             axis="Z", segments=12)
    box("Top clamp", (0.012, 0.012, 0.014), (0, ry, tz), chrome, stand, bevel=0.001)
    cylinder("Cross bar", 0.0028, 0.128, (0, ry, tz), chrome, stand, axis="X", segments=12)
    for side in (-1, 1):
        name = "left" if side < 0 else "right"
        arm = component(whole, f"arm_{name}")
        ball = (side * 0.060, ry, tz)
        cylinder(f"Ball joint {name}", 0.0055, 0.011, ball, chrome, arm, axis="X", segments=12)
        hand = (side * 0.068, -0.020, 0.100)
        d = unit(tuple(b - a for a, b in zip(ball, hand)))
        mid = scale(add(ball, hand), 0.5)
        length = math.dist(ball, hand)
        ocyl(f"Arm rod {name}", mid, d, 0.0025, length, chrome, arm, segments=10)
        cylinder(f"Wing nut {name}", 0.004, 0.006, hand, chrome, arm, axis="X", segments=10,
                 collision=False)
        # Clip hangs down-forward from the arm end, jaws opening downward.
        ex = unit((0, -0.55, -1.0))
        ey = (1, 0, 0)
        ez = cross(ex, ey)
        f = frame(add(hand, scale(ez, -0.0045)), ex, ey, ez)
        alligator(f"Clip {name}", f, chrome, chrome, arm, size=1.0, boot=False)
    lens = component(whole, "magnifier")
    pivot = (0, ry, tz + 0.012)
    elbow = (0, -0.006, tz + 0.034)
    for i, (a, b) in enumerate(((pivot, elbow),)):
        d = unit(tuple(y - x for x, y in zip(a, b)))
        ocyl(f"Lens arm {i}", scale(add(a, b), 0.5), d, 0.0025, math.dist(a, b), chrome, lens,
             segments=10)
    normal = unit((0, -math.sin(math.radians(50)), math.cos(math.radians(50))))
    centre = (0, -0.040, tz + 0.020)
    up_in_plane = unit(cross(normal, (1, 0, 0)))
    hinge = add(centre, scale(up_in_plane, 0.036))
    ocyl("Lens yoke", scale(add(elbow, hinge), 0.5), unit(tuple(y - x for x, y in zip(elbow, hinge))),
         0.0022, math.dist(elbow, hinge), chrome, lens, segments=10)
    hull_collider("Lens", disc(add(centre, scale(normal, -0.003)), normal, 0.0365, 16)
                  + disc(add(centre, scale(normal, 0.003)), normal, 0.0365, 16), lens)
    rim_path = disc(centre, normal, 0.034, 40, phase=0)
    sweep("Lens rim", rim_path + rim_path[:1], 0.0025, rim, lens, sides=8)
    ocyl("Lens glass", centre, normal, 0.0335, 0.003, glass, lens, segments=40, collision=False)


def heat_sink(whole, p):
    L, W, H = p["size"]
    bh, n, t = p["base_h"], p["fins"], p["fin_t"]
    alu = material("Brushed aluminium", (0.68, 0.70, 0.72), 0.32, metallic=0.95)
    base = component(whole, "base")
    box("Base plate", (L, W, bh), (0, 0, bh / 2), alu, base, bevel=0.0005)
    fins = component(whole, "fins")
    pitch = (L - t) / (n - 1)
    for i in range(n):
        x = -L / 2 + t / 2 + i * pitch
        box(f"Fin {i:02d}", (t, W, H - bh + 0.0005), (x, 0, (H + bh - 0.0005) / 2), alu, fins)


def screwdriver_set(whole, p):
    L, W, H = p["size"]
    clear = material("Clear blister plastic", (0.86, 0.90, 0.94), 0.05, transmission=0.92)
    clear.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.3
    card_blue = material("Blue header card", (0.05, 0.22, 0.62), 0.55)
    card_red = material("Red card band", (0.62, 0.05, 0.05), 0.55)
    tray_mat = material("Grey insert tray", (0.70, 0.72, 0.74), 0.5)
    red = material("Red driver handle", (0.60, 0.04, 0.04), 0.35)
    black = material("Black swivel cap", (0.02, 0.02, 0.022), 0.35)
    steel = material("Chrome shaft", (0.62, 0.64, 0.66), 0.22, metallic=0.92)
    white = material("White header panel", (0.90, 0.90, 0.90), 0.5)
    case = component(whole, "case")
    rounded_box("Clear case", (L, W, H), (0, 0, H / 2), clear, case, radius=0.003)
    card = component(whole, "card")
    hx = L / 2 - p["header"]
    box("Header card", (p["header"] - 0.002, W - 0.004, 0.0012), (hx + p["header"] / 2, 0, 0.0018),
        card_blue, card, collision=False)
    box("Header panel", (p["header"] * 0.45, W * 0.55, 0.0003), (hx + p["header"] * 0.55, 0.008, 0.0026),
        white, card, collision=False)
    box("Red band", (0.012, W - 0.004, 0.0012), (hx - 0.006, 0, 0.0018), card_red, card,
        collision=False)
    box("Insert tray", (hx - 0.012 + L / 2 - 0.002, W - 0.006, 0.003),
        ((hx - 0.012 - L / 2 + 0.002) / 2, 0, 0.0027), tray_mat, card, collision=False)
    drivers = component(whole, "screwdrivers")
    n, pitch = p["drivers"], p["pitch"]
    x_back = -L / 2 + 0.006
    for i in range(n):
        y = (i - (n - 1) / 2) * pitch
        r = 0.0068
        zc = 0.0042 + r
        length = 0.078 + 0.0035 * i
        hx0 = x_back + 0.005
        cylinder(f"Cap {i}", r * 0.85, 0.005, (x_back + 0.0025, y, zc), black, drivers, segments=12,
                 collision=False)
        cylinder(f"Handle {i}", r, 0.050, (hx0 + 0.025, y, zc), red, drivers, radius_end=r * 0.85,
                 segments=12, collision=False)
        cylinder(f"Nose {i}", r * 0.85, 0.008, (hx0 + 0.054, y, zc), red, drivers, radius_end=0.0028,
                 segments=12, collision=False)
        shaft = length - 0.063
        cylinder(f"Shaft {i}", 0.0012 + 0.0001 * i, shaft, (hx0 + 0.058 + shaft / 2, y, zc), steel,
                 drivers, segments=8, collision=False)
        for k in range(4):
            a = k * math.tau / 4 + math.pi / 4
            cylinder(f"Grip flute {i} {k}", 0.0011, 0.040, (hx0 + 0.024, y + (r - 0.0006) * math.cos(a),
                     zc + (r - 0.0006) * math.sin(a)), black, drivers, segments=6, collision=False)


def scope_probe(whole, p):
    z = p["axis_z"]
    black = material("Black probe body", (0.02, 0.02, 0.022), 0.35)
    grey = material("Grey hook sleeve", (0.45, 0.47, 0.50), 0.4)
    steel = material("Hook wire", (0.66, 0.68, 0.70), 0.22, metallic=0.92)
    lead_mat = material("Black ground lead", (0.03, 0.03, 0.035), 0.45)
    probe = component(whole, "body")
    x_solid("Probe body", [(-0.050, 0.0050), (-0.045, 0.0058), (0.030, 0.0058), (0.036, 0.0062),
                           (0.038, 0.0085), (0.043, 0.0085), (0.045, 0.0060), (0.058, 0.0050)],
            z, black, probe, segments=16)
    x_solid("Strain relief", [(-0.085, 0.0030), (-0.0495, 0.0048)], z, black, probe, segments=12)
    for i in range(6):
        cylinder(f"Grip ring {i}", 0.0060, 0.0018, (-0.030 + i * 0.008, 0, z),
                 material(f"Grip rubber {i}", (0.06, 0.06, 0.065), 0.6), probe, segments=16,
                 collision=False)
    tip = component(whole, "hook_tip")
    x_solid("Hook sleeve", [(0.0575, 0.0042), (0.086, 0.0040), (0.092, 0.0026)], z, grey, tip,
            segments=16)
    hook = [(0.091, 0, z), (0.097, 0, z), (0.101, 0, z + 0.0005), (0.103, 0, z + 0.0022),
            (0.101, 0, z + 0.0038), (0.098, 0, z + 0.0040)]
    sweep("Hook wire", spline(hook, 4), 0.0005, steel, tip, sides=6)
    cable = component(whole, "cable")
    path = [(-0.084, 0, z), (-0.094, 0.0, z - 0.002), (-0.106, 0.004, 0.0040), (-0.120, 0.010, 0.0025),
            (-0.140, 0.020, 0.0025), (-0.160, 0.034, 0.0025), (-0.172, 0.046, 0.0025)]
    wire("Cable stub", path, 0.0025, black, cable)
    cylinder("Cable cut", 0.0018, 0.0004, path[-1], material("Copper core", (0.75, 0.42, 0.18), 0.4,
             metallic=0.8), cable, segments=10, collision=False).rotation_euler.z = math.atan2(
        path[-1][1] - path[-2][1], path[-1][0] - path[-2][0])
    ground = component(whole, "ground_lead")
    cylinder("Ground collar", 0.0062, 0.004, (0.050, 0, z), black, ground, segments=16,
             collision=False)
    lead = [(0.050, 0.0055, z + 0.001), (0.050, 0.012, 0.005), (0.044, 0.022, 0.0013),
            (0.030, 0.034, 0.0013), (0.012, 0.042, 0.0013), (-0.006, 0.046, 0.0013)]
    wire("Ground lead", lead, 0.0013, lead_mat, ground)
    head = (-0.006, 0.046)
    heading = math.pi + 0.08
    ex = (math.cos(heading), math.sin(heading), 0)
    ey = (-math.sin(heading), math.cos(heading), 0)
    f = frame((head[0], head[1], 0.0), ex, ey, (0, 0, 1))
    alligator("Ground clip", f, lead_mat, steel, ground, size=0.85)


def clip_lead(whole, p):
    boot = material("Clip boot", p["color"], 0.45)
    steel = material("Clip steel", (0.66, 0.68, 0.70), 0.25, metallic=0.9)
    wire_r = 0.001
    ends = []
    for name in ("a", "b"):
        (x, y), heading = p[name]
        clip = component(whole, f"clip_{name}")
        ex = (math.cos(heading), math.sin(heading), 0)
        ey = (-math.sin(heading), math.cos(heading), 0)
        alligator(f"Clip {name}", frame((x, y, 0.0), ex, ey, (0, 0, 1)), boot, steel, clip)
        ends.append(((x, y), ex))
    lead = component(whole, "wire")
    (ax, ay), ea = ends[0]
    (bx, by), eb = ends[1]
    mid = [(x, y, wire_r) for x, y in p["path"]]
    controls = ([(ax + ea[0] * 0.003, ay + ea[1] * 0.003, 0.0026)] + mid
                + [(bx + eb[0] * 0.003, by + eb[1] * 0.003, 0.0026)])
    wire("Lead wire", controls, wire_r, boot, lead, samples=6)


BUILDERS = {"soldering_station": soldering_station, "iron_holder": iron_holder,
            "flux_tin": flux_tin, "desolder_wick": desolder_wick, "helping_hands": helping_hands,
            "heat_sink": heat_sink, "screwdriver_set": screwdriver_set, "scope_probe": scope_probe,
            "clip_lead": clip_lead}


def build_soldering_asset(category, variant="default"):
    root = asset(category, "bottom centre of the resting pose",
                 "+X longest horizontal side; +Y width; +Z up", "-Y is the preview front")
    whole = body(category + ".body", root)
    BUILDERS[category](whole, PROFILES[category][variant])
    finish_whole(whole)
