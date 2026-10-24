"""Electronics-workbench clutter (scene 000101): paper scraps, wire offcuts, cable coils,
keyboard, empty can and bottle, small modules. Each asset is one complete rigid body.

Dimensions are engineering estimates in metres from the scene 000101 reference photo.
Origins sit at the bottom centre of the resting pose (bounding-box centre in XY, lowest
point at Z = 0); +X is the longest horizontal side, +Z up, -Y the preview front. No
branding or text is modelled.
"""

import math
import random

import bpy
from mathutils import Vector

from table_1000.modeling.asset_builders import asset, body, box, cylinder, material, mesh, tube
from table_1000.modeling.pen_holder_assets import outline
from table_1000.modeling.small_props_assets import component, finish_whole
from table_1000.modeling.workbench_assets import (
    PROFILES as WORKBENCH, box_collider, convex, hull, hull_collider, lathe, ring, ring_sectors, slab,
    x_solid,
)


# --------------------------------------------------------------------------- geometry helpers

def rotz(points, angle, center=(0.0, 0.0)):
    c, s = math.cos(angle), math.sin(angle)
    return [(center[0] + (x - center[0]) * c - (y - center[1]) * s,
             center[1] + (x - center[0]) * s + (y - center[1]) * c, z) for x, y, z in points]


def _frames(path, closed):
    """Unit tangent and horizontal side vector at every path point."""
    n = len(path)
    frames = []
    for i in range(n):
        if closed:
            a, b = Vector(path[i - 1]), Vector(path[(i + 1) % n])
        else:
            a, b = Vector(path[max(i - 1, 0)]), Vector(path[min(i + 1, n - 1)])
        t = (b - a).normalized()
        side = t.cross(Vector((0, 0, 1)))
        side = side.normalized() if side.length > 1e-6 else Vector((0, 1, 0))
        frames.append((t, side, side.cross(t).normalized()))
    return frames


def sweep(name, path, radius, surface, parent, sides=8, closed=False):
    """Visual round tube along a polyline (wire, cable, coil loop)."""
    frames = _frames(path, closed)
    vertices = []
    for p, (_, side, up) in zip(path, frames):
        for k in range(sides):
            a = (k + 0.5) * math.tau / sides
            v = Vector(p) + radius * (math.cos(a) * side + math.sin(a) * up)
            vertices.append(tuple(v))
    n = len(path)
    faces = []
    for i in range(n if closed else n - 1):
        a, b = i * sides, ((i + 1) % n) * sides
        faces.extend((a + k, a + (k + 1) % sides, b + (k + 1) % sides, b + k) for k in range(sides))
    if not closed:
        faces.append(tuple(range(sides - 1, -1, -1)))
        faces.append(tuple((n - 1) * sides + k for k in range(sides)))
    obj = mesh(name, vertices, faces, parent, surface)
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    return obj


def octagon(p, side, w, h):
    """Flat-bottomed octagon of width w and height h centred on p in the (side, Z) plane."""
    pts = []
    for k in range(8):
        a = (k + 0.5) * math.tau / 8
        s = w / 2 * math.cos(a) / math.cos(math.pi / 8)
        z = h / 2 * math.sin(a) / math.sin(3 * math.pi / 8)
        pts.append((p[0] + side[0] * s, p[1] + side[1] * s, p[2] + z))
    return pts


def path_colliders(name, path, w, h, parent, per=3, closed=False):
    """Chain of convex hulls (octagonal prisms) covering a lying wire or cable path."""
    frames = _frames(path, closed)
    n = len(path)
    starts = list(range(0, n - 1, per - 1)) if not closed else list(range(0, n, per - 1))
    for k, s in enumerate(starts):
        idx = [(s + j) % n for j in range(per) if closed or s + j < n]
        if len(idx) < 2:
            continue
        pts = []
        for i in idx:
            pts += octagon(path[i], frames[i][1], w, h)
        hull_collider(f"{name} {k:02d}", pts, parent)


def sheet(name, profile, width, thickness, surfaces, parent, collision=True):
    """Bent sheet: a polyline profile in XZ (bottom surface) extruded along Y.

    Each straight run is one convex slab (visual and collider); ``surfaces`` gives
    the material of every run.
    """
    for k, ((x0, z0), (x1, z1)) in enumerate(zip(profile, profile[1:])):
        dx, dz = x1 - x0, z1 - z0
        length = math.hypot(dx, dz)
        nx, nz = -dz / length, dx / length
        pts = [(x + nx * t, y, z + nz * t) for x, z in ((x0, z0), (x1, z1))
               for y in (-width / 2, width / 2) for t in (0, thickness)]
        hull(f"{name} {k:02d}", pts, surfaces[k], parent, collision=collision)


def arc_profile(length, sag, segments, x0=None):
    """Concave-up arc of the given chord-wise length whose ends lift by ``sag``."""
    # Solve R from sag = R (1 - cos(L / 2R)).
    lo, hi = length / 2, 1000.0
    for _ in range(80):
        r = (lo + hi) / 2
        if r * (1 - math.cos(length / (2 * r))) > sag:
            lo = r
        else:
            hi = r
    half = length / (2 * r)
    pts = [(r * math.sin(-half + 2 * half * i / segments),
            r * (1 - math.cos(-half + 2 * half * i / segments))) for i in range(segments + 1)]
    zmin = min(z for _, z in pts)
    return [(x, z - zmin) for x, z in pts]


def surface_slab(name, cx, cy, sx, sy, z_of, height, surface, parent, taper=0.0, sink=0.0003):
    """Key cap / panel standing on a sloped surface z_of(x, y); top shrinks by ``taper``."""
    pts = []
    for fx in (-1, 1):
        for fy in (-1, 1):
            x, y = cx + fx * sx / 2, cy + fy * sy / 2
            pts.append((x, y, z_of(x, y) - sink))
            xt, yt = cx + fx * (sx / 2 - taper), cy + fy * (sy / 2 - taper)
            pts.append((xt, yt, z_of(xt, yt) + height))
    return hull(name, pts, surface, parent, collision=False)


def lathe_colliders(name, rings, parent, collision_segments=16):
    """Collider hulls of each convex run of a Z-axis profile (no visual)."""
    slopes = [(r1 - r0) / (z1 - z0) for (z0, r0), (z1, r1) in zip(rings, rings[1:])]
    breaks = [0] + [i for i in range(1, len(slopes)) if slopes[i] > slopes[i - 1] + 1e-9]
    breaks.append(len(rings) - 1)
    per = max(2, 64 // collision_segments)
    k = 0
    for first, last in zip(breaks, breaks[1:]):
        while True:
            end = min(last, first + per - 1)
            pts = []
            for z, r in rings[first:end + 1]:
                pts.extend(ring(z, r, "Z", collision_segments, circumscribed=True))
            hull_collider(f"{name} run {k}", pts, parent)
            k += 1
            if end == last:
                break
            first = end


def recentre(whole):
    """Move every leaf mesh so the XY bounding-box centre is the origin and the lowest point Z = 0."""
    bpy.context.view_layer.update()
    leaves = [o for o in whole.children_recursive if o.type == "MESH" and len(o.data.vertices)]
    lo = Vector((1e9, 1e9, 1e9))
    hi = -lo
    for obj in leaves:
        for corner in obj.bound_box:
            w = obj.matrix_world @ Vector(corner)
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    offset = Vector((-(lo.x + hi.x) / 2, -(lo.y + hi.y) / 2, -lo.z))
    for obj in leaves:
        obj.location += offset
    bpy.context.view_layer.update()


# --------------------------------------------------------------------------- profiles

YELLOW_PAPER = (1.0, 0.58, 0.02)

PROFILES = {
    "paper_label": {
        "flat": {"size": (0.035, 0.012), "kind": "flat", "colors": ["yellow"]},
        "curled": {"size": (0.036, 0.012), "kind": "curl", "sag": 0.0035, "colors": ["yellow"] * 7},
        "creased": {"size": (0.034, 0.013), "kind": "crease", "flat_len": 0.022, "lift_deg": 22,
                    "colors": ["yellow", "yellow"]},
        "white_tab": {"size": (0.033, 0.013), "kind": "curl", "sag": 0.0018,
                      "colors": ["white"] * 5 + ["yellow"] * 2},
    },
    "wire_scrap": {
        "twisted_pair": {"length": 0.070, "radius": 0.0009, "pitch": 0.011, "bow": 0.006},
        "s_bend": {"length": 0.045, "radius": 0.0009, "bare": 0.005, "amplitude": 0.0045},
    },
    "plastic_scrap": {"default": {
        "bottom": [(-0.0062, -0.0028), (-0.0040, -0.0042), (0.0032, -0.0041), (0.0060, -0.0014),
                   (0.0055, 0.0026), (0.0012, 0.0040), (-0.0048, 0.0033)],
        "top_z": [0.0019, 0.0021, 0.0018, 0.0016, 0.0020, 0.0022, 0.0020], "inset": 0.0007}},
    "zip_bag": {"default": {"size": (0.080, 0.060, 0.0015), "zip_x": 0.030, "seal": 0.003}},
    "receipt": {"default": {"size": (0.090, 0.055), "thickness": 0.001, "fold_x": 0.022,
                            "lift_deg": 9}},
    "trash_bin": {"default": {"bottom": (0.126, 0.096, 0.012), "top": (0.140, 0.110, 0.015),
                              "height": 0.120, "wall": 0.0035, "floor": 0.005, "rim": 0.002},
                  "floor": {"bottom": (0.240, 0.180, 0.030), "top": (0.290, 0.220, 0.035),
                            "height": 0.300, "wall": 0.004, "floor": 0.006, "rim": 0.003}},
    "drink_can": {"opened": {}},
    "water_bottle": {"empty": {
        "rings": [(0.0, 0.0255), (0.003, 0.0305), (0.008, 0.0322), (0.030, 0.0322),
                  (0.032, 0.0310), (0.036, 0.0310), (0.038, 0.0322), (0.052, 0.0322),
                  (0.054, 0.0310), (0.058, 0.0310), (0.060, 0.0322), (0.074, 0.0322),
                  (0.076, 0.0310), (0.080, 0.0310), (0.082, 0.0322), (0.122, 0.0322),
                  (0.140, 0.0300), (0.156, 0.0250), (0.168, 0.0185), (0.177, 0.0140),
                  (0.188, 0.0140)],
        "collision": [(0.0, 0.0255), (0.003, 0.0305), (0.008, 0.0322), (0.122, 0.0322),
                      (0.140, 0.0300), (0.156, 0.0250), (0.168, 0.0185), (0.177, 0.0140),
                      (0.188, 0.0140)],
        "collar": (0.179, 0.0025, 0.0170), "cap": (0.188, 0.210, 0.0155)}},
    "calculator": {"graphing": {"size": (0.185, 0.085), "z_front": 0.0165, "z_back": 0.0195,
                                "key_h": 0.0018}},
    "computer_mouse": {"wireless": {
        "stations": [(-0.0540, 0.020, 0.008), (-0.0520, 0.038, 0.017), (-0.0470, 0.052, 0.026),
                     (-0.0380, 0.060, 0.033), (-0.0250, 0.064, 0.0365), (-0.0100, 0.064, 0.037),
                     (0.0060, 0.062, 0.0350), (0.0210, 0.058, 0.0305), (0.0350, 0.053, 0.0245),
                     (0.0450, 0.046, 0.0180), (0.0515, 0.034, 0.0125), (0.0540, 0.020, 0.0090)],
        "wheel_x": 0.024}},
    "keyboard": {"default": {"size": (0.440, 0.165), "unit": 0.0185, "palm_depth": 0.033,
                             "z_palm": 0.0165, "z_deck_front": 0.0155, "z_deck_back": 0.0225,
                             "key_h": 0.0075}},
    "cable_coil": {
        "black": {"cable_r": 0.0020, "mean_r": 0.0485, "spread": 0.0055, "loops": 8,
                  "color": (0.012, 0.012, 0.014), "tie": True, "plug": False, "seed": 3,
                  "tail": 0.040},
        "network": {"cable_r": 0.0028, "mean_r": 0.0615, "spread": 0.0050, "loops": 6,
                    "color": (0.05, 0.20, 0.62), "tie": False, "plug": True, "seed": 7,
                    "tail": 0.030},
    },
    "lcd_module": {"default": {"size": (0.050, 0.035, 0.0016), "pins": 8}},
    "db_adapter": {"default": {}},
    "pen": {"grey_ballpoint": {
        "radius": 0.0050,
        "tip": [(-0.0705, 0.0004), (-0.0695, 0.0007)],
        "cone": [(-0.0697, 0.0011), (-0.0600, 0.0034), (-0.0570, 0.0040)],
        "grip": [(-0.0572, 0.0043), (-0.0550, 0.0046), (-0.0200, 0.0046), (-0.0180, 0.0044)],
        "barrel": [(-0.0185, 0.0047), (-0.0165, 0.0050), (0.0600, 0.0050), (0.0615, 0.0048)],
        "cap": [(0.0612, 0.0045), (0.0660, 0.0043), (0.0700, 0.0028)],
        "clip": (0.012, 0.0615, 0.0030, 0.0010, 0.0004)}},
}


# --------------------------------------------------------------------------- builders

def paper_label(whole, p):
    L, W = p["size"]
    t = 0.0010  # real paper is ~0.1 mm; 1 mm keeps the contact solver sane
    mats = {"yellow": material("Yellow sticky paper", YELLOW_PAPER, 0.75),
            "white": material("White label paper", (0.90, 0.90, 0.87), 0.75)}
    strip = component(whole, "strip")
    if p["kind"] == "flat":
        profile = [(-L / 2, 0), (L / 2, 0)]
    elif p["kind"] == "curl":
        profile = arc_profile(L, p["sag"], len(p["colors"]))
    else:
        a = math.radians(p["lift_deg"])
        f = p["flat_len"]
        profile = [(0, 0), (f, 0), (f + (L - f) * math.cos(a), (L - f) * math.sin(a))]
    sheet("Paper strip", profile, W, t, [mats[c] for c in p["colors"]], strip)
    # Faint pencil marks (no legible text) on the segment lying on the table.
    ink = material("Pencil scribble", (0.35, 0.30, 0.22), 0.8)
    marks = component(whole, "marks")
    if p["kind"] == "crease":
        xc, seg = p["flat_len"] / 2, p["flat_len"]
    else:
        k = (len(profile) - 1) // 2
        xc, seg = (profile[k][0] + profile[k + 1][0]) / 2, profile[k + 1][0] - profile[k][0]
    for i, (dx, dy, lx) in enumerate(((-0.0008, 0.0016, 0.007), (0.0006, -0.0014, 0.005))):
        lx = min(lx, seg * 0.8)
        box(f"Scribble {i}", (lx, 0.0005, 0.00008), (xc + dx * (seg > 0.01), dy, t + 0.00004), ink,
            marks, collision=False)


def wire_scrap(whole, p):
    black = material("Black PVC insulation", (0.015, 0.015, 0.017), 0.40)
    copper = material("Bare copper", (0.80, 0.42, 0.20), 0.30, metallic=0.95)
    r = p["radius"]
    wire = component(whole, "insulation")
    ends = component(whole, "copper_ends")
    if "pitch" in p:
        L, a = p["length"], r
        n = 84
        axis = [(-L / 2 + L * i / n, p["bow"] * (1 - (2 * i / n - 1) ** 2), 0.0) for i in range(n + 1)]
        frames = _frames(axis, False)
        for k in range(2):
            strand = []
            for (x, y, _), (_, side, _) in zip(axis, frames):
                phi = math.tau * (x + L / 2) / p["pitch"] + k * math.pi
                strand.append((x + side.x * a * math.cos(phi), y + side.y * a * math.cos(phi),
                               2 * r + a * math.sin(phi)))
            sweep(f"Strand {k}", strand, r, black, wire, sides=8)
            for end, sign in ((strand[0], -1), (strand[-1], 1)):
                tip = (end[0] + sign * 0.0025, end[1], end[2])
                sweep(f"Copper tip {k} {'a' if sign < 0 else 'b'}", [end, tip], r * 0.5, copper, ends,
                      sides=6)
        path_colliders("Twisted pair", axis[::6] + ([axis[-1]] if n % 6 else []),
                       4 * r, 4 * r, wire)
        # Lift the collider centre line to the bundle centre (z = 2 r).
        for obj in wire.children:
            if obj.get("geometry_role") == "collision":
                for v in obj.data.vertices:
                    v.co.z += 2 * r
    else:
        L, A, bare = p["length"], p["amplitude"], p["bare"]
        n = 30
        path = [(-L / 2 + L * i / n, A * math.sin(math.tau * i / n), r) for i in range(n + 1)]
        k0, k1 = round(n * bare / L), n - round(n * bare / L)
        sweep("Insulated strand", path[k0:k1 + 1], r, black, wire, sides=10)
        sweep("Bare end a", path[:k0 + 1], r * 0.55, copper, ends, sides=8)
        sweep("Bare end b", path[k1:], r * 0.55, copper, ends, sides=8)
        path_colliders("Insulated strand", path[k0:k1 + 1:2], 2 * r, 2 * r, wire)


def plastic_scrap(whole, p):
    green = material("Green plastic offcut", (0.08, 0.48, 0.18), 0.45)
    pts = [(x, y, 0.0) for x, y in p["bottom"]]
    cx = sum(x for x, _ in p["bottom"]) / len(p["bottom"])
    cy = sum(y for _, y in p["bottom"]) / len(p["bottom"])
    s = p["inset"]
    for (x, y), z in zip(p["bottom"], p["top_z"]):
        d = math.hypot(x - cx, y - cy)
        pts.append((x - (x - cx) / d * s, y - (y - cy) / d * s, z))
    hull("Plastic fragment", pts, green, component(whole, "fragment"))


def zip_bag(whole, p):
    L, W, T = p["size"]
    film = material("Clear polyethylene film", (0.86, 0.90, 0.93), 0.12, transmission=0.80)
    film.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.5
    seal = material("Heat seal edge", (0.80, 0.83, 0.86), 0.25, transmission=0.45)
    zip_mat = material("Red zip track", (0.70, 0.05, 0.05), 0.35)
    write = material("White write-on panel", (0.92, 0.92, 0.90), 0.6)
    bag = component(whole, "film")
    box("Bag film", (L, W, T), (0, 0, T / 2), film, bag, bevel=0.0004)
    edges = component(whole, "seals")
    s = p["seal"]
    for name, dims, pos in (("front", (L, s, 0.0002), (0, -W / 2 + s / 2)),
                            ("back", (L, s, 0.0002), (0, W / 2 - s / 2)),
                            ("bottom", (s, W - 2 * s, 0.0002), (-L / 2 + s / 2, 0))):
        box(f"Seal {name}", dims, (*pos, T + 0.0001), seal, edges, collision=False)
    zipper = component(whole, "zip")
    zx = p["zip_x"]
    for i, dx in enumerate((0.0, 0.0022)):
        box(f"Zip ridge {i}", (0.0011, W - 0.001, 0.0004), (zx + dx, 0, T + 0.0002),
            zip_mat if i == 0 else seal, zipper, collision=False)
    box("Write-on panel", (0.016, 0.034, 0.0002), (zx - 0.016, 0.006, T + 0.0001), write, zipper,
        collision=False)


def receipt(whole, p):
    L, W = p["size"]
    t, fx = p["thickness"], p["fold_x"]
    paper = material("White thermal paper", (0.90, 0.90, 0.88), 0.7)
    ink = material("Black barcode print", (0.03, 0.03, 0.035), 0.6)
    grey = material("Grey print lines", (0.45, 0.46, 0.47), 0.7)
    a = math.radians(p["lift_deg"])
    flat = L / 2 + fx
    profile = [(-L / 2, 0), (fx, 0), (fx + (L - flat) * math.cos(a), (L - flat) * math.sin(a))]
    slip = component(whole, "slip")
    sheet("Paper slip", profile, W, t, [paper, paper], slip)
    box("Fold crease", (0.0006, W, 0.0001), (fx, 0, t + 0.00005), grey, slip, collision=False)
    printing = component(whole, "print")
    rng = random.Random(11)
    x = -0.036
    bars = 0
    while x < -0.002:
        w = rng.choice((0.0004, 0.0004, 0.0008, 0.0012))
        box(f"Bar {bars:02d}", (w, 0.014, 0.0001), (x + w / 2, 0.012, t + 0.00005), ink, printing,
            collision=False)
        x += w + rng.choice((0.0004, 0.0008, 0.0008))
        bars += 1
    for i, (x0, x1, y) in enumerate(((-0.040, 0.004, -0.004), (-0.040, -0.010, -0.009),
                                     (-0.040, 0.010, -0.014), (-0.040, -0.016, -0.019),
                                     (-0.040, 0.000, 0.0225), (-0.016, 0.012, -0.019))):
        box(f"Print line {i}", (x1 - x0, 0.0018, 0.0001), ((x0 + x1) / 2, y, t + 0.00005), grey,
            printing, collision=False)


def trash_bin(whole, p):
    plastic = material("Dark grey bin plastic", (0.045, 0.047, 0.05), 0.55)
    (w0, d0, r0), (w1, d1, r1) = p["bottom"], p["top"]
    h, t, f, rim = p["height"], p["wall"], p["floor"], p["rim"]

    def outer(z):
        u = z / h
        return outline(w0 + (w1 - w0) * u, d0 + (d1 - d0) * u, r0 + (r1 - r0) * u)

    def inner(z):
        u = z / h
        return outline(w0 + (w1 - w0) * u - 2 * t, d0 + (d1 - d0) * u - 2 * t,
                       r0 + (r1 - r0) * u - t)

    floor = component(whole, "floor")
    hull("Floor plate", [(x, y, z) for z in (0, f) for x, y in outer(z)], plastic, floor)
    walls = component(whole, "walls")
    ob, ot, ib, it = outer(f), outer(h), inner(f), inner(h)
    n = len(ob)
    for i in range(n):
        j = (i + 1) % n
        pts = [(*ob[k], f) for k in (i, j)] + [(*ot[k], h) for k in (i, j)]
        pts += [(*ib[k], f) for k in (i, j)] + [(*it[k], h) for k in (i, j)]
        hull(f"Wall strip {i:02d}", pts, plastic, walls)
    lip = component(whole, "rim")
    ro, rt = outline(w1 + 2 * rim, d1 + 2 * rim, r1 + rim), outer(h)
    ri = outer(h - 0.008)
    for i in range(n):
        j = (i + 1) % n
        pts = [(*ro[k], z) for k in (i, j) for z in (h - 0.007, h)]
        pts += [(*rt[k], h) for k in (i, j)] + [(*ri[k], h - 0.008) for k in (i, j)]
        hull(f"Rim strip {i:02d}", pts, plastic, lip)
    # Shallow moulded band a third of the way up (visual only).
    band = material("Moulded band", (0.07, 0.072, 0.075), 0.45)
    zb = 0.040
    ob2 = outline(w0 + (w1 - w0) * zb / h + 0.0008, d0 + (d1 - d0) * zb / h + 0.0008,
                  r0 + (r1 - r0) * zb / h)
    for i in range(n):
        j = (i + 1) % n
        pts = [(*ob2[k], z) for k in (i, j) for z in (zb, zb + 0.004)]
        pts += [(*outer(zb)[k], z) for k in (i, j) for z in (zb, zb + 0.004)]
        hull(f"Band strip {i:02d}", pts, band, lip, collision=False)


def drink_can_opened(whole, p):
    rings = WORKBENCH["drink_can"]["default"]["rings"]
    alu = material("Brushed aluminium", (0.76, 0.77, 0.79), 0.22, metallic=0.95)
    red = material("Red can print", (0.72, 0.04, 0.05), 0.30, metallic=0.3)
    black = material("Black can print", (0.02, 0.02, 0.025), 0.30, metallic=0.2)
    shell = component(whole, "can")
    lathe("Can body", rings, alu, shell, segments=40)
    band = component(whole, "print_band")
    r = 0.0330
    tube("Black band", r + 0.0001, r, 0.010, (0, 0, 0.029), black, band, axis="Z", segments=40,
         collision=False)
    tube("Red band", r + 0.0001, r, 0.044, (0, 0, 0.056), red, band, axis="Z", segments=40,
         collision=False)
    tube("Black stripe", r + 0.00015, r + 0.00005, 0.004, (0, 0, 0.070), black, band, axis="Z",
         segments=40, collision=False)
    white = material("White can stripe", (0.88, 0.88, 0.86), 0.35)
    tube("White stripe", r + 0.00015, r + 0.00005, 0.0025, (0, 0, 0.042), white, band, axis="Z",
         segments=40, collision=False)
    top = rings[-1][0]
    lid = component(whole, "lid")
    hole = material("Dark can opening", (0.01, 0.01, 0.012), 0.6)
    tab_mat = material("Tab metal", (0.62, 0.63, 0.65), 0.3, metallic=0.9)
    cylinder("Lid panel", 0.0250, 0.0004, (0, 0, top + 0.0001), alu, lid, axis="Z", segments=32,
             collision=False)
    opening = [(0.0125 + 0.0068 * math.cos(a) * (1.1 if math.cos(a) > 0 else 0.9),
                0.0062 * math.sin(a), z) for a in [k * math.tau / 16 for k in range(16)]
               for z in (top + 0.0003, top + 0.0005)]
    hull("Drinking opening", opening, hole, lid, collision=False)
    cylinder("Tab rivet", 0.0022, 0.0008, (0.002, 0, top + 0.0007), tab_mat, lid, axis="Z",
             segments=12, collision=False)
    # Pull tab levered up: the nose stays by the opening, the ring end rises toward -X.
    lift = math.radians(28)
    xh, length = 0.006, 0.021
    tab = [(xh - s * math.cos(lift) - dz * math.sin(lift), y,
            top + 0.0008 + s * math.sin(lift) + dz * math.cos(lift))
           for s in (0.0, length) for y in (-0.0055, 0.0055) for dz in (0.0, 0.0006)]
    hull("Lifted pull tab", tab, tab_mat, lid, collision=False)
    ring_hole = [(xh - s * math.cos(lift) - 0.0007 * math.sin(lift), y,
                  top + 0.0008 + s * math.sin(lift) + 0.0007 * math.cos(lift))
                 for s in (0.010, 0.017) for y in (-0.0028, 0.0028)]
    ring_hole += [(x, y, z - 0.0002) for x, y, z in ring_hole]
    hull("Tab finger hole", ring_hole, hole, lid, collision=False)


def water_bottle_empty(whole, p):
    pet = material("Clear empty PET", (0.86, 0.91, 0.95), 0.05, transmission=0.95)
    pet.node_tree.nodes.get("Principled BSDF").inputs["IOR"].default_value = 1.45
    cap_mat = material("Blue bottle cap", (0.03, 0.16, 0.62), 0.35)
    shell = component(whole, "bottle")
    lathe("Bottle body", p["rings"], pet, shell, segments=36, collision=False)
    lathe_colliders("Bottle body", p["collision"], shell)
    collar = component(whole, "neck_ring")
    cz, ch, cr = p["collar"]
    cylinder("Neck ring", cr, ch, (0, 0, cz + ch / 2), pet, collar, axis="Z", segments=24)
    cap = component(whole, "cap")
    z0, z1, r = p["cap"]
    cylinder("Cap", r, z1 - z0, (0, 0, (z0 + z1) / 2), cap_mat, cap, axis="Z", segments=24)
    for i in range(20):
        a = i * math.tau / 20
        box(f"Cap rib {i}", (0.0008, 0.0012, (z1 - z0) * 0.75),
            (r * math.cos(a), r * math.sin(a), (z0 + z1) / 2 - 0.001), cap_mat, cap,
            collision=False).rotation_euler.z = a
    # Tamper ring left on the neck after opening.
    cylinder("Tamper ring", 0.0152, 0.003, (0, 0, z0 - 0.0035), cap_mat, cap, axis="Z", segments=24,
             collision=False)


def graphing_calculator(whole, p):
    L, W = p["size"]
    zf, zb, kh = p["z_front"], p["z_back"], p["key_h"]
    body_mat = material("Black calculator body", (0.018, 0.018, 0.020), 0.40)
    face = material("Charcoal face plate", (0.055, 0.057, 0.062), 0.45)
    lcd = material("Grey LCD", (0.46, 0.50, 0.44), 0.20)
    dark = material("Dark grey keys", (0.10, 0.10, 0.11), 0.40)
    num = material("Grey number keys", (0.36, 0.37, 0.39), 0.40)
    light = material("Light grey function keys", (0.62, 0.63, 0.65), 0.40)
    blue = material("Blue second key", (0.12, 0.32, 0.72), 0.40)
    green = material("Green alpha key", (0.18, 0.50, 0.24), 0.40)

    def top(x, y=0.0):
        return zf + (x + L / 2) / L * (zb - zf)

    shell = component(whole, "body")
    c = 0.006
    pts = []
    for x in (-L / 2, L / 2):
        xi = x - math.copysign(c, x)
        for y in (-W / 2, W / 2):
            yi = y - math.copysign(c, y)
            pts += [(x, yi, 0.002), (xi, y, 0.002), (x, yi, top(x) - 0.003), (xi, y, top(x) - 0.003),
                    (xi, yi, top(x)), (xi * 0.99, yi * 0.97, 0)]
    vertices, faces = convex(pts)
    mesh("Calculator shell", vertices, faces, shell, body_mat)
    hull_collider("Calculator shell and keys",
                  pts + [(x, y, top(x) + kh) for x in (-L / 2 + 0.008, L / 2 - 0.008)
                         for y in (-W / 2 + 0.008, W / 2 - 0.008)], shell)
    display = component(whole, "display")
    sx0, sx1 = L / 2 - 0.064, L / 2 - 0.007
    slab("Screen bezel", sx0, sx1, -W / 2 + 0.007, W / 2 - 0.007, top, 0.0004, face, display)
    slab("LCD", sx0 + 0.008, sx1 - 0.007, -0.030, 0.030, lambda x: top(x) + 0.0004, 0.0003, lcd,
         display)
    keypad = component(whole, "keys")
    slab("Key panel", -L / 2 + 0.006, sx0 - 0.002, -W / 2 + 0.006, W / 2 - 0.006, top, 0.0003,
         face, keypad)
    kz = kh - 0.0003

    def key(name, cx, cy, sx, sy, surf):
        surface_slab(name, cx, cy, sx, sy, lambda x, y: top(x) + 0.0003, kz, surf, keypad,
                     taper=0.0006)

    # Function row under the screen.
    xr = sx0 - 0.0055
    for k in range(5):
        y = -0.030 + k * 0.015
        key(f"Function key {k}", xr, y, 0.0045, 0.0115, light)
    # Two rows of mode keys on the left, arrow pad on the right (-Y is the user's right).
    for row, x in enumerate((xr - 0.0105, xr - 0.0205)):
        for col, y in enumerate((0.030, 0.016, 0.002)):
            surf = dark
            if (row, col) == (0, 0):
                surf = blue
            if (row, col) == (1, 0):
                surf = green
            key(f"Mode key {row}_{col}", x, y, 0.0065, 0.0115, surf)
    ax, ay = xr - 0.0155, -0.023
    for name, dx, dy in (("up", 0.0068, 0), ("down", -0.0068, 0), ("left", 0, 0.0080),
                         ("right", 0, -0.0080)):
        key(f"Arrow {name}", ax + dx, ay + dy, 0.0055 if dy == 0 else 0.0085,
            0.0085 if dy == 0 else 0.0055, num)
    # Main grid: 7 rows x 5 columns, number block lighter grey, right column operators.
    x_top, x_bot = xr - 0.0340, -L / 2 + 0.012
    rows = 7
    pitch = (x_top - x_bot) / (rows - 1)
    for r in range(rows):
        x = x_top - r * pitch
        for col in range(5):
            y = 0.030 - col * 0.015
            surf = dark
            if r >= 3 and 1 <= col <= 3:
                surf = num
            if col == 4:
                surf = light if r >= 2 else dark
            key(f"Key {r}_{col}", x, y, 0.0068, 0.0118, surf)


def mouse_wireless(whole, p):
    navy = material("Navy blue mouse body", (0.025, 0.055, 0.22), 0.30, metallic=0.25)
    silver = material("Silver top panel", (0.62, 0.64, 0.68), 0.28, metallic=0.55)
    rubber = material("Black wheel", (0.01, 0.01, 0.012), 0.5)
    seam_mat = material("Dark seam", (0.02, 0.03, 0.08), 0.4)
    shell = component(whole, "shell")
    na, zs, inset = 14, 0.005, 0.003
    stations = p["stations"]
    rings = []
    for x, w, h in stations:
        pts = [(x, w / 2 - inset, 0.0)]
        for k in range(na + 1):
            a = k * math.pi / na
            pts.append((x, w / 2 * math.cos(a), zs + (h - zs) * math.sin(a) ** 0.75))
        pts.append((x, -(w / 2 - inset), 0.0))
        rings.append(pts)
    m = len(rings[0])
    vertices = [v for r in rings for v in r]
    faces = [tuple(range(m - 1, -1, -1)), tuple((len(rings) - 1) * m + i for i in range(m))]
    for s in range(len(rings) - 1):
        a, b = s * m, (s + 1) * m
        faces.extend((a + i, a + (i + 1) % m, b + (i + 1) % m, b + i) for i in range(m))
    obj = mesh("Mouse shell", vertices, faces, shell, navy)
    obj.data.materials.append(silver)
    for f, face in enumerate(obj.data.polygons):
        face.use_smooth = len(face.vertices) == 4
        s_idx, i = divmod(f - 2, m)
        # Silver top band: arc faces between 0.29 pi and 0.71 pi, rear lip stays navy.
        if f >= 2 and 5 <= i < 11 and s_idx >= 1:
            face.material_index = 1
    front = [v for v in vertices if v[0] >= stations[5][0]]
    back = [v for v in vertices if v[0] <= stations[6][0]]
    hull_collider("Mouse front", front, shell)
    hull_collider("Mouse back", back, shell)
    wheel = component(whole, "wheel")
    wx = p["wheel_x"]
    for (xa, _, ha), (xb, _, hb) in zip(stations, stations[1:]):
        if xa <= wx <= xb:
            hz = ha + (hb - ha) * (wx - xa) / (xb - xa)
    seam = [(x, y, h + dz) for x, _, h in stations if x > 0.03
            for y in (-0.0007, 0.0007) for dz in (-0.0008, 0.0003)]
    seam += [(wx + 0.008, y, hz + dz) for y in (-0.0007, 0.0007) for dz in (-0.0008, 0.0003)]
    hull("Button seam", seam, seam_mat, wheel, collision=False)
    # Navy saddle across the palm between the silver panel and the buttons.
    xs = -0.004
    for (xa, _, ha), (xb, _, hb) in zip(stations, stations[1:]):
        if xa <= xs <= xb:
            hs = ha + (hb - ha) * (xs - xa) / (xb - xa)
    hull("Palm divider", [(xs + dx, y, hs - 0.0010 + dz - abs(y) * 0.10) for dx in (-0.0012, 0.0012)
                          for y in (-0.018, 0.0, 0.018) for dz in (0, 0.0012)], seam_mat, wheel,
         collision=False)
    cylinder("Scroll wheel", 0.0055, 0.0065, (wx, 0, hz - 0.0024), rubber, wheel, axis="Y",
             segments=20)


KEY_ROWS = [  # (unit width or negative gap) per row, back (function row) to front
    [1, -1, 1, 1, 1, 1, -0.5, 1, 1, 1, 1, -0.5, 1, 1, 1, 1],
    [1] * 13 + [2],
    [1.5] + [1] * 12 + [1.5],
    [1.75] + [1] * 11 + [2.25],
    [2.25] + [1] * 10 + [2.75],
    [1.25, 1.25, 1.25, 6.25, 1.25, 1.25, 1.25, 1.25],
]


def keyboard(whole, p):
    L, D = p["size"]
    u = p["unit"]
    black = material("Black keyboard case", (0.022, 0.022, 0.025), 0.45)
    palm_mat = material("Black palm rest", (0.035, 0.036, 0.040), 0.65)
    key_mat = material("Black key caps", (0.012, 0.012, 0.014), 0.35)
    blue = material("Blue accent key", (0.10, 0.20, 0.50), 0.35)
    led = material("Green indicator", (0.10, 0.75, 0.25), 0.3)
    pd = p["palm_depth"]
    y_front, y_deck = -D / 2, -D / 2 + pd
    zdf, zdb, zp, kh = p["z_deck_front"], p["z_deck_back"], p["z_palm"], p["key_h"]

    def deck(x, y):
        return zdf + (y - y_deck) / (D / 2 - y_deck) * (zdb - zdf)

    case = component(whole, "case")
    c = 0.006
    pts = []
    for x in (-L / 2, L / 2):
        xi = x - math.copysign(c, x)
        for y in (y_deck - 0.004, D / 2):
            yi = y - math.copysign(0.004, y - y_deck)
            pts += [(xi, yi, 0), (x, yi, 0.003), (xi, y, 0.003), (x, yi, deck(x, y) - 0.003),
                    (xi, y, deck(x, y) - 0.003), (xi, yi, deck(x, yi))]
    hull("Keyboard case", pts, black, case)
    palm = component(whole, "palm_rest")
    pp = []
    for x in (-L / 2, L / 2):
        xi = x - math.copysign(c, x)
        pp += [(xi, y_front + 0.004, 0), (xi, y_deck, 0), (x, y_front + 0.006, 0.004),
               (xi, y_front, 0.006), (xi, y_front + 0.002, zp - 0.004), (xi, y_front + 0.008, zp),
               (xi, y_deck, zp), (x, y_deck, zp - 0.004), (x, y_front + 0.008, zp - 0.004)]
    hull("Palm rest", pp, palm_mat, palm)
    keys = component(whole, "keys")
    gap = 0.4 * u
    x_main = -L / 2 + 0.010
    x_nav = x_main + 15 * u + gap
    x_pad = x_nav + 3 * u + gap
    y_top = D / 2 - 0.011

    def row_y(r):
        return y_top - (r + 0.5) * u - (gap if r > 0 else 0)

    cap = u - 0.0035

    def key(name, x0, r, w, h=1, surf=key_mat):
        cx = x0 + w * u / 2
        cy = row_y(r) - (h - 1) * u / 2
        surface_slab(name, cx, cy, w * u - 0.0035, h * u - 0.0035, deck, kh, surf, keys,
                     taper=0.0018, sink=0.0004)

    n = 0
    for r, row in enumerate(KEY_ROWS):
        x = x_main
        for w in row:
            if w < 0:
                x += -w * u
                continue
            key(f"Key {n:03d}", x, r, w)
            n += 1
            x += w * u
    nav = [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1), (0, 2), (1, 2), (2, 2), (1, 4), (0, 5),
           (1, 5), (2, 5)]
    for cx, r in nav:
        key(f"Key {n:03d}", x_nav + cx * u, r, 1)
        n += 1
    pad = [(0, 1, 1, 1), (1, 1, 1, 1), (2, 1, 1, 1), (3, 1, 1, 1), (0, 2, 1, 1), (1, 2, 1, 1),
           (2, 2, 1, 1), (3, 2, 1, 2), (0, 3, 1, 1), (1, 3, 1, 1), (2, 3, 1, 1), (0, 4, 1, 1),
           (1, 4, 1, 1), (2, 4, 1, 1), (3, 4, 1, 2), (0, 5, 2, 1), (2, 5, 1, 1)]
    for cx, r, w, h in pad:
        key(f"Key {n:03d}", x_pad + cx * u, r, w, h, blue if (cx, r) == (3, 4) else key_mat)
        n += 1
    for i in range(3):
        x = x_pad + (0.6 + i * 1.2) * u
        surface_slab(f"Indicator {i}", x, row_y(0), 0.004, 0.0025, deck, 0.0003, led, keys)
    # One collider over all key caps on the sloped deck.
    kx0, kx1 = x_main - 0.001, x_pad + 4 * u + 0.001
    ky0, ky1 = row_y(5) - u / 2 - 0.001, y_top + 0.001
    hull_collider("Key caps", [(x, y, deck(x, y) + dz) for x in (kx0, kx1) for y in (ky0, ky1)
                               for dz in (-0.002, kh)], keys)


def cable_coil(whole, p):
    cable = material("Cable jacket", p["color"], 0.40)
    rng = random.Random(p["seed"])
    r, R, spread, n = p["cable_r"], p["mean_r"], p["spread"], p["loops"]
    coil = component(whole, "coil")
    layer_z = (r, r + 1.75 * r)
    height = layer_z[1] + r
    for i in range(n):
        z = layer_z[i % 2]
        rad = R + spread * (rng.random() * 2 - 1) * 0.55
        dx, dy = (rng.random() * 2 - 1) * spread * 0.35, (rng.random() * 2 - 1) * spread * 0.35
        ex, ph = 1 + (rng.random() * 2 - 1) * 0.04, rng.random() * math.tau
        pts = []
        m = 40
        for k in range(m):
            a = k * math.tau / m
            pr = rad * (1 + 0.03 * math.sin(3 * a + ph))
            pts.append((dx + pr * ex * math.cos(a), dy + pr / ex * math.sin(a), z))
        sweep(f"Loop {i}", pts, r, cable, coil, sides=8, closed=True)
    r_in, r_out = R - spread - r - 0.001, R + spread + r + 0.001
    ring_sectors("Coil bundle", r_in, r_out, 0, height, coil, n=10)
    tail = component(whole, "tail")
    t_len = p["tail"]
    a0 = -0.25
    start = (R * math.cos(a0), R * math.sin(a0), r)
    path = [start]
    for k in range(1, 9):
        s = k / 8
        path.append((start[0] + 0.004 + s * t_len * 0.95, start[1] - 0.006 * s - 0.010 * s * s, r))
    sweep("Cable end", path, r, cable, tail, sides=8)
    path_colliders("Cable end", path[2:], 2 * r, 2 * r, tail, per=4)
    if p["tie"]:
        tie_mat = material("Blue twist tie", (0.05, 0.25, 0.85), 0.45)
        tie = component(whole, "twist_tie")
        a = 2.3
        th, wv = 0.0007, 0.0022
        u0, u1 = r_in - 0.0006, r_out + 0.0006
        pieces = [[(u0, -wv, height), (u1, -wv, height), (u0, wv, height), (u1, wv, height)],
                  [(u0, -wv, 0.0005), (u0 + th, -wv, 0.0005), (u0, wv, 0.0005), (u0 + th, wv, 0.0005)],
                  [(u1 - th, -wv, 0.0005), (u1, -wv, 0.0005), (u1 - th, wv, 0.0005), (u1, wv, 0.0005)]]
        tops = [[(x, y, z + th) for x, y, z in pieces[0]],
                [(x, y, height + th) for x, y, z in pieces[1]],
                [(x, y, height + th) for x, y, z in pieces[2]]]
        for k, (lo, hi) in enumerate(zip(pieces, tops)):
            hull(f"Tie wrap {k}", rotz(lo + hi, a), tie_mat, tie, collision=False)
        # Twisted tails lying along the top of the bundle.
        um = (u0 + u1) / 2
        for s, sign in enumerate((-1, 1)):
            pts = [(um + 0.002 * sign, wv, height + th), (um + 0.002 * sign + 0.0015, wv + 0.009,
                                                           height + th)]
            pts = [(x + dx, y, z + dz) for x, y, z in pts for dx in (-0.0006, 0.0006)
                   for dz in (0, 0.0009)]
            hull(f"Tie tail {s}", rotz(pts, a), tie_mat, tie, collision=False)
    if p["plug"]:
        plug = component(whole, "rj45_plug")
        clear = material("Clear plug plastic", (0.85, 0.88, 0.90), 0.10, transmission=0.85)
        boot = material("Blue strain boot", tuple(c * 0.9 for c in p["color"]), 0.45)
        gold = material("Gold contacts", (0.82, 0.64, 0.22), 0.25, metallic=0.95)
        ex, ey, _ = path[-1]
        bl, pl = 0.014, 0.021
        cylinder("Strain boot", 0.0036, bl, (ex + bl / 2, ey, 0.0045), boot, plug,
                 radius_end=0.0045, segments=16)
        x0 = ex + bl
        box("Plug body", (pl, 0.0117, 0.0081), (x0 + pl / 2, ey, 0.00405), clear, plug, bevel=0.0004)
        box("Latch", (0.012, 0.006, 0.0012), (x0 + 0.010, ey, 0.0087), clear, plug, collision=False)
        colours = [(0.9, 0.5, 0.1), (0.9, 0.9, 0.9), (0.1, 0.5, 0.15), (0.1, 0.2, 0.8),
                   (0.9, 0.9, 0.9), (0.1, 0.5, 0.15), (0.45, 0.25, 0.1), (0.9, 0.9, 0.9)]
        for k in range(8):
            y = ey - 0.0049 + k * 0.0014
            box(f"Contact {k}", (0.0035, 0.0008, 0.0004), (x0 + pl - 0.0025, y, 0.0080), gold, plug,
                collision=False)
            box(f"Wire core {k}", (0.010, 0.0008, 0.0008), (x0 + 0.008, y, 0.0035),
                material(f"Core colour {k}", colours[k], 0.5), plug, collision=False)


def lcd_module(whole, p):
    L, W, T = p["size"]
    pcb = material("Blue solder mask", (0.04, 0.22, 0.62), 0.35)
    silk = material("White silkscreen frame", (0.90, 0.90, 0.88), 0.55)
    glass = material("Black display glass", (0.015, 0.02, 0.03), 0.10)
    active = material("Dark blue display area", (0.03, 0.06, 0.12), 0.15)
    chip = material("Black IC package", (0.02, 0.02, 0.025), 0.35)
    tin = material("Tinned pads", (0.70, 0.71, 0.72), 0.25, metallic=0.9)
    gold = material("Gold pins", (0.80, 0.62, 0.20), 0.25, metallic=0.9)
    fpc = material("Amber flex cable", (0.75, 0.42, 0.08), 0.4)
    board = component(whole, "pcb")
    box("Printed circuit board", (L, W, T), (0, 0, T / 2), pcb, board, bevel=0.0003)
    box("Silkscreen frame", (0.040, 0.033, 0.00005), (0.004, 0, T + 0.000025), silk, board,
        collision=False)
    for i, (x, y) in enumerate((x, y) for x in (-L / 2 + 0.0028, L / 2 - 0.0028)
                               for y in (-W / 2 + 0.0028, W / 2 - 0.0028)):
        cylinder(f"Mounting pad {i}", 0.0018, 0.0001, (x, y, T + 0.00005), tin, board, axis="Z",
                 segments=12, collision=False)
        cylinder(f"Mounting hole {i}", 0.0010, 0.0001, (x, y, T + 0.00012), chip, board, axis="Z",
                 segments=10, collision=False)
    display = component(whole, "display")
    dh = 0.0025
    box("Display module", (0.036, 0.031, dh), (0.005, 0, T + dh / 2), glass, display, bevel=0.0003)
    box("Active area", (0.029, 0.026, 0.0001), (0.006, 0, T + dh + 0.00005), active, display,
        collision=False)
    box("Flex tail", (0.003, 0.012, 0.0003), (0.0245, 0, T + 0.00015), fpc, display, collision=False)
    parts = component(whole, "parts")
    for i, (dims, x, y) in enumerate((((0.004, 0.0025, 0.0010), -0.015, 0.011),
                                      ((0.002, 0.0012, 0.0006), -0.015, -0.010),
                                      ((0.002, 0.0012, 0.0006), -0.0125, -0.010))):
        box(f"SMD part {i}", dims, (x, y, T + dims[2] / 2), chip if i == 0 else tin, parts,
            collision=False)
    header = component(whole, "header")
    pins = p["pins"]
    hl = pins * 0.00254
    hx, hh, ph = -L / 2 + 0.0035, 0.0025, 0.0060
    box("Header strip", (0.0025, hl, hh), (hx, 0, T + hh / 2), chip, header, collision=False)
    box_collider("Header", (0.0025, hl, hh + ph), (hx, 0, T + (hh + ph) / 2), header)
    for k in range(pins):
        y = -hl / 2 + 0.00127 + k * 0.00254
        box(f"Pin {k}", (0.0006, 0.0006, ph), (hx, y, T + hh + ph / 2), gold, header,
            collision=False)


def db_adapter(whole, p):
    beige = material("Beige hood plastic", (0.78, 0.72, 0.58), 0.55)
    steel = material("Nickel shell", (0.66, 0.67, 0.69), 0.25, metallic=0.9)
    insert = material("Black insulator", (0.03, 0.03, 0.03), 0.5)
    holes = material("Socket holes", (0.70, 0.55, 0.20), 0.3, metallic=0.9)
    knurl = material("Knurled screw head", (0.55, 0.56, 0.58), 0.35, metallic=0.85)
    zc = 0.008
    hood = component(whole, "hood")
    fy = -0.0135  # back face of the metal flange
    pts = []
    for y, w in ((fy, 0.042), (0.015, 0.032)):
        for x in (-w / 2, w / 2):
            for z, inset in ((0.0, 0.0012), (0.0015, 0.0), (0.0145, 0.0), (0.016, 0.0018)):
                pts.append((x - math.copysign(inset, x), y, z))
    pts += [(x * 0.9, 0.0175, z) for x in (-0.015, 0.015) for z in (0.002, 0.014)]
    hull("Hood shell", pts, beige, hood)
    cylinder("Cable boss", 0.0050, 0.004, (0, 0.0195, zc), beige, hood, axis="Y", segments=16)
    face = component(whole, "shell_face")
    box("Flange", (0.053, 0.0010, 0.0126), (0, fy - 0.0005, zc), steel, face)
    ds = []
    for y in (fy - 0.0010, fy - 0.0068):
        for x, z in ((-0.0192, zc + 0.0040), (0.0192, zc + 0.0040), (-0.0172, zc - 0.0040),
                     (0.0172, zc - 0.0040)):
            ds.append((x, y, z))
    hull("D shell", ds, steel, face)
    ins = [(x * 0.92, fy - 0.0069, z) for x, _, z in ds[:4]] + \
          [(x * 0.92, fy - 0.0070, z) for x, _, z in ds[:4]]
    ins = [(x, y, zc + (z - zc) * 0.78) for x, y, z in ins]
    hull("Insulator face", ins, insert, face, collision=False)
    for row, (count, z) in enumerate(((13, zc + 0.0015), (12, zc - 0.0015))):
        for k in range(count):
            x = (k - (count - 1) / 2) * 0.00277
            cylinder(f"Socket {row}_{k}", 0.0005, 0.0002, (x, fy - 0.0071, z), holes, face, axis="Y",
                     segments=6, collision=False)
    screws = component(whole, "thumbscrews")
    for side, sx in (("left", -0.0235), ("right", 0.0235)):
        cylinder(f"Thumbscrew head {side}", 0.0040, 0.0075, (sx, fy + 0.0045, zc), knurl, screws,
                 axis="Y", segments=16)
        for i in range(8):
            a = i * math.tau / 8
            box(f"Knurl {side} {i}", (0.0006, 0.0060, 0.0006),
                (sx + 0.0041 * math.cos(a), fy + 0.0045, zc + 0.0041 * math.sin(a)), knurl, screws,
                collision=False)
        cylinder(f"Screw thread {side}", 0.0015, 0.0060, (sx, fy - 0.0035, zc), steel, screws,
                 axis="Y", segments=12)


def pen(whole, p):
    silver = material("Silver grey barrel", (0.52, 0.54, 0.57), 0.30, metallic=0.6)
    grip_mat = material("Dark grey rubber grip", (0.10, 0.10, 0.11), 0.55)
    chrome = material("Chrome trim", (0.75, 0.76, 0.78), 0.18, metallic=0.95)
    ball = material("Ball point", (0.30, 0.30, 0.32), 0.25, metallic=0.9)
    z = p["radius"]
    tip = component(whole, "tip")
    (tx0, tr0), (tx1, tr1) = p["tip"]
    cylinder("Pen tip ball", tr0, tx1 - tx0, ((tx0 + tx1) / 2, 0, z), ball, tip, radius_end=tr1,
             segments=12, collision=False)
    for name, surf in (("cone", chrome), ("grip", grip_mat), ("barrel", silver), ("cap", chrome)):
        node = component(whole, name)
        x_solid(f"Pen {name}", p[name], z, surf, node, segments=16,
                collision_sides=8 if name in ("grip", "barrel") else None)
    # A box keel inside the barrel's flat bottom face: in the Isaac CPU settling check
    # the octagon face alone let the light pen roll ~4 deg, the box contact holds it.
    barrel = [n for n in whole.children if n.name.endswith(".barrel")][0]
    box_collider("Barrel keel", (0.070, 0.003, 0.002), (0.020, 0, 0.001), barrel)
    # Clip on top of the rear barrel; the octagonal grip/barrel colliders give the
    # flat (4 mm face) the pen rests on, so it does not roll on the table.
    x0, x1, w, th, gap = p["clip"]
    r = p["radius"]
    uo = r + gap + th
    theta = math.pi

    def place(u, v, x):
        return (x, -u * math.sin(theta) + v * math.cos(theta), z - u * math.cos(theta) - v * math.sin(theta))

    clip = component(whole, "clip")
    bar = [place(u, v, x) for x in (x0, x1) for u in (r + gap, uo) for v in (-w / 2, w / 2)]
    hull("Clip bar", bar, chrome, clip)
    foot = [place(u, v, x) for x in (x0, x0 + 0.003) for u in (r - 0.0003, r + gap + 0.0001)
            for v in (-w / 2, w / 2)]
    hull("Clip foot", foot, chrome, clip, collision=False)
    root = [place(u, v, x) for x in (x1 - 0.005, x1) for u in (r - 0.0005, uo) for v in (-w / 2, w / 2)]
    hull("Clip root", root, chrome, clip)


BUILDERS = {"paper_label": paper_label, "wire_scrap": wire_scrap, "plastic_scrap": plastic_scrap,
            "zip_bag": zip_bag, "receipt": receipt, "trash_bin": trash_bin,
            "drink_can": drink_can_opened, "water_bottle": water_bottle_empty,
            "calculator": graphing_calculator, "computer_mouse": mouse_wireless,
            "keyboard": keyboard, "cable_coil": cable_coil, "lcd_module": lcd_module,
            "db_adapter": db_adapter, "pen": pen}


def build_clutter_asset(category, variant="default"):
    root = asset(category, "bottom centre of the resting pose",
                 "+X longest horizontal side; +Y width; +Z up", "-Y is the preview front")
    whole = body(category + ".body", root)
    BUILDERS[category](whole, PROFILES[category][variant])
    recentre(whole)
    finish_whole(whole)
