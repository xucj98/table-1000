"""Background props behind the electronics workbench (scene 000101), one rigid body each.

Dimensions are engineering estimates in metres from the scene 000101 reference photo
(wall positions use roughly 1.35 mm per photo pixel across the 1.30 m wall). Origins sit at
the bottom centre of the resting footprint; +Z up, -Y the front. No branding or text.
"""

import math

import bpy

from table_1000.modeling.asset_builders import asset, body, box, cylinder, material
from table_1000.modeling.small_props_assets import component, finish_whole
from table_1000.modeling.soldering_assets import (
    add, frame, hull, hull_collider, ocyl, scale, sweep, unit,
)
from table_1000.modeling.workbench_assets import box_collider


PROFILES = {
    "bench_backboard": {"default": {
        "wall": (1.30, 0.018, 0.75), "foot": (1.30, 0.14, 0.02),
        # Wall-mounted items: x centre / z range on the wall, depth out of the wall face.
        "small_cabinet": {"x": (0.06, 0.30), "z": (0.44, 0.54), "depth": 0.10, "grid": (3, 3)},
        "parts_cabinet": {"x": (0.32, 0.62), "z": (0.35, 0.65), "depth": 0.12, "grid": (4, 7)},
        # Steel pegs to hang pliers on (through the gap between the handles): x on the wall,
        # height, pin radius and reach out of the wall face, knob radius and length at the tip.
        "pegs": {"x": (-0.262, -0.187, -0.112, -0.040), "z": 0.33, "radius": 0.003,
                 "reach": 0.034, "knob": (0.0045, 0.004)},
        "papers": [  # x range, z range, roll (deg)
            ((-0.025, 0.190), (0.235, 0.425), 1.5), ((0.200, 0.405), (0.200, 0.340), -1.0)],
        "tape": {"x": (0.07, 0.29), "z": (0.418, 0.432)},
    }},
    "desk_lamp": {"default": {
        "base_r": 0.080, "base_h": 0.028, "shoulder": (0.0, 0.0, 0.060),
        "elbow": (0.0, -0.075, 0.445), "wrist": (0.0, -0.320, 0.430),
        "head": (0.070, 0.300, 0.040), "head_tilt_deg": 7.0,
    }},
}


def emissive(name, color, strength):
    surface = material(name, color, 0.5)
    shader = surface.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Emission Color"].default_value = (*color, 1)
    shader.inputs["Emission Strength"].default_value = strength
    return surface


def wall_plate(name, x, z, y_back, thickness, surface, parent, roll_deg=0.0, collision=False):
    """Thin plate lying on the wall plane (back face at ``y_back``), rolled about Y."""
    cx, cz = (x[0] + x[1]) / 2, (z[0] + z[1]) / 2
    hw, hh = (x[1] - x[0]) / 2, (z[1] - z[0]) / 2
    c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    pts = [(cx + u * c - v * s, y, cz + u * s + v * c)
           for u in (-hw, hw) for v in (-hh, hh) for y in (y_back, y_back - thickness)]
    return hull(name, pts, surface, parent, collision=collision)


# --------------------------------------------------------------------------- backboard

def drawer_cabinet(whole, name, spec, wall_y, frame_mat, drawer_mat, label_mat, parts_mats):
    x0, x1 = spec["x"]
    z0, z1 = spec["z"]
    depth = spec["depth"]
    cols, rows = spec["grid"]
    node = component(whole, name)
    w, h = x1 - x0, z1 - z0
    cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
    front = wall_y - depth
    t = 0.006
    # Frame: back panel, sides, top, bottom and grid dividers (visual); one box collider.
    box(f"{name} back", (w, t, h), (cx, wall_y - t / 2 - 0.0002, cz), frame_mat, node, collision=False)
    for side in (-1, 1):
        box(f"{name} side {side:+d}", (t, depth - t, h), (cx + side * (w / 2 - t / 2),
            front + (depth - t) / 2 - 0.0002, cz), frame_mat, node, collision=False)
    for zz in (z0 + t / 2, z1 - t / 2):
        box(f"{name} shelf {zz:.3f}", (w - 2 * t, depth - t, t),
            (cx, front + (depth - t) / 2 - 0.0002, zz), frame_mat, node, collision=False)
    cw = (w - 2 * t) / cols
    ch = (h - 2 * t) / rows
    rib = 0.003
    for i in range(1, cols):
        box(f"{name} column rib {i}", (rib, 0.008, h - 2 * t),
            (x0 + t + i * cw, front + 0.004, cz), frame_mat, node, collision=False)
    for j in range(1, rows):
        box(f"{name} row rib {j}", (w - 2 * t, 0.008, rib),
            (cx, front + 0.004, z0 + t + j * ch), frame_mat, node, collision=False)
    k = 0
    for i in range(cols):
        for j in range(rows):
            dx = x0 + t + (i + 0.5) * cw
            dz = z0 + t + (j + 0.5) * ch
            fw, fh = cw - rib - 0.002, ch - rib - 0.002
            # Drawer front (thin plate) recessed 1 mm behind the frame lip.
            box(f"{name} drawer {i}{j}", (fw, 0.003, fh), (dx, front + 0.0025, dz),
                drawer_mat, node, collision=False)
            box(f"{name} label {i}{j}", (fw * 0.62, 0.0006, fh * 0.22),
                (dx, front + 0.0007, dz + fh * 0.20), label_mat, node, collision=False)
            box(f"{name} pull {i}{j}", (fw * 0.30, 0.004, 0.003),
                (dx, front - 0.0010, dz - fh * 0.28), drawer_mat, node, collision=False)
            # Small parts behind the clear front.
            if parts_mats:
                mat = parts_mats[k % len(parts_mats)]
                box(f"{name} parts {i}{j}", (fw * 0.7, 0.02, fh * 0.35),
                    (dx, front + 0.016, dz - fh * 0.22), mat, node, collision=False)
            k += 1
    box_collider(name, (w, depth, h), (cx, wall_y - depth / 2 - 0.0002, cz), node)


def pegs(node, p, wall_y, steel):
    """Pins out of the wall face, each with a knob at its tip so what hangs on it stays on."""
    radius, reach, (knob_r, knob_l) = p["radius"], p["reach"], p["knob"]
    for i, x in enumerate(p["x"]):
        ocyl(f"Peg {i}", (x, wall_y - (reach - knob_l) / 2, p["z"]), (0, 1, 0), radius, reach - knob_l,
             steel, node, segments=12)
        ocyl(f"Peg {i} knob", (x, wall_y - reach + knob_l / 2, p["z"]), (0, 1, 0), knob_r, knob_l,
             steel, node, segments=12)


def bench_backboard(whole, p):
    W, T, H = p["wall"]
    fw, fd, fh = p["foot"]
    wall_y = fd / 2 - T  # front face of the wall panel
    wall_mat = material("Warm grey wall paint", (0.40, 0.35, 0.32), 0.85)
    rail_mat = material("Dark steel foot rail", (0.05, 0.05, 0.055), 0.45, metallic=0.6)
    dark = material("Dark grey cabinet plastic", (0.035, 0.037, 0.042), 0.45)
    smoke = material("Smoked drawer plastic", (0.09, 0.09, 0.10), 0.25, transmission=0.35)
    blue = material("Blue cabinet frame", (0.02, 0.09, 0.40), 0.40)
    clear = material("Clear drawer plastic", (0.70, 0.74, 0.78), 0.10, transmission=0.75)
    label = material("White label strip", (0.85, 0.85, 0.82), 0.70)
    parts = [material("Brass parts", (0.55, 0.38, 0.10), 0.4, metallic=0.7),
             material("Black parts", (0.03, 0.03, 0.03), 0.5),
             material("Steel parts", (0.55, 0.57, 0.60), 0.3, metallic=0.9),
             material("Red parts", (0.50, 0.04, 0.03), 0.4),
             material("Yellow parts", (0.80, 0.55, 0.04), 0.4),
             material("Green parts", (0.05, 0.25, 0.08), 0.4)]
    steel = material("Peg steel", (0.32, 0.33, 0.35), 0.35, metallic=0.85)
    paper = material("Lined paper", (0.86, 0.86, 0.84), 0.85)
    ink = material("Ruled blue lines", (0.10, 0.22, 0.50), 0.7)
    margin = material("Ruled red margin", (0.65, 0.15, 0.15), 0.7)
    pin = material("Push pin heads", (0.70, 0.10, 0.06), 0.3)
    tape = material("White tape", (0.88, 0.88, 0.86), 0.6)

    panel = component(whole, "panel")
    box("Wall panel", (W, T, H), (0, fd / 2 - T / 2, H / 2), wall_mat, panel)
    foot = component(whole, "foot_rail")
    box("Foot rail", (fw, fd - T, fh), (0, -T / 2, fh / 2), rail_mat, foot, bevel=0.002)

    drawer_cabinet(whole, "small_cabinet", p["small_cabinet"], wall_y, dark, smoke, label, [])
    drawer_cabinet(whole, "parts_cabinet", p["parts_cabinet"], wall_y, blue, clear, label, parts)

    pegs(component(whole, "pegs"), p["pegs"], wall_y, steel)

    sheets = component(whole, "papers")
    for i, ((x0, x1), (z0, z1), roll) in enumerate(p["papers"]):
        y0 = wall_y - 0.0003
        wall_plate(f"Paper {i}", (x0, x1), (z0, z1), y0, 0.0005, paper, sheets, roll)
        cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
        c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        rows = int((z1 - z0 - 0.03) / 0.0085)
        for r in range(rows):
            v = z1 - cz - 0.025 - r * 0.0085
            lx, lz = cx - v * s, cz + v * c
            wall_plate(f"Paper {i} line {r}", (lx - (x1 - x0) / 2 + 0.004, lx + (x1 - x0) / 2 - 0.004),
                       (lz - 0.0007, lz + 0.0007), y0 - 0.0005, 0.0002, ink, sheets, roll)
        u = x0 - cx + 0.025
        wall_plate(f"Paper {i} margin", (cx + u * c - 0.0005, cx + u * c + 0.0005),
                   (z0 + 0.004 + u * s, z1 - 0.004 + u * s), y0 - 0.0005, 0.0002, margin, sheets, 0)
        for u in (x0 - cx + 0.012, x1 - cx - 0.012):
            v = z1 - cz - 0.010
            cylinder(f"Paper {i} pin {u:+.3f}", 0.0045, 0.006,
                     (cx + u * c - v * s, y0 - 0.0005 - 0.003, cz + u * s + v * c), pin, sheets,
                     axis="Y", segments=12, collision=False)
    strip = component(whole, "tape")
    wall_plate("Tape strip", p["tape"]["x"], p["tape"]["z"], wall_y - 0.0012, 0.0004, tape, strip)


# --------------------------------------------------------------------------- desk lamp

def helix(a, b, radius, turns, samples_per_turn=10):
    d = unit(tuple(y - x for x, y in zip(a, b)))
    e1 = (1, 0, 0)
    e2 = unit((d[1] * e1[2] - d[2] * e1[1], d[2] * e1[0] - d[0] * e1[2], d[0] * e1[1] - d[1] * e1[0]))
    n = int(turns * samples_per_turn)
    out = []
    for k in range(n + 1):
        t = k / n
        ang = t * turns * math.tau
        c = tuple(x + (y - x) * t for x, y in zip(a, b))
        out.append(add(c, add(scale(e1, radius * math.cos(ang)), scale(e2, radius * math.sin(ang)))))
    return out


def arm_segment(node, name, a, b, black, spring_mat, spring_offset):
    d = unit(tuple(y - x for x, y in zip(a, b)))
    length = math.dist(a, b)
    mid = scale(add(a, b), 0.5)
    for side in (-1, 1):
        off = (side * 0.011, 0, 0)
        ocyl(f"{name} rod {side:+d}", add(mid, off), d, 0.0042, length, black, node,
             segments=10, collision=False)
    # One hull around both rods.
    r = 0.0046
    pts = []
    for end in (a, b):
        for sx in (-1, 1):
            for dy, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
                pts.append((end[0] + sx * (0.011 + r), end[1] + dy, end[2] + dz))
    hull_collider(name, pts, node)
    # Tension spring beside the rods (visual), attached by short links.
    s0 = add(add(a, scale(d, length * 0.12)), spring_offset)
    s1 = add(add(a, scale(d, length * 0.55)), spring_offset)
    sweep(f"{name} spring", helix(s0, s1, 0.0045, turns=18), 0.0009, spring_mat, node, sides=6)
    for i, s in enumerate((s0, s1)):
        anchor = add(a, scale(d, length * (0.08 if i == 0 else 0.62)))
        dd = unit(tuple(y - x for x, y in zip(anchor, s)))
        ocyl(f"{name} spring link {i}", scale(add(anchor, s), 0.5), dd, 0.0012,
             math.dist(anchor, s), spring_mat, node, segments=6, collision=False)


def desk_lamp(whole, p):
    black = material("Black lamp enamel", (0.018, 0.018, 0.02), 0.40, metallic=0.2)
    rubber = material("Black rubber foot", (0.01, 0.01, 0.012), 0.8)
    spring_mat = material("Dark spring steel", (0.10, 0.10, 0.11), 0.30, metallic=0.9)
    glow = emissive("Warm white lit diffuser", (1.0, 0.86, 0.62), 6.0)
    R, bh = p["base_r"], p["base_h"]
    base = component(whole, "base")
    ocyl("Base disc", (0, 0, 0.004 + (bh - 0.004) / 2), (0, 0, 1), R, bh - 0.004, black, base,
         radius_end=R - 0.008, segments=32, collision=False)
    hull_collider("Base disc", [(r * math.cos((k + 0.5) * math.tau / 32), r * math.sin((k + 0.5) * math.tau / 32), z)
                                for z, r in ((0.0, R - 0.003), (0.004, R), (bh, R - 0.008))
                                for k in range(32)], base)
    ocyl("Rubber foot ring", (0, 0, 0.002), (0, 0, 1), R - 0.003, 0.004, rubber, base,
         segments=32, collision=False)
    sh = p["shoulder"]
    ocyl("Pivot turret", (0, 0, bh + 0.010), (0, 0, 1), 0.024, 0.020, black, base, segments=20,
         collision=False)
    box("Shoulder fork", (0.034, 0.020, 0.030), (0, 0, sh[2] - 0.004), black, base, collision=False)
    cylinder("Shoulder knob", 0.010, 0.046, sh, black, base, axis="X", segments=16, collision=False)
    hull_collider("Turret and fork", [(x, y, z) for x in (-0.024, 0.024) for y in (-0.024, 0.024)
                                      for z in (bh - 0.001, sh[2] + 0.010)], base)

    arm = component(whole, "arm")
    el, wr = p["elbow"], p["wrist"]
    arm_segment(arm, "Lower arm", sh, el, black, spring_mat, (0.0, 0.016, -0.004))
    cylinder("Elbow knuckle", 0.012, 0.044, el, black, arm, axis="X", segments=16, collision=False)
    arm_segment(arm, "Upper arm", el, wr, black, spring_mat, (0.0, 0.0, 0.016))
    cylinder("Wrist knuckle", 0.010, 0.040, wr, black, arm, axis="X", segments=16, collision=False)

    head = component(whole, "head")
    hx, hl, hz = p["head"]
    a = math.radians(p["head_tilt_deg"])
    # Head frame: local x along the lamp width (X), y along the head (towards -Y, nose down).
    ey = (0, -math.cos(a), -math.sin(a))
    ez = (0, -math.sin(a), math.cos(a))
    top_back = (0, wr[1] + 0.020, wr[2] - 0.016)
    f = frame(top_back, (1, 0, 0), ey, ez)
    c = 0.010
    shell = []
    for v in (0.0, hl):
        for sx in (-1, 1):
            shell += [f((sx * hx / 2, v, -c)), f((sx * (hx / 2 - c), v, 0.0)),
                      f((sx * hx / 2, v, -hz + 0.004)), f((sx * (hx / 2 - 0.004), v, -hz))]
    hull("Lamp head shell", shell, black, head, collision=False)
    hull_collider("Lamp head", [f((sx * hx / 2, v, z)) for sx in (-1, 1) for v in (0.0, hl)
                                for z in (0.0, -hz)], head)
    hull("Diffuser", [f((sx * (hx / 2 - 0.007), v, -hz - 0.0012 * k))
                      for sx in (-1, 1) for v in (0.015, hl - 0.015) for k in (-0.5, 1)],
         glow, head, collision=False)
    hull("Rear switch cap", [f((sx * 0.018, v, z)) for sx in (-1, 1) for v in (-0.012, 0.004)
                             for z in (-0.005, -0.030)], black, head, collision=False)
    box("Wrist bracket", (0.030, 0.026, 0.020), (0, wr[1], wr[2] - 0.008), black, head,
        collision=False)
    for i in range(5):
        v = 0.06 + i * 0.04
        hull(f"Cooling rib {i}", [f((sx * (hx / 2 - 0.012), v + dv, dz)) for sx in (-1, 1)
                                  for dv in (0, 0.006) for dz in (-0.001, 0.003)],
             black, head, collision=False)


BUILDERS = {"bench_backboard": bench_backboard, "desk_lamp": desk_lamp}


def build_backdrop_asset(category, variant="default"):
    root = asset(category, "bottom centre of the resting footprint",
                 "+X wall/lamp width; +Y back; +Z up", "-Y is the front")
    whole = body(category + ".body", root)
    BUILDERS[category](whole, PROFILES[category][variant])
    finish_whole(whole)
    bpy.context.view_layer.update()
