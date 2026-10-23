"""Bench instruments for the electronics workbench (scene 000101), one rigid body each.

Dimensions are engineering estimates in metres from the scene 000101 reference
photo. Origins sit at the bottom centre of the resting pose; +Z up, -Y is the
operating front (the bench power supply is deeper than wide, so its long side
runs along Y). No branding or text is modelled.
"""

import math

from table_1000.modeling.asset_builders import asset, body, box, material
from table_1000.modeling.small_props_assets import component, finish_whole, rounded_box
from table_1000.modeling.soldering_assets import hull, ocyl
from table_1000.modeling.workbench_assets import box_collider

FRONT = (0.0, -1.0, 0.0)


def glow(name, color, strength, roughness=0.3):
    """Matte surface with a little self-illumination (lit screens)."""
    surface = material(name, color, roughness)
    shader = surface.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Emission Color"].default_value = (*color, 1)
    shader.inputs["Emission Strength"].default_value = strength
    return surface


def ellipse(rx, ry, z, n=32, cx=0.0, cy=0.0):
    return [(cx + rx * math.cos(k * math.tau / n), cy + ry * math.sin(k * math.tau / n), z)
            for k in range(n)]


def knob(name, x, y_face, z, radius, depth, surface, cap, pointer, parent, segments=16):
    """Front-panel knob along -Y: skirt cylinder, lighter cap and a white pointer line."""
    ocyl(name, (x, y_face - depth / 2, z), FRONT, radius, depth, surface, parent,
         radius_end=radius * 0.88, segments=segments, collision=False)
    ocyl(name + " cap", (x, y_face - depth - 0.0003, z), FRONT, radius * 0.78, 0.0006, cap, parent,
         segments=segments, collision=False)
    box(name + " pointer", (0.0012, 0.0006, radius * 0.7),
        (x, y_face - depth - 0.0007, z + radius * 0.4), pointer, parent, collision=False)


# --------------------------------------------------------------------------- profiles

PROFILES = {
    "monitor": {"default": {"bezel": (0.420, 0.028, 0.360), "panel_z": 0.060, "bezel_y": -0.005,
                            "screen": (0.386, 0.290), "screen_z": 0.100,
                            "base": (0.110, 0.090, 0.016)}},
    "oscilloscope": {"default": {"plate": (0.340, 0.200, 0.020), "case": (0.330, 0.150, 0.170),
                                 "case_y": -0.010}},
    "bench_power_supply": {"default": {"size": (0.170, 0.280, 0.150)}},
    "instrument_box": {"default": {"size": (0.280, 0.150, 0.070),
                                   "jacks": [(-0.100, (0.85, 0.72, 0.05)), (-0.020, (0.86, 0.86, 0.84)),
                                             (0.012, (0.07, 0.07, 0.075)), (0.062, (0.72, 0.04, 0.03)),
                                             (0.095, (0.72, 0.04, 0.03))]}},
}


# --------------------------------------------------------------------------- builders

def monitor(whole, p):
    bw, bd, bh = p["bezel"]
    z0, by = p["panel_z"], p["bezel_y"]
    black = material("Black monitor plastic", (0.020, 0.020, 0.022), 0.45)
    satin = material("Satin black stand", (0.035, 0.035, 0.038), 0.30)
    desktop = glow("Blue desktop", (0.015, 0.16, 0.80), 0.35)
    sky = glow("Light blue desktop band", (0.08, 0.36, 0.92), 0.35)
    taskbar = glow("Dark taskbar", (0.03, 0.07, 0.30), 0.4)
    start = glow("Green start button", (0.10, 0.55, 0.12), 0.6)
    icon = glow("White desktop icon", (0.85, 0.88, 0.92), 0.6)
    led = glow("Power LED", (0.15, 0.45, 1.00), 2.0)
    stand = component(whole, "stand")
    rx, ry, h = p["base"]
    hull("Oval base", ellipse(rx, ry, 0.0) + ellipse(rx - 0.004, ry - 0.004, h - 0.003)
         + ellipse(rx - 0.012, ry - 0.012, h), satin, stand, smooth=True)
    hull("Neck", [(x, y, z) for x in (-0.034, 0.034) for y in (0.018, 0.044) for z in (h - 0.002,)]
         + [(x, y, z) for x in (-0.030, 0.030) for y in (0.020, 0.040) for z in (0.175,)],
         satin, stand)
    housing = component(whole, "housing")
    yb = by + bd / 2
    box("Front bezel", (bw, bd, bh), (0, by, z0 + bh / 2), black, housing, bevel=0.004)
    zc = z0 + bh / 2
    hull("Rear housing", [(x, yb - 0.001, zc + z) for x in (-0.180, 0.180) for z in (-0.150, 0.150)]
         + [(x, 0.044, zc + z) for x in (-0.125, 0.125) for z in (-0.095, 0.095)], black, housing)
    ocyl("Power button", (0.175, by - bd / 2 - 0.001, z0 + 0.018), FRONT, 0.0045, 0.002, satin,
         housing, segments=12, collision=False)
    ocyl("Power LED", (0.163, by - bd / 2 - 0.0006, z0 + 0.018), FRONT, 0.0012, 0.0012, led,
         housing, segments=8, collision=False)
    screen = component(whole, "screen")
    sw, sh = p["screen"]
    zs = p["screen_z"]
    yf = by - bd / 2
    # Two blue tones: a lighter upper sky band over the deeper blue desktop.
    box("Desktop", (sw, 0.001, sh * 0.62), (0, yf - 0.0004, zs + sh * 0.31), desktop, screen,
        collision=False)
    box("Desktop sky", (sw, 0.001, sh * 0.38), (0, yf - 0.0004, zs + sh * 0.81), sky, screen,
        collision=False)
    box("Taskbar", (sw, 0.001, 0.013), (0, yf - 0.0008, zs + 0.0065), taskbar, screen,
        collision=False)
    box("Start button", (0.040, 0.001, 0.011), (-sw / 2 + 0.021, yf - 0.0011, zs + 0.0065), start,
        screen, collision=False)
    for i in range(4):
        box(f"Icon {i}", (0.014, 0.001, 0.013), (-sw / 2 + 0.018, yf - 0.0008, zs + sh - 0.020 - i * 0.030),
            icon, screen, collision=False)


def oscilloscope(whole, p):
    px, py, ph = p["plate"]
    cw, cd, ch = p["case"]
    cy = p["case_y"]
    teal = material("Teal base plate", (0.10, 0.30, 0.38), 0.55)
    beige = material("Beige scope case", (0.70, 0.68, 0.60), 0.50)
    bezel = material("Light front bezel", (0.78, 0.77, 0.71), 0.45)
    top = material("Slate top cover", (0.16, 0.20, 0.25), 0.55)
    screen_mat = glow("Dark scope screen", (0.020, 0.030, 0.035), 0.05, 0.12)
    grid = material("Faint graticule", (0.07, 0.11, 0.11), 0.3)
    knob_mat = material("Grey knob", (0.55, 0.56, 0.56), 0.40)
    knob_cap = material("Light knob cap", (0.80, 0.80, 0.78), 0.40)
    pointer = material("Dark pointer", (0.05, 0.05, 0.05), 0.5)
    button = material("Pale grey button", (0.62, 0.62, 0.60), 0.45)
    metal = material("BNC nickel", (0.62, 0.63, 0.64), 0.25, metallic=0.9)
    black = material("BNC insulator", (0.03, 0.03, 0.03), 0.5)
    base = component(whole, "base_plate")
    box("Base plate", (px, py, ph), (0, 0, ph / 2), teal, base, bevel=0.002)
    case = component(whole, "case")
    yf, yb = cy - cd / 2, cy + cd / 2
    zb = ph
    # Full-size front shell tapering to a smaller rear (typical scope casing).
    hull("Case", [(x, y, zb + z) for x in (-cw / 2, cw / 2) for y in (yf, yf + 0.070)
                  for z in (0.0, ch)]
         + [(x, yb, zb + z) for x in (-cw / 2 + 0.015, cw / 2 - 0.015) for z in (0.0, ch - 0.025)],
         beige, case)
    box("Top cover", (cw - 0.006, 0.066, 0.002), (0, yf + 0.037, zb + ch + 0.0005), top, case,
        collision=False)
    zf = zb  # front panel z offset
    panel = component(whole, "display")
    box("Screen bezel", (0.200, 0.002, 0.135), (-0.058, yf - 0.0005, zf + 0.090), bezel, panel,
        collision=False)
    box("Screen", (0.155, 0.002, 0.108), (-0.066, yf - 0.0012, zf + 0.098), screen_mat, panel,
        collision=False)
    for i in range(1, 4):
        box(f"Graticule h{i}", (0.150, 0.0005, 0.0006), (-0.066, yf - 0.0023, zf + 0.044 + i * 0.027),
            grid, panel, collision=False)
    for i in range(1, 5):
        box(f"Graticule v{i}", (0.0006, 0.0005, 0.104), (-0.141 + i * 0.030, yf - 0.0023, zf + 0.098),
            grid, panel, collision=False)
    for i in range(5):
        box(f"Menu button {i}", (0.012, 0.004, 0.007), (0.025, yf - 0.002, zf + 0.055 + i * 0.022),
            button, panel, bevel=0.0008, collision=False)
    controls = component(whole, "controls")
    for i in range(6):
        box(f"Panel button {i}", (0.011, 0.004, 0.006), (0.048 + i * 0.019, yf - 0.002, zf + 0.158),
            button, controls, bevel=0.0008, collision=False)
    columns = (0.055, 0.095, 0.135)
    for row, (z, r) in enumerate(((0.135, 0.0085), (0.107, 0.0085), (0.078, 0.0110), (0.049, 0.0090))):
        for c, x in enumerate(columns):
            knob(f"Knob {row}{c}", x, yf, zf + z, r + (0.001 if c == 1 and row == 0 else 0.0),
                 0.011, knob_mat, knob_cap, pointer, controls)
    bnc = component(whole, "bnc")
    for i, x in enumerate((0.045, 0.080, 0.115, 0.150)):
        ocyl(f"BNC {i} flange", (x, yf - 0.0015, zf + 0.022), FRONT, 0.0070, 0.003, metal, bnc,
             segments=16, collision=False)
        ocyl(f"BNC {i} barrel", (x, yf - 0.007, zf + 0.022), FRONT, 0.0048, 0.010, metal, bnc,
             segments=16, collision=False)
        ocyl(f"BNC {i} insulator", (x, yf - 0.0121, zf + 0.022), FRONT, 0.0030, 0.0004, black, bnc,
             segments=12, collision=False)
    ocyl("Power button", (-0.140, yf - 0.002, zf + 0.012), FRONT, 0.005, 0.004, button, bnc,
         segments=12, collision=False)
    # One box keeps the knob/connector field solid for bumping objects.
    box_collider("Front controls", (0.135, 0.012, 0.150), (0.098, yf - 0.006, zf + 0.085), bnc)


def bench_power_supply(whole, p):
    w, d, h = p["size"]
    grey = material("Light grey case", (0.62, 0.63, 0.62), 0.45)
    panel_mat = material("Off-white front panel", (0.74, 0.74, 0.71), 0.45)
    dark = material("Vent slot", (0.05, 0.05, 0.055), 0.6)
    groove = material("Handle groove", (0.25, 0.26, 0.26), 0.5)
    window = glow("Dark display window", (0.020, 0.025, 0.025), 0.05, 0.10)
    frame_mat = material("Grey display frame", (0.45, 0.46, 0.46), 0.4)
    sticker = material("Yellow sticker", (0.92, 0.75, 0.10), 0.5)
    knob_mat = material("Pale knob", (0.80, 0.80, 0.77), 0.40)
    knob_cap = material("Knob cap", (0.88, 0.88, 0.85), 0.40)
    pointer = material("Black pointer", (0.03, 0.03, 0.03), 0.5)
    black = material("Black post", (0.03, 0.03, 0.035), 0.4)
    red = material("Red post", (0.72, 0.04, 0.03), 0.4)
    green = material("Green post", (0.05, 0.40, 0.12), 0.4)
    metal = material("Post stud", (0.62, 0.62, 0.62), 0.3, metallic=0.85)
    case = component(whole, "case")
    rounded_box("Case", (w, d - 0.003, h), (0, 0.0015, h / 2), grey, case, radius=0.004)
    top = component(whole, "vented_top")
    box("Handle groove", (0.030, d - 0.030, 0.0004), (0, 0.004, h + 0.0001), groove, top,
        collision=False)
    for side in (-1, 1):
        for c in range(3):
            for r in range(14):
                box(f"Vent {side} {c} {r}", (0.011, 0.0035, 0.0004),
                    (side * (0.030 + c * 0.016), -0.108 + r * 0.0175, h + 0.0001), dark, top,
                    collision=False)
    yf = -d / 2
    front = component(whole, "front_panel")
    box("Front panel", (w - 0.006, 0.003, h - 0.006), (0, yf + 0.0015, h / 2), panel_mat, front,
        collision=False)
    box("Display frame", (0.140, 0.002, 0.040), (0, yf - 0.0005, 0.095), frame_mat, front,
        collision=False)
    box("Display window", (0.126, 0.002, 0.028), (0, yf - 0.0010, 0.095), window, front,
        collision=False)
    box("Sticker strip", (0.140, 0.0006, 0.010), (0, yf - 0.0002, 0.127), sticker, front,
        collision=False)
    box("Power switch", (0.018, 0.004, 0.010), (-0.055, yf - 0.001, 0.022), black, front,
        collision=False)
    controls = component(whole, "controls")
    for i, x in enumerate((-0.062, -0.030, 0.026, 0.058)):
        knob(f"Knob {i}", x, yf, 0.056, 0.0095, 0.013, knob_mat, knob_cap, pointer, controls)
    box("Mode switch", (0.006, 0.006, 0.010), (-0.002, yf - 0.003, 0.056), black, controls,
        collision=False)
    posts = component(whole, "binding_posts")
    for i, (x, surface) in enumerate(((0.012, red), (0.037, black), (0.062, green))):
        ocyl(f"Post {i}", (x, yf - 0.007, 0.022), FRONT, 0.0062, 0.014, surface, posts,
             segments=16, collision=False)
        ocyl(f"Post {i} stud", (x, yf - 0.0142, 0.022), FRONT, 0.0022, 0.0006, metal, posts,
             segments=10, collision=False)
    box_collider("Front controls", (0.140, 0.014, 0.060), (0, yf - 0.007, 0.040), posts)


def instrument_box(whole, p):
    w, d, h = p["size"]
    black = material("Black enclosure", (0.022, 0.022, 0.024), 0.50)
    hole = material("Jack hole", (0.005, 0.005, 0.005), 0.7)
    label = material("Blank white label", (0.90, 0.90, 0.87), 0.6)
    case = component(whole, "case")
    rounded_box("Enclosure", (w, d, h), (0, 0, h / 2), black, case, radius=0.003)
    labels = component(whole, "labels")
    for i, (x, width) in enumerate(((-0.098, 0.030), (-0.030, 0.022), (0.030, 0.026),
                                    (0.090, 0.024))):
        box(f"Top label {i}", (width, 0.008, 0.0004), (x, -d / 2 + 0.010, h + 0.0001), label, labels,
            collision=False)
    jacks = component(whole, "banana_jacks")
    yf = -d / 2
    z = h * 0.48
    for i, (x, color) in enumerate(p["jacks"]):
        collar = material(f"Jack collar {i}", color, 0.35)
        ocyl(f"Jack {i}", (x, yf - 0.004, z), FRONT, 0.0058, 0.008, collar, jacks, segments=16,
             collision=False)
        ocyl(f"Jack {i} rim", (x, yf - 0.0085, z), FRONT, 0.0045, 0.001, collar, jacks,
             radius_end=0.0040, segments=16, collision=False)
        ocyl(f"Jack {i} hole", (x, yf - 0.0091, z), FRONT, 0.0021, 0.0004, hole, jacks,
             segments=10, collision=False)
    box_collider("Jack strip", (0.210, 0.008, 0.012), (-0.002, yf - 0.004, z), jacks)


BUILDERS = {"monitor": monitor, "oscilloscope": oscilloscope,
            "bench_power_supply": bench_power_supply, "instrument_box": instrument_box}


def build_instrument_asset(category, variant="default"):
    root = asset(category, "bottom centre of the resting pose",
                 "+X width; +Y depth (rear); +Z up", "-Y is the operating front")
    whole = body(category + ".body", root)
    BUILDERS[category](whole, PROFILES[category][variant])
    finish_whole(whole)
