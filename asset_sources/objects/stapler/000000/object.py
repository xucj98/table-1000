"""White/blue miniature stapler with a native hinge and an open throat.

Local +X points to the rear hinge; +Y width; +Z up. The entire upper
cover/magazine/driver assembly moves together in this shape-stage model.
"""
import math
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Vector

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, box, collider, cylinder, generate, joint, material, mesh


PIVOT = Vector((0.024, 0, 0.017))
OPEN_ANGLE = math.radians(12)
CHANNEL_FRONT = -0.0313
CHANNEL_BOTTOM = 0.0145
ANVIL_TOP = 0.00955
# The front underside of the metal channel meets the anvil top at the stop.
contact_dx = PIVOT.x - CHANNEL_FRONT
contact_dz = CHANNEL_BOTTOM - PIVOT.z
CONTACT_ANGLE = (math.asin((ANVIL_TOP - PIVOT.z) / math.hypot(contact_dx, contact_dz))
                 - math.atan2(contact_dz, contact_dx))
PRESS_LIMIT = CONTACT_ANGLE - OPEN_ANGLE


def slab(name, length, width, bottom, front_top, rear_top, radius, position, surface, parent,
         rear_width=None):
    contour = []
    for x, y, start in [(length / 2 - radius, width / 2 - radius, 0),
                         (-length / 2 + radius, width / 2 - radius, 90),
                         (-length / 2 + radius, -width / 2 + radius, 180),
                         (length / 2 - radius, -width / 2 + radius, 270)]:
        for step in range(3):
            a = math.radians(start + step * 45)
            contour.append((x + radius * math.cos(a), y + radius * math.sin(a)))
    contour.sort(key=lambda p: math.atan2(p[1], p[0]))
    if rear_width:
        contour = [(x, y * (1 + (rear_width / width - 1) * (x / length + 0.5)))
                   for x, y in contour]
    count = len(contour)
    vertices = [(x, y, bottom) for x, y in contour]
    vertices.extend((x, y, front_top + (rear_top - front_top) * (x / length + 0.5))
                    for x, y in contour)
    faces = [tuple(range(count - 1, -1, -1)), tuple(range(count, 2 * count))]
    faces.extend((i, (i + 1) % count, (i + 1) % count + count, i + count)
                 for i in range(count))
    obj = mesh(name, vertices, faces, parent, surface)
    obj.location = position
    collider(obj)
    return obj


def build():
    white = material("Ivory white molded plastic", (0.78, 0.80, 0.78), 0.29)
    blue = material("Baby blue upper cover", (0.27, 0.54, 0.72), 0.29)
    metal = material("Stapler steel", (0.65, 0.68, 0.70), 0.36, metallic=0.85)
    dark = material("Throat and clinch recess", (0.025, 0.03, 0.035), 0.65)
    root = asset("15_Mini_Stapler", "bottom center of the base", "+X length toward hinge; +Y width; +Z up",
                 "staple mouth at -X")
    base = body("Stapler base", root)
    slab("White bottom shell", 0.070, 0.030, -0.00325, 0.00325, 0.00325, 0.0055,
         (0, 0, 0.00325), white, base)
    box("Steel lower plate", (0.058, 0.022, 0.0015), (-0.002, 0, 0.00725), metal, base)
    box("Staple anvil", (0.011, 0.015, 0.0015), (-0.026, 0, 0.0088), metal, base,
        bevel=0.0005)
    box("Anvil long clinch groove", (0.007, 0.002, 0.00008), (-0.026, 0, 0.0096), dark, base,
        collision=False)
    box("Anvil cross clinch groove", (0.0015, 0.010, 0.00008), (-0.026, 0, 0.00965), dark, base,
        collision=False)
    for y in (-0.0125, 0.0125):
        box("Rear hinge cheek", (0.011, 0.003, 0.011), (0.024, y, 0.013), blue, base,
            bevel=0.001)
    cylinder("Recessed pivot axle", 0.0014, 0.022, (0.024, 0, 0.017), metal, base, axis="Y")
    upper = body("Stapler moving upper", root)
    slab("Complete blue upper cover", 0.069, 0.028, 0, 0.008, 0.010, 0.006,
         (0.002, 0, 0.024), blue, upper, rear_width=0.020)
    # Butt the floor/walls against the stop, avoiding coincident outer faces.
    channel_rear_of_stop = CHANNEL_FRONT + 0.0018
    channel_length = 0.018 - channel_rear_of_stop
    channel_center = (0.018 + channel_rear_of_stop) / 2
    box("Steel staple channel floor", (channel_length, 0.0205, 0.0015),
        (channel_center, 0, CHANNEL_BOTTOM + 0.00075), metal, upper, bevel=0.0002)
    for side, y in (("left", -0.0095), ("right", 0.0095)):
        box(f"Steel staple channel {side} wall", (channel_length, 0.0015, 0.0065),
            (channel_center, y, 0.01925), metal, upper, bevel=0.0002)
        box(f"Channel to cover {side} bracket", (0.003, 0.0025, 0.0035),
            (0.012, y, 0.02325), metal, upper, bevel=0.00015)
    box("Steel channel front stop", (0.0018, 0.0205, 0.008),
        (CHANNEL_FRONT + 0.0009, 0, 0.0185), metal, upper, bevel=0.0002)
    box("Steel driver", (0.001, 0.009, 0.005), (-0.0295, 0, 0.022), metal, upper,
        bevel=0.00015)
    box("Blue rear bridge", (0.011, 0.020, 0.006), (0.024, 0, 0.022), blue, upper)
    bpy.context.view_layer.update()
    upper.matrix_basis = (Matrix.Translation(PIVOT) @ Matrix.Rotation(OPEN_ANGLE, 4, "Y")
                          @ Matrix.Translation(-PIVOT))
    joint("upper_press", "HINGE", base, upper, PIVOT, (PRESS_LIMIT, 0), (-math.pi / 2, 0, 0))


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
