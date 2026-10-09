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
    metal = material("Stapler steel", (0.50, 0.54, 0.57), 0.24, metallic=0.92)
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
    slab("Complete blue upper cover", 0.069, 0.028, -0.005, 0.004, 0.006, 0.006,
         (0.002, 0, 0.024), blue, upper, rear_width=0.020)
    box("Dark central throat", (0.053, 0.010, 0.001), (-0.004, 0, 0.0185), dark, upper,
        collision=False)
    box("Steel magazine spine", (0.056, 0.011, 0.0015), (-0.004, 0, 0.0192), metal, upper)
    for y in (-0.0062, 0.0062):
        box("Steel magazine rail", (0.054, 0.0018, 0.0042), (-0.004, y, 0.021), metal, upper)
    box("Magazine folded nose", (0.004, 0.014, 0.0045), (-0.031, 0, 0.0205), metal, upper)
    box("Steel driver", (0.0025, 0.009, 0.006), (-0.031, 0, 0.021), metal, upper)
    box("Blue rear bridge", (0.011, 0.020, 0.005), (0.024, 0, 0.0215), blue, upper)
    pivot = Vector((0.024, 0, 0.017))
    bpy.context.view_layer.update()
    upper.matrix_basis = (Matrix.Translation(pivot) @ Matrix.Rotation(math.radians(7), 4, "Y")
                          @ Matrix.Translation(-pivot))
    joint("upper_press", "HINGE", base, upper, pivot, (-0.27, 0.70), (-math.pi / 2, 0, 0))


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
