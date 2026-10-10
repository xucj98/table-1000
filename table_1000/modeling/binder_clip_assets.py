"""Closed storage binder clips, with open wire loops and a hollow folded shell.

The sheet/loop proportions and colors adapt the existing scene-002 clip model.
The missing spring back and rolled lips are explicit geometry; no spring or
clamping mechanism is simulated in this single-body shape-stage asset.
"""
from pathlib import Path
import shutil

import bpy
from mathutils import Vector
from asset_builders import asset, body, box, collider, cylinder, generate, material, mesh


COLORS = {"blue": (0.045, 0.26, 0.48), "gold": (0.58, 0.43, 0.13)}


def wire_segment(name, start, end, surface, parent):
    start, end = Vector(start), Vector(end)
    direction = end - start
    obj = cylinder(name, 0.00065, direction.length, (start + end) / 2,
                   surface, parent, axis="Z", segments=10, collision=False)
    rotation = direction.to_track_quat("Z", "Y").to_matrix()
    # Bake rotation so both visual and convex proxy have unit object scale.
    for vertex in obj.data.vertices:
        vertex.co = rotation @ vertex.co
    collider(obj)


def build(color):
    paint = material(f"{color.title()} enamel steel", COLORS[color], 0.28, 0.65)
    steel = material("Plated steel wire", (0.66, 0.70, 0.73), 0.2, 0.98)
    root = asset(f"binder_clip_{color}", "center of flat spring back at its underside",
                 "+X jaw width; +Y rear handle; +Z toward closed lips and extended handles",
                 "front handle at -Y")
    whole = body("clip", root)
    whole["interaction_scope"] = "closed storage; no jaw opening or handle articulation"
    # 22 mm jaw width. Two convex sloped sheets preserve the triangular void.
    box("Spring back", (0.022, 0.0127, 0.0007), (0, 0, 0.00035), paint, whole)
    for sign, side in ((-1, "front"), (1, "rear")):
        section = [(sign * 0.00635, 0.0007), (sign * 0.0007, 0.0155),
                   (sign * 0.00005, 0.0155), (sign * 0.00565, 0.0007)]
        vertices = [(x, y, z) for x in (-0.011, 0.011) for y, z in section]
        faces = [(3, 2, 1, 0), (4, 5, 6, 7)]
        faces += [(i, (i+1)%4, (i+1)%4+4, i+4) for i in range(4)]
        sheet = mesh(f"{side} folded jaw", vertices, faces, whole, paint)
        collider(sheet)
        # Closed contacting lips and sockets connect the two ends of each loop.
        cylinder(f"{side} rolled lip", 0.00075, 0.022,
                 (0, sign * 0.00075, 0.0155), paint, whole, axis="X", segments=12)
        points = [(-0.0075, sign * 0.001, 0.0155),
                  (-0.0085, sign * 0.0105, 0.022),
                  (-0.004, sign * 0.01235, 0.02635),
                  (0.004, sign * 0.01235, 0.02635),
                  (0.0085, sign * 0.0105, 0.022),
                  (0.0075, sign * 0.001, 0.0155)]
        for index, (start, end) in enumerate(zip(points, points[1:])):
            wire_segment(f"{side} wire {index}", start, end, steel, whole)
    # Join the complete visual so individual wire/sheet pieces are not selectable.
    # Move the saved active compound root; lock child edits and proxy selection.
    bpy.context.view_layer.update()
    visuals = [o for o in whole.children if o.get("geometry_role") == "visual"]
    bpy.ops.object.select_all(action="DESELECT")
    for obj in visuals:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = visuals[0]
    bpy.ops.object.join()
    visual = bpy.context.object
    visual.name = "Complete clip visual"
    # Use the joined visual as an ordinary selectable child. Selecting its parent
    # moves the complete physical object; lock child channels against stray edits.
    for obj in whole.children:
        obj.lock_location = (True, True, True)
        obj.lock_rotation = (True, True, True)
        obj.lock_scale = (True, True, True)
        if obj.get("geometry_role") == "collision":
            obj.hide_select = True
    bpy.ops.object.select_all(action="DESELECT")
    whole.select_set(True)
    bpy.context.view_layer.objects.active = whole


def generate_clip(color, source_file, argv=None):
    generate(lambda: build(color), source_file, argv)
    output = Path(bpy.data.filepath).parent
    original = Path(__file__).resolve()
    target = output / original.name
    if original != target:
        shutil.copyfile(original, target)
