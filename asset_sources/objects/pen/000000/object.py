"""Generate only the black ballpoint pen; Blender 4.5, no external dependencies.

CLI: blender -b --python-exit-code 1 -P object.py -- --output DIRECTORY
Output: DIRECTORY/object.blend and the generating source DIRECTORY/object.py.
Local frame: +X length, +Y width, +Z up; tip -X, button +X, barrel Z=0.006 m.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil
import sys

import bpy
from mathutils import Matrix


def material(name, color, roughness, metallic=0.0):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    shader = result.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Transmission Weight"].default_value = 0.0
    shader.inputs["IOR"].default_value = 1.46
    result.diffuse_color = (*color, 1)
    return result


def attach(obj, parent, collection, surface):
    for previous in list(obj.users_collection):
        previous.objects.unlink(obj)
    collection.objects.link(obj)
    obj.parent = parent
    obj.data.materials.append(surface)
    return obj


def bevel(obj, width):
    modifier = obj.modifiers.new("Manufactured edge radius", "BEVEL")
    modifier.width = width
    modifier.segments = 1
    modifier = obj.modifiers.new("Weighted corner normals", "WEIGHTED_NORMAL")
    modifier.keep_sharp = True


def box(name, dimensions, position, surface, parent, collection, radius=0.001):
    bpy.ops.mesh.primitive_cube_add(size=1, location=position)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    attach(obj, parent, collection, surface)
    bevel(obj, radius)
    return obj


def cylinder(name, radius, depth, position, surface, parent, collection, tip_radius=None):
    if tip_radius is None:
        bpy.ops.mesh.primitive_cylinder_add(vertices=16, radius=radius, depth=depth, location=position)
    else:
        bpy.ops.mesh.primitive_cone_add(vertices=16, radius1=radius, radius2=tip_radius,
                                      depth=depth, location=position)
    obj = bpy.context.object
    obj.name = name
    obj.rotation_euler.x = math.pi / 2
    attach(obj, parent, collection, surface)
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    return obj


def build():
    black = material("Black pen lacquer", (0.007, 0.010, 0.012), 0.19)
    metal = material("Brushed nickel hardware", (0.50, 0.54, 0.57), 0.24, metallic=0.92)
    collection = bpy.data.collections.new("13_Black_Ballpoint")
    bpy.context.scene.collection.children.link(collection)
    root = bpy.data.objects.new("13_Black_Ballpoint__MOVE", bpy.data.meshes.new("PenFrame"))
    root["rigid_body_root"] = True
    collection.objects.link(root)
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.035
    root["asset_id"] = "13_Black_Ballpoint"
    root["coordinate_frame_normalized"] = True
    cylinder("Black lacquer barrel", 0.0056, 0.113, (0, 0, 0.006), black, root, collection)
    cylinder("Ballpoint tapered metal nose", 0.0056, 0.024, (0, -0.0685, 0.006), metal, root, collection, 0.0011)
    cylinder("Ballpoint push button", 0.0030, 0.012, (0, 0.062, 0.006), metal, root, collection)
    for y in (-0.010, -0.005, 0, 0.005):
        cylinder("Silver barrel grip band", 0.0058, 0.0017, (0, y, 0.006), metal, root, collection)
    box("Ballpoint pocket clip", (0.0024, 0.045, 0.0025), (-0.005, 0.031, 0.009), black, root, collection)
    box("Pocket clip bridge", (0.0055, 0.004, 0.003), (-0.0025, 0.050, 0.011), black, root, collection)
    # Preserve the source geometry while defining X as the pen's length axis.
    inverse = Matrix.Rotation(-math.pi / 2, 4, "Z")
    for child in root.children:
        child.matrix_basis = inverse @ child.matrix_basis
    root["axis_convention"] = "local +X = primary length; local +Y = width/depth; local +Z = up"
    root["semantic_front"] = "ballpoint tip at -X, push button at +X"
    root["origin_convention"] = "below barrel midpoint; barrel center at local Z = 0.006 m"



def add_physics():
    root = bpy.data.objects["13_Black_Ballpoint__MOVE"]
    visuals = list(root.children)
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = root
    root.select_set(True)
    bpy.ops.rigidbody.object_add()
    root.rigid_body.collision_shape = "COMPOUND"
    root.rigid_body.mesh_source = "BASE"
    root.rigid_body.kinematic = False
    for visual in visuals:
        visual["geometry_role"] = "visual"
        # Grip rings are visual detail; the barrel collider includes their radius.
        if visual.name.startswith("Silver barrel grip band"):
            continue
        # The base meshes are cylinders/frusta/boxes; bevel is visual only.
        mesh = visual.data.copy()
        if visual.name == "Black lacquer barrel":
            for vertex in mesh.vertices:
                vertex.co.x *= 0.0058 / 0.0056
                vertex.co.y *= 0.0058 / 0.0056
        proxy = bpy.data.objects.new("Collision_" + visual.name, mesh)
        bpy.context.scene.collection.objects.link(proxy)
        proxy.parent = root
        proxy.matrix_basis = visual.matrix_basis.copy()
        proxy["geometry_role"] = "collision"
        proxy.hide_render = True
        proxy.display_type = "WIRE"
        bpy.ops.object.select_all(action="DESELECT")
        bpy.context.view_layer.objects.active = proxy
        proxy.select_set(True)
        bpy.ops.rigidbody.object_add()
        proxy.rigid_body.collision_shape = "BOX" if "clip" in visual.name.lower() else "CONVEX_HULL"
        proxy.rigid_body.mesh_source = "BASE"
        proxy.rigid_body.use_margin = True
        proxy.rigid_body.collision_margin = 0.0
    bpy.context.scene.rigidbody_world.enabled = False
    bpy.context.scene["penetration_tolerance_m"] = 0.0002

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Directory for object.blend and object.py")
    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    output = parser.parse_args(argv).output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "METERS"
    build()
    add_physics()
    scene.frame_set(1)
    bpy.context.view_layer.update()
    bpy.context.preferences.filepaths.save_version = 0
    destination = output / "object.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(destination))
    source = Path(__file__).resolve()
    if source != output / "object.py":
        shutil.copyfile(source, output / "object.py")
    for name in ("preview.json", "metadata.json"):
        if source.parent / name != output / name:
            shutil.copyfile(source.parent / name, output / name)
    print("MODEL_BUILT", "pen", destination, flush=True)


if __name__ == "__main__":
    main()
