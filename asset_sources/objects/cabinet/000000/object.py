"""Generate only the three-drawer cabinet; Blender 4.5, no external dependencies.

CLI: blender -b --python-exit-code 1 -P object.py -- --output DIRECTORY
Output: DIRECTORY/object.blend and the generating source DIRECTORY/object.py.
Local frame: bottom-center origin; +X width, +Y depth, +Z up; front at -Y.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

import bpy


def material(name, color, roughness, transmission=0.0):
    result = bpy.data.materials.new(name)
    result.use_nodes = True
    shader = result.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Metallic"].default_value = 0.0
    shader.inputs["Transmission Weight"].default_value = transmission
    shader.inputs["IOR"].default_value = 1.46
    result.diffuse_color = (*color, 1)
    return result


def asset(name, position=(0, 0, 0), parent=None):
    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)
    root = bpy.data.objects.new(name + "__MOVE", None)
    collection.objects.link(root)
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.035
    root.location = position
    root.parent = parent
    root["asset_id"] = name
    root["coordinate_frame_normalized"] = True
    return root, collection


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
    if name == "White pull handle":
        bevel(obj, radius)
    return obj


def rounded_rect(width, height, radius):
    points = []
    for x, z, start in [
        (width / 2 - radius, height / 2 - radius, 0),
        (-width / 2 + radius, height / 2 - radius, 90),
        (-width / 2 + radius, -height / 2 + radius, 180),
        (width / 2 - radius, -height / 2 + radius, 270),
    ]:
        for step in range(2):
            angle = math.radians(start + 90 * step)
            points.append((x + radius * math.cos(angle), z + radius * math.sin(angle)))
    return points


def cabinet_shell(root, collection, surface):
    outer = rounded_rect(0.286, 0.284, 0.012)
    inner = rounded_rect(0.272, 0.270, 0.005)
    count = len(outer)
    vertices = [(x, y, z + 0.142)
                for y, contour in [(-0.138, outer), (0.138, outer), (-0.138, inner), (0.138, inner)]
                for x, z in contour]
    faces = []
    for index in range(count):
        following = (index + 1) % count
        faces.extend([
            (index, count + index, count + following, following),
            (2 * count + following, 3 * count + following, 3 * count + index, 2 * count + index),
            (index, following, 2 * count + following, 2 * count + index),
            (count + following, count + index, 3 * count + index, 3 * count + following),
        ])
    mesh = bpy.data.meshes.new("Continuous rounded cabinet shell_mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new("Continuous rounded cabinet shell", mesh)
    collection.objects.link(obj)
    attach(obj, root, collection, surface)


def rigid_settings(obj, shape):
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.rigidbody.object_add()
    obj.select_set(False)
    obj.rigid_body.collision_shape = shape
    obj.rigid_body.mesh_source = "BASE"
    obj.rigid_body.use_margin = True
    obj.rigid_body.collision_margin = 0.0


def compound_body(root):
    bpy.context.view_layer.update()
    body = bpy.data.objects.new(root.name + "_body", bpy.data.meshes.new("CompoundFrame"))
    bpy.context.scene.collection.objects.link(body)
    body.parent = root.parent
    body.matrix_world = root.matrix_world.copy()
    for key in root.keys():
        body[key] = root[key]
    body["rigid_body_root"] = True
    for child in list(root.children):
        world = child.matrix_world.copy()
        child.parent = body
        child.matrix_world = world
        child["geometry_role"] = "visual"
    name = root.name
    bpy.data.objects.remove(root, do_unlink=True)
    body.name = name
    rigid_settings(body, "COMPOUND")
    body.rigid_body.kinematic = False
    return body


def collider(body, name, vertices, faces, shape="CONVEX_HULL", matrix=None):
    mesh = bpy.data.meshes.new(name + "_mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = body
    if matrix is not None:
        obj.matrix_basis = matrix
    obj["geometry_role"] = "collision"
    obj.hide_render = True
    obj.display_type = "WIRE"
    rigid_settings(obj, shape)
    return obj


def collision_parts(body):
    for visual in list(body.children):
        if visual.name.startswith("Continuous rounded"):
            outer = rounded_rect(0.286, 0.284, 0.012)
            inner = rounded_rect(0.272, 0.270, 0.005)
            for i in range(len(outer)):
                j = (i + 1) % len(outer)
                contour = [outer[i], outer[j], inner[j], inner[i]]
                vertices = [(x, y, z + 0.142) for y in (-0.138, 0.138) for x, z in contour]
                faces = [(3, 2, 1, 0), (4, 5, 6, 7), (0, 1, 5, 4),
                         (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
                collider(body, f"Shell_collision_{i:02d}", vertices, faces)
        else:
            collider(body, "Collision_" + visual.name,
                     [tuple(v.co) for v in visual.data.vertices],
                     [tuple(face.vertices) for face in visual.data.polygons],
                     "BOX", visual.matrix_basis.copy())


def drawer_joint(housing_body, drawer_body, index, z):
    joint = bpy.data.objects.new(f"drawer{index}_open", None)
    bpy.context.scene.collection.objects.link(joint)
    joint.empty_display_type = "ARROWS"
    joint.empty_display_size = 0.03
    joint.location = (0, -0.133, z)
    joint.rotation_euler.z = -math.pi / 2  # Slider local +X = asset local -Y.
    bpy.ops.object.select_all(action="DESELECT")
    bpy.context.view_layer.objects.active = joint
    joint.select_set(True)
    bpy.ops.rigidbody.constraint_add()
    joint.select_set(False)
    constraint = joint.rigid_body_constraint
    constraint.type = "SLIDER"
    constraint.disable_collisions = False
    constraint.object1 = housing_body
    constraint.object2 = drawer_body
    constraint.use_limit_lin_x = True
    constraint.limit_lin_x_lower = 0.0
    constraint.limit_lin_x_upper = 0.173
    joint["preview_joint"] = True


def build():
    white = material("Ivory white molded plastic", (0.78, 0.80, 0.78), 0.29)
    smoke = material("Smoky translucent drawer plastic", (0.51, 0.56, 0.58), 0.22, transmission=0.73)
    root, collection = asset("05_Plastic_Cabinet")
    convention = "local +X = width; local +Y = depth/assembly travel; local +Z = up"
    root["axis_convention"] = convention
    root["semantic_front"] = "assembly/front face; drawer travel remains along local -Y"
    root["origin_convention"] = "bottom center of the cabinet housing"
    housing, housing_parts = asset("05_Plastic_Cabinet_Housing", parent=root)
    housing["axis_convention"] = convention
    housing["origin_convention"] = root["origin_convention"]
    cabinet_shell(housing, housing_parts, white)
    box("Cabinet back panel", (0.267, 0.007, 0.255), (0, 0.134, 0.142), white, housing, housing_parts, 0.014)
    for z in (0.093, 0.184):
        box("Slim drawer shelf", (0.266, 0.263, 0.0035), (0, 0, z), white, housing, housing_parts)
    housing_body = compound_body(housing)
    collision_parts(housing_body)
    for index, z in enumerate((0.231, 0.140, 0.049), 1):
        drawer, parts = asset(f"{5 + index:02d}_Plastic_Drawer_{index}", (0, -0.133, z), root)
        drawer["axis_convention"] = convention
        drawer["semantic_front"] = root["semantic_front"]
        box("Smoky translucent front", (0.259, 0.004, 0.080), (0, 0, 0), smoke, drawer, parts, 0.005)
        for x in (-0.128, 0.128):
            box("Translucent drawer side", (0.0035, 0.256, 0.080), (x, 0.131, 0), smoke, drawer, parts, 0.002)
        box("Translucent drawer bottom", (0.258, 0.256, 0.0035), (0, 0.131, -0.0385), smoke, drawer, parts, 0.002)
        box("Translucent drawer rear", (0.258, 0.0035, 0.080), (0, 0.260, 0), smoke, drawer, parts, 0.002)
        box("White pull handle", (0.048, 0.010, 0.012), (0, -0.009, -0.006), white, drawer, parts, 0.0025)
        drawer_body = compound_body(drawer)
        collision_parts(drawer_body)
        drawer_joint(housing_body, drawer_body, index, z)


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
    scene.frame_start = 0
    scene.frame_end = 72
    build()
    scene.rigidbody_world.enabled = False
    scene["complexity_budget"] = json.dumps({"visual_triangles": 600, "colliders": 32, "convex_vertices": 80, "convex_faces": 60})
    scene["penetration_tolerance_m"] = 0.0002
    scene.frame_set(0)
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
    print("MODEL_BUILT", "cabinet", destination, flush=True)


if __name__ == "__main__":
    main()
