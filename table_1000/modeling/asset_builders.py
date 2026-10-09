"""Small Blender modeling helpers shared by the stationery asset sources.

The builder copies this module beside object.py so generated source snapshots
can be rebuilt without importing the repository package.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil
import sys

import bpy
import bmesh


def material(name, color, roughness=0.35, metallic=0.0, transmission=0.0):
    surface = bpy.data.materials.new(name)
    surface.use_nodes = True
    shader = surface.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Transmission Weight"].default_value = transmission
    shader.inputs["IOR"].default_value = 1.46
    surface.diffuse_color = (*color, 1)
    return surface


def asset(name, origin, axes, front):
    root = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(root)
    root["asset_id"] = name
    root["coordinate_frame_normalized"] = True
    root["origin_convention"] = origin
    root["axis_convention"] = axes
    root["semantic_front"] = front
    root.empty_display_size = 0.02
    return root


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


def body(name, root):
    obj = bpy.data.objects.new(name, bpy.data.meshes.new(name + "_frame"))
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = root
    obj["rigid_body_root"] = True
    rigid_settings(obj, "COMPOUND")
    return obj


def mesh(name, vertices, faces, parent, surface=None, shape=None):
    data = bpy.data.meshes.new(name + "_mesh")
    data.from_pydata(vertices, [], faces)
    data.update()
    bm = bmesh.new()
    bm.from_mesh(data)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(data)
    bm.free()
    obj = bpy.data.objects.new(name, data)
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = parent
    if shape:
        obj["geometry_role"] = "collision"
        obj.hide_render = True
        obj.display_type = "WIRE"
        rigid_settings(obj, shape)
    else:
        obj["geometry_role"] = "visual"
        data.materials.append(surface)
    return obj


def collider(visual, shape="CONVEX_HULL"):
    proxy = bpy.data.objects.new("Collision_" + visual.name, visual.data.copy())
    bpy.context.scene.collection.objects.link(proxy)
    proxy.parent = visual.parent
    proxy.matrix_basis = visual.matrix_basis.copy()
    proxy["geometry_role"] = "collision"
    proxy.hide_render = True
    proxy.display_type = "WIRE"
    rigid_settings(proxy, shape)
    return proxy


BOX_FACES = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]


def box(name, dimensions, position, surface, parent, bevel=0.0, collision=True):
    vertices = [(x * dimensions[0] / 2, y * dimensions[1] / 2, z * dimensions[2] / 2)
                for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    obj = mesh(name, vertices, BOX_FACES, parent, surface)
    obj.location = position
    if bevel:
        modifier = obj.modifiers.new("Manufactured edge", "BEVEL")
        modifier.width = bevel
        modifier.segments = 1
        modifier = obj.modifiers.new("Corner normals", "WEIGHTED_NORMAL")
        modifier.keep_sharp = True
    if collision:
        collider(obj, "BOX")
    return obj


def axis_point(axial, u, v, axis):
    if axis == "X":
        return (axial, u, v)
    if axis == "Y":
        return (u, axial, v)
    return (u, v, axial)


def cylinder(name, radius, depth, position, surface, parent, radius_end=None,
             axis="X", segments=16, collision=True):
    radius_end = radius if radius_end is None else radius_end
    vertices = [axis_point(end, r * math.cos(i * math.tau / segments),
                           r * math.sin(i * math.tau / segments), axis)
                for end, r in ((-depth / 2, radius), (depth / 2, radius_end))
                for i in range(segments)]
    faces = [tuple(range(segments - 1, -1, -1)), tuple(range(segments, 2 * segments))]
    faces.extend((i, (i + 1) % segments, (i + 1) % segments + segments, i + segments)
                 for i in range(segments))
    obj = mesh(name, vertices, faces, parent, surface)
    obj.location = position
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    if collision:
        collider(obj)
    return obj


def tube(name, outer, inner, depth, position, surface, parent, axis="X",
         outer_end=None, inner_end=None, segments=16, collision=True):
    """Closed hollow tube; each collision sector is a closed convex prism."""
    outer_end = outer if outer_end is None else outer_end
    inner_end = inner if inner_end is None else inner_end
    vertices = [axis_point(end, r * math.cos(i * math.tau / segments),
                           r * math.sin(i * math.tau / segments), axis)
                for end, r in ((-depth / 2, outer), (depth / 2, outer_end),
                               (-depth / 2, inner), (depth / 2, inner_end))
                for i in range(segments)]
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        sector_faces = [(i, j, segments + j, segments + i),
                        (2 * segments + j, 2 * segments + i, 3 * segments + i, 3 * segments + j),
                        (i, 2 * segments + i, 2 * segments + j, j),
                        (segments + i, segments + j, 3 * segments + j, 3 * segments + i)]
        faces.extend(sector_faces)
        if collision:
            indices = [i, j, segments + i, segments + j,
                       2 * segments + i, 2 * segments + j, 3 * segments + i, 3 * segments + j]
            local = {index: n for n, index in enumerate(indices)}
            closed = sector_faces + [(i, segments + i, 3 * segments + i, 2 * segments + i),
                                     (j, 2 * segments + j, 3 * segments + j, segments + j)]
            proxy = mesh(f"Collision_{name}_{i:02d}", [vertices[k] for k in indices],
                         [tuple(local[k] for k in face) for face in closed], parent,
                         shape="CONVEX_HULL")
            proxy.location = position
    obj = mesh(name, vertices, faces, parent, surface)
    obj.location = position
    for i, face in enumerate(obj.data.polygons):
        face.use_smooth = i % 4 in (0, 1)
    return obj


def label_patch(name, radius, depth, position, surface, parent, angle=math.pi / 2,
                span=1.12, segments=4):
    """A thin closed label following the upper surface of an X-axis cylinder."""
    count = segments + 1
    vertices = [(x, r * math.cos(angle - span / 2 + i * span / segments),
                  r * math.sin(angle - span / 2 + i * span / segments))
                for x, r in ((-depth / 2, radius), (depth / 2, radius),
                             (-depth / 2, radius - 0.00005), (depth / 2, radius - 0.00005))
                for i in range(count)]
    faces = []
    for i in range(segments):
        j = i + 1
        faces.extend([(i, j, count + j, count + i),
                      (2 * count + j, 2 * count + i, 3 * count + i, 3 * count + j),
                      (i, 2 * count + i, 2 * count + j, j),
                      (count + i, count + j, 3 * count + j, 3 * count + i)])
    for i in (0, segments):
        faces.append((i, count + i, 3 * count + i, 2 * count + i))
    obj = mesh(name, vertices, faces, parent, surface)
    obj.location = position
    for i, face in enumerate(obj.data.polygons):
        face.use_smooth = i % 4 in (0, 1)
    return obj


def joint(name, kind, fixed, moving, position, limits, rotation=(0, 0, 0)):
    obj = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(obj)
    obj.location = position
    obj.rotation_euler = rotation
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.rigidbody.constraint_add()
    obj.select_set(False)
    c = obj.rigid_body_constraint
    c.type = kind
    c.object1, c.object2 = fixed, moving
    c.disable_collisions = False
    if kind == "SLIDER":
        c.use_limit_lin_x = True
        c.limit_lin_x_lower, c.limit_lin_x_upper = limits
    else:
        c.use_limit_ang_z = True
        c.limit_ang_z_lower, c.limit_ang_z_upper = limits
    obj["preview_joint"] = True
    return obj


def generate(build, source_file, argv=None):
    parser = argparse.ArgumentParser(description="Build this object and a reusable source snapshot")
    parser.add_argument("--output", type=Path, required=True)
    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    output = parser.parse_args(argv).output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "METERS"
    scene.frame_start, scene.frame_end = 0, 48
    build()
    scene.rigidbody_world.enabled = False
    scene["penetration_tolerance_m"] = 0.0002
    scene.frame_set(0)
    bpy.context.view_layer.update()
    bpy.context.preferences.filepaths.save_version = 0
    destination = output / "object.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(destination))
    source = Path(source_file).resolve()
    for original in (source, source.parent / "metadata.json", source.parent / "preview.json",
                     Path(__file__).resolve()):
        target = output / original.name
        if original != target:
            shutil.copyfile(original, target)
    print("MODEL_BUILT", destination, flush=True)
