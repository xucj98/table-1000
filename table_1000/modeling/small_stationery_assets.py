"""Small stationery with semantic parts, imported through the modeling package."""

from __future__ import annotations

import math

import bpy

from table_1000.modeling.asset_builders import (
    asset, body, box, collider, material, mesh, part,
)


def rounded_prism(name, dimensions, position, radius, surface, parent,
                  *, segments=3, collision=True):
    """Closed convex rounded rectangle extruded along Z, without bevel layers."""
    length, width, height = dimensions
    outline = []
    for cx, cy, start in (
        (length / 2 - radius, width / 2 - radius, 0),
        (-length / 2 + radius, width / 2 - radius, math.pi / 2),
        (-length / 2 + radius, -width / 2 + radius, math.pi),
        (length / 2 - radius, -width / 2 + radius, 3 * math.pi / 2),
    ):
        for index in range(segments + 1):
            angle = start + index * math.pi / (2 * segments)
            point = (cx + radius * math.cos(angle), cy + radius * math.sin(angle))
            if not outline or math.dist(point, outline[-1]) > 1e-10:
                outline.append(point)
    if math.dist(outline[0], outline[-1]) < 1e-10:
        outline.pop()
    count = len(outline)
    vertices = [(x, y, z) for z in (-height / 2, height / 2) for x, y in outline]
    faces = [tuple(range(count - 1, -1, -1)), tuple(range(count, 2 * count))]
    faces.extend((i, (i + 1) % count, (i + 1) % count + count, i + count)
                 for i in range(count))
    obj = mesh(name, vertices, faces, parent, surface)
    obj.location = position
    if collision:
        collider(obj)
    return obj


def printed_rectangle(name, dimensions, position, surface, parent, *, underside=False):
    """A single planar printed face; no tiny boxes or contact bodies."""
    length, width = dimensions
    points = [(-length / 2, -width / 2, 0), (length / 2, -width / 2, 0),
              (length / 2, width / 2, 0), (-length / 2, width / 2, 0)]
    face = (3, 2, 1, 0) if underside else (0, 1, 2, 3)
    obj = mesh(name, points, [face], parent, surface)
    obj.location = position
    return obj


def printed_text(name, text, size, position, surface, parent):
    """Generic printed lettering, using Blender's font and converted to mesh."""
    data = bpy.data.curves.new(name, "FONT")
    data.body = text
    data.size = size
    data.align_x = "CENTER"
    data.align_y = "CENTER"
    data.resolution_u = 1
    obj = bpy.data.objects.new(name, data)
    bpy.context.scene.collection.objects.link(obj)
    obj.parent = parent
    obj.location = position
    data.materials.append(surface)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.convert(target="MESH")
    bpy.context.object["geometry_role"] = "visual"
    return bpy.context.object


def finish_part(node):
    """Join visual patches within a semantic part and normalize complete names."""
    visuals = [obj for obj in node.children if obj.get("geometry_role") == "visual"]
    bpy.ops.object.select_all(action="DESELECT")
    for obj in visuals:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = visuals[0]
    if len(visuals) > 1:
        bpy.ops.object.join()
    bpy.context.object.name = node.name + ".visual"
    bpy.context.object.data.name = bpy.context.object.name + ".mesh"
    collisions = sorted((obj for obj in node.children
                         if obj.get("geometry_role") == "collision"), key=lambda obj: obj.name)
    for index, obj in enumerate(collisions, 1):
        obj.name = node.name + ".collision" + (str(index) if len(collisions) > 1 else "")
        obj.data.name = obj.name + ".mesh"
    for obj in node.children:
        obj.hide_select = True
    node.hide_select = True


def finish_body(rigid):
    """Select the complete transport body; nested parts are never independent."""
    for node in rigid.children:
        finish_part(node)
    bpy.ops.object.select_all(action="DESELECT")
    rigid.select_set(True)
    bpy.context.view_layer.objects.active = rigid


def stationery_body(name):
    root = asset(name, "bottom footprint center",
                 "+X length toward rear; +Y width; +Z up", "top face is +Z")
    return body(name + ".body", root)


def build_red_eraser(dimensions):
    """White rubber, an attached red paper sleeve, and integral printed marks."""
    length, width, height = dimensions
    rigid = stationery_body("eraser")
    rubber_part = part("eraser.body.rubber", rigid)
    sleeve = part("eraser.body.paper_sleeve", rigid)
    printing = part("eraser.body.sleeve_print", rigid)
    rubber = material("Warm white rubber", (0.84, 0.83, 0.77), 0.76)
    red = material("Red paper sleeve", (0.62, 0.016, 0.023), 0.72)
    paper = material("White sleeve print", (0.87, 0.86, 0.82), 0.77)
    ink = material("Dark sleeve print", (0.015, 0.012, 0.012), 0.8)
    skin = 0.0002
    sleeve_length = length * 0.82
    rounded_prism(rubber_part.name + ".visual", (length, width - 2 * skin, height - 2 * skin),
                  (0, 0, height / 2), 0.00055, rubber, rubber_part, segments=2)
    for index, z in enumerate((skin / 2, height - skin / 2), 1):
        box(sleeve.name + f".face{index}", (sleeve_length, width, skin),
            (0, 0, z), red, sleeve)
    for index, y in enumerate((-width / 2 + skin / 2, width / 2 - skin / 2), 1):
        box(sleeve.name + f".side{index}", (sleeve_length, skin, height - 2 * skin),
            (0, y, height / 2), red, sleeve)
    print_z = height + 0.000003
    for index, x in enumerate((-0.011, 0.011), 1):
        points = [(width * 0.29 * math.cos(i * math.tau / 12),
                   width * 0.29 * math.sin(i * math.tau / 12), 0) for i in range(12)]
        obj = mesh(printing.name + f".seal{index}", points, [tuple(range(12))], printing, paper)
        obj.location = (x, 0, print_z)
        for stripe, offset in enumerate((-0.0015, 0, 0.0015), 1):
            printed_rectangle(printing.name + f".seal{index}_stripe{stripe}",
                              (0.006, 0.00055), (x, offset, print_z + 0.000003), red, printing)
    printed_rectangle(printing.name + ".end_band", (0.003, width),
                      (-sleeve_length / 2 + 0.002, 0, print_z), ink, printing)
    printed_text(printing.name + ".grade", "2B", 0.0028,
                 (sleeve_length / 2 - 0.0025, 0, print_z + 0.000003), paper, printing)
    printed_rectangle(printing.name + ".barcode_patch", (0.025, 0.010),
                      (0, 0, -0.000003), paper, printing, underside=True)
    for index, bar_width in enumerate((0.00035, 0.0007, 0.00025, 0.0005, 0.0003,
                                       0.00065, 0.00025, 0.00045, 0.0007, 0.0003,
                                       0.00035, 0.00055, 0.00025, 0.0006, 0.0003)):
        printed_rectangle(printing.name + f".barcode{index + 1}", (bar_width, 0.007),
                          (-0.0105 + index * 0.0014, 0, -0.000006), ink, printing, underside=True)
    finish_body(rigid)


def build_white_knife(dimensions):
    """White short housing and a fixed, enclosed blade; no cutting mechanism."""
    length, width, height = dimensions
    rigid = stationery_body("utility_knife")
    housing = part("utility_knife.body.housing", rigid)
    slider = part("utility_knife.body.slider", rigid)
    track = part("utility_knife.body.slider_track", rigid)
    guide = part("utility_knife.body.blade_guide", rigid)
    blade = part("utility_knife.body.retracted_blade", rigid)
    white = material("White knife ABS", (0.82, 0.84, 0.83), 0.32)
    rim = material("White upper shell", (0.91, 0.92, 0.90), 0.35)
    green = material("Pale green fixed slider", (0.48, 0.59, 0.37), 0.44)
    dark = material("Recessed slider track", (0.018, 0.022, 0.023), 0.52)
    steel = material("Recessed steel guide", (0.39, 0.42, 0.44), 0.32, metallic=0.8)
    # One hull covers the closed housing; the thin face plate is only visual.
    rounded_prism(housing.name + ".shell", (length, width, height * 0.93),
                  (0, 0, height * 0.465), width / 2, white, housing, segments=4)
    rounded_prism(housing.name + ".face_plate", (length - 0.001, width - 0.001, height * 0.05),
                  (0, 0, height * 0.905), (width - 0.001) / 2, rim, housing,
                  segments=4, collision=False)
    top = height * 0.93
    printed_rectangle(track.name + ".visual", (length * 0.43, 0.0022),
                      (0.001, 0, top + 0.000004), dark, track)
    rounded_prism(slider.name + ".visual", (0.015, 0.007, height * 0.07),
                  (0.010, 0, height * 0.965), 0.0027, green, slider, segments=2)
    printed_rectangle(guide.name + ".mouth", (0.0055, 0.0018),
                      (-length * 0.365, 0, top + 0.000003), steel, guide)
    printed_rectangle(guide.name + ".rear_mark", (0.004, 0.0005),
                      (length * 0.36, 0, top + 0.000003), steel, guide)
    # A simple steel insert entirely within the closed shell, with no redundant
    # interior contact proxy and no exposed tip. This is a fixed visual part.
    box(blade.name + ".visual", (length * 0.58, 0.009, 0.0003),
        (-0.004, 0, height * 0.45), steel, blade, collision=False)
    finish_body(rigid)


def build_white_board_eraser(dimensions):
    """White shell, pad mount and integral felt approximation, without wiping."""
    length, width, height = dimensions
    rigid = stationery_body("board_eraser")
    shell = part("board_eraser.body.shell", rigid)
    mount = part("board_eraser.body.pad_mount", rigid)
    pad = part("board_eraser.body.felt_pad", rigid)
    marking = part("board_eraser.body.shell_print", rigid)
    white = material("White board eraser shell", (0.88, 0.90, 0.88), 0.36)
    seam = material("Dark pad mount edge", (0.025, 0.029, 0.03), 0.6)
    felt = material("Black felt approximation", (0.012, 0.015, 0.018), 0.96)
    nodes = felt.node_tree.nodes
    noise = nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 140
    noise.inputs["Detail"].default_value = 2
    bump = nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.18
    bump.inputs["Distance"].default_value = 0.00015
    felt.node_tree.links.new(noise.outputs["Fac"], bump.inputs["Height"])
    felt.node_tree.links.new(bump.outputs["Normal"],
                             nodes.get("Principled BSDF").inputs["Normal"])
    pad_height, mount_height = 0.004, 0.001
    rounded_prism(pad.name + ".visual", (length - 0.002, width - 0.002, pad_height),
                  (0, 0, pad_height / 2), 0.008, felt, pad)
    rounded_prism(mount.name + ".visual", (length - 0.001, width - 0.001, mount_height),
                  (0, 0, pad_height + mount_height / 2), 0.0085, seam, mount, collision=False)
    shell_height = height - pad_height - mount_height
    rounded_prism(shell.name + ".visual", (length, width, shell_height),
                  (0, 0, height - shell_height / 2), 0.009, white, shell)
    # The mount's 1 mm boundary shares the shell contact material for now.
    # Extend only the shell collision to cover it; keep the felt contact separate.
    shell_collision = next(obj for obj in shell.children if obj.get("geometry_role") == "collision")
    for vertex in shell_collision.data.vertices:
        if vertex.co.z < 0:
            vertex.co.z -= mount_height
    grey = material("Subtle generic shell print", (0.35, 0.38, 0.39), 0.75)
    printed_text(marking.name + ".visual", "ERASER", 0.0035,
                 (0, 0, height + 0.000005), grey, marking)
    finish_body(rigid)
