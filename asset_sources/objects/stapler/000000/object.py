"""White/blue miniature stapler with a native hinge and an open throat.

Local +X points to the rear hinge; +Y width; +Z up. The entire upper
cover/magazine/driver assembly moves together in this shape-stage model.
"""
import math

import bpy
from mathutils import Matrix, Vector

from table_1000.modeling.asset_builders import asset, body, box, collider, cylinder, generate, joint, material, mesh, group_part


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


def staple_channel(parent, surface):
    """One continuous steel trough; separate boxes supply convex collision."""
    front, stop_rear, rear = CHANNEL_FRONT, CHANNEL_FRONT + 0.0018, 0.018
    outer, inner = 0.01025, 0.00875
    bottom, floor_top, top = CHANNEL_BOTTOM, 0.016, 0.0225
    profile = [(-outer, bottom), (outer, bottom), (outer, top), (inner, top),
               (inner, floor_top), (-inner, floor_top), (-inner, top), (-outer, top)]
    vertices = [(front, y, z) for y, z in (profile[0], profile[1], profile[2], profile[7])]
    vertices += [(stop_rear, y, z) for y, z in profile[3:7]]
    vertices += [(rear, y, z) for y, z in profile]
    faces = [(0, 3, 2, 1), (0, 1, 9, 8), (1, 2, 10, 9), (3, 0, 8, 15),
             (2, 3, 15, 14, 7, 4, 11, 10), (4, 7, 6, 5), (4, 5, 12, 11),
             (5, 6, 13, 12), (6, 7, 14, 13), tuple(range(8, 16))]
    obj = mesh("Steel staple channel", vertices, faces, parent, surface)
    # Bevel outer lower folds and mouth corners, leaving panel rims sharp.
    weights = obj.data.attributes.new("bevel_weight_edge", "FLOAT", "EDGE")
    for edge in obj.data.edges:
        a, b = (obj.data.vertices[i].co for i in edge.vertices)
        lower_fold = (abs(a.z - bottom) < 1e-7 and abs(b.z - bottom) < 1e-7
                      and abs(a.y - b.y) < 1e-7 and abs(abs(a.y) - outer) < 1e-7)
        mouth_corner = (abs(a.x - front) < 1e-7 and abs(b.x - front) < 1e-7
                        and abs(a.y - b.y) < 1e-7 and abs(abs(a.y) - outer) < 1e-7)
        weights.data[edge.index].value = float(lower_fold or mouth_corner)
    modifier = obj.modifiers.new("Steel outer folds", "BEVEL")
    modifier.limit_method = "WEIGHT"
    modifier.width = 0.0008
    modifier.segments = 1
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
    channel_parts = [box("Steel staple channel floor", (channel_length, 0.0205, 0.0015),
                         (channel_center, 0, CHANNEL_BOTTOM + 0.00075), metal, upper)]
    for side, y in (("left", -0.0095), ("right", 0.0095)):
        channel_parts.append(box(f"Steel staple channel {side} wall", (channel_length, 0.0015, 0.0065),
                                 (channel_center, y, 0.01925), metal, upper))
        box(f"Channel to cover {side} bracket", (0.003, 0.0025, 0.0035),
            (0.012, y, 0.02325), metal, upper)
    channel_parts.append(box("Steel channel front stop", (0.0018, 0.0205, 0.008),
                             (CHANNEL_FRONT + 0.0009, 0, 0.0185), metal, upper))
    # Retain the convex proxies and replace their visuals with one steel trough.
    for obj in channel_parts:
        data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        bpy.data.meshes.remove(data)
    staple_channel(upper, metal)
    box("Steel driver", (0.001, 0.009, 0.005), (-0.0295, 0, 0.022), metal, upper)
    box("Blue rear bridge", (0.011, 0.020, 0.006), (0.024, 0, 0.022), blue, upper)
    bpy.context.view_layer.update()
    upper.matrix_basis = (Matrix.Translation(PIVOT) @ Matrix.Rotation(OPEN_ANGLE, 4, "Y")
                          @ Matrix.Translation(-PIVOT))
    joint("upper_press", "HINGE", base, upper, PIVOT, (PRESS_LIMIT, 0), (-math.pi / 2, 0, 0))

    organize_parts()


def organize_parts():
    """Group the original geometry while preserving every saved world pose."""
    root = bpy.data.objects['15_Mini_Stapler']
    root.name = 'stapler'
    root["asset_id"] = 'stapler/000000'
    owner = bpy.data.objects['Stapler base']
    owner.name = 'stapler.base'
    for component, names in [
        ('shell', ['White bottom shell']),
        ('plate', ['Steel lower plate']),
        ('anvil', ['Staple anvil']),
        ('clinch_groove1', ['Anvil long clinch groove']),
        ('clinch_groove2', ['Anvil cross clinch groove']),
        ('hinge_cheek1', ['Rear hinge cheek']),
        ('hinge_cheek2', ['Rear hinge cheek.001']),
        ('axle', ['Recessed pivot axle']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        group_part(owner.name + "." + component, owner, visuals, collisions)
    owner = bpy.data.objects['Stapler moving upper']
    owner.name = 'stapler.upper'
    for component, names in [
        ('cover', ['Complete blue upper cover']),
        ('channel', ['Steel staple channel']),
        ('bracket1', ['Channel to cover left bracket']),
        ('bracket2', ['Channel to cover right bracket']),
        ('driver', ['Steel driver']),
        ('rear_bridge', ['Blue rear bridge']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        if component == "channel":
            collisions = [bpy.data.objects[name] for name in [
                "Collision_Steel staple channel floor", "Collision_Steel staple channel left wall",
                "Collision_Steel staple channel right wall", "Collision_Steel channel front stop",
            ]]
        group_part(owner.name + "." + component, owner, visuals, collisions)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
