"""Three solid disk magnet variants; no magnetic force model.

The bottom contact plane is Z=0. The inherited 25 mm cap and 20 mm base
overlap by 1.05 mm, giving 7.45 mm actual height (the old 9 mm was nominal).
"""

import bpy

from table_1000.modeling.asset_builders import (
    asset, body, collider, cylinder, generate, material, part,
)


VARIANTS = {
    "orange": ((0.97, 0.37, 0.025), 0.23),
    "green": ((0.08, 0.43, 0.06), 0.22),
    "red": ((0.65, 0.018, 0.025), 0.20),
}


def rounded_disk(name, radius, depth, z, surface, parent):
    """One semantic disk with a matching low-poly convex collision mesh."""
    component = part(name, parent)
    # 24 radial segments deviate from the analytic cap circle by at most
    # 0.107 mm. One bevel segment retains the 0.5 mm manufactured edge.
    obj = cylinder(name + ".visual", radius, depth, (0, 0, z), surface, component,
                   axis="Z", segments=24, collision=False)
    bpy.context.view_layer.objects.active = obj
    bevel = obj.modifiers.new("Manufactured rim", "BEVEL")
    bevel.width = 0.0005
    bevel.segments = 1
    bpy.ops.object.modifier_apply(modifier=bevel.name)
    collider(obj).name = name + ".collision"
    return obj


def build_magnet(color):
    rgb, roughness = VARIANTS[color]
    root = asset("magnet", "center of the base contact plane",
                 "+X width; +Y back; +Z from base toward colored cap",
                 "rotationally symmetric; -Y is the preview front")
    whole = body("magnet.body", root)
    # Preserved engineering estimate, not a calibrated dynamics parameter.
    whole.rigid_body.mass = 0.008
    base = material("Magnet base", (0.18, 0.20, 0.22), 0.30, metallic=0.90)
    cap = material(f"{color.title()} ABS cap", rgb, roughness)
    rounded_disk("magnet.body.magnetic_base", 0.010, 0.004, 0.002, base, whole)
    rounded_disk("magnet.body.plastic_cap", 0.0125, 0.0045, 0.0052, cap, whole)
    # Make the complete object the default editable selection. Geometry remains
    # available in the Outliner but cannot accidentally be dragged on its own.
    bpy.ops.object.select_all(action="DESELECT")
    root.hide_select = True
    for obj in whole.children_recursive:
        obj.hide_select = True
    whole.select_set(True)
    bpy.context.view_layer.objects.active = whole


def generate_magnet(color, source_file, argv=None):
    generate(lambda: build_magnet(color), source_file, argv)
