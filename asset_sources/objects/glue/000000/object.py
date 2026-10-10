"""White superglue bottle with a yellow tapered nozzle cap; one rigid body."""
import math

import bpy

from table_1000.modeling.asset_builders import asset, body, cylinder, generate, label_patch, material, group_part


def build():
    white = material("White bottle plastic", (0.78, 0.80, 0.78), 0.29)
    yellow = material("Yellow nozzle and label", (0.70, 0.79, 0.012), 0.32)
    black = material("Label ink", (0.009, 0.012, 0.014), 0.70)
    root = asset("12_Superglue_Bottle", "bottom center of the upright bottle",
                 "+X width; +Y back; +Z upright toward nozzle", "label faces -Y")
    bottle = body("Superglue bottle", root)
    cylinder("White glue bottle", 0.012, 0.039, (0, 0, 0.025), white, bottle, axis="Z")
    cylinder("Yellow label band", 0.0121, 0.026, (0, 0, 0.027), yellow, bottle,
             axis="Z", collision=False)
    cylinder("White base rim", 0.0124, 0.006, (0, 0, 0.003), white, bottle, axis="Z")
    cylinder("White bottle neck", 0.0075, 0.004, (0, 0, 0.0455), white, bottle, axis="Z")
    cylinder("Yellow screw collar", 0.009, 0.010, (0, 0, 0.051), yellow, bottle, axis="Z")
    cylinder("Tapered nozzle cap", 0.0078, 0.039, (0, 0, 0.075), yellow, bottle,
             axis="Z", radius_end=0.0021)
    label_patch("Black label stripe", 0.01236, 0.0025, (0, 0, 0.013), black, bottle,
                span=1.5, segments=6, axis="Z", angle=-math.pi / 2)
    label_patch("Black label block", 0.01236, 0.011, (0, 0, 0.027), black, bottle,
                axis="Z", angle=-math.pi / 2)
    for z in (0.024, 0.028, 0.032):
        label_patch("Label print", 0.01245, 0.0015, (0, 0, z), yellow, bottle,
                    span=0.7, axis="Z", angle=-math.pi / 2)

    organize_parts()


def organize_parts():
    """Group the original geometry while preserving every saved world pose."""
    root = bpy.data.objects['12_Superglue_Bottle']
    root.name = 'glue'
    root["asset_id"] = 'glue/000000'
    owner = bpy.data.objects['Superglue bottle']
    owner.name = 'glue.bottle'
    for component, names in [
        ('container', ['White glue bottle']),
        ('label_band', ['Yellow label band']),
        ('base_rim', ['White base rim']),
        ('neck', ['White bottle neck']),
        ('screw_collar', ['Yellow screw collar']),
        ('nozzle_cap', ['Tapered nozzle cap']),
        ('label_stripe', ['Black label stripe']),
        ('label_block', ['Black label block']),
        ('label_print1', ['Label print']),
        ('label_print2', ['Label print.001']),
        ('label_print3', ['Label print.002']),
    ]:
        visuals = [bpy.data.objects[name] for name in names]
        collisions = sorted([obj for obj in owner.children
                             if any(obj.name == "Collision_" + name
                                    or obj.name.startswith("Collision_" + name + "_")
                                    for name in names)], key=lambda obj: obj.name)
        group_part(owner.name + "." + component, owner, visuals, collisions)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
