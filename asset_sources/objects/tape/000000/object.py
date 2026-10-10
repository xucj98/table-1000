"""Hollow milky tape roll, one compound body with an open cardboard core."""

import bpy

from table_1000.modeling.asset_builders import asset, body, generate, material, tube, group_part


def build():
    tape = material("Milky adhesive tape", (0.78, 0.80, 0.72), 0.30, transmission=0.35)
    core = material("Cardboard core", (0.48, 0.40, 0.28), 0.70)
    root = asset("09_Tape_Roll", "bottom center of the roll", "+X and +Y radial; +Z roll axis",
                 "radially symmetric")
    roll = body("Tape roll", root)
    tube("Wound tape", 0.0305, 0.0173, 0.020, (0, 0, 0.0102), tape, roll, axis="Z")
    tube("Cardboard inner core", 0.0173, 0.0158, 0.0204, (0, 0, 0.0102), core, roll, axis="Z")

    organize_parts()


def organize_parts():
    """Group the original geometry while preserving every saved world pose."""
    root = bpy.data.objects['09_Tape_Roll']
    root.name = 'tape'
    root["asset_id"] = 'tape/000000'
    owner = bpy.data.objects['Tape roll']
    owner.name = 'tape.roll'
    for component, names in [
        ('winding', ['Wound tape']),
        ('core', ['Cardboard inner core']),
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
