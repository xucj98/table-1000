"""Build this capped pen as two independent top-level rigid-body subtrees."""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
import bpy
from asset_builders import capped_pen, generate, group_part

PROFILE = {'geometry': {'segments': 16,
              'seat_x_m': -0.032,
              'barrel_end_x_m': 0.086,
              'barrel_radius_m': 0.009,
              'neck_length_m': 0.026,
              'neck_radius_m': 0.0078,
              'nib_length_m': 0.013,
              'nib_tip_radius_m': 0.0022,
              'nib_base_radius_m': 0.0042,
              'cap_radius_m': 0.01,
              'cap_length_m': 0.061,
              'end_thickness_m': 0.0015,
              'lip_length_m': 0.003,
              'shell_inner_radius_m': 0.0086,
              'rigid_lip_clearance_m': 0.0002,
              'label_length_m': 0.046},
 'barrel_color': [0.008, 0.014, 0.09],
 'cap_color': [0.67, 0.7, 0.69],
 'cap_transmission': 0.48,
 'body_mass_kg': 0.012,
 'cap_mass_kg': 0.003,
 'metal_point': False,
 'label_color': [0.82, 0.82, 0.73]}


def build():
    capped_pen("pen/000006", PROFILE)
    for root_name, part_name, visual_name in [
        ("body", "barrel", "Barrel"), ("body", "label", "Pale paper label"),
        ("body", "neck", "Neck"), ("body", "tip", "Nib"),
        ("cap", "shell", "Cap shell"), ("cap", "lip", "Cap lip"), ("cap", "end", "Cap end"),
    ]:
        owner = bpy.data.objects[root_name]
        visual = bpy.data.objects[visual_name]
        collisions = sorted([obj for obj in owner.children if obj.name == "Collision_" + visual_name
                             or obj.name.startswith("Collision_" + visual_name + "_")], key=lambda obj: obj.name)
        group_part(root_name + "." + part_name, owner, [visual], collisions)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
