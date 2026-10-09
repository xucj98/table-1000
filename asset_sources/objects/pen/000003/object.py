"""Build this capped pen as two independent top-level rigid-body subtrees."""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import capped_pen, generate

PROFILE = {'geometry': {'segments': 16,
              'seat_x_m': -0.032,
              'barrel_end_x_m': 0.0545,
              'barrel_radius_m': 0.009,
              'neck_length_m': 0.021,
              'neck_radius_m': 0.0078,
              'nib_length_m': 0.01,
              'nib_tip_radius_m': 0.0022,
              'nib_base_radius_m': 0.0042,
              'rear_start_x_m': 0.0535,
              'rear_end_x_m': 0.093,
              'rear_radius_m': 0.01,
              'cap_radius_m': 0.0098,
              'cap_length_m': 0.054,
              'end_thickness_m': 0.0015,
              'lip_length_m': 0.003,
              'shell_inner_radius_m': 0.0086,
              'rigid_lip_clearance_m': 0.0002},
 'barrel_color': [0.82, 0.82, 0.73],
 'cap_color': [0.008, 0.014, 0.09],
 'clip_color': [0.008, 0.014, 0.09],
 'rear_color': [0.67, 0.7, 0.69],
 'cap_transmission': 0,
 'rear_transmission': 0.48,
 'clip_dimensions_m': [0.039, 0.0034, 0.0023],
 'clip_position_m': [-0.0265, 0, 0.01095],
 'body_mass_kg': 0.012,
 'cap_mass_kg': 0.003,
 'metal_point': False}


def build():
    capped_pen("pen/000003", PROFILE)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
