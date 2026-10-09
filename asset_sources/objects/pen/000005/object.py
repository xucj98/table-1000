"""Build this capped pen as two independent top-level rigid-body subtrees."""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import capped_pen, generate

PROFILE = {'geometry': {'segments': 16,
              'seat_x_m': -0.0365,
              'barrel_end_x_m': 0.07,
              'barrel_radius_m': 0.0048,
              'neck_length_m': 0.014,
              'neck_radius_m': 0.0038,
              'nib_length_m': 0.008,
              'nib_tip_radius_m': 0.0007,
              'nib_base_radius_m': 0.0026,
              'rear_start_x_m': 0.069,
              'rear_end_x_m': 0.0735,
              'rear_radius_m': 0.0048,
              'cap_radius_m': 0.0055,
              'cap_length_m': 0.045,
              'end_thickness_m': 0.001,
              'lip_length_m': 0.002,
              'shell_inner_radius_m': 0.0047,
              'rigid_lip_clearance_m': 0.0002},
 'barrel_color': [0.24, 0.43, 0.6],
 'cap_color': [0.61, 0.75, 0.82],
 'clip_color': [0.24, 0.43, 0.6],
 'rear_color': [0.78, 0.8, 0.78],
 'cap_transmission': 0.48,
 'rear_transmission': 0,
 'clip_dimensions_m': [0.033, 0.0018, 0.0018],
 'clip_position_m': [-0.025, 0.0015, 0.0058],
 'body_mass_kg': 0.006,
 'cap_mass_kg': 0.0015,
 'metal_point': True}


def build():
    capped_pen("pen/000005", PROFILE)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
