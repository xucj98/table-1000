"""Drawer unit physics: housing articulation, drawers on friction slides; design values, not calibration."""

from table_1000.physics.drawers import author as author_drawers

HOUSING_KG = 0.9
DRAWER_KG = 0.15
FRICTION = (.30, .22)
FOOT_FRICTION = (.85, .70)


def author(geometry_path, output_path):
    author_drawers(geometry_path, output_path, housing_kg=HOUSING_KG, drawer_kg=DRAWER_KG,
                   friction=FRICTION, foot_friction=FOOT_FRICTION)
