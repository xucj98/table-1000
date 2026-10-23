"""One rigid body (black breakout box with banana jacks): mass and friction are design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 1.2
FRICTION = (0.6, 0.5)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
