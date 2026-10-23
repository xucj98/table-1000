"""One rigid body (power_adapter): mass and friction are design values (plastic), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.25
FRICTION = (0.5, 0.4)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
