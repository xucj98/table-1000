"""One rigid body (snack): mass and friction are design values (food packaging), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.03
FRICTION = (0.6, 0.5)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
