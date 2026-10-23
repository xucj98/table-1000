"""One rigid body (staple box): mass and friction are design values (metal), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.01
FRICTION = (0.3, 0.25)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
