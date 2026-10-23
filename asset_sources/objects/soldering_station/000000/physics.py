"""One rigid body (soldering station: steel/plastic case with transformer): mass and friction are design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 1.4
FRICTION = (0.6, 0.5)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
