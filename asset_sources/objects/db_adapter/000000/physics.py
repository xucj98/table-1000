"""One rigid body (db_adapter): mass and friction are design values (plastic and steel), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.035
FRICTION = (0.4, 0.35)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
