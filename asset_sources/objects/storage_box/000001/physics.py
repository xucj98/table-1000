"""One rigid body (jar with lid): mass and friction are design values (glass), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.12
FRICTION = (0.35, 0.3)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
