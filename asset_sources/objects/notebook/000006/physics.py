"""One rigid body (A5 notebook): mass and friction are design values (paper cover), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.2
FRICTION = (0.5, 0.4)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
