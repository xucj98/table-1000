"""One rigid body (laptop): mass and friction are design values (rubber feet), not calibration.
"""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 1.37
FRICTION = (0.7, 0.6)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
