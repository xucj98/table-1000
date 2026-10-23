"""One rigid body (highlighter): mass and friction are design values (plastic), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.015
FRICTION = (0.4, 0.3)
ANGULAR_DAMPING = 2.     # 1/s, stands in for rolling resistance (the body is round)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, angular_damping=ANGULAR_DAMPING)
