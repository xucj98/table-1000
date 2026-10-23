"""One rigid body (battery): mass and friction are design values (metal), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.023
FRICTION = (0.4, 0.3)
ANGULAR_DAMPING = 2.0     # 1/s, stands in for rolling resistance (the body is round)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, angular_damping=ANGULAR_DAMPING)
