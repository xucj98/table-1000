"""One rigid body (oscilloscope probe with ground lead and cable stub): mass and friction are design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.04
FRICTION = (0.6, 0.5)
ANGULAR_DAMPING = 1.0     # 1/s, stands in for rolling resistance (round probe body)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, angular_damping=ANGULAR_DAMPING)
