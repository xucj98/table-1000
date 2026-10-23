"""One rigid body (cream_jar): mass and friction are design values (glass/painted wood), not calibration.
"""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.12
FRICTION = (0.5, 0.4)
ANGULAR_DAMPING = 2.0     # 1/s, stands in for rolling resistance (round body)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, angular_damping=ANGULAR_DAMPING)
