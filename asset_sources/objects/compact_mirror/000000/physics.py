"""One rigid body (compact_mirror): mass and friction are design values (metal), not calibration.
"""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.08
FRICTION = (0.4, 0.3)
ANGULAR_DAMPING = 1.0     # 1/s, stands in for rolling resistance (round body)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, angular_damping=ANGULAR_DAMPING)
