"""One rigid body (cable_coil): mass and friction are design values (PVC cable), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.09
FRICTION = (0.5, 0.45)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
