"""One rigid body (electrical_tape): mass and friction are design values (PVC), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.04
FRICTION = (0.6, 0.5)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
