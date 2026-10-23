"""One rigid body (screwdriver set: six screwdrivers in a clear PET case): mass and friction are design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.16
FRICTION = (0.4, 0.3)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
