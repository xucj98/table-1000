"""One rigid body (eraser): mass and friction are design values (rubber), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.015
FRICTION = (0.8, 0.7)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
