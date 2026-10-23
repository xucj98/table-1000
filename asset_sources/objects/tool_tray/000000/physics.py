"""One rigid body (tool_tray): mass and friction are design values (plastic), not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.45
FRICTION = (0.5, 0.4)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION)
