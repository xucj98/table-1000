"""One rigid body (helping hands: cast-iron base carries most of the mass): mass and friction are design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.45
FRICTION = (0.6, 0.5)
CENTER_OF_MASS = (0.0, 0.004, 0.010)   # m, heavy cast base: real mass sits low


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, center_of_mass=CENTER_OF_MASS)
