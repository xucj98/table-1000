"""One rigid body (iron holder: cast base dominates the mass, spring and tray are light): mass and friction are design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.45
FRICTION = (0.6, 0.5)
CENTER_OF_MASS = (-0.002, 0.004, 0.009)   # m, heavy cast base: real mass sits low


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, center_of_mass=CENTER_OF_MASS)
