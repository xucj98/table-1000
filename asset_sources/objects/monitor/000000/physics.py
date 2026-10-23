"""One rigid body (LCD monitor: plastic bezel and housing on a weighted oval stand): mass and friction are design values, not calibration.
The stand base is weighted, so the centre of mass sits low instead of where uniform collider density would put it."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 3.5
FRICTION = (0.6, 0.5)
CENTER_OF_MASS = (0.0, 0.01, 0.09)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, center_of_mass=CENTER_OF_MASS)
