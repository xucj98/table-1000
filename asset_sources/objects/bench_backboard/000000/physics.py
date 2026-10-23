"""One rigid body (freestanding bench backboard): the heavy foot rail carries the mass, so the centre of mass is set low and over the foot; design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 12.0
FRICTION = (0.6, 0.5)
CENTER_OF_MASS = (0.0, -0.015, 0.060)   # m, low and forward over the foot rail


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, center_of_mass=CENTER_OF_MASS)
