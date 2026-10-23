"""One rigid body (desk lamp in a fixed pose): the cast base carries most of the mass, so the centre of mass sits inside the base footprint; design values, not calibration."""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 2.5
FRICTION = (0.6, 0.5)
CENTER_OF_MASS = (0.0, -0.030, 0.040)   # m, inside the 160 mm base, low


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, center_of_mass=CENTER_OF_MASS)
