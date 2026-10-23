"""One rigid body (reed_diffuser): mass and friction are design values (glass/painted wood), not calibration.

The collision hulls stand in for light reeds above a heavy glass base, so their
uniform-density centroid sits far too high; the centre of mass is set low in the base instead.
"""

from table_1000.physics.rigid import author as author_rigid

MASS_KG = 0.35
FRICTION = (0.5, 0.4)
CENTER_OF_MASS_M = (0., 0., 0.032)


def author(geometry_path, output_path):
    author_rigid(geometry_path, output_path, MASS_KG, FRICTION, center_of_mass=CENTER_OF_MASS_M)
