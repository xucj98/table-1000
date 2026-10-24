"""Drawer unit physics: housing articulation, drawers on friction slides; design values, not calibration."""

from table_1000.physics.drawers import author as author_drawers

HOUSING_KG = 1.6
DRAWER_KG = 0.30
FRICTION = (.35, .25)
FOOT_FRICTION = (.85, .70)
# Steel drawers are heavy: 1 N of slide friction keeps one shut until the chest tilts about 20 degrees.
SLIDE_FRICTION_N = (1., .7)


def author(geometry_path, output_path):
    author_drawers(geometry_path, output_path, housing_kg=HOUSING_KG, drawer_kg=DRAWER_KG,
                   friction=FRICTION, foot_friction=FOOT_FRICTION,
                   static_friction_n=SLIDE_FRICTION_N[0], dynamic_friction_n=SLIDE_FRICTION_N[1])
