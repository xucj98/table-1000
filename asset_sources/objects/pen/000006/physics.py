"""Native cap detent masses, contact materials and spring drives.

Engineering estimates; effective follower mass is an inertia proxy.
COM and inertia are estimated by PhysX from collision geometry.
"""
from table_1000.physics.pen_assets import author as author_pen

MASSES = {'body': 0.012, 'cap': 0.003}
IS_CLICK = False
METAL_PARTS = ()
RUBBER_PARTS = ()


def author(geometry_path, output_path):
    return author_pen(geometry_path, output_path, masses=MASSES,
                      metal_parts=METAL_PARTS, rubber_parts=RUBBER_PARTS,
                      articulation=IS_CLICK, detent=DETENT)

DETENT = {'seat': -0.032, 'neck': 0.0078, 'segments': 12, 'balls': 3, 'radius': 0.0006, 'height': 0.00033749999999999996, 'travel': 0.0006, 'mass': 0.00042187499999999983, 'stiffness': 600.0, 'damping': 0.22499999999999995, 'preload': 0.0001, 'flat': False, 'dt': 0.001}
