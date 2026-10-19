"""Native cap detent masses, contact materials and spring drives.

Engineering estimates; effective follower mass is an inertia proxy.
COM and inertia are estimated by PhysX from collision geometry.
"""
from table_1000.physics.pen_assets import author as author_pen

MASSES = {'body': 0.011, 'cap': 0.002}
IS_CLICK = False
METAL_PARTS = ()
RUBBER_PARTS = ()


def author(geometry_path, output_path):
    return author_pen(geometry_path, output_path, masses=MASSES,
                      metal_parts=METAL_PARTS, rubber_parts=RUBBER_PARTS,
                      articulation=IS_CLICK, detent=DETENT)

DETENT = {'seat': -0.025, 'neck': 0.0056, 'segments': 12, 'balls': 3, 'radius': 0.00045, 'height': 0.000253125, 'travel': 0.00045000000000000004, 'mass': 0.000177978515625, 'stiffness': 984.375, 'damping': 0.16874999999999998, 'preload': 0.00016874999999999998, 'flat': False, 'dt': 0.001}
