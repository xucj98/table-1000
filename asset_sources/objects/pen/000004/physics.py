"""Native cap detent masses, contact materials and spring drives.

Engineering estimates; effective follower mass is an inertia proxy.
COM and inertia are estimated by PhysX from collision geometry.
"""
from table_1000.physics.pen_assets import author as author_pen

MASSES = {'body': 0.006, 'cap': 0.0015}
IS_CLICK = False
METAL_PARTS = ('body.point',)
RUBBER_PARTS = ()


def author(geometry_path, output_path):
    return author_pen(geometry_path, output_path, masses=MASSES,
                      metal_parts=METAL_PARTS, rubber_parts=RUBBER_PARTS,
                      articulation=IS_CLICK, detent=DETENT)

DETENT = {'seat': -0.0365, 'neck': 0.0038, 'segments': 16, 'balls': 3, 'radius': 0.00035, 'height': 0.000196875, 'travel': 0.00035, 'mass': 8.3740234375e-05, 'stiffness': 765.625, 'damping': 0.13125, 'preload': 0.00013125, 'flat': False, 'dt': 0.001}
