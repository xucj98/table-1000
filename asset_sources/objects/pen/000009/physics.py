"""Approved engineering click drives; native force and reaction transmission."""
from table_1000.physics.pen_assets import author as author_pen
MASSES = {'pen.shell': .006, 'pen.button': .0004, 'pen.refill': .0016}
CLICK = {'button': {'stiffness': 300., 'damping': .08, 'max_force': 2.},
         'refill': {'stiffness': 800., 'damping': .15, 'max_force': 3.}}
def author(geometry_path, output_path):
    return author_pen(geometry_path, output_path, masses=MASSES,
                      metal_parts=('pen.refill.tip',), rubber_parts=('pen.shell.grip',),
                      articulation=True, click=CLICK)
