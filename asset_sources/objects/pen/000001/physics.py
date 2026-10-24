"""Engineering estimates for native finite-force click drives and body masses."""
from table_1000.physics.pen_assets import author as author_pen
MASSES = {'pen.shell': 0.006, 'pen.button': 0.0004, 'pen.refill': 0.0016}
CLICK = {'button': {'stiffness': 300., 'damping': .08, 'max_force': 2.},
         'refill': {'stiffness': 800., 'damping': .15, 'max_force': 3.}}
def author(geometry_path, output_path):
    return author_pen(geometry_path, output_path, masses=MASSES,
                      metal_parts=('pen.refill.tip',), rubber_parts=(),
                      articulation=True, click=CLICK)
