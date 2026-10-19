"""Engineering estimates for native finite-force click drives and body masses."""
from table_1000.physics.pen_assets import author as author_pen
MASSES = {'pen.shell': 0.01, 'pen.button': 0.001, 'pen.refill': 0.002}
CLICK = {'button': {'stiffness': 300., 'damping': .08, 'max_force': 2.},
         'refill': {'stiffness': 800., 'damping': .15, 'max_force': 3.}}
def author(geometry_path, output_path):
    return author_pen(geometry_path, output_path, masses=MASSES,
                      metal_parts=('pen.shell.nose', 'pen.shell.grip_band1', 'pen.shell.grip_band2', 'pen.shell.grip_band3', 'pen.shell.grip_band4', 'pen.button.button', 'pen.refill.tip'), rubber_parts=(),
                      articulation=True, click=CLICK)
