"""RM001 simulation tuning, without calibration against the real gripper.

K=50 N m/rad, D=20 N m s/rad reduces squeezing/rolling compared with K=100.
The build explicitly copies URDF effort limits (20 N m for the fingers).
High friction and torsional resistance belong only to the separate inner pads;
the finger structure uses the body's material. Pads are rigid collision shapes,
not a deformation model of rubber.
"""

from table_1000.robots import source_dir, source_module

_X1PRO = source_module(source_dir('x1pro'), 'physics')

DRIVES = {**_X1PRO.DRIVES, '_arm_gripper_left_joint': (50., 20.)}
MATERIALS = {**_X1PRO.MATERIALS, 'finger': (.4, .3), 'pad': (2., 1.6)}


def author(import_path, output_path):
    """Write ``output_path`` as a layer over the imported robot."""
    _X1PRO.author(import_path, output_path, DRIVES, MATERIALS)
