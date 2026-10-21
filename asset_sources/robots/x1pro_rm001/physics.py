"""Joint drives, solver settings and contact materials for X1Pro with the rm001 gripper.

Everything as for x1pro (../x1pro/physics.py) except the finger drives: the
fingers pivot, so their gains are per radian. 100 N m/rad holds a 14 mm stick
at the fingertips under the acceptance shake (30 N m/rad, the x1pro finger
gain at the 0.14 m finger length, lets wide objects slip); damping 10 N m s/rad
closes the fingers with a 0.1 s time constant, about as fast as x1pro's
(without it they close at the 11.5 rad/s URDF limit, 1.6 m/s at the
fingertips, and dent a thin object by 4 mm). Simulation values, not calibrated
against the real controller.
"""

from table_1000.robots import source_dir, source_module

_X1PRO = source_module(source_dir('x1pro'), 'physics')

DRIVES = {**_X1PRO.DRIVES, '_arm_gripper_left_joint': (100., 10.)}


def author(import_path, output_path):
    """Write ``output_path`` as a layer over the imported robot."""
    _X1PRO.author(import_path, output_path, DRIVES)
