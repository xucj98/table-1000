"""Joint drives, solver settings and contact materials for X1Pro with the rm001 gripper.

Everything as for x1pro (../x1pro/physics.py) except the fingers, which pivot
(gains per radian) and hold objects at their tips:

- drive 100 N m/rad, damping 20 N m s/rad. 30 N m/rad, the x1pro finger gain at
  the 0.14 m finger length, lets a box and a can slip 6 mm under the acceptance
  shake; 300 N m/rad squeezes a thin stick out. Damping closes the fingers
  with a 0.2 s time constant; without it they close at the 11.5 rad/s URDF
  limit (1.6 m/s at the fingertips) and dent a thin object by 4 mm;
- pad friction (2.0, 1.6), averaged with the object's: with x1pro's (1.0, 0.8)
  held objects drift 1.6–3 mm in the shake, with these under 1 mm
  (outputs/robots/x1pro_rm001/experiments/grasp-sweep).

Simulation values, not calibrated against the real controller.
"""

from table_1000.robots import source_dir, source_module

_X1PRO = source_module(source_dir('x1pro'), 'physics')

DRIVES = {**_X1PRO.DRIVES, '_arm_gripper_left_joint': (100., 20.)}
MATERIALS = {**_X1PRO.MATERIALS, 'finger': (2., 1.6)}


def author(import_path, output_path):
    """Write ``output_path`` as a layer over the imported robot."""
    _X1PRO.author(import_path, output_path, DRIVES, MATERIALS)
