"""RM001 simulation tuning, without calibration against the real gripper.

Finger drive K=50 N m/rad, D=20 N m s/rad.
The build explicitly copies URDF effort limits (20 N m for the fingers).
High friction and torsional resistance belong only to the separate inner pads;
the finger structure uses the body's material. Pads are rigid collision shapes,
not a deformation model of rubber.

The fingers carry an armature, of the order of a small motor's reflected inertia.
Without it the light pivoting fingers, held by the drive, the mimic coupling, a
grasped object and the table at once, are thrown past their limits in one step
when a fingertip touches the table while closing, and the mimic coupling yields
(both fingers and the object shift sideways by 25 mm).
"""

from table_1000.robots import source_dir, source_module

_X1PRO = source_module(source_dir('x1pro'), 'physics')

DRIVES = {**_X1PRO.DRIVES, '_arm_gripper_left_joint': (50., 20.)}
MATERIALS = {**_X1PRO.MATERIALS, 'finger': (.4, .3), 'pad': (2., 1.6)}
ARMATURE = {'_arm_gripper_': .001}   # kg m^2
# 16 position iterations hold small objects in these fingers under a push
# (grasp_and_push); more fling them.
SOLVER_ITERATIONS = (16, 4)


def author(import_path, output_path):
    """Write ``output_path`` as a layer over the imported robot."""
    _X1PRO.author(import_path, output_path, DRIVES, MATERIALS, ARMATURE, SOLVER_ITERATIONS)
