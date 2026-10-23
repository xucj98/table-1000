"""X1Pro with the rm001 gripper (:data:`SPEC`, see table_1000/robots/spec.py).

The same robot as x1pro (../x1pro/robot.py: joints, base, cameras; the
``ready`` keyframe with a higher torso) with a gripper whose two fingers pivot about the gripper's Z axis, 0.025 m to
either side of its centre line, 0.077 m beyond ``<side>_arm_link6``. Each
finger is 0.14 m long; its pad point is the fingertip frame
``<side>_arm_gripper_<finger>_tip_link``, 0.015 m inside the finger axis. The
fingers are driven in radians, the right at minus the left angle; the pad gap
is the distance between the two fingertip frames, 0 at -0.072 rad (the lower
limit, -0.073) to 0.245 m at the upper limit 0.872 rad.

TCP: a frame on ``<side>_arm_link6`` 0.198 m along the gripper axis, centred
between the fingers (0.01 m below the axis, like them); +X is the approach
direction, +Y the direction in which the left finger opens, +Z points to the
wrist camera. Up to a gap of 0.1 m the fingertips are 0.018 to 0.020 m beyond
the TCP, as on x1pro (0.0185 m).
"""

from dataclasses import replace

from table_1000.robots import source_dir, source_module

_X1PRO = source_module(source_dir('x1pro'), 'robot')

TCP_OFFSET = (.198, 0., -.01)
# (pivot offset from the centre line, fingertip (x, y) in the finger frame)
FINGER_PIVOT = (.025, (.14, -.015))
MAX_GAP = .245

# As x1pro's, with the torso 0.05 m higher: the TCP is 0.055 m further from the wrist, and top-down
# grasps 0.17 m above the table in front of the robot (the acceptance grasps) need the higher torso.
READY = {**_X1PRO.READY, 'lift_joint': .45}

SPEC = replace(
    _X1PRO.SPEC, name='x1pro_rm001', urdf='asset_sources/robots/x1pro_rm001/robot.urdf', keyframes={'ready': READY},
    arms={name: replace(arm, tcp=(arm.tcp[0], TCP_OFFSET), finger_pivot=FINGER_PIVOT, max_gap=MAX_GAP)
          for name, arm in _X1PRO.SPEC.arms.items()})
