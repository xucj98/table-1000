"""X1Pro description (:data:`SPEC`) and the legacy right-arm planner.

TCP: a frame on ``right_arm_link6`` 0.145 m along the gripper axis, centred
between the finger pads; +X is the approach direction, +Y the direction in
which the left finger opens, +Z points to the wrist camera. The fingertips are
0.0185 m beyond the TCP.

Fingers are driven in metres: the left finger at ``+gap/2`` and the right at
``-gap/2`` gives a pad gap of ``gap`` (0 to 0.0975 m).
"""

from pathlib import Path

import numpy as np

from table_1000.robots.agent import ArmSpec, BaseSpec, RobotSpec

ARM = [f'right_arm_joint{i}' for i in range(1, 7)]
FINGERS = ['right_arm_gripper_left_joint', 'right_arm_gripper_right_joint']
FINGER_LINKS = ['right_arm_gripper_left_link', 'right_arm_gripper_right_link']
TCP_LINK, TCP_PARENT, TCP_OFFSET = 'right_tcp', 'right_arm_link6', (.145, 0., 0.)
TIP_BEYOND_TCP = .0185
MAX_GAP = .0975
# Joint-space limits for time parameterization (rad/s, rad/s^2), well below
# the URDF velocity limits.
VELOCITY_LIMIT, ACCELERATION_LIMIT = .8, 1.5
# Planning hulls (except the fingers, which touch objects on purpose) are grown
# by this much, so plans keep the robot clear of itself and of objects.
PLANNING_PADDING = .002
# Bodies whose collision hulls overlap at every configuration: the head
# housing seated on the torso and the closed finger pads.
SEATED = [('head_pitch_link', 'lift_link'),
          ('left_arm_gripper_left_link', 'left_arm_gripper_right_link'),
          ('right_arm_gripper_left_link', 'right_arm_gripper_right_link')]


def _arm(side):
    return ArmSpec(
        name=side, joints=tuple(f'{side}_arm_joint{i}' for i in range(1, 7)),
        tcp_link=f'{side}_tcp', tcp=(f'{side}_arm_link6', TCP_OFFSET),
        fingers=(f'{side}_arm_gripper_left_joint', f'{side}_arm_gripper_right_joint'),
        finger_links=(f'{side}_arm_gripper_left_link', f'{side}_arm_gripper_right_link'),
        max_gap=MAX_GAP, mount='lift_link', velocity_limit=VELOCITY_LIMIT, acceleration_limit=ACCELERATION_LIMIT)


# Arms raised in front of the torso, head looking at the work surface. (At all
# zeros the folded arms' link2 and link4 hulls overlap.)
_READY = {'lift_joint': .40, 'head_pitch_joint': .30,
          **{f'{side}_arm_joint{i}': value for side in ('left', 'right')
             for i, value in zip((2, 3), (.4, -1.))}}

SPEC = RobotSpec(
    name='x1pro', urdf='asset_sources/robots/x1pro/robot.urdf',
    arms={'right': _arm('right'), 'left': _arm('left')},
    # Wheels on base_link at (0, +-0.24, 0.086), 5 N m each (URDF): at most
    # 2 x 5 / 0.086 = 116 N of push and 116 x 0.24 = 28 N m of turn. Speed
    # limits are simulation values, not the real controller's.
    base=BaseSpec(wheels=('left_wheel_joint', 'right_wheel_joint'), wheel_radius=.086, track=.48,
                  max_speed=(.5, 1.), max_acceleration=(1., 2.), max_force=(116., 28.)),
    lift=('lift_joint',), head=('head_yaw_joint', 'head_pitch_joint'),
    keyframes={'ready': _READY},
    seated=tuple(SEATED),
    unpadded=tuple(f'{side}_arm_gripper_{finger}_link' for side in ('left', 'right') for finger in ('left', 'right')),
    planning_padding=PLANNING_PADDING)


HEAD_CAMERA = 'camera_head_front_color_optical_frame'


def camera_mount(urdf, frame=HEAD_CAMERA):
    """``(parent_link, T_parent_camera)`` of a camera frame fixed to a body,
    with the optical convention (+Z forward, +Y down) turned into USD's
    (-Z forward, +Y up)."""
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation
    joint = next(joint for joint in ET.parse(urdf).getroot().findall('joint')
                 if joint.find('child').get('link') == frame)
    origin = joint.find('origin')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
    transform[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
    return joint.find('parent').get('link'), transform @ np.diag([1., -1., -1., 1.])


def finger_targets(gap):
    gap = float(np.clip(gap, 0., MAX_GAP))
    return {FINGERS[0]: gap / 2, FINGERS[1]: -gap / 2}


def build_planner(source, output, frozen):
    """MPlib planner for the right arm; every other joint fixed at ``frozen``.

    ``source`` is the robot's ``robot.urdf``; the planning URDF, SRDF and mesh
    copies go to ``output``. The planner works in the frame of base_link:
    MPlib 0.2.1 poses attached bodies from link poses without the
    articulation's base pose, so a non-identity base pose would misplace them.
    """
    import mplib
    from table_1000.planning.model import write_planning_urdf, write_srdf
    output = Path(output)
    urdf = write_planning_urdf(source, output / 'robot.urdf', ARM + FINGERS, frozen,
                               (TCP_PARENT, TCP_LINK, TCP_OFFSET, (0., 0., 0.)), PLANNING_PADDING,
                               [f'{side}_arm_gripper_{finger}_link' for side in ('left', 'right') for finger in ('left', 'right')])
    write_srdf(urdf, output / 'robot.srdf', SEATED)
    planner = mplib.Planner(str(urdf), TCP_LINK, srdf=str(output / 'robot.srdf'), use_convex=True,
                            joint_vel_limits=np.full(len(ARM), VELOCITY_LIMIT),
                            joint_acc_limits=np.full(len(ARM), ACCELERATION_LIMIT))
    if planner.user_joint_names != ARM + FINGERS:
        raise RuntimeError(f'unexpected planning joints {planner.user_joint_names}')
    return planner
