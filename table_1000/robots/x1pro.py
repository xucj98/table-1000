"""X1Pro right-arm conventions and its MPlib planner.

TCP: a frame on ``right_arm_link6`` 0.145 m along the gripper axis, centred
between the finger pads; +X is the approach direction, +Y the direction in
which the left finger opens, +Z points to the wrist camera. The fingertips are
0.0185 m beyond the TCP.

Fingers are driven in metres: the left finger at ``+gap/2`` and the right at
``-gap/2`` gives a pad gap of ``gap`` (0 to 0.0975 m).
"""

from pathlib import Path

import numpy as np

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
