"""Simulator-independent description of a mobile manipulator.

A robot module (for example :mod:`table_1000.robots.x1pro`) fills a
:class:`RobotSpec`: its joint groups, end effectors, mobile base and named
keyframes. Kinematics, planning and controllers are built from the spec, so
they work the same for every simulator backend.

Frames: the robot's ``base_link`` sits on the floor at the planar base pose
``(x, y, yaw)`` in the world. Arm targets are given in the world frame and
converted with that pose.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ArmSpec:
    """One arm and its parallel gripper.

    ``tcp`` is ``(parent_link, xyz)``: a frame fixed to ``parent_link`` with the
    parent's orientation. Fingers are driven in metres; ``finger_signs`` maps a
    pad gap ``g`` to finger targets ``sign * g / 2``.
    """
    name: str
    joints: tuple
    tcp_link: str
    tcp: tuple
    fingers: tuple
    finger_links: tuple
    finger_signs: tuple = (1., -1.)
    max_gap: float = .1
    # Joint-space limits for time parameterization (rad/s, rad/s^2).
    velocity_limit: float = .8
    acceleration_limit: float = 1.5

    def finger_targets(self, gap):
        gap = float(np.clip(gap, 0., self.max_gap))
        return {joint: sign * gap / 2 for joint, sign in zip(self.fingers, self.finger_signs)}


@dataclass(frozen=True)
class BaseSpec:
    """Differential-drive base moved by three virtual planar joints.

    The simulator drives ``virtual_joints`` (world x, world y, yaw); the wheels
    only turn to match. Speeds are limited to ``max_speed`` (m/s, rad/s) and
    changed at most by ``max_acceleration`` (m/s^2, rad/s^2).
    """
    virtual_joints: tuple = ('base_x_joint', 'base_y_joint', 'base_yaw_joint')
    wheels: tuple = ()
    wheel_radius: float = .1
    track: float = .5
    max_speed: tuple = (.5, 1.)
    max_acceleration: tuple = (1., 2.)


@dataclass(frozen=True)
class RobotSpec:
    name: str
    urdf: str                      # robot source URDF, relative to the repository root
    arms: dict                     # name -> ArmSpec
    base: BaseSpec
    lift: tuple = ()               # prismatic torso joints
    head: tuple = ()               # head joints (yaw, pitch)
    keyframes: dict = field(default_factory=dict)   # name -> {joint: value}
    # Body pairs whose collision hulls overlap at every configuration; never checked.
    seated: tuple = ()
    # Links whose planning hulls are not grown (they touch objects on purpose).
    unpadded: tuple = ()
    planning_padding: float = .002

    @property
    def body_joints(self):
        """Every driven joint of the articulation, base virtual joints excluded."""
        joints = [*self.lift, *self.head]
        for arm in self.arms.values():
            joints += [*arm.joints, *arm.fingers]
        return joints

    def keyframe(self, name):
        q = {joint: 0. for joint in self.body_joints}
        q.update(self.keyframes[name])
        return q
