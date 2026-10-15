"""Simulator-independent description of a robot with arms and parallel grippers.

A robot's ``asset_sources/robots/<name>/robot.py`` fills a :class:`RobotSpec`:
its arms (joints, TCP, two-finger gripper), and optionally a torso lift, a
head and a mobile base, plus named keyframes and cameras. Kinematics, planning,
controllers and camera images are built from the spec, so they work the same for every
simulator backend and every such robot: a fixed single arm (a Franka on a
stand) as well as a mobile dual-arm robot. Legged robots are not covered.

Frames: the root link of the URDF is the robot's base frame. A mobile base
moves it on the floor at the planar pose ``(x, y, yaw)``; a fixed robot stays
where its scene places it. Arm targets are given in the world frame and
converted with that pose.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ArmSpec:
    """One arm and its parallel gripper.

    ``tcp`` is ``(parent_link, xyz)``: a frame fixed to ``parent_link`` with the
    parent's orientation. ``mount`` is the link the arm is mounted on; TCP
    targets of the pose controller are kept in its frame, so they move with
    the torso and base. Fingers are driven in metres; ``finger_signs`` maps a
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
    mount: str = 'base_link'       # set it to the arm's mount link (the root link for a fixed arm)
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
    changed at most by ``max_acceleration`` (m/s^2, rad/s^2). The virtual drives
    push with at most ``max_force`` (N, N m for yaw), what the wheel motors can
    deliver, so a blocked base stops instead of shoving the obstacle; its target
    stays within ``max_lag`` (m, rad) of the measured pose.
    """
    virtual_joints: tuple = ('base_x_joint', 'base_y_joint', 'base_yaw_joint')
    wheels: tuple = ()
    wheel_radius: float = .1
    track: float = .5
    max_speed: tuple = (.5, 1.)
    max_acceleration: tuple = (1., 2.)
    max_force: tuple = (200., 50.)
    max_lag: tuple = (.05, .1)


@dataclass(frozen=True)
class CameraSpec:
    """One image stream of a camera fixed to a robot body, as the real camera reports it.

    ``frame`` is the stream's optical frame in the URDF (OpenCV convention: +Z
    forward, +X right and +Y down in the image). ``intrinsics`` are
    ``(fx, fy, cx, cy)`` in pixels for ``resolution`` ``(width, height)``.
    ``model`` is OpenCV's ``'pinhole'`` (``distortion`` k1, k2, p1, p2[, k3]) or
    ``'fisheye'`` (k1 to k4); :func:`table_1000.robots.cameras.project` maps
    points to pixels with it. ``kind`` is ``'rgb'`` (uint8 images) or ``'depth'``
    (metres along +Z, 0 where nothing is hit). The simulator renders the stream
    with the same lens model, clipping closer than ``near`` (the camera's own
    window). ``self_view`` bounds the share of pixels showing the robot itself
    in the acceptance poses.
    """
    frame: str
    resolution: tuple
    intrinsics: tuple
    model: str = 'pinhole'
    distortion: tuple = ()
    kind: str = 'rgb'
    fps: float = 30.
    near: float = .01
    self_view: float = .1


@dataclass(frozen=True)
class RobotSpec:
    name: str
    urdf: str                      # robot source URDF, relative to the repository root
    arms: dict                     # name -> ArmSpec; the first is the primary arm
    base: BaseSpec = None          # mobile base, or None for a fixed robot
    lift: tuple = ()               # prismatic torso joints
    head: tuple = ()               # head joints (yaw, pitch)
    keyframes: dict = field(default_factory=dict)   # name -> {joint: value}; 'zero' is implied
    cameras: dict = field(default_factory=dict)     # name -> CameraSpec
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

    @property
    def primary_arm(self):
        return next(iter(self.arms))

    def keyframe(self, name):
        """Every body joint at keyframe ``name`` (unlisted joints at zero; ``'zero'`` is all zeros)."""
        q = {joint: 0. for joint in self.body_joints}
        q.update({} if name == 'zero' else self.keyframes[name])
        return q

    @property
    def keyframe_names(self):
        return ['zero', *[name for name in self.keyframes if name != 'zero']]
