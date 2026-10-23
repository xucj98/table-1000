"""Simulator-independent description of a robot with arms and two-finger grippers.

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

from dataclasses import dataclass, field, replace

import numpy as np


@dataclass(frozen=True)
class ArmSpec:
    """One arm and its two-finger gripper.

    ``tcp`` is ``(parent_link, xyz)``: a frame fixed to ``parent_link`` with the
    parent's orientation. ``mount`` is the link the arm is mounted on; TCP
    targets of the pose controller are kept in its frame, so they move with
    the torso and base.

    The gripper is commanded by the pad gap ``g`` (0 to ``max_gap``), the two
    fingers moving symmetrically: finger targets are ``sign * s`` with
    ``finger_signs``. Sliding fingers are driven in metres, ``s = g / 2``.
    Pivoting fingers (``finger_pivot``) are driven in radians: ``finger_pivot``
    is ``(offset, (x, y))``, the distance of the first finger's pivot from the
    centre line and its pad point in the finger frame at angle 0 (``y`` towards
    the opening side), so the pad lies ``offset + x sin s + y cos s`` from the
    centre line and ``g`` is twice that.
    """
    name: str
    joints: tuple
    tcp_link: str
    tcp: tuple
    fingers: tuple
    finger_links: tuple
    finger_signs: tuple = (1., -1.)
    max_gap: float = .1
    finger_pivot: tuple = None
    mount: str = 'base_link'       # set it to the arm's mount link (the root link for a fixed arm)
    # Joint-space limits for time parameterization (rad/s, rad/s^2).
    velocity_limit: float = .8
    acceleration_limit: float = 1.5

    def finger_targets(self, gap):
        half = float(np.clip(gap, 0., self.max_gap)) / 2
        if self.finger_pivot is not None:
            offset, (x, y) = self.finger_pivot
            half = np.arctan2(-y, x) + np.arcsin((half - offset) / np.hypot(x, y))
        return {joint: sign * float(half) for joint, sign in zip(self.fingers, self.finger_signs)}

    def gap(self, q):
        """Pad gap of finger positions ``q`` (their symmetric part)."""
        s = float(np.dot(q, self.finger_signs)) / 2
        if self.finger_pivot is not None:
            offset, (x, y) = self.finger_pivot
            s = offset + x * np.sin(s) + y * np.cos(s)
        return 2 * s


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
class Sensor:
    """What one image stream of a camera delivers, as the real camera reports it.

    ``intrinsics`` are ``(fx, fy, cx, cy)`` in pixels for ``resolution``
    ``(width, height)``. ``model`` is OpenCV's ``'pinhole'`` (``distortion``
    k1, k2, p1, p2[, k3]) or ``'fisheye'`` (k1 to k4);
    :func:`table_1000.robots.cameras.project` maps points to pixels with it.
    ``kind`` is ``'rgb'`` (uint8 images) or ``'depth'`` (metres along +Z, 0
    where nothing is hit). The simulator renders the stream with the same lens
    model, clipping closer than ``near`` (the camera's own window).
    """
    resolution: tuple
    intrinsics: tuple
    model: str = 'pinhole'
    distortion: tuple = ()
    kind: str = 'rgb'
    near: float = .01

    def resized(self, width, height):
        """The same lens at another resolution: intrinsics scale with the image."""
        sx, sy = width / self.resolution[0], height / self.resolution[1]
        fx, fy, cx, cy = self.intrinsics
        return replace(self, resolution=(width, height), intrinsics=(fx * sx, fy * sy, cx * sx, cy * sy))


@dataclass(frozen=True)
class CameraSpec:
    """A :class:`Sensor` mounted on the robot.

    ``mount`` is the stream's optical frame (OpenCV convention: +Z forward, +X
    right and +Y down in the image): a fixed frame of the URDF by name, or
    ``(link, xyz, rpy)`` placing it on a link without editing the URDF.
    ``self_view`` bounds the share of pixels showing the robot itself in the
    acceptance poses.
    """
    sensor: Sensor
    mount: object
    self_view: float = .1


def override_cameras(cameras, config):
    """Cameras of a robot changed by ``config``, as ManiSkill's ``sensor_configs``.

    Keys naming a camera apply to that camera only (``None`` drops it); other
    keys apply to every camera. Values set :class:`Sensor` or
    :class:`CameraSpec` fields; ``resize=(width, height)`` scales the intrinsics
    with the image (:meth:`Sensor.resized`).
    """
    def apply(camera, changes):
        changes = dict(changes)
        sensor = camera.sensor
        if 'resize' in changes:
            sensor = sensor.resized(*changes.pop('resize'))
        sensor_fields = {key: changes.pop(key) for key in list(changes) if key in Sensor.__dataclass_fields__}
        unknown = set(changes) - set(CameraSpec.__dataclass_fields__)
        if unknown:
            raise KeyError(f'unknown camera settings {sorted(unknown)}')
        return replace(camera, sensor=replace(sensor, **sensor_fields), **changes)

    common = {key: value for key, value in config.items() if key not in cameras}
    result = {}
    for name, camera in cameras.items():
        own = config.get(name, {})
        if own is None:
            continue
        result[name] = apply(apply(camera, common), own)
    return result


@dataclass(frozen=True)
class RobotSpec:
    name: str
    urdf: str                      # robot source URDF, relative to the repository root
    arms: dict                     # name -> ArmSpec; the first is the primary arm
    base: BaseSpec = None          # mobile base, or None for a fixed robot
    lift: tuple = ()               # prismatic torso joints
    # Torso lift limits for time parameterization (m/s, m/s^2).
    lift_velocity_limit: float = .1
    lift_acceleration_limit: float = .2
    head: tuple = ()               # head joints (yaw, pitch)
    keyframes: dict = field(default_factory=dict)   # name -> {joint: value}; 'zero' is implied
    cameras: dict = field(default_factory=dict)     # name -> CameraSpec (see override_cameras)
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
