"""Controllers: one action per control step -> joint targets per physics step.

Modelled on ManiSkill3's controllers. An action is a dict of arrays, one row per
environment (``N`` rows); a group missing from the action holds its previous
target. Each control step spans ``substeps`` physics steps; position targets
move linearly from the previous to the new target across them, with the matching
velocity as feed-forward, so a 25 Hz command stream reaches the 250 Hz drives
without steps.

Groups of a :class:`RobotSpec`:

========================  ===============================================  ==========
group                     action row                                       modes
========================  ===============================================  ==========
``base``                  ``(v, w)`` forward speed (m/s), yaw rate (rad/s)  velocity
``lift``, ``head``        joint positions                                  pos, delta
``<arm>``                 6 joint positions, or a TCP pose delta           pos, delta, ee_delta
``<arm>_gripper``         pad gap (m)                                      pos
========================  ===============================================  ==========

``ee_delta`` rows are ``(dx, dy, dz, rx, ry, rz)``: a translation and a rotation
vector in the frame of the arm's mount link, applied to the previous TCP target
(kept in that frame, so it moves with the torso lift and the base); the joint
target comes from IK on the environment's :class:`RobotModel`.
"""

import numpy as np
from scipy.spatial.transform import Rotation

CONTROL_HZ = 25
PHYSICS_DT = .004
SUBSTEPS = round(1 / (CONTROL_HZ * PHYSICS_DT))


class JointPosition:
    def __init__(self, joints, limits, delta=False):
        self.joints, self.delta = list(joints), delta
        self.lower, self.upper = np.asarray(limits, dtype=float).T

    @property
    def dim(self):
        return len(self.joints)

    def reset(self, q):
        self.start = self.target = np.array(q, dtype=float)

    def set_action(self, action):
        action = np.asarray(action, dtype=float).reshape(self.target.shape)
        self.start = self.target
        self.target = np.clip(self.target + action if self.delta else action, self.lower, self.upper)

    def hold(self):
        self.start = self.target

    def substep(self, k, substeps, dt):
        """Targets after physics step ``k`` (1..substeps) of this control step."""
        alpha = k / substeps
        velocity = (self.target - self.start) / (substeps * dt)
        return self.start + alpha * (self.target - self.start), velocity


class Gripper(JointPosition):
    """Pad gap -> finger targets of one :class:`ArmSpec`."""

    def __init__(self, arm):
        self.arm = arm
        super().__init__(arm.fingers, [(-1., 1.)] * 2)
        self.signs = np.asarray(arm.finger_signs)

    @property
    def dim(self):
        return 1

    def set_action(self, action):
        gap = np.clip(np.asarray(action, dtype=float).reshape(-1, 1), 0., self.arm.max_gap)
        self.start = self.target
        self.target = gap * self.signs / 2

    def substep(self, k, substeps, dt):
        # The finger drives close on their own; contact stops them at the object.
        return self.target, np.zeros_like(self.target)


class BaseVelocity:
    """``(v, w)`` -> targets of the virtual (x, y, yaw) joints and the wheel speeds.

    The commanded speeds are clipped to ``max_speed`` and approached at no more
    than ``max_acceleration``; the virtual joints follow the integrated pose,
    so the base does not drift while it stands still.
    """

    def __init__(self, base):
        self.base = base
        self.joints = list(base.virtual_joints) + list(base.wheels)
        self.max_speed, self.max_acceleration = np.asarray(base.max_speed), np.asarray(base.max_acceleration)

    @property
    def dim(self):
        return 2

    def reset(self, pose):
        """``pose``: ``[N, 3]`` world (x, y, yaw) of base_link."""
        self.pose = np.array(pose, dtype=float).reshape(-1, 3)
        self.speed = np.zeros((len(self.pose), 2))
        self.command = np.zeros_like(self.speed)
        self.wheels = np.zeros((len(self.pose), len(self.base.wheels)))

    def set_action(self, action):
        self.command = np.clip(np.asarray(action, dtype=float).reshape(-1, 2), -self.max_speed, self.max_speed)

    def hold(self):
        pass

    def substep(self, k, substeps, dt):
        change = np.clip(self.command - self.speed, -self.max_acceleration * dt, self.max_acceleration * dt)
        self.speed = self.speed + change
        v, w = self.speed.T
        yaw = self.pose[:, 2]
        velocity = np.stack([v * np.cos(yaw), v * np.sin(yaw), w], axis=1)
        self.pose = self.pose + velocity * dt
        wheel_speed = np.zeros_like(self.wheels)
        if len(self.base.wheels) == 2:
            # left, right wheel angular speeds of a differential drive
            half = self.base.track / 2
            wheel_speed = np.stack([v - w * half, v + w * half], axis=1) / self.base.wheel_radius
        self.wheels = self.wheels + wheel_speed * dt
        return np.concatenate([self.pose, self.wheels], axis=1), np.concatenate([velocity, wheel_speed], axis=1)


class EndEffectorDelta(JointPosition):
    """TCP pose deltas in the arm's mount frame -> arm joint targets by IK.

    A delta whose IK fails keeps the previous target; :attr:`failed` marks the
    environments where that happened in the last control step.
    """

    def __init__(self, arm, models):
        self.arm, self.models = arm, models
        model = models[0]
        indices = model.arm_indices(arm.name)
        super().__init__(arm.joints, model.limits[indices])

    @property
    def dim(self):
        return 6

    def reset(self, q):
        super().reset(q)
        self.poses = []
        for model, row in zip(self.models, self.target):
            full = model.vector(dict(zip(self.joints, row)))
            self.poses.append(np.linalg.inv(model.fk(self.arm.mount, full)) @ model.tcp(self.arm.name, full))
        self.failed = np.zeros(len(self.target), dtype=bool)

    def set_action(self, action):
        action = np.asarray(action, dtype=float).reshape(-1, 6)
        self.start = self.target
        target = self.target.copy()
        for i, (model, row) in enumerate(zip(self.models, action)):
            goal = self.poses[i].copy()
            goal[:3, 3] += row[:3]
            goal[:3, :3] = Rotation.from_rotvec(row[3:]).as_matrix() @ goal[:3, :3]
            q = model.vector(dict(zip(self.joints, self.target[i])))
            status, solution = model.ik(self.arm.name, model.fk(self.arm.mount, q) @ goal, q, attempts=1,
                                        check_collision=True)
            self.failed[i] = status != 'Success'
            if not self.failed[i]:
                target[i] = solution[model.arm_indices(self.arm.name)]
                self.poses[i] = goal
        self.target = target


class RobotController:
    """Every group of a robot; ``modes`` maps arm, lift and head groups to a mode.

    :attr:`joints` lists the joints it drives, in the column order of
    :meth:`substep`'s output.
    """

    def __init__(self, spec, models, modes=None, physics_dt=PHYSICS_DT, substeps=SUBSTEPS):
        self.spec, self.models, self.dt, self.substeps = spec, models, physics_dt, substeps
        modes = {'lift': 'pos', 'head': 'pos', **{arm: 'pos' for arm in spec.arms}, **(modes or {})}
        limits = models[0].limits
        index = models[0].index

        def joint(names, mode):
            return JointPosition(names, limits[[index[n] for n in names]], delta=mode == 'delta')

        self.groups = {'base': BaseVelocity(spec.base)}
        if spec.lift:
            self.groups['lift'] = joint(spec.lift, modes['lift'])
        if spec.head:
            self.groups['head'] = joint(spec.head, modes['head'])
        for name, arm in spec.arms.items():
            self.groups[name] = (EndEffectorDelta(arm, models) if modes[name] == 'ee_delta'
                                 else joint(arm.joints, modes[name]))
            self.groups[f'{name}_gripper'] = Gripper(arm)
        self.modes = modes
        self.joints = [name for group in self.groups.values() for name in group.joints]

    def action_space(self):
        """``{group: dim}`` of the action dict."""
        return {name: group.dim for name, group in self.groups.items()}

    def reset(self, q, base_pose):
        """``q``: ``{joint: [N]}`` measured body joints; ``base_pose``: ``[N, 3]``."""
        for name, group in self.groups.items():
            if name == 'base':
                group.reset(base_pose)
            else:
                group.reset(np.stack([np.asarray(q[j], dtype=float).reshape(-1) for j in group.joints], axis=1))

    def set_action(self, action):
        unknown = set(action) - set(self.groups)
        if unknown:
            raise KeyError(f'unknown action groups {sorted(unknown)}')
        for name, group in self.groups.items():
            if name in action:
                group.set_action(action[name])
            else:
                group.hold()

    def substep(self, k):
        """``(positions, velocities)``, each ``[N, len(joints)]``, for physics step ``k`` (1..substeps)."""
        parts = [group.substep(k, self.substeps, self.dt) for group in self.groups.values()]
        return np.concatenate([p for p, _ in parts], axis=1), np.concatenate([v for _, v in parts], axis=1)
