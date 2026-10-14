"""Batched task environment: ``reset(layout)`` and ``step(action)`` at 25 Hz.

Simulator-independent: it drives a backend (``table_1000.sim.isaac``) through
the robot's :class:`~table_1000.robots.controllers.RobotController`. One
``step`` applies an action dict (see the controllers module) and advances
``SUBSTEPS`` physics steps; every array has a leading environment axis.

Observation ``obs``:

- ``qpos``, ``qvel``: ``{joint: [N]}`` for every robot joint, base joints included
- ``base_pose``: ``[N, 3]`` world (x, y, yaw) of the robot's root link (fixed
  at the scene's placement for a robot without a mobile base)
- ``tcp``: ``{arm: [N, 4, 4]}`` world TCP poses
- ``time``: simulated seconds since the reset
"""

import numpy as np

from table_1000.robots.controllers import SUBSTEPS, RobotController


def base_transform(pose, z=0.):
    x, y, yaw = pose
    transform = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    transform[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    transform[:3, 3] = [x, y, z]
    return transform


def scene_pose(scene):
    """``(x, y, yaw)`` and height of the robot in ``scene``."""
    position = scene.ROBOT['position']
    return np.array([position[0], position[1], scene.ROBOT['yaw']]), float(position[2])


class Env:
    def __init__(self, scene, backend, spec, models, modes=None):
        """``models``: one :class:`RobotModel` per environment."""
        self.scene, self.backend, self.spec, self.models = scene, backend, spec, models
        self.num_envs = backend.num_envs
        self.controller = RobotController(spec, models, modes, physics_dt=backend.physics_dt)
        self.observers = []        # called as observer(env) after every control step
        self.caption = lambda env: f't={env.time:5.2f}s'
        self.capture = backend.cameras is not None
        self.steps = 0

    @property
    def time(self):
        return self.backend.time

    def initial_robot_q(self):
        """Scene joints (others at zero) and, for a mobile base, its pose as virtual joint values."""
        robot = self.scene.ROBOT
        q = dict(zip(self.spec.base.virtual_joints, scene_pose(self.scene)[0])) if self.spec.base else {}
        q.update(robot['joints'])
        return q

    def reset(self, layout, robot_q=None):
        robot_q = robot_q or self.initial_robot_q()
        self.backend.reset(layout, robot_q)
        self.layout = layout
        self.steps = 0
        obs = self.observe()
        self.controller.reset(obs['qpos'], obs['base_pose'])
        return obs

    def observe(self):
        q, dq = self.backend.joint_state()
        joints = self.backend.robot_joints
        qpos = {name: q[:, i] for i, name in enumerate(joints)}
        qvel = {name: dq[:, i] for i, name in enumerate(joints)}
        if self.spec.base:
            base = np.stack([qpos[name] for name in self.spec.base.virtual_joints], axis=1)
            height = 0.
        else:
            pose, height = scene_pose(self.scene)
            base = np.repeat(pose[None], self.num_envs, axis=0)
        tcp = {arm: [] for arm in self.spec.arms}
        for i, model in enumerate(self.models):
            model.set_state({name: values[i] for name, values in qpos.items()}, base_transform(base[i], height))
            for arm in self.spec.arms:
                tcp[arm].append(model.tcp(arm, world=True))
        self.obs = {'qpos': qpos, 'qvel': qvel, 'base_pose': base,
                    'tcp': {arm: np.stack(poses) for arm, poses in tcp.items()}, 'time': self.time}
        return self.obs

    def step(self, action):
        """Apply ``action`` for one control step; returns ``(obs, info)``."""
        self.controller.measure(self.obs['base_pose'])
        self.controller.set_action(action)
        for k in range(1, SUBSTEPS + 1):
            positions, velocities = self.controller.substep(k)
            self.backend.set_targets(self.controller.joints, positions, velocities)
            self.backend.step()
        self.steps += 1
        obs = self.observe()
        info = {'ik_failed': {name: group.failed.copy() for name, group in self.controller.groups.items()
                              if hasattr(group, 'failed')}}
        for observer in self.observers:
            observer(self)
        if self.capture:
            self.backend.capture(self.caption(self))
        return obs, info

    def targets(self, joints):
        """``[N, len(joints)]`` current position targets of the named robot joints."""
        index = {name: i for i, name in enumerate(self.backend.robot_joints)}
        return self.backend.targets[:, [index[name] for name in joints]]


class EnvView:
    """One environment of a batched :class:`Env`, with unbatched accessors (for experts)."""

    def __init__(self, env, index=0):
        self.env, self.index = env, index
        self.scene, self.backend, self.model = env.scene, env.backend, env.models[index]

    @property
    def time(self):
        return self.env.time

    @property
    def contacts(self):
        return self.backend.contacts

    def qpos(self, joints):
        return np.array([self.env.obs['qpos'][name][self.index] for name in joints])

    def targets(self, joints):
        return self.env.targets(joints)[self.index]

    def body_poses(self):
        return {key: value[self.index] for key, value in self.backend.body_poses().items()}

    def center_of_mass(self, prefix):
        return self.backend.center_of_mass(prefix)[self.index]

    def object_joint(self, name, joint):
        return float(self.backend.object_joints(name)[joint][self.index])

    def link_pose(self, link):
        return self.backend.link_pose(link)[self.index]
