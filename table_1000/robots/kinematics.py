"""Whole-body kinematics and self-collision planning from a :class:`RobotSpec`.

One planning model (MPlib, Pinocchio and FCL) holds every body joint, so a
plan for one arm is checked against the torso, head and the other arm at their
current positions. Each arm gets its own MPlib planner, whose move group is the
chain from the root link to the arm's TCP; a torso lift on that chain is held
fixed, so only the arm's own joints move, unless a call frees it (``lift=True``):
then IK and RRTConnect move the lift together with the arm.

Poses here are in the frame of the URDF's root link; :attr:`RobotModel.base`
(world from root: a mobile base's planar pose, or a fixed robot's placement)
converts world-frame targets. MPlib 0.2.1 places attached bodies
from link poses without the articulation's base pose, so the planner itself
always works in the base frame.

The planning world contains only the robot unless a caller adds objects to an
arm's planner (see :class:`table_1000.planning.world.PlanningScene`).
"""

from pathlib import Path

import time

import numpy as np

from table_1000.robots.planning_model import write_planning_urdf, write_srdf
from table_1000.planning.screw import plan_screw
from table_1000.planning.world import pose

ROOT = Path(__file__).resolve().parents[2]


def matrix(mplib_pose):
    return mplib_pose.to_transformation_matrix()


def pose_error(a, b):
    """Position (m) and rotation (rad) between two 4x4 transforms."""
    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), float(rotation)


class RobotModel:
    """Planning model of one robot; ``output`` receives the planning URDF and SRDF."""

    def __init__(self, spec, output, root=ROOT):
        import mplib
        self.spec, self.root = spec, Path(root)
        output = Path(output)
        tcps = [(arm.tcp[0], arm.tcp_link, arm.tcp[1], (0., 0., 0.)) for arm in spec.arms.values()]
        urdf = write_planning_urdf(Path(root) / spec.urdf, output / 'robot.urdf', spec.body_joints, {}, tcps,
                                   spec.planning_padding, spec.unpadded)
        srdf = write_srdf(urdf, output / 'robot.srdf', spec.seated)
        self.planners = {}
        for name, arm in spec.arms.items():
            chain = len(spec.lift) + len(arm.joints)
            lift = len(spec.lift)
            self.planners[name] = mplib.Planner(
                str(urdf), arm.tcp_link, srdf=str(output / 'robot.srdf'), use_convex=True,
                joint_vel_limits=np.r_[np.full(lift, spec.lift_velocity_limit), np.full(chain - lift, arm.velocity_limit)],
                joint_acc_limits=np.r_[np.full(lift, spec.lift_acceleration_limit),
                                       np.full(chain - lift, arm.acceleration_limit)])
        first = next(iter(self.planners.values()))
        self.joints = list(first.user_joint_names)
        self.index = {name: i for i, name in enumerate(self.joints)}
        self.limits = np.asarray(first.joint_limits)
        for name, planner in self.planners.items():
            group = [self.joints[i] for i in planner.move_group_joint_indices]
            if group != [*spec.lift, *spec.arms[name].joints]:
                raise RuntimeError(f'unexpected move group for {name}: {group}')
        self.q = np.zeros(len(self.joints))
        self.base = np.eye(4)

    # -- configuration ---------------------------------------------------------
    def vector(self, values, q=None):
        """Full configuration from ``{joint: value}``, other joints from ``q`` (default: current)."""
        q = np.array(self.q if q is None else q, dtype=float)
        for name, value in values.items():
            if name in self.index:
                q[self.index[name]] = value
        return q

    def values(self, q=None, joints=None):
        q = self.q if q is None else q
        return {name: float(q[self.index[name]]) for name in (joints or self.joints)}

    def set_state(self, values=None, base=None):
        """Set measured joint values (any subset) and the world pose of the root link."""
        if values is not None:
            self.q = self.vector(values)
        if base is not None:
            self.base = np.asarray(base, dtype=float)
        for planner in self.planners.values():
            planner.robot.set_qpos(self.q, True)

    def arm_indices(self, arm):
        return [self.index[name] for name in self.spec.arms[arm].joints]

    def lift_indices(self):
        return [self.index[name] for name in self.spec.lift]

    # -- kinematics --------------------------------------------------------------
    def fk(self, link, q=None, world=False):
        """Pose of ``link`` in the base frame (or the world with ``world=True``)."""
        planner = next(iter(self.planners.values()))
        planner.pinocchio_model.compute_forward_kinematics(self.q if q is None else q)
        local = matrix(planner.pinocchio_model.get_link_pose(planner.link_name_2_idx[link]))
        return self.base @ local if world else local

    def tcp(self, arm, q=None, world=False):
        return self.fk(self.spec.arms[arm].tcp_link, q, world)

    def ik(self, arm, target, q=None, *, world=False, attempts=20, closest=True, check_collision=True, lift=False):
        """Joint configuration placing ``arm``'s TCP at ``target``; ``(status, q)``.

        Only the arm's joints change (and the torso lift with ``lift=True``).
        Solutions that collide in the planning world are rejected unless
        ``check_collision`` is False.
        """
        planner = self.planners[arm]
        target = np.linalg.inv(self.base) @ target if world else np.asarray(target)
        start = np.clip(self.q if q is None else q, self.limits[:, 0], self.limits[:, 1])
        mask = np.ones(len(self.joints), dtype=bool)
        mask[self.arm_indices(arm) + (self.lift_indices() if lift else [])] = False
        if check_collision:
            status, result = planner.IK(pose(target), start, mask, n_init_qpos=attempts, return_closest=closest)
            return status, result
        planner.robot.set_qpos(start, True)
        q_goal, success, _ = planner.pinocchio_model.compute_IK_CLIK(planner.link_name_2_idx[planner.move_group],
                                                                    pose(target), start, mask)
        planner.robot.set_qpos(self.q, True)
        return ('Success' if success else 'IK Failed'), (np.asarray(q_goal) if success else None)

    def self_collisions(self, q=None):
        """Robot body pairs in collision at ``q`` (default: current)."""
        planner = next(iter(self.planners.values()))
        planner.robot.set_qpos(self.q if q is None else q, True)
        pairs = sorted({tuple(sorted((c.link_name1, c.link_name2))) for c in planner.check_for_self_collision()})
        planner.robot.set_qpos(self.q, True)
        return pairs

    def look_at(self, point, camera, q=None, world=True):
        """Head joint values that point ``camera``'s optical +Z at ``point``."""
        if not self.spec.head:
            raise ValueError(f'{self.spec.name} has no head to point')
        from scipy.optimize import least_squares
        point = np.asarray(point, dtype=float)
        if world:
            point = (np.linalg.inv(self.base) @ np.r_[point, 1.])[:3]
        q0 = np.array(self.q if q is None else q, dtype=float)
        head = [self.index[name] for name in self.spec.head]

        def residual(angles):
            q = q0.copy()
            q[head] = angles
            frame = self.fk(camera, q)
            direction = point - frame[:3, 3]
            return frame[:3, 2] - direction / np.linalg.norm(direction)

        bounds = self.limits[head].T
        solution = least_squares(residual, np.clip(q0[head], *bounds), bounds=bounds).x
        return dict(zip(self.spec.head, map(float, solution)))

    # -- planning ------------------------------------------------------------------
    def _arm_path(self, arm, result, lift=False):
        """Keep the arm joints of an MPlib result (the lift is held fixed); with ``lift``, the lift
        joints' rows go to ``result['lift']``."""
        if result['status'] != 'Success':
            return result
        count = len(self.spec.lift)
        if lift:
            result['lift'] = np.asarray(result['position'])[:, :count]
        for key in ('position', 'velocity', 'acceleration'):
            result[key] = np.asarray(result[key])[:, count:]
        result['joints'] = list(self.spec.arms[arm].joints)
        return result

    def plan_rrt(self, arm, target, q=None, *, world=False, time_step=.04, planning_time=3., goals=None, lift=False):
        """RRTConnect to an IK solution of ``target`` (or to ``goals``, full configurations). The
        torso lift is held unless ``lift`` frees it; its path is then ``result['lift']``."""
        planner = self.planners[arm]
        start = np.clip(self.q if q is None else q, self.limits[:, 0], self.limits[:, 1])
        started = time.perf_counter()
        if goals is None:
            status, goals = self.ik(arm, target, start, world=world, closest=False, lift=lift)
            if status != 'Success':
                return {'status': status, 'reason': status, 'ik_s': time.perf_counter() - started}
        ik_s = time.perf_counter() - started
        # The torso lift is the first joint of the move group.
        result = planner.plan_qpos(list(goals), start, time_step=time_step, planning_time=planning_time,
                                   fixed_joint_indices=[] if lift else list(range(len(self.spec.lift))))
        planner.robot.set_qpos(self.q, True)
        if result['status'] != 'Success':
            result['reason'] = result['status']
        result.update(ik_s=ik_s, rrt_s=time.perf_counter() - started - ik_s)
        return self._arm_path(arm, result, lift)

    def check_path(self, arm, positions, step=.01, lift=None):
        """First collision along a path of ``arm`` joint rows (with ``lift`` rows of the torso
        lift, else the lift where it is), interpolated so that no joint moves more than ``step``
        rad (or m) between checks.

        Returns ``(checks, pairs)``; ``pairs`` is empty when the path is free.
        Contacts present at the start (a hand just released next to an object)
        are tolerated until the path first leaves them.
        """
        world = self.planners[arm].planning_world
        positions = np.asarray(positions, dtype=float)
        if lift is None:
            lift = np.repeat(self.q[self.lift_indices()][None], len(positions), axis=0)
        positions = np.c_[np.asarray(lift, dtype=float), positions]

        def colliding(row):
            world.set_qpos_all(row)
            return {(c.link_name1, c.object_name2, c.link_name2) for c in world.check_collision()}

        rows = [positions[0]]
        for start, end in zip(positions[:-1], positions[1:]):
            count = max(1, int(np.ceil(np.abs(end - start).max() / step)))
            rows += [start + (end - start) * k / count for k in range(1, count + 1)]
        checks, leaving = 0, None
        try:
            for row in rows:
                checks += 1
                pairs = colliding(row)
                if leaving is None:
                    leaving = pairs
                if not pairs:
                    leaving = set()
                elif not (leaving and pairs <= leaving):
                    return checks, sorted(pairs)
            return checks, []
        finally:
            self.planners[arm].robot.set_qpos(self.q, True)

    def plan_screw(self, arm, target, q=None, *, world=False, time_step=.04):
        """Straight screw motion of the TCP from its pose at ``q`` to ``target``."""
        planner = self.planners[arm]
        target = np.linalg.inv(self.base) @ target if world else np.asarray(target)
        start = self.q if q is None else q
        locked = [i for i in range(len(self.joints)) if i not in self.arm_indices(arm)]
        result = plan_screw(planner, pose(target), np.array(start, dtype=float), time_step=time_step,
                            wrt_world=False, locked=locked)
        planner.robot.set_qpos(self.q, True)
        return self._arm_path(arm, result)
