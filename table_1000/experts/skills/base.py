"""The base: driving it, and choosing where it stands so that poses are reached; the free arms back home."""

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.experts.context import steps
from table_1000.experts.skills.common import IK_ATTEMPTS
from table_1000.experts.solver import PlanningFailed

# The base drives at most BASE_SPEED (m/s) and turns at most BASE_TURN (rad/s), proportionally to the
# error (BASE_GAIN) until within BASE_TOLERANCE (m, rad), then settles for BASE_SETTLE s. make_room
# tries it BASE_BACKS (m) behind its start and turned by BASE_TURNS (deg), for every joint
# BASE_MARGIN (rad) inside its limits.
BASE_SPEED, BASE_TURN, BASE_GAIN = .15, .4, 2.
BASE_TOLERANCE, BASE_SETTLE = (.005, np.radians(1.)), .5
BASE_BACKS = (0., .05, .10, .15, .20)
BASE_TURNS = (0., 10., -10., 20., -20.)
BASE_MARGIN = .25


class Base:
    """The base skills of :class:`~table_1000.experts.skills.Skills` (and ``ready``)."""

    # -- the base --------------------------------------------------------------------------
    def base_pose(self):
        return np.asarray(self.ctx.env.obs['agent']['base_pose'][0], dtype=float)

    def move_base(self, forward=0., turn=0.):
        """Drive the base straight by ``forward`` (m along its heading, back when negative), then turn
        it in place by ``turn`` (rad); the arms keep their joints."""
        ctx = self.ctx
        x, y, yaw = self.base_pose()
        goal, heading = np.array([x, y]) + forward * np.array([np.cos(yaw), np.sin(yaw)]), yaw + turn

        def wrapped(angle):
            return float((angle + np.pi) % (2 * np.pi) - np.pi)

        limit = steps(abs(forward) / BASE_SPEED + abs(turn) / BASE_TURN + BASE_SETTLE)
        for _ in range(limit):
            x, y, now = self.base_pose()
            along = float((goal - [x, y]) @ [np.cos(now), np.sin(now)])
            if abs(along) < BASE_TOLERANCE[0]:
                break
            ctx.step({'base': [[np.clip(BASE_GAIN * along, -BASE_SPEED, BASE_SPEED),
                                np.clip(BASE_GAIN * wrapped(yaw - now), -BASE_TURN, BASE_TURN)]]})
        for _ in range(limit):
            error = wrapped(heading - self.base_pose()[2])
            if abs(error) < BASE_TOLERANCE[1]:
                break
            ctx.step({'base': [[0., np.clip(BASE_GAIN * error, -BASE_TURN, BASE_TURN)]]})
        ctx.step({'base': [[0., 0.]]})
        ctx.hold(BASE_SETTLE)
        for solver in self.solvers.values():
            solver.sync()
        ctx.note('base moved', forward=forward, turn=turn, pose=self.base_pose().tolist())

    def base_for(self, poses, arms=None, need=None, path=False):
        """``(back, turn)`` from the episode's start pose of the base (backed off along its start
        heading, then turned) from where one of ``arms`` (default the free ones) reaches ``need``
        (default all) of ``poses`` with each joint ``BASE_MARGIN`` inside its limits; the least move
        that does, else the one reaching most, with the widest margin. Each of ``poses`` is a world
        TCP pose or a list of them along a straight move, each solved from the one before at the
        torso height of the first (``path``: ``poses`` are one such list)."""
        model = self.model
        arms = list(arms) if arms else [a for a in self.arms if a not in self.held]
        groups = [list(poses)] if path else [[pose] if np.ndim(pose) == 2 else list(pose) for pose in poses]
        need = len(groups) if need is None else need
        actual = model.base.copy()
        best, most = None, (-1, -np.inf)
        try:
            for back, turn in sorted(((b, t) for b in BASE_BACKS for t in np.radians(BASE_TURNS)),
                                     key=lambda option: option[0] / .05 + abs(option[1]) / np.radians(10.)):
                model.base = self.base_at(back, turn)
                for arm in arms:
                    self.solvers[arm].sync()
                    margins = [self.path_margin(arm, group) for group in groups]
                    reached = sum(margin >= BASE_MARGIN for margin in margins)
                    if reached >= need:
                        return back, turn
                    score = (reached, max(margins, default=np.inf))
                    if score > most:
                        best, most = (back, turn), score
        finally:
            model.base = actual
            for solver in self.solvers.values():
                solver.sync()
        return best

    def path_margin(self, arm, poses):
        """Least distance (rad) of ``arm``'s joints to their limits through ``poses`` (world TCP), each
        solved from the one before (one branch) at the torso height of the first, as a straight move
        does; -inf where one has no IK."""
        model, indices = self.model, self.model.arm_indices(arm)
        q, margin = None, np.inf
        for index, pose in enumerate(poses):
            status, q = model.ik(arm, pose, q, world=True, lift=index == 0, attempts=IK_ATTEMPTS)
            if status != 'Success':
                return -np.inf
            margin = min(margin, float(np.min(np.minimum(q[indices] - model.limits[indices, 0],
                                                         model.limits[indices, 1] - q[indices]))))
        return margin

    def base_at(self, back, turn):
        """World pose of the root link with the base ``back`` behind its start pose, turned by ``turn``."""
        x, y, yaw = self.start_base
        transform = self.model.base.copy()
        transform[:3, :3] = Rotation.from_euler('z', yaw + turn).as_matrix()
        transform[:2, 3] = np.array([x, y]) - back * np.array([np.cos(yaw), np.sin(yaw)])
        return transform

    def make_room(self, poses, arms=None, need=None, path=False):
        """Move the base (see :meth:`base_for`) to where ``poses`` are reachable with a margin:
        back to the start heading, along it, then turned."""
        if not self.ctx.spec.base:
            return
        chosen = self.base_for(poses, arms, need, path)
        self.ctx.note('room', chosen=None if chosen is None else [float(v) for v in chosen])
        if chosen is None:
            return
        back, turn = chosen
        x, y, yaw = self.base_pose()
        x0, y0, yaw0 = self.start_base
        now_back = float(([x0, y0] - np.array([x, y])) @ [np.cos(yaw0), np.sin(yaw0)])
        now_turn = float((yaw - yaw0 + np.pi) % (2 * np.pi) - np.pi)
        if abs(now_back - back) < BASE_TOLERANCE[0] * 2 and abs(now_turn - turn) < BASE_TOLERANCE[1] * 2:
            return
        self.ready()        # the free arms in, so that they sweep nothing as the base turns
        self.move_base(0., -now_turn)
        self.move_base(now_back - back, turn)

    def ready(self):
        """Back to the start pose with each arm holding nothing (a planned move; skipped where it
        does not plan), the fingers open."""
        for arm in self.arms:
            if arm in self.held:
                continue
            solver, indices = self.solvers[arm], self.model.arm_indices(arm)
            goal = self.model.q.copy()
            goal[indices] = self.home[indices]
            if np.abs(goal - self.model.q).max() < .05:
                continue
            try:
                solver.open_gripper()
                solver.move_to_pose_with_RRTConnect(None, goals=[goal])
            except PlanningFailed as error:
                self.ctx.note('not ready', arm=arm, reason=error.reason)
