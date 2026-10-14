"""Right-arm motion layer of the X1Pro experts, on the robot model and the 25 Hz Env.

Modelled on ManiSkill3's ``TwoFingerGripperMotionPlanningSolver``: free-space
moves use RRTConnect to an IK goal, contact moves use a straight screw path,
and every trajectory is executed as one joint-position action per control step.
Motions are generators that yield actions; the caller sends each to
``env.step`` (see :func:`run_expert`), so one expert per environment can run in
lockstep. The planning world (only the robot, plus the objects a task adds) is
refreshed from live body poses before each plan; planning failures raise
:class:`PlanningFailed` with the stage and reason.
"""

import time

import numpy as np

from table_1000.planning.world import PlanningScene, asset_objects, box_object, pose
from table_1000.robots.controllers import CONTROL_HZ

CONTROL_DT = 1 / CONTROL_HZ
# Free-space (RRTConnect) plans keep this clearance from the table and cabinet;
# contact moves (screw paths next to the handle or pen) use the bare geometry.
# The handle and pen, which the fingers touch on purpose, are never grown.
PADDING = .003
# RRTConnect only checks its edges coarsely; its time-parameterized path is
# re-checked at this joint-space step and re-planned if any check collides.
VALIDATION_STEP = .01
ARRIVAL_TOLERANCE = .01   # rad, final joint error before a move counts as done
SETTLE_LIMIT = .5         # s of extra holding allowed to reach it
GRIPPER_TIME = .6         # s for the fingers to close or open


class PlanningFailed(RuntimeError):
    def __init__(self, stage, reason, details=None):
        super().__init__(f'{stage}: {reason}')
        self.stage, self.reason, self.details = stage, reason, details or {}


def steps(seconds):
    return max(1, round(seconds / CONTROL_DT))


class MotionPlanner:
    """Motions of one arm of the robot in ``view`` (a :class:`~table_1000.sim.env.EnvView`)."""

    def __init__(self, view, arm='right'):
        self.view, self.model, self.arm = view, view.model, view.model.spec.arms[arm]
        self.planner = self.model.planners[arm]
        # The planner works in the base frame (see table_1000.robots.kinematics).
        self.world = PlanningScene(self.planner, np.linalg.inv(self.model.base))
        self.timings = {'planning_s': 0., 'executing_s': 0.}
        self.log = []
        self.max_tracking_error = 0.

    def close(self):
        """Remove this expert's objects from the shared planning world."""
        self.world.clear()

    def add_box(self, name, size, transform):
        self.world.add(*box_object(name, size, transform))

    def add_asset(self, model, prefix, group, padded=lambda name: True, merged=lambda name: False):
        """Planning objects of an asset instance (see ``asset_objects``), at their live poses."""
        objects = asset_objects(model, prefix, group, padded, merged)
        poses = self.view.body_poses()
        for obj in objects:
            self.world.add(obj, poses[obj.body])
        return objects

    # -- state ---------------------------------------------------------------
    def qpos(self):
        return self.view.qpos(self.arm.joints)

    def sync(self):
        """Move planning objects to their live poses (the robot follows every observation)."""
        self.world.update(self.view.body_poses())

    def tcp(self):
        return self.model.tcp(self.arm.name, world=True)

    def allow(self, names, allowed=True):
        """Permit (or revoke) planned contact between the fingers and objects."""
        self.world.allow(names, self.arm.finger_links, allowed)

    # -- planning ------------------------------------------------------------
    def _plan(self, stage, kind, function):
        self.world.set_padding(PADDING if kind == 'rrt' else 0.)
        self.sync()
        started = time.perf_counter()
        result = function()
        elapsed = time.perf_counter() - started
        self.timings['planning_s'] += elapsed
        entry = {'stage': stage, 'kind': kind, 'status': result['status'], 'planning_s': elapsed,
                 'time': self.view.time}
        if result['status'] != 'Success':
            entry.update({key: value for key, value in result.items()
                          if key not in ('position', 'velocity', 'acceleration', 'time')})
            self.log.append(entry)
            raise PlanningFailed(stage, result.get('reason', result['status']), entry)
        entry['duration_s'] = float(result['duration'])
        if 'validation_checks' in result:
            entry['validation_checks'] = result['validation_checks']
        self.log.append(entry)
        return result

    def validated(self, result):
        """Fail ``result`` if its path collides anywhere at VALIDATION_STEP."""
        if result['status'] != 'Success':
            return result
        checks, pairs = self.model.check_path(self.arm.name, result['position'], VALIDATION_STEP)
        if pairs:
            return {'status': 'path collision', 'reason': 'path collision', 'collisions': pairs,
                    'validation_checks': checks}
        result['validation_checks'] = checks
        return result

    def move_to_pose_with_RRTConnect(self, stage, target, planning_time=3., attempts=3):
        """Plan with RRTConnect to an IK goal (an approximate or colliding path is retried), then follow it."""
        for attempt in range(attempts):
            try:
                result = self._plan(stage, 'rrt', lambda: self.validated(self.model.plan_rrt(
                    self.arm.name, target, world=True, time_step=CONTROL_DT, planning_time=planning_time)))
                break
            except PlanningFailed as error:
                if not error.reason.startswith(('RRTConnect', 'path collision')) or attempt == attempts - 1:
                    raise
        yield from self.follow(stage, result)

    def move_to_pose_with_screw(self, stage, target):
        result = self._plan(stage, 'screw', lambda: self.model.plan_screw(
            self.arm.name, target, world=True, time_step=CONTROL_DT))
        yield from self.follow(stage, result)

    # -- execution -----------------------------------------------------------
    def follow(self, stage, result):
        """Send the path one waypoint per control step, then hold its end until arrival."""
        started = time.perf_counter()
        positions = np.asarray(result['position'])
        worst = 0.
        for target in positions:
            yield {self.arm.name: target[None]}
            worst = max(worst, float(np.abs(self.qpos() - target).max()))
        final = positions[-1]
        held = 0
        while np.abs(self.qpos() - final).max() > ARRIVAL_TOLERANCE:
            if held * CONTROL_DT >= SETTLE_LIMIT:
                raise PlanningFailed(stage, 'tracking', {'final_error_rad': float(np.abs(self.qpos() - final).max())})
            yield {}
            held += 1
        self.max_tracking_error = max(self.max_tracking_error, worst)
        self.log[-1].update(max_tracking_error_rad=worst, settle_steps=held)
        self.timings['executing_s'] += time.perf_counter() - started

    def gripper(self, gap, seconds=GRIPPER_TIME):
        for _ in range(steps(seconds)):
            yield {f'{self.arm.name}_gripper': [[gap]]}

    def open_gripper(self, gap=.08):
        yield from self.gripper(gap)

    def close_gripper(self):
        yield from self.gripper(0.)

    def finger_gap(self):
        left, right = self.view.qpos(self.arm.fingers)
        return float(left - right)

    def hold(self, seconds):
        for _ in range(steps(seconds)):
            yield {}

    # -- held objects ----------------------------------------------------------
    def attach(self, name, object_world):
        """Carry planning object ``name`` with the TCP at its measured relative pose."""
        relative = np.linalg.inv(self.tcp()) @ object_world
        world = self.planner.planning_world
        world.attach_object(name, self.planner.robot.get_name(), self.planner.link_name_2_idx[self.arm.tcp_link],
                            pose(relative), list(self.arm.finger_links))
        return relative

    def detach(self, name):
        self.planner.planning_world.detach_object(name, False)

    def carry(self, names, contacts):
        """Attach planning objects held through a joint (a drawer pulled by its
        handle) so they move with the TCP; each may touch ``contacts``."""
        poses = self.view.body_poses()
        for name in names:
            self.attach(name, poses[self.world.objects[name].body])
        self.world.allow(names, contacts)

    def release(self, names, contacts):
        for name in names:
            self.detach(name)
        self.world.allow(names, contacts, False)
        self.sync()


def run_expert(env, experts):
    """Step ``env`` with one expert generator per environment until all finish.

    An expert that finishes (or raises) holds its robot still while the others
    go on; returns one ``None`` or exception per environment.
    """
    outcomes = [None] * len(experts)
    active = list(range(len(experts)))
    while active:
        actions = {}
        for i in list(active):
            try:
                action = next(experts[i])
            except StopIteration:
                active.remove(i)
                continue
            except Exception as error:   # the expert's failure, reported per environment
                outcomes[i] = error
                active.remove(i)
                continue
            if len(experts) != 1:
                raise NotImplementedError('batched actions are merged once a backend has several environments')
            actions = action
        if active:
            env.step(actions)
    return outcomes
