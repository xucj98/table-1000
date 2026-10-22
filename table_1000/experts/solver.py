"""Motion primitives of one arm, after ManiSkill3's ``TwoFingerGripperMotionPlanningSolver``.

Free-space moves plan with RRTConnect to an IK goal, contact moves with a
straight screw path (both MPlib, see :class:`~table_1000.robots.kinematics.RobotModel`),
and a planned path is sent as one joint-position action per control step. The
planning world holds the robot and whatever the expert adds (a table box,
asset objects from their collision geometry); objects follow their live poses
before every plan. Contact with an object is planned only where the expert
allows it (:meth:`MotionSolver.allow`), and held objects move with the hand
(:meth:`MotionSolver.attach`, :meth:`MotionSolver.carry`). A plan that cannot be
made raises :class:`PlanningFailed` with the reason (``IK Failed``,
``RRTConnect ...``, ``joint_limit``, ``collision``, ``path collision``,
``tracking``).
"""

import time

import numpy as np

from table_1000.experts.context import DT, ExpertFailed, steps
from table_1000.planning.world import PlanningScene, asset_objects, box_object, pose

# Free-space (RRTConnect) plans keep this clearance from padded objects; contact
# moves use the bare geometry. Objects the hand touches on purpose are never grown.
PADDING = .003
# RRTConnect checks its edges coarsely; its time-parameterized path is re-checked
# at this joint step and re-planned if any check collides.
VALIDATION_STEP = .01
ARRIVAL_TOLERANCE = .01   # rad, final joint error before a move counts as done
SETTLE_LIMIT = .5         # s of extra holding allowed to reach it
GRIPPER_TIME = .6         # s for the fingers to close or open
OPEN_GAP = .08            # m


class PlanningFailed(ExpertFailed):
    pass


class MotionSolver:
    def __init__(self, ctx, arm):
        self.ctx, self.model = ctx, ctx.model
        self.arm = ctx.spec.arms[arm]
        self.planner = self.model.planners[arm]
        # The planner works in the base frame (see table_1000.robots.kinematics).
        self.world = PlanningScene(self.planner, np.linalg.inv(self.model.base))
        self.log = []
        self.timings = {'planning_s': 0., 'executing_s': 0.}

    # -- planning world --------------------------------------------------------------
    def add_box(self, name, size, transform):
        """A static box (e.g. the table top) of ``size`` centred on ``transform``."""
        self.world.add(*box_object(name, size, transform))

    def add_table(self):
        table = self.ctx.scene.TABLE
        top = np.eye(4)
        top[:3, 3] = [*table['center'], table['height'] - table['top_thickness'] / 2]
        self.add_box('table', (*table['size'], table['top_thickness']), top)

    def add_object(self, name, group, padded=lambda part: True, merged=lambda part: False):
        """Planning objects of asset object ``name`` from its collision geometry: ``group(body, part)``
        returns ``(planning name, frame body)`` (see :func:`~table_1000.planning.world.asset_objects`);
        planning names get the prefix ``<name>:``."""
        objects = asset_objects(self.ctx.model_path(name), f'{name}:', group, padded, merged)
        poses = self.ctx.poses()
        for obj in objects:
            self.world.add(obj, poses[obj.body])
        return objects

    def sync(self):
        """Move planning objects to their live poses."""
        self.world.update(self.ctx.poses())

    def allow(self, names, allowed=True):
        """Permit (or revoke) planned contact between the fingers and planning objects ``names``."""
        self.world.allow(names, self.arm.finger_links, allowed)

    def attach(self, name, object_world):
        """Carry planning object ``name`` with the TCP at its measured pose relative to it;
        returns that relative pose."""
        relative = np.linalg.inv(self.tcp()) @ object_world
        self.planner.planning_world.attach_object(
            name, self.planner.robot.get_name(), self.planner.link_name_2_idx[self.arm.tcp_link], pose(relative),
            list(self.arm.finger_links))
        return relative

    def detach(self, name):
        self.planner.planning_world.detach_object(name, False)

    def carry(self, names, contacts):
        """Attach planning objects moved through a joint (a drawer pulled by its handle) so they
        move with the TCP; each may touch ``contacts``."""
        poses = self.ctx.poses()
        for name in names:
            self.attach(name, poses[self.world.objects[name].body])
        self.world.allow(names, contacts)

    def release(self, names, contacts):
        for name in names:
            self.detach(name)
        self.world.allow(names, contacts, False)
        self.sync()

    # -- state ---------------------------------------------------------------------------
    def qpos(self, lift=False):
        return self.ctx.env.qpos([*self.arm.joints, *(self.ctx.spec.lift if lift else ())])[0]

    def tcp(self):
        return self.model.tcp(self.arm.name, world=True)

    def finger_gap(self):
        return self.arm.gap(self.ctx.env.qpos(self.arm.fingers)[0])

    # -- planning ------------------------------------------------------------------------
    def _plan(self, kind, function):
        self.world.set_padding(PADDING if kind == 'rrt' else 0.)
        self.sync()
        # A hand starting closer to an object than the padding (just let go of it) plans unpadded.
        closer = self.model.collisions(self.arm.name, self.model.q) if kind == 'rrt' else []
        if closer:
            self.world.set_padding(0.)
        # The robot's planning links are grown, so they may overlap where the bare links do not (an
        # elbow by the torso after a pull): a straight move starting so is planned with them allowed.
        robot = self.planner.robot.get_name()
        overlaps = [(link, other) for link, name, other in self.model.collisions(self.arm.name, self.model.q)
                    if name == robot] if kind == 'screw' else []
        for link, other in overlaps:
            self.world.allow([link], [other])
        started = time.perf_counter()
        try:
            result = function()
        finally:
            for link, other in overlaps:
                self.world.allow([link], [other], False)
        elapsed = time.perf_counter() - started
        self.timings['planning_s'] += elapsed
        entry = {'subtask': self.ctx.current, 'step': self.ctx.steps, 'kind': kind, 'status': result['status'],
                 'planning_s': elapsed}
        if closer:
            entry['unpadded'] = closer
        if overlaps:
            entry['self_overlaps'] = overlaps
        self.log.append(entry)
        if result['status'] != 'Success':
            entry.update({key: value for key, value in result.items()
                          if key not in ('position', 'velocity', 'acceleration', 'time')})
            raise PlanningFailed(self.ctx.current, result.get('reason', result['status']), plan=entry)
        entry['duration_s'] = float(result['duration'])
        return result

    def _validated(self, result):
        """Fail ``result`` if its path collides anywhere at VALIDATION_STEP."""
        if result['status'] != 'Success':
            return result
        checks, pairs = self.model.check_path(self.arm.name, result['position'], VALIDATION_STEP, result.get('lift'))
        if pairs:
            return {'status': 'path collision', 'reason': 'path collision', 'collisions': pairs}
        return result

    def move_to_pose_with_RRTConnect(self, target, planning_time=3., attempts=3, lift=False, goals=None):
        """Plan to an IK goal of world TCP pose ``target`` (or to ``goals``, full configurations of
        the planning model) with RRTConnect and follow it; an approximate or colliding path is
        re-planned up to ``attempts`` times. ``lift`` moves the torso lift as well."""
        for attempt in range(attempts):
            try:
                result = self._plan('rrt', lambda: self._validated(self.model.plan_rrt(
                    self.arm.name, target, world=True, time_step=DT, planning_time=planning_time, lift=lift,
                    goals=goals)))
                break
            except PlanningFailed as error:
                if not error.reason.startswith(('RRTConnect', 'path collision')) or attempt == attempts - 1:
                    raise
        self.follow(result)

    def move_to_pose_with_screw(self, target):
        """Move the TCP along a straight screw path to world pose ``target``."""
        self.follow(self._plan('screw', lambda: self.model.plan_screw(self.arm.name, target, world=True,
                                                                      time_step=DT)))

    def first_reachable(self, targets, move=None):
        """Move to the first of ``targets`` that plans (``move``: default the screw move); a plan
        failing for lack of a path tries the next, any other failure propagates."""
        move = move or self.move_to_pose_with_screw
        for index, target in enumerate(targets):
            try:
                return move(target)
            except PlanningFailed as error:
                if index == len(targets) - 1 or not error.reason.startswith(
                        ('IK Failed', 'joint_limit', 'collision', 'path collision', 'RRTConnect')):
                    raise

    # -- execution -----------------------------------------------------------------------
    def follow(self, result):
        """Send the path one waypoint per control step (with the torso lift's, if it moves), then
        hold its end until arrival."""
        started = time.perf_counter()
        positions = np.asarray(result['position'])
        lift = result.get('lift')
        worst = 0.
        for index, target in enumerate(positions):
            action = {self.arm.name: target[None]}
            if lift is not None:
                action['lift'] = lift[index][None]
            self.ctx.step(action)
            worst = max(worst, float(np.abs(self.qpos() - target).max()))
        if lift is not None:
            positions = np.c_[positions, lift]
        final, held = positions[-1], 0
        while np.abs(self.qpos(lift is not None) - final).max() > ARRIVAL_TOLERANCE:
            if held * DT >= SETTLE_LIMIT:
                raise PlanningFailed(self.ctx.current, 'tracking',
                                     final_error_rad=float(np.abs(self.qpos(lift is not None) - final).max()))
            self.ctx.step()
            held += 1
        self.log[-1].update(max_tracking_error_rad=worst, settle_steps=held)
        self.timings['executing_s'] += time.perf_counter() - started

    def gripper(self, gap, seconds=GRIPPER_TIME):
        for _ in range(steps(seconds)):
            self.ctx.step({f'{self.arm.name}_gripper': [[gap]]})

    def open_gripper(self, gap=OPEN_GAP):
        self.gripper(gap)

    def close_gripper(self):
        self.gripper(0.)
