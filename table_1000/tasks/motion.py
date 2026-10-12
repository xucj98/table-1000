"""Thin X1Pro right-arm motion layer between MPlib and the Isaac environment.

Modelled on ManiSkill3's ``TwoFingerGripperMotionPlanningSolver``: free-space
moves use RRTConnect to an IK goal, contact moves use a straight screw path,
and every trajectory is executed as per-step joint position and velocity
targets. The planning world is refreshed from live body poses before each plan;
planning failures raise :class:`PlanningFailed` with the stage and reason.
"""

import time

import numpy as np

from table_1000.planning.screw import plan_screw
from table_1000.planning.world import PlanningScene, asset_objects, box_object, pose
from table_1000.robots import x1pro
from table_1000.tasks.environment import DT

# Free-space (RRTConnect) plans keep this clearance from the table and cabinet;
# contact moves (screw paths next to the handle or pen) use the bare geometry.
# The handle and pen, which the fingers touch on purpose, are never grown.
PADDING = .003
ARRIVAL_TOLERANCE = .01   # rad, final joint error before a move counts as done
SETTLE_LIMIT = .5         # s of extra holding allowed to reach it
GRIPPER_TIME = .6         # s for the fingers to close or open


class PlanningFailed(RuntimeError):
    def __init__(self, stage, reason, details=None):
        super().__init__(f'{stage}: {reason}')
        self.stage, self.reason, self.details = stage, reason, details or {}


class MotionPlanner:
    def __init__(self, env, robot_source, output, assets):
        self.env = env
        frozen = {name: value for name, value in env.scene.ROBOT['joints'].items() if name not in x1pro.ARM}
        from table_1000.scenes.stage import robot_base
        # Planning happens in the robot base frame (see x1pro.build_planner);
        # this class takes and returns world-frame poses.
        self.base = robot_base(env.scene)
        self.planner = x1pro.build_planner(robot_source, output, frozen)
        self.world = PlanningScene(self.planner, np.linalg.inv(self.base))
        table = env.scene.TABLE
        top = np.eye(4)
        top[:3, 3] = [*table['center'], table['height'] - table['top_thickness'] / 2]
        self.world.add(*box_object('table', (*table['size'], table['top_thickness']), top))
        # Cabinet: one object per body, handles split out for contact permission.
        # Pen: its bodies stay assembled, so one object in the pen body frame.
        self.objects = (
            asset_objects(f'{assets}/{env.scene.CABINET["asset"]}/model.json', 'cabinet:',
                          lambda body, part: (part if part.endswith('.handle') else body, body),
                          lambda name: not name.endswith('.handle'))
            + asset_objects(f'{assets}/{env.scene.PEN["asset"]}/model.json', 'pen:',
                            lambda body, part: ('pen', 'body'), lambda name: False))
        poses = env.body_poses()
        for obj in self.objects:
            self.world.add(obj, poses[obj.body])
        self.timings = {'planning_s': 0., 'executing_s': 0.}
        self.log = []
        self.max_tracking_error = 0.

    # -- state ---------------------------------------------------------------
    def qpos(self):
        return self.env.joint_positions(x1pro.ARM + x1pro.FINGERS)

    def sync(self):
        """Move planning objects to their live poses and the robot to its live q."""
        self.world.update(self.env.body_poses())
        self.planner.robot.set_qpos(self.qpos(), True)

    def tcp(self, q=None):
        q = self.qpos() if q is None else q
        model = self.planner.pinocchio_model
        model.compute_forward_kinematics(q)
        local = model.get_link_pose(self.planner.link_name_2_idx[x1pro.TCP_LINK]).to_transformation_matrix()
        return self.base @ local

    def allow(self, names, allowed=True):
        """Permit (or revoke) planned contact between the fingers and objects."""
        self.world.allow(names, x1pro.FINGER_LINKS, allowed)

    # -- planning ------------------------------------------------------------
    def _plan(self, stage, kind, function):
        self.world.set_padding(PADDING if kind == 'rrt' else 0.)
        self.sync()
        started = time.perf_counter()
        result = function()
        elapsed = time.perf_counter() - started
        self.timings['planning_s'] += elapsed
        entry = {'stage': stage, 'kind': kind, 'status': result['status'], 'planning_s': elapsed,
                 'time': self.env.time}
        if result['status'] != 'Success':
            entry.update({key: value for key, value in result.items() if key not in ('position', 'velocity', 'acceleration', 'time')})
            self.log.append(entry)
            raise PlanningFailed(stage, result.get('reason', result['status']), entry)
        entry['duration_s'] = float(result['duration'])
        self.log.append(entry)
        return result

    def move_to_pose_with_RRTConnect(self, stage, target, planning_time=3., attempts=3):
        """Plan with RRTConnect to an IK goal; an approximate solution is retried."""
        q = self.qpos()
        goal = pose(np.linalg.inv(self.base) @ target)
        for attempt in range(attempts):
            try:
                result = self._plan(stage, 'rrt', lambda: self.planner.plan_pose(
                    goal, q, mask=[False] * 6 + [True] * 2, time_step=DT, planning_time=planning_time))
                break
            except PlanningFailed as error:
                if 'RRTConnect' not in error.reason or attempt == attempts - 1:
                    raise
        return self.follow(stage, result)

    def move_to_pose_with_screw(self, stage, target):
        q = self.qpos()
        goal = pose(np.linalg.inv(self.base) @ target)
        result = self._plan(stage, 'screw', lambda: plan_screw(self.planner, goal, q, time_step=DT))
        return self.follow(stage, result)

    # -- execution -----------------------------------------------------------
    def follow(self, stage, result):
        """Execute a time-parameterized arm path, then hold its end until arrival."""
        started = time.perf_counter()
        positions, velocities = np.asarray(result['position']), np.asarray(result['velocity'])
        worst = 0.
        for target, velocity in zip(positions, velocities):
            self.env.set_targets(dict(zip(x1pro.ARM, target)), dict(zip(x1pro.ARM, velocity)))
            self.env.step()
            worst = max(worst, float(np.abs(self.env.joint_positions(x1pro.ARM) - target).max()))
        final = positions[-1]
        self.env.set_targets(dict(zip(x1pro.ARM, final)), {})
        held = 0
        while np.abs(self.env.joint_positions(x1pro.ARM) - final).max() > ARRIVAL_TOLERANCE:
            if held * DT >= SETTLE_LIMIT:
                raise PlanningFailed(stage, 'tracking', {'final_error_rad': float(np.abs(self.env.joint_positions(x1pro.ARM) - final).max())})
            self.env.step()
            held += 1
        self.max_tracking_error = max(self.max_tracking_error, worst)
        self.log[-1].update(max_tracking_error_rad=worst, settle_steps=held)
        self.timings['executing_s'] += time.perf_counter() - started
        return worst

    def gripper(self, gap, seconds=GRIPPER_TIME):
        self.env.set_targets(x1pro.finger_targets(gap))
        self.env.step(round(seconds / DT))
        return self.finger_gap()

    def open_gripper(self, gap=.08):
        return self.gripper(gap)

    def close_gripper(self):
        return self.gripper(0.)

    def finger_gap(self):
        left, right = self.env.joint_positions(x1pro.FINGERS)
        return float(left - right)

    def hold(self, seconds):
        self.env.step(round(seconds / DT))

    # -- held objects ----------------------------------------------------------
    def attach(self, name, object_world):
        """Carry planning object ``name`` with the TCP at its measured relative pose."""
        tcp = self.tcp()
        relative = np.linalg.inv(tcp) @ object_world
        world = self.planner.planning_world
        world.attach_object(name, self.planner.robot.get_name(), self.planner.link_name_2_idx[x1pro.TCP_LINK],
                            pose(relative), x1pro.FINGER_LINKS)
        return relative

    def detach(self, name):
        self.planner.planning_world.detach_object(name, False)

    def carry(self, names, contacts):
        """Attach planning objects held through a joint (a drawer pulled by its
        handle) so they move with the TCP; each may touch ``contacts``."""
        poses = self.env.body_poses()
        for name in names:
            self.attach(name, poses[self.world.objects[name].body])
        self.world.allow(names, contacts)

    def release(self, names, contacts):
        for name in names:
            self.detach(name)
        self.world.allow(names, contacts, False)
        self.sync()
