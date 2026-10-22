"""Pick: grasp an object resting on a support (the table, a box on it, ...) with either arm and lift it.

``pick(ctx, key, shape)`` grasps the object body ``key`` (a scene primitive's
name, or ``'<object>:<body>'``) whose collision shape is ``shape`` (a
:class:`~table_1000.experts.grasps.Box` or
:class:`~table_1000.experts.grasps.Cylinder`, placed by ``offset`` in the body
frame), then lifts it. It is one blocking skill on top of
:class:`~table_1000.experts.solver.MotionSolver`:

1. **choose** -- grasp candidates of the shape at its live pose
   (:func:`~table_1000.experts.grasps.grasp_candidates`, cheapest first: close to a
   reach-out-and-down approach from the arm's shoulder, centred on the centre of
   mass) are tried per arm, the arm whose shoulder is nearer first. Every object of
   the arm's planning world is an obstacle of the candidates' hand; the ones within
   ``SUPPORT_DISTANCE`` of the object are its supports, which the hand may touch (overlapping
   them by at most ``-SUPPORT_CLEARANCE``; closer than the planning world's padding, so the
   fingers' planned contact with them is allowed).
   A candidate is kept when the grasp, a pre-grasp ``APPROACH`` behind it and the
   grasp ``LIFT`` higher (unless the torso lift can rise that far) have collision-free
   IK solutions, the torso lift free for the grasp and the lifted pose and held for the
   pre-grasp;
2. **reach** -- RRTConnect to the pre-grasp, moving the torso lift with the arm,
   fingers open to the object's width plus a margin; then a straight move in,
   the only move in which the fingers may touch the object;
3. **grasp** -- the fingers close fully (the drives squeeze the object); the
   finger gap must show the object between the pads;
4. **lift** -- up by ``LIFT``: straight, else by raising the torso and the arm for the
   rest (see :meth:`Picker.lift`); the object must have risen with the hand.

A candidate whose motion cannot be planned is replaced by the next one; a
grasp that misses is backed out of and the object re-observed, at most
``MAX_REGRASPS`` times. Each change is noted (``ctx.note``); anything else fails
the subtask (``ctx.fail``) or raises the planning failure.
"""

import time

import numpy as np

from table_1000.experts.context import DT, steps
from table_1000.experts.grasps import SUPPORT_CLEARANCE, Box, Cylinder, Hand, grasp_candidates, standoff, translated
from table_1000.experts.solver import PlanningFailed
from table_1000.planning.world import PlanningObject

APPROACH = .05          # m, pre-grasp distance behind the grasp along the approach
LIFT = .10              # m
CLOSE_TIME = .8         # s
HELD_FRACTION = .5      # the closed gap must be at least this share of the object's width
MAX_REGRASPS = 2
MAX_IK = 60             # candidates checked by IK per arm and choice
MAX_PLANS = 6           # candidates whose motion is tried per grasp attempt
IK_ATTEMPTS = 20        # IK restarts per pose
MIN_RISE = .05          # m the object must rise in the lift
SUPPORT_DISTANCE = .003 # m, planning objects this close to the picked one support it


def planning_object(name, key, shape, offset=np.eye(4)):
    """Planning object ``name`` of ``shape`` moving with body ``key``."""
    if isinstance(shape, Box):
        entry = (np.asarray(offset, dtype=float), 'box', tuple(shape.size))
    else:
        entry = (np.asarray(offset, dtype=float), 'cylinder', (shape.radius, shape.length))
    return PlanningObject(name, key, [entry], padded=False)


class Picker:
    """Picks with ``ctx``'s arms (``arms``: names, default all). The table is added to each
    arm's planning world once; other obstacles are added by the caller."""

    def __init__(self, ctx, arms=None):
        self.ctx, self.model = ctx, ctx.model
        self.arms = list(arms or ctx.spec.arms)
        self.solvers = {arm: ctx.solver(arm) for arm in self.arms}
        self.hands = {arm: Hand(self.model, arm) for arm in self.arms}
        for solver in self.solvers.values():
            if 'table' not in solver.world.objects:
                solver.add_table()
        self.log, self.rejected = [], {}

    # -- choice ------------------------------------------------------------------------
    def shoulder(self, arm):
        return self.model.fk(self.ctx.spec.arms[arm].joints[1].replace('joint', 'link'), world=True)[:3, 3]

    def obstacles(self, arm, key, transform, shape):
        """``([(FCL object, clearance to keep)], supports)``: the objects of ``arm``'s planning world
        other than ``key`` (at ``transform``, ``shape``), and the names of those supporting it."""
        from mplib.collision_detection import fcl
        from table_1000.planning.world import geometry, pose
        self.solvers[arm].sync()
        world = self.solvers[arm].world
        target = fcl.FCLObject(key, pose(transform), [fcl.CollisionObject(geometry(*shape.fcl_shape()))],
                               [pose(np.eye(4))])
        obstacles, supports = [], []
        for name, obj in world.objects.items():
            if name == key or world.planner.planning_world.is_object_attached(name):
                continue
            body = obj.fcl(world.poses[name])
            support = fcl.distance(target, body).min_distance < SUPPORT_DISTANCE
            if support:
                supports.append(name)
            obstacles.append((body, SUPPORT_CLEARANCE if support else 0.))
        return obstacles, supports

    def options(self, key, shape, offset, skip):
        """Feasible ``(arm, grasp, supports, q_grasp, q_pre)``, in the order tried; ``skip`` holds the
        ids of candidates already used. ``self.rejected`` counts the candidates dropped, by reason."""
        self.rejected = {'grasp IK': 0, 'pre-grasp IK': 0, 'lift': 0}
        transform = self.ctx.poses()[key] @ offset
        centre = transform[:3, 3]
        arms = sorted(self.arms, key=lambda arm: np.linalg.norm((self.shoulder(arm) - centre)[:2]))
        for arm in arms:
            solver, side = self.solvers[arm], self.ctx.spec.arms[arm]
            reach = normalized(centre - self.shoulder(arm))
            preferred = normalized(reach + [0., 0., -1.])
            obstacles, supports = self.obstacles(arm, key, transform, shape)
            candidates = grasp_candidates(shape, transform, self.hands[arm], obstacles, preferred,
                                          approach_distance=APPROACH)
            checked = 0
            for index, grasp in enumerate(candidates):
                if (arm, index) in skip:
                    continue
                if checked >= MAX_IK:
                    break
                checked += 1
                start = self.model.vector(side.finger_targets(grasp.open_gap))
                solver.sync()
                # The fingers touch the object and come closer to its supports than the padding.
                solver.allow([key, *supports])
                status, q_grasp = self.model.ik(arm, grasp.pose, start, world=True, lift=True, attempts=IK_ATTEMPTS)
                solver.allow([key, *supports], False)
                if status != 'Success':
                    self.rejected['grasp IK'] += 1
                    continue
                status, q_pre = self.model.ik(arm, standoff(grasp.pose, APPROACH), q_grasp, world=True,
                                              attempts=IK_ATTEMPTS)
                if status != 'Success':
                    self.rejected['pre-grasp IK'] += 1
                    continue
                # Close to the body the hand may not reach LIFT higher; then the torso must rise that far.
                status, _ = self.model.ik(arm, translated(grasp.pose, [0., 0., LIFT]), q_grasp, world=True,
                                          lift=True, attempts=IK_ATTEMPTS)
                headroom = self.model.limits[self.model.lift_indices(), 1] - q_grasp[self.model.lift_indices()]
                if status != 'Success' and np.min(headroom) < LIFT:
                    self.rejected['lift'] += 1
                    continue
                skip.add((arm, index))
                yield arm, grasp, supports, q_grasp, q_pre

    # -- skill --------------------------------------------------------------------------
    def pick(self, key, shape, offset=np.eye(4)):
        """Grasp body ``key`` by ``shape`` (at ``offset`` in the body frame) and lift it. Returns
        ``{'arm', 'grasp', 'finger_gap', 'tcp_object', ...}``."""
        ctx = self.ctx
        offset = np.asarray(offset, dtype=float)
        for solver in self.solvers.values():
            if key not in solver.world.objects:
                solver.world.add(planning_object(key, key, shape, offset), ctx.poses()[key])
        skip, regrasps, started = set(), 0, time.perf_counter()
        while True:
            reached = self.reach(key, shape, offset, skip)
            if reached is None:
                ctx.fail('no reachable grasp', candidates_tried=len(skip), rejected=self.rejected)
            arm, grasp, supports = reached
            solver = self.solvers[arm]
            solver.close_gripper()
            gap = solver.finger_gap()
            if gap >= HELD_FRACTION * grasp.width:
                break
            # Missed: back out, re-observe the object and choose again.
            ctx.note('missed grasp', arm=arm, finger_gap=gap, width=grasp.width)
            solver.open_gripper(grasp.open_gap)
            solver.first_reachable([standoff(solver.tcp(), APPROACH), translated(solver.tcp(), [0., 0., APPROACH])])
            solver.allow([key, *supports], False)
            regrasps += 1
            if regrasps > MAX_REGRASPS:
                ctx.fail('object not held', finger_gap=gap, width=grasp.width)
        before = ctx.poses()[key]
        solver.attach(key, before @ offset)
        self.lift(arm, supports)
        solver.allow(supports, False)
        ctx.hold(.2)
        after = ctx.poses()[key]
        rise = float(after[2, 3] - before[2, 3])
        if rise < MIN_RISE:
            ctx.fail('object not lifted', rise=rise, finger_gap=solver.finger_gap())
        result = {'arm': arm, 'grasp': {'pose': grasp.pose.tolist(), 'open_gap': grasp.open_gap, 'width': grasp.width,
                                        'backoff': grasp.backoff, 'cost': grasp.cost, **grasp.info},
                  'supports': supports,
                  'finger_gap': solver.finger_gap(), 'rise': rise, 'regrasps': regrasps,
                  'choice_s': time.perf_counter() - started,
                  'tcp_object': (np.linalg.inv(solver.tcp()) @ after).tolist()}
        self.log.append(result)
        return result

    def reach(self, key, shape, offset, skip):
        """Move to the first grasp of :meth:`options` whose motion plans, fingers open around the
        object; ``(arm, grasp, supports)``, or None when no candidate is left."""
        ctx = self.ctx
        for tried, (arm, grasp, supports, q_grasp, q_pre) in enumerate(self.options(key, shape, offset, skip)):
            if tried >= MAX_PLANS:
                return None
            solver = self.solvers[arm]
            try:
                solver.open_gripper(grasp.open_gap)
                solver.move_to_pose_with_RRTConnect(None, lift=True, goals=[q_pre])
                solver.allow([key, *supports])
                solver.move_to_pose_with_screw(grasp.pose)
                return arm, grasp, supports
            except PlanningFailed as error:
                solver.allow([key, *supports], False)
                ctx.note('next grasp', arm=arm, cost=grasp.cost, reason=error.reason)
        return None

    def lift(self, arm, supports):
        """Up by LIFT: straight with the arm; else (the arm close to the body may hit it going
        straight up) by raising the torso, then the arm for what the lift's travel leaves, straight
        or, the object clear of its ``supports``, along a planned path."""
        solver, ctx = self.solvers[arm], self.ctx
        try:
            solver.move_to_pose_with_screw(translated(solver.tcp(), [0., 0., LIFT]))
            return
        except PlanningFailed as error:
            straight = error.reason
        lift = ctx.env.qpos(ctx.spec.lift)[0]
        upper = self.model.limits[self.model.lift_indices(), 1]
        target = np.minimum(lift + LIFT, upper)
        ctx.note('lift with the torso', reason=straight, lift=lift.tolist(), target=target.tolist())
        count = steps(float(np.max(target - lift)) / ctx.spec.lift_velocity_limit)
        for k in range(1, count + 1):
            ctx.step({'lift': (lift + (target - lift) * k / count)[None]})
        ctx.hold(.3)
        rest = LIFT - float(np.min(target - lift))
        if rest < .005:
            return
        up = translated(solver.tcp(), [0., 0., rest])
        solver.allow(supports, False)
        for move in (solver.move_to_pose_with_screw, lambda pose: solver.move_to_pose_with_RRTConnect(pose, lift=True)):
            try:
                move(up)
                return
            except PlanningFailed as error:
                reason = error.reason
        ctx.note('lift short', rest=rest, reason=reason)


def normalized(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def pick(ctx, key, shape, offset=np.eye(4), arms=None):
    """:meth:`Picker.pick` with a picker made for this call."""
    return Picker(ctx, arms).pick(key, shape, offset)
