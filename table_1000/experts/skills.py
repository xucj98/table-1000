"""Atomic skills for experts: grasp, pick, move by a displacement, push, release, place.

Named after RoboTwin 2.0's task API (``grasp_actor``, ``move_by_displacement``,
``place_actor``, ``open_gripper``/``close_gripper``) but blocking like the rest of
``solve(ctx)``: each call returns when the motion is done, checked in the
simulation, and fails the subtask (:class:`~table_1000.experts.context.ExpertFailed`,
or :class:`~table_1000.experts.solver.PlanningFailed` for a move that cannot be
planned) when it is not. Lower-level moves stay available through
``ctx.solver(arm)`` and mix freely with these::

    from table_1000.experts import skills

    g = skills.grasp(ctx, 'cabinet', 'cabinet.drawer1.handle', then=-pull)  # either arm, the nearer first
    skills.move_by_displacement(ctx, g['arm'], *-pull, drag=('cabinet', 'cabinet.drawer1'))
    skills.release(ctx, g['arm'])
    p = skills.pick(ctx, 'pen')                                              # grasp, then lift
    skills.place(ctx, 'pen', (table_frame, (x0, y0), (x1, y1)), near=(x, y, table_height))

What is grasped is an object of the scene (an asset, by name), or a part or body
of it: its shape is the collision box of that part in the frame of its body, or
one given (a :class:`~table_1000.experts.grasps.Box` or
:class:`~table_1000.experts.grasps.Cylinder` placed by ``offset`` in the body
frame, also for a scene primitive that is not an asset). Every asset of the scene
is in each arm's planning world, one planning object per part (``<object>:<part>``);
the table too.

**grasp** -- candidates of the shape at its live pose
(:func:`~table_1000.experts.grasps.grasp_candidates`, cheapest first: close to the
``approach`` direction, default reaching out and down from the arm's shoulder;
centred on ``point``, default the shape's centre) are tried per arm, the arm whose
shoulder is nearer first. Every other planning object is an obstacle of the
candidates' hand; the ones within ``SUPPORT_DISTANCE`` of the shape are its supports,
which the hand may touch (overlapping them by at most ``-SUPPORT_CLEARANCE``). A
candidate is kept when the grasp and a pre-grasp ``APPROACH`` behind it have
collision-free IK solutions, and so does the grasp moved by ``then`` (the motion that
follows, if given). The hand plans (RRTConnect, the torso lift with the arm) to the
pre-grasp with the fingers open to the object's width plus a margin, moves straight
in (the only move in which the fingers may touch the object), and closes; the finger
gap must stop within ``HELD_TOLERANCE`` of the gap at which the pads touch the object, else it backs out, re-observes the
object and chooses again (at most ``MAX_REGRASPS`` times). The grasped part then
moves with the hand in the planning world until :func:`release`.

**move_by_displacement** -- the TCP straight by a world displacement, with the torso
lift too where the arm alone runs into a joint limit, or (``least``) as far along as
plans; straight up beyond the arm's reach, the torso lift rises and the arm does the rest. ``drag``
names a body moved through a joint by the held part (a drawer pulled by its
handle): its planning objects move with the hand and may touch the rest of their
object and whatever rests on them.

**push** -- the closed fingertips against the face of a part that ``direction``
points into, then straight on by ``distance`` (a drawer pushed shut by its front);
the hand comes level with the push or tilted down (``PUSH_TILTS``), and what the
part's body carries moves with it as in a drag.

**release** -- open the fingers to the grasp's opening and back out along the
approach (or up).

**place** -- put the held object down anywhere in a region: a rectangle on a surface
(a frame whose z = 0 plane is the surface), its footprint inside by a margin and out
of keep-out rectangles. Positions every ``PLACE_STEP`` and turns about the surface
normal every ``PLACE_TURN_STEP`` are ranked nearest a given point, then least turned
(``PLACE_TURNS`` per position);
IK screens them above and at the release (``MAX_PLACE_IK``), and the first of those that has a motion
(``MAX_PLACES`` planned) is taken: above it, straight down to ``clearance`` over the
surface, then :func:`release`.

**make_room** -- move the base (backed off along its start heading, turned) to the
least move from where given TCP poses are reachable with every joint a margin inside
its limits, the free arms back at their start pose first (``move_base`` drives it
straight and turns it in place).

**retry** -- a step run again when it fails (``ATTEMPTS`` in all, within
``RETRY_BUDGET`` s of the episode), the arms that hold nothing back at their start pose
first, so that its choices start afresh.

Each choice of another candidate is noted (``ctx.note``).
"""

from dataclasses import dataclass
import time

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.experts.context import ExpertFailed, steps
from table_1000.experts.grasps import SUPPORT_CLEARANCE, Box, Cylinder, Hand, grasp_candidates, standoff, translated
from table_1000.experts.solver import PlanningFailed
from table_1000.planning.world import PlanningObject

APPROACH = .05          # m, pre-grasp distance behind the grasp along the approach
LIFT = .10              # m, pick's lift
HELD_TOLERANCE = .005   # m the closed gap may fall short of the gap at which the pads touch the object
MAX_REGRASPS = 2
MAX_IK = 60             # candidates checked by IK per arm and choice
MAX_PLANS = 6           # candidates whose motion is tried per grasp attempt
IK_ATTEMPTS = 20        # IK restarts per pose
MIN_RISE = .05          # m the object must rise in pick's lift
SUPPORT_DISTANCE = .003 # m, planning objects this close to the grasped one support it
PLACE_ABOVE = .08       # m, place comes down from this high above the release pose
PLACE_CLEARANCE = .01   # m between the held object and the surface at the release
ATTEMPTS = 3            # tries of a step under retry
RETRY_BUDGET = 240.     # s of an episode (wall clock) after which a failed step is not tried again
# The base drives at most BASE_SPEED (m/s) and turns at most BASE_TURN (rad/s), proportionally to the
# error (BASE_GAIN) until within BASE_TOLERANCE (m, rad), then settles for BASE_SETTLE s. make_room
# tries it BASE_BACKS (m) behind its start and turned by BASE_TURNS (deg), for every joint
# BASE_MARGIN (rad) inside its limits.
BASE_SPEED, BASE_TURN, BASE_GAIN = .15, .4, 2.
BASE_TOLERANCE, BASE_SETTLE = (.005, np.radians(1.)), .5
BASE_BACKS = (0., .05, .10, .15, .20)
BASE_TURNS = (0., 10., -10., 20., -20.)
BASE_MARGIN = .25
MAX_PLACES = 8          # placements whose motion place plans
MAX_PLACE_IK = 150      # placements place checks by IK
ROOM_PLACES = 10        # first placements the base moves to reach when none is in reach
PLACE_STEP = .02        # m between the positions place samples in its region
PLACE_TURN_STEP = 15.   # deg between the turns place samples
PLACE_TURNS = 4         # turns place tries per position, the least first
PLACE_MARGIN = .01      # m the footprint keeps inside the region and off its keep-out areas
RETREATS = (APPROACH, .03)   # m, back-off after releasing, tried in order
PARTIALS = (.75, .5, .25)     # fractions of a straight move tried when all of it does not plan
SHORT_RETREAT = .02     # m back along the approach that takes the fingers off what they held
WHY_LIMIT = 20          # rejected grasps per choice whose collisions are counted for the report
PUSH_GAP = .03          # m, push starts with the fingertips this far in front of the face
PUSH_TILTS = np.radians([0., 20., 40., 60.])   # push approaches, tilted down from the push direction


def collisions(error):
    """The collisions a :class:`PlanningFailed` reports (of its path, start or goal)."""
    plan = error.details.get('plan', {})
    return {key: plan[key] for key in ('collisions', 'start_collisions', 'goal_collisions') if plan.get(key)}


def normalized(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def planning_object(name, key, shape, offset=np.eye(4)):
    """Planning object ``name`` of ``shape`` moving with body ``key``."""
    if isinstance(shape, Box):
        entry = (np.asarray(offset, dtype=float), 'box', tuple(shape.size))
    else:
        entry = (np.asarray(offset, dtype=float), 'cylinder', (shape.radius, shape.length))
    return PlanningObject(name, key, [entry], padded=False)


@dataclass
class Target:
    """What a grasp holds: planning objects ``names``, shaped (for the grasp) by ``shape`` at
    ``offset`` in the frame of body ``body`` (a key of ``ctx.poses()``); ``points`` are the whole
    held object's collision points in that frame (for placing it)."""
    obj: str
    names: list
    body: str
    shape: object
    offset: np.ndarray
    points: np.ndarray


class Skills:
    """The skills of ``ctx``'s arms (``arms``: names, default all); see the module docstring."""

    def __init__(self, ctx, arms=None):
        self.ctx, self.model = ctx, ctx.model
        self.arms = list(arms or ctx.spec.arms)
        self.solvers = {arm: ctx.solver(arm) for arm in self.arms}
        self.hands = {arm: Hand(self.model, arm) for arm in self.arms}
        for solver in self.solvers.values():
            if 'table' not in solver.world.objects:
                solver.add_table()
            for name in ctx.scene.OBJECTS:
                if not any(key.startswith(name + ':') for key in solver.world.objects):
                    # Parts of one asset touch or overlap each other (a cap on a pen, a drawer in its
                    # housing); only their contacts with the robot and other objects are planned.
                    # A part that is about convex (or too thin to get into) is checked as one hull.
                    parts = [obj.name for obj in solver.add_object(name, lambda body, part: (part, body),
                                                                   merged=ctx.geometry[name].solid)]
                    solver.world.allow(parts, parts)
        self.home = self.model.q.copy()     # the arms' poses at the start, to return to (see ready)
        self.start_base = self.base_pose() if ctx.spec.base else None
        self.held = {}      # arm -> {'target', 'supports', 'grasp'}
        self.log, self.rejected = [], {}

    # -- what is grasped -----------------------------------------------------------------
    def target(self, obj, part=None, shape=None, offset=None, body=None):
        """The :class:`Target` of scene object ``obj`` (an asset, or a primitive with ``shape``):
        ``part`` (a part or body name, default all of it), shaped by its collision box in the frame
        of ``body`` (default the first body of the part), or by ``shape`` at ``offset`` there."""
        ctx = self.ctx
        offset = np.eye(4) if offset is None else np.asarray(offset, dtype=float)
        if obj not in ctx.scene.OBJECTS:
            if shape is None:
                raise ValueError(f'{obj!r} is not an asset of the scene: give its shape')
            for solver in self.solvers.values():
                if obj not in solver.world.objects:
                    solver.world.add(planning_object(obj, obj, shape, offset), ctx.poses()[obj])
            points = shape.points() @ offset[:3, :3].T + offset[:3, 3]
            return Target(obj, [obj], obj, shape, offset, points)
        geometry = ctx.geometry[obj]
        parts = geometry.parts(part)
        if not parts:
            raise KeyError(f'{obj} has no part {part!r}')
        body = body or parts[0][0]
        if shape is None:
            low, high = geometry.bounds(part, body)
            shape, offset = Box(tuple(high - low)), np.eye(4)
            offset[:3, 3] = (low + high) / 2
        return Target(obj, [f'{obj}:{name}' for _, name in parts], f'{obj}:{body}', shape, offset,
                      geometry.points(None, body))

    def holding(self, obj):
        """The arm holding ``obj``, or None."""
        return next((arm for arm, held in self.held.items() if held['target'].obj == obj), None)

    # -- choice ------------------------------------------------------------------------
    def shoulder(self, arm):
        return self.model.fk(self.ctx.spec.arms[arm].joints[1].replace('joint', 'link'), world=True)[:3, 3]

    def obstacles(self, arm, target, transform):
        """``([(FCL object, clearance to keep)], supports)``: the objects of ``arm``'s planning world
        other than ``target``'s (its shape at ``transform``), and the names of those supporting it."""
        from mplib.collision_detection import fcl
        from table_1000.planning.world import geometry, pose
        self.solvers[arm].sync()
        world = self.solvers[arm].world
        shape = fcl.FCLObject('target', pose(transform), [fcl.CollisionObject(geometry(*target.shape.fcl_shape()))],
                              [pose(np.eye(4))])
        obstacles, supports = [], []
        for name, obj in world.objects.items():
            if name in target.names or world.planner.planning_world.is_object_attached(name):
                continue
            body = obj.fcl(world.poses[name])
            support = fcl.distance(shape, body).min_distance < SUPPORT_DISTANCE
            if support:
                supports.append(name)
            obstacles.append((body, SUPPORT_CLEARANCE if support else 0.))
        return obstacles, supports

    def options(self, target, arms, skip, point=None, approach=None, then=None, max_angle=None):
        """Feasible ``(arm, grasp, supports, q_grasp, q_pre)``, in the order tried; ``skip`` holds the
        ids of candidates already used. ``self.rejected`` counts the candidates dropped, by reason."""
        self.rejected = {'grasp IK': 0, 'pre-grasp IK': 0, 'then': 0, 'collisions': {}}
        transform = self.ctx.poses()[target.body] @ target.offset
        centre = transform[:3, 3]
        then = None if then is None else np.asarray(then, dtype=float)
        upward = then is not None and np.linalg.norm(then[:2]) < 1e-9 and then[2] > 0.
        for arm in sorted(arms, key=lambda arm: np.linalg.norm((self.shoulder(arm) - centre)[:2])):
            solver, side = self.solvers[arm], self.ctx.spec.arms[arm]
            preferred = approach if approach is not None else normalized(centre - self.shoulder(arm)) + [0., 0., -1.]
            obstacles, supports = self.obstacles(arm, target, transform)
            candidates = grasp_candidates(target.shape, transform, self.hands[arm], obstacles, preferred,
                                          center_of_mass=point, approach_distance=APPROACH)
            checked = 0
            for index, grasp in enumerate(candidates):
                if (arm, index) in skip:
                    continue
                if checked >= MAX_IK or (max_angle is not None and grasp.info['approach_deg'] > np.degrees(max_angle) + 1e-6):
                    break
                checked += 1
                start = self.model.vector(side.finger_targets(grasp.open_gap))
                solver.sync()
                # The fingers touch the object and come closer to its supports than the padding.
                solver.allow([*target.names, *supports])
                status, q_grasp = self.ik(arm, grasp.pose, start)
                solver.allow([*target.names, *supports], False)
                if status != 'Success':
                    self.rejected['grasp IK'] += 1
                    if self.rejected['grasp IK'] <= WHY_LIMIT:
                        self.why(arm, grasp.pose, start, [*target.names, *supports])
                    continue
                status, q_pre = self.model.ik(arm, standoff(grasp.pose, APPROACH), q_grasp, world=True,
                                              attempts=IK_ATTEMPTS)
                if status != 'Success':
                    self.rejected['pre-grasp IK'] += 1
                    continue
                if then is not None:
                    status, _ = self.model.ik(arm, translated(grasp.pose, then), q_grasp, world=True, lift=True,
                                              attempts=IK_ATTEMPTS)
                    # Close to the body the hand may not reach higher; then the torso must rise that far.
                    headroom = self.model.limits[self.model.lift_indices(), 1] - q_grasp[self.model.lift_indices()]
                    if status != 'Success' and not (upward and np.min(headroom) >= then[2]):
                        self.rejected['then'] += 1
                        continue
                skip.add((arm, index))
                yield arm, grasp, supports, q_grasp, q_pre

    def ik(self, arm, pose, start):
        """IK of world ``pose`` keeping the torso lift where it is, else moving it too (a far lift
        counts little among the arm's angles, and a raised torso makes later plans longer)."""
        status, q = self.model.ik(arm, pose, start, world=True, attempts=IK_ATTEMPTS)
        if status == 'Success':
            return status, q
        return self.model.ik(arm, pose, start, world=True, lift=True, attempts=IK_ATTEMPTS)

    def why(self, arm, pose, start, allowed):
        """Count, in ``self.rejected['collisions']``, what an IK solution of ``pose`` that ignores
        collisions would collide with (``'no IK'`` when there is none)."""
        status, q = self.model.ik(arm, pose, start, world=True, lift=True, attempts=IK_ATTEMPTS, check_collision=False)
        if status != 'Success':
            keys = ['no IK']
        else:
            solver = self.solvers[arm]
            solver.allow(allowed)
            keys = sorted({f'{link} / {obj}' for link, obj, _ in self.model.collisions(arm, q)}) or ['none']
            solver.allow(allowed, False)
        counts = self.rejected['collisions']
        for key in keys:
            counts[key] = counts.get(key, 0) + 1

    # -- skills --------------------------------------------------------------------------
    def grasp(self, target, arm=None, point=None, approach=None, then=None, max_angle=None):
        """Grasp ``target`` (:class:`Target`) with ``arm`` (default either); see the module docstring.
        Returns ``{'arm', 'grasp', 'supports', 'finger_gap', 'regrasps', 'choice_s', 'tcp_object'}``."""
        ctx = self.ctx
        arms = [arm] if arm else [a for a in self.arms if a not in self.held]
        skip, regrasps, started = set(), 0, time.perf_counter()
        while True:
            reached = self.reach(target, arms, skip, point, approach, then, max_angle)
            if reached is None:
                ctx.fail('no reachable grasp', target=target.names, candidates_tried=len(skip), rejected=self.rejected)
            arm, grasp, supports = reached
            solver = self.solvers[arm]
            solver.close_gripper()
            gap = solver.finger_gap()
            if gap >= grasp.info['contact_gap'] - HELD_TOLERANCE:
                break
            # Missed: back out, re-observe the object and choose again.
            ctx.note('missed grasp', arm=arm, finger_gap=gap, width=grasp.width, pose=grasp.pose.tolist(), **grasp.info)
            solver.open_gripper(grasp.open_gap)
            solver.first_reachable([standoff(solver.tcp(), APPROACH), translated(solver.tcp(), [0., 0., APPROACH])])
            solver.allow([*target.names, *supports], False)
            regrasps += 1
            if regrasps > MAX_REGRASPS:
                ctx.fail('object not held', finger_gap=gap, width=grasp.width)
        poses = ctx.poses()
        for name in target.names:
            solver.attach(name, poses[solver.world.objects[name].body])
        chosen = {'pose': grasp.pose.tolist(), 'open_gap': grasp.open_gap, 'width': grasp.width,
                  'backoff': grasp.backoff, 'cost': grasp.cost, **grasp.info}
        self.held[arm] = {'target': target, 'supports': supports, 'grasp': chosen}
        ctx.note('grasped', arm=arm, target=target.names[0], approach=grasp.pose[:3, 0].tolist(),
                 closing=grasp.pose[:3, 1].tolist(), finger_gap=solver.finger_gap(), **grasp.info)
        result = {'arm': arm, 'grasp': chosen, 'supports': supports, 'finger_gap': solver.finger_gap(),
                  'regrasps': regrasps, 'choice_s': time.perf_counter() - started,
                  'tcp_object': (np.linalg.inv(solver.tcp()) @ poses[target.body]).tolist()}
        self.log.append(result)
        return result

    def reach(self, target, arms, skip, point, approach, then, max_angle=None):
        """Move to the first grasp of :meth:`options` whose motion plans, fingers open around the
        object; ``(arm, grasp, supports)``, or None when no candidate is left."""
        ctx = self.ctx
        for tried, (arm, grasp, supports, q_grasp, q_pre) in enumerate(
                self.options(target, arms, skip, point, approach, then, max_angle)):
            if tried >= MAX_PLANS:
                return None
            solver = self.solvers[arm]
            try:
                solver.open_gripper(grasp.open_gap)
                solver.move_to_pose_with_RRTConnect(None, lift=True, goals=[q_pre])
                solver.allow([*target.names, *supports])
                solver.move_to_pose_with_screw(grasp.pose)
                return arm, grasp, supports
            except PlanningFailed as error:
                solver.allow([*target.names, *supports], False)
                ctx.note('next grasp', arm=arm, cost=grasp.cost, reason=error.reason, **collisions(error))
        return None

    def pick(self, target, arm=None, point=None, approach=None, height=LIFT):
        """:meth:`grasp` with the lift checked, then up by ``height``; the object must have risen."""
        ctx = self.ctx
        result = self.grasp(target, arm, point, approach, then=(0., 0., height))
        before = ctx.poses()[target.body]
        self.move_by_displacement(result['arm'], (0., 0., height))
        ctx.hold(.2)
        after = ctx.poses()[target.body]
        rise = float(after[2, 3] - before[2, 3])
        if rise < MIN_RISE:
            self.release(result['arm'])     # so that it can be picked again
            ctx.fail('object not lifted', rise=rise, finger_gap=self.solvers[result['arm']].finger_gap(),
                     arm=result['arm'], grasp=result['grasp'], object_before=before.tolist())
        result.update(rise=rise, finger_gap=self.solvers[result['arm']].finger_gap(),
                      tcp_object=(np.linalg.inv(self.solvers[result['arm']].tcp()) @ after).tolist())
        return result

    def move_by_displacement(self, arm, displacement, drag=None, least=1.):
        """The TCP straight by world ``displacement`` (or a part of it down to ``least``, see
        :meth:`straight`); ``drag``: ``(object, body)`` moved through a joint by the held part.
        Supports of the held object may be touched in this move only. Returns the fraction moved."""
        solver = self.solvers[arm]
        displacement = np.asarray(displacement, dtype=float)
        held = self.held.get(arm)
        supports = held['supports'] if held else []
        dragged, riders, contacts = self.dragged(arm, drag) if drag else ([], [], [])
        if dragged:
            solver.carry(dragged, contacts + riders)
            solver.carry(riders, contacts)
        try:
            return self.straight(arm, displacement, least)
        finally:
            if dragged:
                solver.release(riders, contacts)
                solver.release(dragged, contacts + riders)
            if held:
                solver.allow(supports, False)
                held['supports'] = []

    def dragged(self, arm, drag):
        """``(own, riders, contacts)``: the planning objects of body ``drag`` (``(object, body)``) not
        yet held, every part of the other objects resting on or against it (a pen in a drawer rides
        along), and what they may touch besides each other and the rest of their objects: the
        objects within ``SUPPORT_DISTANCE`` of them."""
        from mplib.collision_detection import fcl
        solver = self.solvers[arm]
        solver.sync()
        world, key = solver.world, f'{drag[0]}:{drag[1]}'
        attached = world.planner.planning_world.is_object_attached
        names = [name for name, obj in world.objects.items() if obj.body == key and not attached(name)]
        bodies = [world.objects[name].fcl(world.poses[name]) for name in names]
        def touching(names, bodies, skip):
            return [name for name, obj in world.objects.items()
                    if name not in names and not attached(name) and not name.startswith(skip) and any(
                        fcl.distance(body, obj.fcl(world.poses[name])).min_distance < SUPPORT_DISTANCE
                        for body in bodies)]

        objects = {name.split(':')[0] for name in touching(names, bodies, drag[0] + ':') if ':' in name}
        riders = [name for name in world.objects
                  if ':' in name and name.split(':')[0] in objects and not attached(name)]
        carried = names + riders
        bodies = [world.objects[name].fcl(world.poses[name]) for name in carried]
        return names, riders, [name for name in touching(carried, bodies, drag[0] + ':')
                               if name.split(':')[0] not in objects]

    def straight(self, arm, displacement, least=1.):
        """Straight by ``displacement`` with the arm, else (``least`` < 1) by the largest of
        ``PARTIALS`` of it down to ``least`` that plans, else with the arm and the torso lift; straight
        up out of their reach (the arm may hit the body close to it), the torso lift rises, then
        the arm does the rest, straight or along a planned path. Returns the fraction moved."""
        solver, ctx = self.solvers[arm], self.ctx
        start = solver.tcp()

        try:
            solver.move_to_pose_with_screw(translated(start, displacement))
            return 1.
        except PlanningFailed as error:
            failed = error
        # Six arm joints alone may run into a limit along the line: part of it, else all of it with
        # the torso lift as well (which follows its path less closely).
        options = [(f, False) for f in PARTIALS if f >= least] + [(1., True)]
        # (A limit or the body, an elbow by the torso, may also be hit only near the end of the line.)
        if failed.reason in ('joint_limit', 'no_progress', 'iteration_limit', 'collision'):
            for fraction, lift in options:
                try:
                    solver.move_to_pose_with_screw(translated(start, displacement * fraction), lift=lift)
                    if fraction < 1.:
                        ctx.note('moved part way', fraction=fraction, reason=failed.reason)
                    return fraction
                except PlanningFailed:
                    pass
        if np.linalg.norm(displacement[:2]) > 1e-9 or displacement[2] <= 0.:
            raise failed
        straight = {'reason': failed.reason, 'collisions': failed.details.get('plan', {}).get('collisions')}
        rise = float(displacement[2])
        lift = ctx.env.qpos(ctx.spec.lift)[0]
        upper = self.model.limits[self.model.lift_indices(), 1]
        target = np.minimum(lift + rise, upper)
        ctx.note('rise with the torso', **straight, lift=lift.tolist(), target=target.tolist())
        count = steps(float(np.max(target - lift)) / ctx.spec.lift_velocity_limit)
        for k in range(1, count + 1):
            ctx.step({'lift': (lift + (target - lift) * k / count)[None]})
        ctx.hold(.3)
        rest = rise - float(np.min(target - lift))
        if rest < .005:
            return 1.
        up = translated(solver.tcp(), [0., 0., rest])
        held = self.held.get(arm)
        if held:
            solver.allow(held['supports'], False)
        for move in (solver.move_to_pose_with_screw, lambda pose: solver.move_to_pose_with_RRTConnect(pose, lift=True)):
            try:
                move(up)
                return 1.
            except PlanningFailed as error:
                reason, collisions = error.reason, error.details.get('plan', {}).get('collisions')
        ctx.note('rise short', rest=rest, reason=reason, collisions=collisions)
        return 1. - rest / rise

    def push(self, target, direction, distance, arm=None):
        """Push ``target`` (:class:`Target`) by ``distance`` along world ``direction``; see the
        module docstring. Returns ``{'arm', 'tilt', 'moved'}`` (how far its body went)."""
        ctx = self.ctx
        direction = normalized(np.asarray(direction, dtype=float))
        transform = ctx.poses()[target.body] @ target.offset
        face = transform[:3, 3] + direction * float((target.shape.points() @ transform[:3, :3].T @ direction).min())
        level = normalized(direction - [0., 0., direction[2]])
        side = normalized(np.cross([0., 0., 1.], level))
        arms = [arm] if arm else [a for a in self.arms if a not in self.held]
        options = []
        for candidate in sorted(arms, key=lambda a: np.linalg.norm((self.shoulder(a) - face)[:2])):
            tip = self.hands[candidate].tip()
            for tilt in PUSH_TILTS:
                approach = normalized(level * np.cos(tilt) - np.array([0., 0., 1.]) * np.sin(tilt))
                for closing in (side, normalized(np.cross(approach, side))):
                    pose = np.eye(4)
                    pose[:3, :3] = np.c_[approach, closing, np.cross(approach, closing)]
                    pose[:3, 3] = face - approach * tip
                    options.append((candidate, tilt, pose))
        before = ctx.poses()[target.body][:3, 3]
        for candidate, tilt, contact in options:
            solver = self.solvers[candidate]
            start = self.model.vector(ctx.spec.arms[candidate].finger_targets(0.))
            status, q_pre = self.ik(candidate, translated(contact, -direction * PUSH_GAP), start)
            if status != 'Success':
                continue
            status, _ = self.model.ik(candidate, translated(contact, direction * distance), q_pre, world=True,
                                      attempts=IK_ATTEMPTS, check_collision=False)
            if status != 'Success':
                continue
            # The fingertips push the part and may touch the rest of its body (a drawer's front).
            touched = [name for name, obj in solver.world.objects.items() if obj.body == target.body] or target.names
            try:
                solver.close_gripper()
                solver.move_to_pose_with_RRTConnect(None, lift=True, goals=[q_pre])
                solver.allow(touched)
                solver.move_to_pose_with_screw(contact)
                break
            except PlanningFailed as error:
                solver.allow(touched, False)
                ctx.note('next push', arm=candidate, tilt=float(tilt), reason=error.reason, **collisions(error))
        else:
            ctx.fail('no reachable push', target=target.names)
        try:
            self.move_by_displacement(candidate, direction * distance, drag=(target.obj, target.body.split(':', 1)[1]))
        finally:
            solver.allow(touched, False)
        moved = float((ctx.poses()[target.body][:3, 3] - before) @ direction)
        # Back off: straight back or up, else along a planned path. The push is done either way.
        tcp = solver.tcp()
        moves = [lambda pose=pose: solver.move_to_pose_with_screw(pose)
                 for pose in [translated(tcp, -direction * d) for d in RETREATS] + [translated(tcp, [0., 0., d]) for d in RETREATS]]
        moves.append(lambda: solver.move_to_pose_with_RRTConnect(translated(tcp, -direction * RETREATS[0]), lift=True))
        try:
            solver.first_reachable(moves, lambda move: move())
        except PlanningFailed as error:
            ctx.note('no retreat', arm=candidate, reason=error.reason, **collisions(error))
        return {'arm': candidate, 'tilt': float(tilt), 'moved': moved}

    def release(self, arm):
        """Open the fingers to the grasp's opening and back out along the approach (straight, else
        planned), or up."""
        held = self.held.pop(arm)
        solver = self.solvers[arm]
        solver.open_gripper(held['grasp']['open_gap'])
        for name in held['target'].names:
            solver.detach(name)
        solver.sync()
        tcp = solver.tcp()
        # Straight out (with the torso lift too, if the arm alone runs into a limit or the body),
        # else out along a planned path, else out just clear of the fingers' grip and up, else up.
        def out_and_up():
            solver.move_to_pose_with_screw(standoff(tcp, SHORT_RETREAT))
            solver.first_reachable([translated(solver.tcp(), [0., 0., d]) for d in RETREATS])

        moves = [lambda d=d, lift=lift: solver.move_to_pose_with_screw(standoff(tcp, d), lift=lift)
                 for d in RETREATS for lift in (False, True)]
        moves.append(lambda: solver.move_to_pose_with_RRTConnect(standoff(tcp, RETREATS[0]), lift=True))
        moves.append(out_and_up)
        moves += [lambda d=d: solver.move_to_pose_with_screw(translated(tcp, [0., 0., d])) for d in RETREATS]
        try:
            solver.first_reachable(moves, lambda move: move())
        finally:
            solver.allow([*held['target'].names, *held['supports']], False)

    def placements(self, target, current, region, near, keep_out, step, margin):
        """``[(object pose at rest on the surface, spot)]`` in ``region`` (see :meth:`place`), the
        footprint at least ``margin`` inside it and out of ``keep_out``, nearest ``near`` first, then
        the least turned from ``current``."""
        frame, low, high = (np.asarray(item, dtype=float) for item in region)
        local = np.linalg.inv(frame)
        boxes = [(np.linalg.inv(np.asarray(f, dtype=float)), np.asarray(l, dtype=float)[:2] - margin,
                  np.asarray(h, dtype=float)[:2] + margin) for f, l, h in keep_out]
        options = []
        for turn in np.radians(np.arange(0., 360., PLACE_TURN_STEP)):
            # Turned about the surface normal, in the region frame.
            rotation = frame[:3, :3] @ Rotation.from_euler('z', turn).as_matrix() @ frame[:3, :3].T @ current[:3, :3]
            points = (target.points @ rotation.T) @ local[:3, :3].T     # in the region frame, about the origin
            centre = (points[:, :2].min(0) + points[:, :2].max(0)) / 2
            spread = np.ptp(points[:, :2], axis=0) / 2
            first, last = low[:2] + spread + margin, high[:2] - spread - margin
            if np.any(first > last + 1e-9):
                continue
            for x in np.arange(first[0], last[0] + 1e-9, step):
                for y in np.arange(first[1], last[1] + 1e-9, step):
                    spot = frame @ [x, y, 0., 1.]
                    footprint = points[:, :2] - centre + [x, y]
                    world = (frame @ np.c_[footprint, np.zeros(len(footprint)), np.ones(len(footprint))].T).T
                    if any(np.any(np.all((low_ <= (inv @ world.T).T[:, :2]) & ((inv @ world.T).T[:, :2] <= high_), axis=1))
                           for inv, low_, high_ in boxes):
                        continue
                    pose = np.eye(4)
                    pose[:3, :3] = rotation
                    pose[:3, 3] = frame[:3, :3] @ np.r_[[x, y] - centre, -points[:, 2].min()] + frame[:3, 3]
                    turned = Rotation.from_matrix(rotation @ current[:3, :3].T).magnitude()
                    options.append((float(np.linalg.norm((spot[:3] - near)[:2])), turned, pose, spot[:3]))
        # Nearest first, at most PLACE_TURNS of the least turned per position: IK checks spread over
        # positions rather than spending themselves on the turns of a few.
        options.sort(key=lambda option: (round(option[0] / step), option[1]))
        kept, counts = [], {}
        for distance, turned, pose, spot in options:
            key = tuple(np.round(spot[:2] / step).astype(int))
            counts[key] = counts.get(key, 0) + 1
            if counts[key] <= PLACE_TURNS:
                kept.append((pose, spot))
        return kept

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
        (default all) of the world TCP ``poses`` with each joint ``BASE_MARGIN`` inside its limits
        (``path``: the poses of one straight move, each solved from the one before at the torso
        height of the first);
        the least move that does, else the one reaching most, with the widest margin."""
        model = self.model
        arms = list(arms) if arms else [a for a in self.arms if a not in self.held]
        need = len(poses) if need is None else need
        actual = model.base.copy()
        best, most = None, (-1, -np.inf)
        try:
            for back, turn in sorted(((b, t) for b in BASE_BACKS for t in np.radians(BASE_TURNS)),
                                     key=lambda option: option[0] / .05 + abs(option[1]) / np.radians(10.)):
                model.base = self.base_at(back, turn)
                for arm in arms:
                    self.solvers[arm].sync()
                    indices = model.arm_indices(arm)
                    # Along a path, each pose from the solution of the one before (one branch) with the
                    # torso lift where the first put it (a straight move moves the arm alone).
                    margins, q = [], None
                    for index, pose in enumerate(poses):
                        chained = path and index > 0
                        if chained and q is None:
                            margins.append(-np.inf)
                            continue
                        status, q = model.ik(arm, pose, q if path else None, world=True, lift=not chained,
                                             attempts=IK_ATTEMPTS)
                        q = None if status != 'Success' else q
                        margins.append(-np.inf if status != 'Success' else float(np.min(np.minimum(
                            q[indices] - model.limits[indices, 0], model.limits[indices, 1] - q[indices]))))
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

    def put_down(self, arm, options, tcp_object, clearance, above):
        """Move the held object down at the first of ``options`` (``(object pose, spot)``) in reach
        (IK above and at the release) whose motion plans; ``{'done', 'spot', 'ik_checked',
        'planned', 'last', ...collisions}``."""
        ctx, solver = self.ctx, self.solvers[arm]
        result = {'done': False, 'spot': None, 'ik_checked': 0, 'planned': 0, 'last': 'no placement in reach'}
        for placed, spot in options:
            if result['ik_checked'] >= MAX_PLACE_IK or result['planned'] >= MAX_PLACES:
                break
            result['ik_checked'] += 1
            release = translated(placed, [0., 0., clearance]) @ np.linalg.inv(tcp_object)
            # Where it is let go (the hand and arm low by a wall), then above it at that torso height:
            # the way down moves the arm alone.
            status, q_release = self.ik(arm, release, self.model.q)
            if status != 'Success':
                continue
            status, q_above = self.model.ik(arm, translated(release, [0., 0., above]), q_release, world=True,
                                            attempts=IK_ATTEMPTS)
            if status != 'Success':
                continue
            result['planned'] += 1
            try:
                solver.move_to_pose_with_RRTConnect(None, lift=True, goals=[q_above])
                solver.move_to_pose_with_screw(release)
                result.update(done=True, spot=spot)
                return result
            except PlanningFailed as error:
                result.update(last=error.reason, **collisions(error))
                ctx.note('next placement', arm=arm, spot=spot.tolist(), reason=error.reason, **collisions(error))
        return result

    def place(self, obj, region, arm=None, near=None, keep_out=(), clearance=PLACE_CLEARANCE, above=PLACE_ABOVE,
              step=PLACE_STEP, margin=PLACE_MARGIN):
        """Put the held ``obj`` down in ``region`` and release it; see the module docstring.
        ``region`` is ``(frame, low, high)``: the rectangle ``low``-``high`` (x, y) on the surface
        z = 0 of the world 4x4 ``frame``; ``keep_out`` are such rectangles (any height) the
        footprint stays out of; ``near`` the world point to place it nearest (default the
        region's centre). Returns ``{'arm', 'spot', 'object'}`` (its pose after)."""
        ctx = self.ctx
        arm = arm or self.holding(obj)
        if arm is None or arm not in self.held or self.held[arm]['target'].obj != obj:
            raise ValueError(f'{obj!r} is not held')
        solver, target = self.solvers[arm], self.held[arm]['target']
        current = ctx.poses()[target.body]
        tcp_object = np.linalg.inv(solver.tcp()) @ current
        frame, low, high = (np.asarray(item, dtype=float) for item in region)
        if near is None:
            near = frame @ np.r_[(low[:2] + high[:2]) / 2, 0., 1.]
        options = self.placements(target, current, region, np.asarray(near, dtype=float)[:3], keep_out, step, margin)
        if not options:
            ctx.fail('no room', region=[frame.tolist(), low.tolist(), high.tolist()])
        # IK first (cheap) over many, then motion for those that have it. Where none is in reach,
        # the base moves to reach the first ones, and they are screened again.
        result = self.put_down(arm, options, tcp_object, clearance, above)
        if result['planned'] == 0 and ctx.spec.base:
            releases = [translated(placed, [0., 0., clearance]) @ np.linalg.inv(tcp_object)
                        for placed, _ in options[:ROOM_PLACES]]
            self.make_room(releases, [arm], need=1)
            current = ctx.poses()[target.body]
            options = self.placements(target, current, region, np.asarray(near, dtype=float)[:3], keep_out, step, margin)
            result = self.put_down(arm, options, tcp_object, clearance, above)
        if not result['done']:
            ctx.fail('no placement', arm=arm, options=len(options), **{k: v for k, v in result.items() if k != 'done'})
        spot = result['spot']
        self.release(arm)
        ctx.hold(.3)
        return {'arm': arm, 'spot': spot.tolist(), 'object': ctx.poses()[target.body].tolist()}


def _skills(ctx):
    return ctx.skills()


def make_room(ctx, poses, arms=None, path=False):
    """Move the base to where the world TCP ``poses`` are reachable with a margin (see :meth:`Skills.make_room`)."""
    _skills(ctx).make_room(poses, arms, path=path)


def retry(ctx, step, attempts=ATTEMPTS, budget=RETRY_BUDGET):
    """``step()``, again up to ``attempts`` times in all when it fails (each choice of grasp, path
    and spot starts afresh from where things are), the free arms back at their start pose first;
    not once the episode has run ``budget`` s (wall clock)."""
    for attempt in range(1, attempts + 1):
        try:
            return step()
        except ExpertFailed as failure:
            if attempt == attempts or ctx.elapsed() > budget:
                raise
            ctx.note('retry', attempt=attempt, reason=failure.reason)
            _skills(ctx).ready()


def grasp(ctx, obj, part=None, *, arm=None, shape=None, offset=None, body=None, point=None, approach=None, then=None,
          max_angle=None):
    """Grasp scene object ``obj`` (or its ``part``); see the module docstring. ``max_angle`` (rad): only
    grasps approaching within it of ``approach``."""
    skills = _skills(ctx)
    return skills.grasp(skills.target(obj, part, shape, offset, body), arm, point, approach, then, max_angle)


def pick(ctx, obj, part=None, *, arm=None, shape=None, offset=None, body=None, point=None, approach=None, height=LIFT):
    """Grasp scene object ``obj`` (or its ``part``) and lift it by ``height``."""
    skills = _skills(ctx)
    return skills.pick(skills.target(obj, part, shape, offset, body), arm, point, approach, height)


def move_by_displacement(ctx, arm, x=0., y=0., z=0., drag=None, least=1.):
    """Move ``arm``'s TCP straight by ``(x, y, z)`` (world), or part way down to the fraction
    ``least``; ``drag``: ``(object, body)`` moved by the held part. Returns the fraction moved."""
    return _skills(ctx).move_by_displacement(arm, (x, y, z), drag, least)


def push(ctx, obj, part=None, *, direction, distance, arm=None, shape=None, offset=None, body=None):
    """See :meth:`Skills.push`; the target as for :func:`grasp`."""
    skills = _skills(ctx)
    return skills.push(skills.target(obj, part, shape, offset, body), direction, distance, arm)


def release(ctx, arm):
    """Let go of what ``arm`` holds and back out."""
    _skills(ctx).release(arm)


def place(ctx, obj, region, *, arm=None, near=None, keep_out=(), clearance=PLACE_CLEARANCE, above=PLACE_ABOVE,
          step=PLACE_STEP, margin=PLACE_MARGIN):
    """Put the held ``obj`` down in ``region`` (see :meth:`Skills.place`) and release it."""
    return _skills(ctx).place(obj, region, arm, near, keep_out, clearance, above, step, margin)


def open_gripper(ctx, arm, gap=None):
    solver = ctx.solver(arm)
    solver.open_gripper() if gap is None else solver.open_gripper(gap)


def close_gripper(ctx, arm):
    ctx.solver(arm).close_gripper()
