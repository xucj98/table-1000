"""Atomic skills for experts: grasp, pick, move by a displacement, release, place.

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
    skills.place(ctx, 'pen', [(x, y, table_height)])

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

**move_by_displacement** -- the TCP straight by a world displacement; straight up
beyond the arm's reach, the torso lift rises and the arm does the rest. ``drag``
names a body moved through a joint by the held part (a drawer pulled by its
handle): its planning objects move with the hand and may touch the rest of their
object and whatever rests on them.

**release** -- open the fingers to the grasp's opening and back out along the
approach (or up).

**place** -- put the held object down at the first of ``spots`` (world points:
the centre of its footprint and the height of the surface under it) and turn about
the vertical that plans: above it, straight down to ``clearance`` over the surface,
then :func:`release`. The object keeps its orientation, turned to ``heading`` (the
world direction of its body's X axis, if given) and by each of ``turns``.

Each choice of another candidate is noted (``ctx.note``).
"""

from dataclasses import dataclass
import time

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.experts.context import steps
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
RETREATS = (APPROACH, .03)   # m, back-off after releasing, tried in order
WHY_LIMIT = 20          # rejected grasps per choice whose collisions are counted for the report


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
                    solver.add_object(name, lambda body, part: (part, body))
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

    def options(self, target, arms, skip, point=None, approach=None, then=None):
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
                if checked >= MAX_IK:
                    break
                checked += 1
                start = self.model.vector(side.finger_targets(grasp.open_gap))
                solver.sync()
                # The fingers touch the object and come closer to its supports than the padding.
                solver.allow([*target.names, *supports])
                status, q_grasp = self.model.ik(arm, grasp.pose, start, world=True, lift=True, attempts=IK_ATTEMPTS)
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
    def grasp(self, target, arm=None, point=None, approach=None, then=None):
        """Grasp ``target`` (:class:`Target`) with ``arm`` (default either); see the module docstring.
        Returns ``{'arm', 'grasp', 'supports', 'finger_gap', 'regrasps', 'choice_s', 'tcp_object'}``."""
        ctx = self.ctx
        arms = [arm] if arm else [a for a in self.arms if a not in self.held]
        skip, regrasps, started = set(), 0, time.perf_counter()
        while True:
            reached = self.reach(target, arms, skip, point, approach, then)
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

    def reach(self, target, arms, skip, point, approach, then):
        """Move to the first grasp of :meth:`options` whose motion plans, fingers open around the
        object; ``(arm, grasp, supports)``, or None when no candidate is left."""
        ctx = self.ctx
        for tried, (arm, grasp, supports, q_grasp, q_pre) in enumerate(
                self.options(target, arms, skip, point, approach, then)):
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
                ctx.note('next grasp', arm=arm, cost=grasp.cost, reason=error.reason)
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
            ctx.fail('object not lifted', rise=rise, finger_gap=self.solvers[result['arm']].finger_gap(),
                     arm=result['arm'], grasp=result['grasp'], object_before=before.tolist())
        result.update(rise=rise, finger_gap=self.solvers[result['arm']].finger_gap(),
                      tcp_object=(np.linalg.inv(self.solvers[result['arm']].tcp()) @ after).tolist())
        return result

    def move_by_displacement(self, arm, displacement, drag=None):
        """The TCP straight by world ``displacement``; ``drag``: ``(object, body)`` moved through a
        joint by the held part. Supports of the held object may be touched in this move only."""
        solver = self.solvers[arm]
        displacement = np.asarray(displacement, dtype=float)
        held = self.held.get(arm)
        supports = held['supports'] if held else []
        dragged, contacts = self.dragged(arm, drag) if drag else ([], [])
        if dragged:
            solver.carry(dragged, contacts)
        try:
            self.straight(arm, displacement)
        finally:
            if dragged:
                solver.release(dragged, contacts)
            if held:
                solver.allow(supports, False)
                held['supports'] = []

    def dragged(self, arm, drag):
        """Planning objects of body ``drag`` (``(object, body)``) not yet held, and what they may
        touch: the rest of their object and the objects within ``SUPPORT_DISTANCE`` of them."""
        from mplib.collision_detection import fcl
        solver = self.solvers[arm]
        solver.sync()
        world, key = solver.world, f'{drag[0]}:{drag[1]}'
        attached = world.planner.planning_world.is_object_attached
        names = [name for name, obj in world.objects.items() if obj.body == key and not attached(name)]
        bodies = [world.objects[name].fcl(world.poses[name]) for name in names]
        # The dragged parts touch each other and the held part they hang on.
        contacts = list(names) + [name for name in world.objects if attached(name)]
        for name, obj in world.objects.items():
            if name in names or attached(name):
                continue
            if name.startswith(drag[0] + ':') or any(
                    fcl.distance(body, obj.fcl(world.poses[name])).min_distance < SUPPORT_DISTANCE for body in bodies):
                contacts.append(name)
        return names, contacts

    def straight(self, arm, displacement):
        """Straight by ``displacement``; straight up out of the arm's reach (it may hit the body
        close to it), the torso lift rises, then the arm does the rest, straight or along a
        planned path."""
        solver, ctx = self.solvers[arm], self.ctx
        try:
            solver.move_to_pose_with_screw(translated(solver.tcp(), displacement))
            return
        except PlanningFailed as error:
            if np.linalg.norm(displacement[:2]) > 1e-9 or displacement[2] <= 0.:
                raise
            straight = {'reason': error.reason, 'collisions': error.details.get('plan', {}).get('collisions')}
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
            return
        up = translated(solver.tcp(), [0., 0., rest])
        held = self.held.get(arm)
        if held:
            solver.allow(held['supports'], False)
        for move in (solver.move_to_pose_with_screw, lambda pose: solver.move_to_pose_with_RRTConnect(pose, lift=True)):
            try:
                move(up)
                return
            except PlanningFailed as error:
                reason, collisions = error.reason, error.details.get('plan', {}).get('collisions')
        ctx.note('rise short', rest=rest, reason=reason, collisions=collisions)

    def release(self, arm):
        """Open the fingers to the grasp's opening and back out along the approach, or up."""
        held = self.held.pop(arm)
        solver = self.solvers[arm]
        solver.open_gripper(held['grasp']['open_gap'])
        for name in held['target'].names:
            solver.detach(name)
        solver.sync()
        tcp = solver.tcp()
        try:
            solver.first_reachable([standoff(tcp, d) for d in RETREATS] + [translated(tcp, [0., 0., d]) for d in RETREATS])
        finally:
            solver.allow([*held['target'].names, *held['supports']], False)

    def place(self, obj, spots, arm=None, heading=None, turns=(0., np.pi), clearance=PLACE_CLEARANCE,
              above=PLACE_ABOVE):
        """Put the held ``obj`` down at the first of ``spots`` and ``turns`` that plans, release it;
        see the module docstring. Returns ``{'arm', 'spot', 'turn', 'object'}`` (its pose after)."""
        ctx = self.ctx
        arm = arm or self.holding(obj)
        if arm is None or arm not in self.held or self.held[arm]['target'].obj != obj:
            raise ValueError(f'{obj!r} is not held')
        solver, target = self.solvers[arm], self.held[arm]['target']
        current = ctx.poses()[target.body]
        tcp_object = np.linalg.inv(solver.tcp()) @ current
        delta = 0.
        if heading is not None:
            axis = current[:3, 0]
            delta = float(np.arctan2(heading[1], heading[0]) - np.arctan2(axis[1], axis[0]))
        options = [(np.asarray(spot, dtype=float), turn) for spot in spots for turn in turns]
        for index, (spot, turn) in enumerate(options):
            rotation = Rotation.from_euler('z', delta + turn).as_matrix() @ current[:3, :3]
            points = target.points @ rotation.T
            placed = np.eye(4)
            placed[:3, :3] = rotation
            placed[:2, 3] = spot[:2] - (points[:, :2].min(0) + points[:, :2].max(0)) / 2
            placed[2, 3] = spot[2] + clearance - points[:, 2].min()
            release = placed @ np.linalg.inv(tcp_object)
            try:
                solver.move_to_pose_with_RRTConnect(translated(release, [0., 0., above]), lift=True)
                solver.move_to_pose_with_screw(release)
                break
            except PlanningFailed as error:
                if index == len(options) - 1:
                    raise
                ctx.note('next placement', arm=arm, spot=spot.tolist(), turn=float(turn), reason=error.reason)
        self.release(arm)
        ctx.hold(.3)
        return {'arm': arm, 'spot': spot.tolist(), 'turn': float(turn), 'object': ctx.poses()[target.body].tolist()}


def _skills(ctx):
    return ctx.skills()


def grasp(ctx, obj, part=None, *, arm=None, shape=None, offset=None, body=None, point=None, approach=None, then=None):
    """Grasp scene object ``obj`` (or its ``part``); see the module docstring."""
    skills = _skills(ctx)
    return skills.grasp(skills.target(obj, part, shape, offset, body), arm, point, approach, then)


def pick(ctx, obj, part=None, *, arm=None, shape=None, offset=None, body=None, point=None, approach=None, height=LIFT):
    """Grasp scene object ``obj`` (or its ``part``) and lift it by ``height``."""
    skills = _skills(ctx)
    return skills.pick(skills.target(obj, part, shape, offset, body), arm, point, approach, height)


def move_by_displacement(ctx, arm, x=0., y=0., z=0., drag=None):
    """Move ``arm``'s TCP straight by ``(x, y, z)`` (world); ``drag``: ``(object, body)`` moved by the held part."""
    _skills(ctx).move_by_displacement(arm, (x, y, z), drag)


def release(ctx, arm):
    """Let go of what ``arm`` holds and back out."""
    _skills(ctx).release(arm)


def place(ctx, obj, spots, *, arm=None, heading=None, turns=(0., np.pi), clearance=PLACE_CLEARANCE, above=PLACE_ABOVE):
    """Put the held ``obj`` down at the first of ``spots`` that plans and release it."""
    return _skills(ctx).place(obj, spots, arm, heading, turns, clearance, above)


def open_gripper(ctx, arm, gap=None):
    solver = ctx.solver(arm)
    solver.open_gripper() if gap is None else solver.open_gripper(gap)


def close_gripper(ctx, arm):
    ctx.solver(arm).close_gripper()
