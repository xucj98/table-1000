"""Coverage of the pick skill (:mod:`table_1000.experts.pick`) on X1Pro at a table, with test objects.

The scene (:func:`scene`): a 1.4 x 0.8 m table 0.75 m high, the robot ``gap`` m
from the middle of its long edge, facing it, base fixed, arms folded (zero pose),
torso lift at ``lift`` and head pitched down. The test objects are primitive
shapes of plastic (1000 kg/m^3) or steel (:data:`OBJECTS`); one at a time stands on the table
in one of its resting poses, the others parked on the floor.

A trial (:meth:`Evaluator.trial`) settles the object, runs ``pick`` and, when it
succeeds, holds the object while a force of ``FORCE`` N pushes its centre of mass
in each of :data:`PUSHES` directions in turn (world axes and random directions):
ramped up over ``RAMP`` s, held ``PUSH`` s, released. The grasp holds when the
object never moves more than ``SLIP_LIMIT`` from where it sat in the hand.
"""

from dataclasses import dataclass
import math
import time
import types

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.experts.grasps import Box, Cylinder

TABLE = {'size': (1.4, .8), 'height': .75, 'top_thickness': .03, 'leg_size': .05, 'center': (0., 0.),
         'friction': (.5, .4)}
PLASTIC, STEEL = 1000., 7850.   # kg/m^3
FORCE, RAMP, PUSH, SETTLE = 5., .2, .8, .3
SLIP_LIMIT = .005    # m, object from its place in the hand (as the robot acceptance)
PENETRATION_LIMIT = .001  # m, contacts deeper than this are reported
DROP_DISTANCE = .03  # m, object this far from its place in the hand is dropped
RANDOM_PUSHES = 8


@dataclass(frozen=True)
class TestObject:
    shape: object         # Box or Cylinder, centred on the body frame
    poses: tuple          # resting poses: names of rotations (see rest_rotation)
    color: tuple
    density: float = PLASTIC

    @property
    def volume(self):
        if isinstance(self.shape, Box):
            return float(np.prod(self.shape.size))
        return math.pi * self.shape.radius ** 2 * self.shape.length

    def primitive(self):
        mass = self.density * self.volume
        if isinstance(self.shape, Box):
            return {'shape': 'box', 'size': tuple(self.shape.size), 'mass': mass, 'color': self.color}
        return {'shape': 'cylinder', 'size': (self.shape.radius, self.shape.length), 'mass': mass, 'color': self.color}

    def rest(self, pose):
        """``(rotation, height of the centre above the support)`` of resting pose ``pose``."""
        if pose == 'standing':
            rotation = np.eye(3)
        else:   # 'lying': the frame's Z (the long or round axis) horizontal
            rotation = Rotation.from_euler('x', 90, degrees=True).as_matrix()
        points = self.shape.points() @ rotation.T
        return rotation, float(-points[:, 2].min())

    def footprint(self):
        """Largest horizontal half extent over the resting poses (keeps the object on the table)."""
        return max(float(np.abs((self.shape.points() @ self.rest(pose)[0].T)[:, :2]).max()) for pose in self.poses)


OBJECTS = {
    'cube1': TestObject(Box((.01, .01, .01)), ('standing',), (.75, .75, .78), STEEL),
    'cube4': TestObject(Box((.04, .04, .04)), ('standing',), (.25, .55, .85)),
    'bar': TestObject(Box((.02, .02, .10)), ('standing', 'lying'), (.9, .7, .15)),
    'rod': TestObject(Cylinder(.01, .10), ('standing', 'lying'), (.3, .75, .35)),
    'disc': TestObject(Cylinder(.015, .01), ('standing',), (.6, .35, .8)),
}


def scene(gap=.12, lift=.6, objects=OBJECTS, robot='x1pro'):
    """Scene source of the evaluation: robot ``robot`` with its base ``gap`` m from the table's near
    edge (world -Y), torso lift ``lift`` m; ``scene.placement(**poses)`` parks the objects not given."""
    width, depth = TABLE['size']
    source = types.SimpleNamespace(
        TABLE=TABLE, OBJECTS={}, PRIMITIVES={name: obj.primitive() for name, obj in objects.items()},
        ROBOT={'name': robot, 'asset': f'robots/{robot}', 'position': (0., -depth / 2 - gap, 0.), 'yaw': math.pi / 2,
               'joints': {'lift_joint': lift, 'head_pitch_joint': .3}})

    def placement(**poses):
        layout = {}
        for index, (name, obj) in enumerate(objects.items()):
            parked = np.eye(4)
            parked[:3, 3] = [-3., -2. + .5 * index, obj.rest(obj.poses[0])[1]]
            layout[name] = poses.get(name, parked)
        return layout

    source.placement = placement
    return source


def table_pose(name, x, y, yaw, pose, objects=OBJECTS):
    """World pose of object ``name`` resting on the table top at ``(x, y)``, turned by ``yaw``."""
    rotation, height = objects[name].rest(pose)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('z', yaw).as_matrix() @ rotation
    transform[:3, 3] = [x, y, TABLE['height'] + height]
    return transform


def sample(rng, name, objects=OBJECTS, margin=.005):
    """A random resting pose of ``name`` anywhere on the table: ``{'x', 'y', 'yaw', 'pose'}``."""
    width, depth = TABLE['size']
    edge = objects[name].footprint() + margin
    return {'x': float(rng.uniform(-width / 2 + edge, width / 2 - edge)),
            'y': float(rng.uniform(-depth / 2 + edge, depth / 2 - edge)),
            'yaw': float(rng.uniform(-math.pi, math.pi)), 'pose': str(rng.choice(objects[name].poses))}


def push_directions(rng, count=RANDOM_PUSHES):
    """The six world axes, then ``count`` random unit directions."""
    axes = [sign * axis for axis in np.eye(3) for sign in (1., -1.)]
    random = rng.normal(size=(count, 3))
    return axes + list(random / np.linalg.norm(random, axis=1, keepdims=True))


class Evaluator:
    """One environment (import after ``start_app``) running trials one after another."""

    def __init__(self, source, assets, output, root, views=None, random_pushes=RANDOM_PUSHES):
        from table_1000.robots import load_spec
        from table_1000.robots.kinematics import RobotModel
        from table_1000.sim.isaaclab.env import RobotEnv
        self.scene, self.assets, self.output = source, assets, output
        self.spec = load_spec(source.ROBOT['name'], root)
        self.env = RobotEnv(source, self.spec, assets, views=views or {}, root=root,
                            contact_watch=('robot', *source.PRIMITIVES))
        self.model = RobotModel(self.spec, output / 'planning', root)
        self.views = dict(views or {})
        self.random_pushes = random_pushes

    def trial(self, name, placement, seed, video=None):
        """Pick object ``name`` placed by ``placement`` (see :func:`sample`); returns the report.
        ``video``: a :class:`~table_1000.sim.isaaclab.video.Video` capturing every step."""
        import mplib.pymp
        import torch
        from table_1000.experts.context import Context, ExpertFailed
        from table_1000.experts.pick import Picker
        started = time.perf_counter()
        pose = table_pose(name, placement['x'], placement['y'], placement['yaw'], placement['pose'])
        self.env.reset(self.scene.placement(**{name: pose}))
        mplib.pymp.set_global_seed(seed + 1)
        ctx = Context(self.env, self.model, self.scene, self.assets, np.random.default_rng([seed, 1]))
        report = {'object': name, 'seed': seed, **placement}
        phase = {'text': 'settle'}
        if video is not None:
            ctx.observers.append(lambda c: video.capture(f'{name} {placement["pose"]}  {phase["text"]}  '
                                                         f't={c.steps / 25:5.2f}s'))
        ctx.hold(.5)
        placed = ctx.poses()[name]
        report['settled_shift'] = float(np.linalg.norm(placed[:3, 3] - pose[:3, 3]))
        picker = Picker(ctx)
        try:
            phase['text'] = 'pick'
            result = picker.pick(name, OBJECTS[name].shape)
            report.update(status='picked', pick=result)
        except ExpertFailed as error:
            report.update(status='failed', failure={'reason': error.reason, 'details': error.details})
        except Exception as error:   # an error in the skill is a result, reported with the trial
            import traceback
            report.update(status='error', failure={'reason': repr(error), 'traceback': traceback.format_exc()})
        if report['status'] == 'picked':
            phase['text'] = 'push 5 N'
            report['hold'] = self.disturb(ctx, name, result['arm'], np.random.default_rng([seed, 2]), phase)
            report['status'] = 'success' if report['hold']['held'] else 'slipped'
        report['events'] = ctx.events
        report['plans'] = [entry for solver in ctx.solvers.values() for entry in solver.log]
        report['max_penetration_m'] = max([entry['max_penetration_m'] for entry in self.env.contacts.intervals()],
                                          default=0.)
        report['penetration'] = {' / '.join(entry['pair']): entry['max_penetration_m']
                                 for entry in self.env.contacts.intervals() if entry['max_penetration_m'] > PENETRATION_LIMIT}
        for solver in ctx.solvers.values():
            solver.world.clear()
        self.push(name, np.zeros(3))
        report['sim_s'], report['wall_s'] = ctx.steps / 25, time.perf_counter() - started
        return report

    def push(self, name, force):
        import torch
        shape = self.env.env.scene[name]
        forces = torch.tensor(np.asarray(force, dtype=np.float32), device=self.env.device).view(1, 1, 3)
        shape.set_external_force_and_torque(forces, torch.zeros_like(forces), is_global=True)

    def disturb(self, ctx, name, arm, rng, phase):
        """Push the held object with FORCE N in each direction; slip of the object in the hand."""
        def offset():
            return np.linalg.inv(ctx.tcp(arm)) @ ctx.poses()[name]

        ctx.hold(SETTLE)
        start = offset()
        pushes = []
        for direction in push_directions(rng, self.random_pushes):
            worst = 0.
            phase['text'] = f'push 5 N ({direction[0]:+.2f}, {direction[1]:+.2f}, {direction[2]:+.2f})'
            for k in range(1, round((RAMP + PUSH) * 25) + 1):
                self.push(name, FORCE * direction * min(1., k / (RAMP * 25)))
                ctx.step()
                worst = max(worst, float(np.linalg.norm(offset()[:3, 3] - start[:3, 3])))
            self.push(name, np.zeros(3))
            ctx.hold(SETTLE)
            pushes.append({'direction': direction.tolist(), 'max_slip_m': worst,
                           'after_m': float(np.linalg.norm(offset()[:3, 3] - start[:3, 3]))})
            if worst > DROP_DISTANCE:
                break
        slip = max(entry['max_slip_m'] for entry in pushes)
        return {'held': slip <= SLIP_LIMIT and len(pushes) == 6 + self.random_pushes,
                'max_slip_m': slip, 'pushes': pushes}

    def close(self):
        self.env.close()
