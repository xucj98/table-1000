"""Robot acceptance in the simulator: physical-sense cases and comparison metrics.

``scripts/robots/check_robot.py <name>`` runs, in one Isaac process, a check
scene (a table in front of the robot and a few primitive shapes, see
:func:`check_scene`) through:

- **cases** with pass/fail criteria, each starting from a reset:
  ``arm_hits_block``, ``arm_pushes_objects``, ``drop_on_robot``,
  ``lift_into_shelf``, ``self_collision``, ``grasp_and_shake``,
  ``base_into_wall``, ``drive_holding``, ``joint_limits``;
- **metrics** recorded for comparison only: model/simulator consistency, IK,
  self-collision planning, joint tracking, base tracking and step cost.

Common criteria of every case: robot contacts penetrate at most
``PENETRATION_LIMIT``; no robot joint moves faster than its URDF velocity
limit (with ``SPEED_MARGIN``) and no object faster than ``OBJECT_SPEED_LIMIT``
(nothing is flung); blocked joints settle (``JITTER_LIMIT``).

World frame of the check scene: the robot's base_link at the origin facing +X,
the table top 0.29 m in front of it. Unused primitives are parked off to the
side.
"""

from pathlib import Path
import time
import types
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.robots.controllers import CONTROL_HZ

PENETRATION_LIMIT = .002     # m, slow or resting contacts
# Objects dropped onto the robot hit it at about 2 m/s, 9 mm per 4 ms step; a
# few millimetres of transient overlap is the price of that step and accepted.
IMPACT_PENETRATION_LIMIT = .01
# Joint speed / URDF velocity limit. A finger knocked by an obstacle is back-driven
# a little past its limit; flung bodies go far beyond.
SPEED_MARGIN = 1.5
OBJECT_SPEED_LIMIT = 2.      # m/s
JITTER_LIMIT = 1e-3          # rad or m, oscillation std of a blocked joint over 0.5 s
HOLD_DRIFT_LIMIT = .005      # m, held object relative to the TCP
DT = 1 / CONTROL_HZ

TABLE = {'size': (.8, 1.2), 'height': .755, 'top_thickness': .03, 'leg_size': .05, 'center': (.69, 0.),
         'friction': (.5, .4)}
TOP = TABLE['height']
# Primitives; a missing mass makes the shape kinematic (it stays where it is put).
PRIMITIVES = {
    'block': {'shape': 'box', 'size': (.06, .06, .14), 'color': (.8, .3, .2)},
    'box': {'shape': 'box', 'size': (.05, .05, .08), 'mass': .2},
    'can': {'shape': 'cylinder', 'size': (.025, .10), 'mass': .2, 'color': (.3, .7, .3)},
    'stick': {'shape': 'cylinder', 'size': (.007, .14), 'mass': .02, 'color': (.9, .8, .2)},
    'ball': {'shape': 'sphere', 'size': .03, 'mass': .1, 'color': (.9, .5, .1)},
    'cube': {'shape': 'box', 'size': (.06, .06, .06), 'mass': .3, 'color': (.6, .3, .8)},
    'shelf': {'shape': 'box', 'size': (.12, .12, .02), 'color': (.5, .5, .5)},
    'wall': {'shape': 'box', 'size': (.1, 1.4, 1.), 'color': (.5, .5, .5)},
}


def translation(x, y, z, rotation=None):
    transform = np.eye(4)
    transform[:3, 3] = [x, y, z]
    if rotation is not None:
        transform[:3, :3] = rotation
    return transform


def resting_height(spec):
    """Height of a primitive's centre when it rests on a flat surface (cylinders upright)."""
    size = spec['size']
    return size if spec['shape'] == 'sphere' else (size[1] if spec['shape'] == 'cylinder' else size[2]) / 2


def parked(index):
    """Out of the way, resting on the floor."""
    spec = list(PRIMITIVES.values())[index]
    return translation(-3., -2. + .6 * index, resting_height(spec))


def check_scene(spec):
    """Scene module for :class:`~table_1000.sim.isaac.IsaacBackend`."""
    scene = types.SimpleNamespace(
        TABLE=TABLE, OBJECTS=(), PRIMITIVES=PRIMITIVES,
        ROBOT={'asset': f'robots/{spec.name}', 'position': (0., 0., 0.), 'yaw': 0.,
               'joints': spec.keyframes['ready']})

    def placement(**poses):
        layout = {name: parked(i) for i, name in enumerate(PRIMITIVES)}
        layout.update(poses)
        return layout

    scene.placement = placement
    return scene


def primitive_surface(name, pose, count=400):
    """Points on a primitive's surface, posed in the world."""
    import trimesh
    spec = PRIMITIVES[name]
    if spec['shape'] == 'sphere':
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=spec['size'])
    elif spec['shape'] == 'box':
        mesh = trimesh.creation.box(spec['size'])
    else:
        mesh = trimesh.creation.cylinder(radius=spec['size'][0], height=spec['size'][1])
    points, _ = trimesh.sample.sample_surface_even(mesh, count, seed=0)
    return points @ pose[:3, :3].T + pose[:3, 3]


def down(tilt_deg=90.):
    """TCP orientation with its approach axis (+X) tilted down by ``tilt_deg``."""
    return Rotation.from_euler('y', tilt_deg, degrees=True).as_matrix()


class VisualModel:
    """Visual meshes of the robot source, posed by the planning model's FK (for gap measurements)."""

    def __init__(self, spec, model, root):
        import trimesh
        urdf = Path(root) / spec.urdf
        self.model, self.parts = model, []
        for link in ET.parse(urdf).getroot().findall('link'):
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                origin = visual.find('origin')
                transform = np.eye(4)
                if origin is not None:
                    transform[:3, :3] = Rotation.from_euler(
                        'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                    transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                self.parts.append((link.get('name'), transform,
                                   trimesh.load(urdf.parent / mesh.get('filename'), force='mesh')))

    def posed(self):
        import trimesh
        meshes = []
        for link, transform, mesh in self.parts:
            posed = mesh.copy()
            posed.apply_transform(self.model.fk(link, world=True) @ transform)
            meshes.append(posed)
        return trimesh.util.concatenate(meshes)

    def surface_below(self, point):
        """Highest visual surface point below ``point`` (world), at the model's current state; None if none."""
        hits, _, _ = self.posed().ray.intersects_location([point], [[0., 0., -1.]])
        return float(hits[:, 2].max()) if len(hits) else None

    def distance(self, points):
        """Smallest distance from ``points`` (world) to the visual surface."""
        _, distances, _ = self.posed().nearest.on_surface(np.asarray(points))
        return float(distances.min())


class Checker:
    def __init__(self, spec, output, assets, root, cameras=None):
        from table_1000.robots.kinematics import RobotModel
        from table_1000.sim.env import Env
        from table_1000.sim.isaac import IsaacBackend
        self.spec, self.output = spec, Path(output)
        self.scene = check_scene(spec)
        started = time.perf_counter()
        self.backend = IsaacBackend(self.scene, self.scene.placement(), assets, cameras=cameras,
                                    contact_objects=tuple(PRIMITIVES), contact_watch=('robot', *PRIMITIVES))
        self.model = RobotModel(spec, self.output / 'planning')
        self.env = Env(self.scene, self.backend, spec, [self.model])
        self.visual = VisualModel(spec, self.model, root)
        self.load_s = time.perf_counter() - started
        self.body_joints = [j for j in spec.body_joints]
        limits = {joint.get('name'): float(joint.find('limit').get('velocity'))
                  for joint in ET.parse(Path(root) / spec.urdf).getroot().findall('joint')
                  if joint.find('limit') is not None}
        self.speed_limits = np.array([limits[j] for j in self.body_joints])
        self.case = ''
        self.env.caption = lambda env: f'{self.case}  t={env.time:5.2f}s'

    # -- driving ---------------------------------------------------------------
    def reset(self, robot=None, **poses):
        q = self.env.initial_robot_q()
        q.update(robot or {})
        self.trace = {'joint_speed': 0., 'joint': None, 'object_speed': 0.}
        return self.env.reset(self.scene.placement(**poses), q)

    def step(self, action=None, count=1):
        for _ in range(count):
            obs, info = self.env.step(action or {})
            ratio = np.abs([obs['qvel'][j][0] for j in self.body_joints]) / self.speed_limits
            if ratio.max() > self.trace['joint_speed']:
                self.trace['joint_speed'], self.trace['joint'] = float(ratio.max()), self.body_joints[int(ratio.argmax())]
            velocities = self.backend.body_velocities()
            if velocities:
                self.trace['object_speed'] = max(self.trace['object_speed'], max(
                    float(np.linalg.norm(v[0, :3])) for v in velocities.values()))
        return obs

    def q(self, joints):
        return np.array([self.env.obs['qpos'][j][0] for j in joints])

    def arm_q(self, arm):
        return self.q(self.spec.arms[arm].joints)

    def ik(self, arm, target):
        status, q = self.model.ik(arm, target, world=True, attempts=40)
        if status != 'Success':
            raise RuntimeError(f'check pose unreachable for {arm}: {target[:3, 3]}')
        return q[self.model.arm_indices(arm)]

    def move_joints(self, arm, target, seconds):
        """Joint-space straight line to ``target`` (no planning: cases collide on purpose)."""
        start = self.arm_q(arm)
        count = max(1, round(seconds / DT))
        for k in range(1, count + 1):
            self.step({arm: (start + (target - start) * k / count)[None]})

    def move_line(self, arm, target):
        """Straight TCP line to the world ``target`` (self-collision checks only), at the planner's speed."""
        result = self.model.plan_screw(arm, target, world=True, time_step=DT)
        if result['status'] != 'Success':
            raise RuntimeError(f'line to {target[:3, 3]} not planned: {result.get("reason")}')
        for row in result['position']:
            self.step({arm: row[None]})

    def jitter(self, joints, seconds=.5):
        """Largest standard deviation of ``joints`` while holding for ``seconds``."""
        rows = []
        for _ in range(round(seconds / DT)):
            self.step()
            rows.append(self.q(joints))
        rows = np.array(rows)
        # Oscillation about a straight-line fit; a slow creep (a hand sliding
        # along an obstacle under saturated torque) is reported separately.
        t = np.arange(len(rows))
        residual = rows - np.polyval(np.polyfit(t, rows, 1), t[:, None]) if len(rows) > 2 else rows * 0.
        self.last_jitter = {'oscillation_std': dict(zip(joints, map(float, residual.std(axis=0)))),
                            'creep': dict(zip(joints, map(float, rows[-1] - rows[0])))}
        return float(residual.std(axis=0).max())

    def penetration(self, robot_only=True):
        """Deepest contact of the case (robot pairs only by default)."""
        depth = 0.
        for entry in self.backend.contacts.intervals():
            if not robot_only or any(side.startswith('robot') for side in entry['pair']):
                depth = max(depth, entry['max_penetration_m'])
        return depth

    def common(self, result):
        result['max_penetration_m'] = self.penetration()
        result.update(max_joint_speed_ratio=self.trace['joint_speed'], fastest_joint=self.trace['joint'],
                      max_object_speed=self.trace['object_speed'])
        checks = result.setdefault('checks', {})
        checks['penetration'] = result['max_penetration_m'] <= PENETRATION_LIMIT
        checks['joint_speed'] = result['max_joint_speed_ratio'] <= SPEED_MARGIN
        checks['object_speed'] = result['max_object_speed'] <= OBJECT_SPEED_LIMIT
        checks = {key: bool(value) for key, value in checks.items()}
        result['checks'] = checks
        result['status'] = 'PASS' if all(checks.values()) else 'FAIL'
        deepest = {}
        for entry in self.backend.contacts.intervals():
            pair = ' / '.join(entry['pair'])
            deepest[pair] = max(deepest.get(pair, 0.), entry['max_penetration_m'])
        result['contacts'] = dict(sorted(deepest.items(), key=lambda item: -item[1]))
        return result

    def tcp(self, arm='right'):
        return self.env.obs['tcp'][arm][0]

    def held(self, body, arm='right'):
        """Pose of ``body`` in the frame of the arm's TCP link, both as simulated."""
        link = self.backend.link_pose(self.spec.arms[arm].tcp[0])[0]
        return np.linalg.inv(link) @ self.backend.body_poses()[f'{body}:body'][0]

    def on_table(self, x, y, height):
        return translation(x, y, TOP + height / 2)

    # -- cases -------------------------------------------------------------------
    def arm_hits_block(self):
        """The right hand sweeps sideways through a fixed block; it must stop, then reach the goal once freed."""
        self.reset(block=self.on_table(.45, -.12, .14))
        start, goal = self.ik('right', translation(.45, -.36, .80, down(70))), self.ik('right', translation(.45, .05, .80, down(70)))
        self.move_joints('right', start, 2.)
        self.move_joints('right', goal, 3.)
        blocked_error = float(np.abs(self.arm_q('right') - goal).max())
        jitter = self.jitter(self.spec.arms['right'].joints)
        self.backend.set_object_pose('block', parked(0))
        self.step(count=round(1.5 / DT))
        result = {'blocked_error_rad': blocked_error, 'blocked_jitter_rad': jitter, 'blocked_motion': self.last_jitter,
                  'freed_error_rad': float(np.abs(self.arm_q('right') - goal).max())}
        result['checks'] = {'stopped_by_block': blocked_error > .05, 'settled': jitter <= JITTER_LIMIT,
                            'reaches_goal_when_freed': result['freed_error_rad'] <= .01}
        return self.common(result)

    def arm_pushes_objects(self):
        """The hand sweeps through a free box and can; they slide or tip, nothing is flung."""
        self.reset(box=self.on_table(.45, -.15, .05), can=self.on_table(.47, -.04, .10))
        start = self.ik('right', translation(.45, -.36, .80, down(70)))
        before = self.backend.body_poses()
        self.move_joints('right', start, 2.)
        self.move_line('right', translation(.45, .1, .80, down(70)))
        self.step(count=round(1. / DT))
        after = self.backend.body_poses()
        moved = {name: float(np.linalg.norm(after[f'{name}:body'][0][:2, 3] - before[f'{name}:body'][0][:2, 3]))
                 for name in ('box', 'can')}
        heights = {name: float(after[f'{name}:body'][0][2, 3]) for name in ('box', 'can')}
        result = {'moved_m': moved, 'final_height_m': heights}
        result['checks'] = {'pushed': all(d > .01 for d in moved.values()),
                            'still_on_table': all(TOP < z < TOP + .1 for z in heights.values())}
        return self.common(result)

    def drop_on_robot(self):
        """A ball and a cube dropped onto the base, torso, arm and head rest on the visible surface or roll off."""
        targets = {'base': 'base_link', 'torso': 'lift_link', 'arm': 'right_arm_link3', 'head': 'head_pitch_link'}
        results = {}
        for name, link in targets.items():
            for body in ('ball', 'cube'):
                self.reset()
                self.step(count=2)
                point = self.model.fk(link, world=True)[:3, 3]
                surface = self.visual.surface_below([point[0], point[1], 3.])
                if surface is None:
                    continue
                half = PRIMITIVES[body]['size'] if body == 'ball' else PRIMITIVES[body]['size'][2] / 2
                self.backend.set_object_pose(body, translation(point[0], point[1], surface + .25 + half))
                self.step(count=round(2.5 / DT))
                pose = self.backend.body_poses()[f'{body}:body'][0]
                speed = float(np.linalg.norm(self.backend.body_velocities()[f'{body}:body'][0, :3]))
                resting_on_robot = pose[2, 3] > .1 and speed < .02
                gap = self.visual.distance(primitive_surface(body, pose)) if resting_on_robot else None
                results[f'{name}/{body}'] = {'link': link, 'resting_on_robot': bool(resting_on_robot),
                                             'gap_to_visual_m': gap, 'penetration_m': self.penetration()}
        gaps = [r['gap_to_visual_m'] for r in results.values() if r['gap_to_visual_m'] is not None]
        result = {'drops': results, 'max_gap_m': max(gaps, default=0.)}
        result['checks'] = {'no_deep_drop_penetration': all(r['penetration_m'] <= IMPACT_PENETRATION_LIMIT
                                                            for r in results.values()),
                            # Resting more than 15 mm above the visible surface: the collision shell is too fat.
                            'rests_on_visible_surface': result['max_gap_m'] <= .015}
        return self.common(result)

    def lift_into_shelf(self):
        """The torso rises into a fixed shelf above the head: it stops there, then lowers again."""
        self.reset()
        self.step(count=2)
        head = self.model.fk('head_pitch_link', world=True)[:3, 3]
        top = self.visual.surface_below([head[0], head[1], 3.])
        self.backend.set_object_pose('shelf', translation(head[0], head[1], top + .10))
        lift = self.spec.lift[0]
        start = self.q([lift])[0]
        self.step({'lift': [[start + .3]]}, count=round(2.5 / DT))
        blocked = self.q([lift])[0]
        jitter = self.jitter([lift])
        self.step({'lift': [[start]]}, count=round(2. / DT))
        result = {'lift_start_m': float(start), 'lift_blocked_m': float(blocked), 'blocked_jitter_m': jitter,
                  'lift_back_error_m': float(abs(self.q([lift])[0] - start))}
        result['checks'] = {'stopped_by_shelf': blocked < start + .25, 'settled': jitter <= JITTER_LIMIT,
                            'lowers_again': result['lift_back_error_m'] <= .005}
        return self.common(result)

    def self_collision(self):
        """The right arm swings into the torso and the left arm: it stops at contact and comes back."""
        self.reset()
        self.step(count=2)
        arm = self.spec.arms['right']
        ready = self.arm_q('right')
        target, contact = self.colliding_target('right', ready)
        collides = target is not None
        self.move_joints('right', target, 3.)
        error = float(np.abs(self.arm_q('right') - target).max())
        jitter = self.jitter(arm.joints)
        self.move_joints('right', ready, 3.)
        self.step(count=round(1. / DT))
        result = {'model_contact': contact, 'target': target.tolist(), 'blocked_error_rad': error, 'blocked_jitter_rad': jitter,
                  'back_error_rad': float(np.abs(self.arm_q('right') - ready).max()),
                  'self_contacts': sorted({' / '.join(e['pair']) for e in self.backend.contacts.intervals()
                                           if all(s.startswith('robot') for s in e['pair'])})}
        result['checks'] = {'stopped_at_contact': error > .05, 'settled': jitter <= JITTER_LIMIT,
                            'comes_back': result['back_error_rad'] <= .01}
        return self.common(result)

    def colliding_target(self, arm, ready, beyond=.3):
        """Arm joints just past the first self-collision along one shoulder or elbow joint from ``ready``."""
        indices = self.model.arm_indices(arm)
        for joint in (0, 1, 2, 3):
            for sign in (1., -1.):
                low, high = self.model.limits[indices[joint]]
                for angle in np.arange(.05, 3., .05):
                    q = ready.copy()
                    q[joint] = ready[joint] + sign * angle
                    if not low <= q[joint] <= high:
                        break
                    pairs = self.model.self_collisions(self.model.vector(dict(zip(self.spec.arms[arm].joints, q))))
                    if pairs:
                        q[joint] = np.clip(q[joint] + sign * beyond, low, high)
                        return q, pairs
        raise RuntimeError('no self-collision reachable along single joints')

    def _grasp(self, body, pose, height, approach=.10):
        """Top-down grasp of ``body`` resting at ``pose``; returns the closed finger gap."""
        self.reset(**{body: pose})
        self.step(count=round(.5 / DT))
        center = self.backend.body_poses()[f'{body}:body'][0][:3, 3]
        # Pads 15 mm below the top, fingertips (18.5 mm past the TCP) clear of the table.
        z = max(center[2] + height / 2 - .015, TOP + .0185 + .004)
        grasp = translation(center[0], center[1], z, down(90))
        self.step({'right_gripper': [[.08]]}, count=round(.6 / DT))
        self.move_joints('right', self.ik('right', translation(*grasp[:3, 3] + [0., 0., approach], down(90))), 2.5)
        self.move_joints('right', self.ik('right', grasp), 1.5)
        self.step({'right_gripper': [[0.]]}, count=round(.8 / DT))
        fingers = self.q(self.spec.arms['right'].fingers)
        return float(fingers[0] - fingers[1])

    def grasp_and_shake(self):
        """Grasp a box, a can and a thin stick, lift and shake each: no slip, no finger penetration."""
        objects = {'box': (self.on_table(.45, -.15, .08), .08), 'can': (self.on_table(.45, -.15, .10), .10),
                   # lying across the closing direction (world Y): its axis along world X
                   'stick': (translation(.45, -.15, TOP + .007, Rotation.from_euler('y', 90, degrees=True).as_matrix()),
                             .014)}
        results = {}
        for body, (pose, height) in objects.items():
            gap = self._grasp(body, pose, height)
            lifted = self.ik('right', translation(*self.tcp()[:3, 3] + [0., 0., .08], down(90)))
            self.move_joints('right', lifted, 1.5)
            self.step(count=round(.5 / DT))
            relative = self.held(body)
            joint6 = self.spec.arms['right'].joints[-1]
            for sign in (1, -1, 1, -1):
                target = lifted.copy()
                target[-1] += sign * .4
                self.move_joints('right', target, .5)
            self.move_joints('right', lifted, .5)
            self.step(count=round(.5 / DT))
            drift = float(np.linalg.norm(self.held(body)[:3, 3] - relative[:3, 3]))
            held_z = self.backend.body_poses()[f'{body}:body'][0][2, 3]
            self.step({'right_gripper': [[.08]]}, count=round(1.5 / DT))
            final = self.backend.body_poses()[f'{body}:body'][0][2, 3]
            results[body] = {'finger_gap_m': gap, 'drift_m': drift, 'falls_when_released': bool(final < held_z - .04),
                             'penetration_m': self.penetration(), 'joint6': joint6}
        result = {'objects': results}
        result['checks'] = {'held_without_slip': all(r['drift_m'] <= HOLD_DRIFT_LIMIT for r in results.values()),
                            'released': all(r['falls_when_released'] for r in results.values())}
        return self.common(result)

    def base_into_wall(self):
        """The base backs into a wall: it stops there without shoving or crossing it."""
        self.reset(wall=translation(-.65, 0., .5))
        self.step({'base': [[-.3, 0.]]}, count=round(3. / DT))
        x_blocked = float(self.env.obs['base_pose'][0, 0])
        jitter = self.jitter(list(self.spec.base.virtual_joints[:2]))
        self.step({'base': [[.3, 0.]]}, count=round(1.5 / DT))
        self.step({'base': [[0., 0.]]}, count=round(1. / DT))
        result = {'base_x_blocked_m': x_blocked, 'blocked_jitter_m': jitter,
                  'base_x_after_m': float(self.env.obs['base_pose'][0, 0])}
        result['checks'] = {'stopped_by_wall': x_blocked > -.5, 'settled': jitter <= JITTER_LIMIT,
                            'drives_away': result['base_x_after_m'] > x_blocked + .2}
        return self.common(result)

    def drive_holding(self):
        """Hold a box, drive at full speed and stop: the box stays in the hand."""
        self._grasp('box', self.on_table(.45, -.15, .08), .08)
        lifted = self.ik('right', translation(*self.tcp()[:3, 3] + [0., 0., .08], down(90)))
        self.move_joints('right', lifted, 1.5)
        self.step(count=round(.5 / DT))
        relative = self.held('box')

        def drift():
            return float(np.linalg.norm(self.held('box')[:3, 3] - relative[:3, 3]))

        phases = {}
        for name, command, seconds in (('drive', [-.5, 0.], 1.5), ('stop', [0., 0.], 1.), ('turn', [0., 1.], 1.),
                                       ('stop after turn', [0., 0.], 1.)):
            self.step({'base': [command]}, count=round(seconds / DT))
            phases[name] = drift()
            fingers = self.q(self.spec.arms['right'].fingers)
            phases[name + ' finger_centre'] = float(fingers.sum() / 2)
        arm = self.spec.arms['right']
        sim = self.backend.link_pose(arm.tcp[0])[0] @ translation(*arm.tcp[1])
        result = {'drift_m': max(v for k, v in phases.items() if 'finger' not in k), 'drift_by_phase_m': phases,
                  'model_vs_sim_tcp_m': _pose_error(sim, self.tcp())[0],
                  'base_pose': self.env.obs['base_pose'][0].tolist()}
        result['checks'] = {'held_without_slip': result['drift_m'] <= HOLD_DRIFT_LIMIT}
        return self.common(result)

    def joint_limits(self):
        """Lift, head and arm commands beyond the limits end at the limits, steady."""
        self.reset()
        self.step(count=2)
        limits = {name: self.model.limits[self.model.index[name]] for name in self.spec.body_joints}
        beyond = {'lift': [[limits['lift_joint'][1] + .2]],
                  'head': [[limits['head_yaw_joint'][1] + .5, limits['head_pitch_joint'][0] - .5]]}
        self.step(beyond, count=round(4. / DT))
        joints = ['lift_joint', 'head_yaw_joint', 'head_pitch_joint']
        q = self.q(joints)
        jitter = self.jitter(joints)
        overshoot = max(max(limits[j][0] - v, v - limits[j][1]) for j, v in zip(joints, q))
        result = {'final': dict(zip(joints, map(float, q))), 'max_overshoot': float(overshoot), 'jitter': jitter}
        result['checks'] = {'within_limits': overshoot <= .01, 'settled': jitter <= JITTER_LIMIT,
                            'reaches_limits': all(min(abs(v - limits[j][0]), abs(v - limits[j][1])) < .02
                                                  for j, v in zip(joints, q))}
        return self.common(result)

    CASES = ('arm_hits_block', 'arm_pushes_objects', 'drop_on_robot', 'lift_into_shelf', 'self_collision',
             'grasp_and_shake', 'base_into_wall', 'drive_holding', 'joint_limits')

    # -- comparison metrics ---------------------------------------------------------
    def metrics(self, samples=100, seed=0):
        rng = np.random.default_rng(seed)
        model = self.model
        self.reset()
        self.step(count=2)
        ready = model.q.copy()
        out = {}
        for arm_name, arm in self.spec.arms.items():
            indices = model.arm_indices(arm_name)
            low, high = model.limits[indices].T
            free = []
            while len(free) < samples:
                q = ready.copy()
                q[indices] = rng.uniform(low, high)
                if not model.self_collisions(q):
                    free.append(q)
            # Model vs simulator TCP at the same joint values.
            # Measured joint values, since a random pose may meet the table and be pushed.
            errors = []
            for q in free[:30]:
                self.reset(robot=dict(zip(arm.joints, q[indices])))
                sim = self.backend.link_pose(arm.tcp[0])[0] @ translation(*arm.tcp[1])
                position, rotation = _pose_error(sim, self.tcp(arm_name))
                errors.append((position, rotation))
            errors = np.array(errors)
            # IK on reachable poses.
            ik_times, solved, ik_errors = [], 0, []
            model.set_state(dict(zip(model.joints, ready)))
            for q in free:
                target = model.tcp(arm_name, q)
                started = time.perf_counter()
                status, solution = model.ik(arm_name, target)
                ik_times.append(time.perf_counter() - started)
                if status == 'Success':
                    solved += 1
                    ik_errors.append(_pose_error(model.tcp(arm_name, solution), target))
            # Self-collision planning between random free configurations.
            plan_times, planned = [], 0
            for start, goal in zip(free[:20], free[20:40]):
                model.set_state(dict(zip(model.joints, start)))
                started = time.perf_counter()
                result = model.plan_rrt(arm_name, None, goals=[goal], planning_time=2.)
                plan_times.append(time.perf_counter() - started)
                planned += result['status'] == 'Success'
            model.set_state(dict(zip(model.joints, ready)))
            out[arm_name] = {
                'fk_position_error_max_m': float(errors[:, 0].max()), 'fk_rotation_error_max_rad': float(errors[:, 1].max()),
                'ik_success_rate': solved / len(free), 'ik_time_median_s': float(np.median(ik_times)),
                'ik_position_error_max_m': float(max((e[0] for e in ik_errors), default=0.)),
                'plan_success_rate': planned / len(plan_times), 'plan_time_median_s': float(np.median(plan_times)),
                'plan_time_p95_s': float(np.percentile(plan_times, 95))}
        out['tracking'] = self.tracking_metrics()
        out['base'] = self.base_metrics()
        out['step_ms'] = self.step_cost()
        return out

    def tracking_metrics(self):
        """Arm following planned paths: worst error during the path and the settle time."""
        self.reset()
        self.step(count=2)
        targets = [translation(.45, -.25, .95, down(45)), translation(.55, -.05, .85, down(70)),
                   translation(.40, -.30, .90, down(90)), translation(.50, -.20, 1.0, down(45))]
        worst, settle = [], []
        for target in targets:
            result = self.model.plan_rrt('right', target, world=True)
            if result['status'] != 'Success':
                continue
            error = 0.
            for row in result['position']:
                self.step({'right': row[None]})
                error = max(error, float(np.abs(self.arm_q('right') - row).max()))
            steps = 0
            while np.abs(self.arm_q('right') - result['position'][-1]).max() > .002 and steps < 50:
                self.step()
                steps += 1
            worst.append(error)
            settle.append(steps * DT)
        return {'paths': len(worst), 'max_error_rad': float(max(worst, default=np.nan)),
                'settle_to_2mrad_s_max': float(max(settle, default=np.nan))}

    def base_metrics(self):
        """Speed tracking (backing away from the table) and an in-place turn in the open."""
        self.reset()
        self.step({'base': [[-.4, 0.]]}, count=round(2. / DT))
        velocity = [self.env.obs['qvel'][j][0] for j in self.spec.base.virtual_joints[:2]]
        speed = float(np.linalg.norm(velocity))
        self.step({'base': [[0., 0.]]}, count=round(1. / DT))
        start = self.env.obs['base_pose'][0].copy()
        self.step({'base': [[0., .8]]}, count=round(2. / DT))
        self.step({'base': [[0., 0.]]}, count=round(1. / DT))
        turned = self.env.obs['base_pose'][0]
        target = self.env.controller.groups['base'].pose[0]
        return {'speed_at_0.4_m_s': speed, 'turn_translation_drift_m': float(np.linalg.norm(turned[:2] - start[:2])),
                'pose_error_after_stop_m': float(np.linalg.norm(turned[:2] - target[:2])),
                'yaw_error_after_stop_rad': float(abs(turned[2] - target[2]))}

    def step_cost(self):
        """Physics ms per step: robot holding still, and moving its arm."""
        self.reset()
        before = self.backend.timings['step_s']
        self.step(count=50)
        hold = (self.backend.timings['step_s'] - before) / (50 * self.env.controller.substeps) * 1e3
        before = self.backend.timings['step_s']
        self.move_joints('right', self.ik('right', translation(.45, -.25, .95, down(45))), 2.)
        moving = (self.backend.timings['step_s'] - before) / (50 * self.env.controller.substeps) * 1e3
        return {'hold': hold, 'arm_moving': moving}


def physics_ms(checker, actions):
    """Median physics time per 4 ms step over the control steps of ``actions``."""
    times = []
    for action in actions:
        before = checker.backend.timings['step_s']
        checker.step(action)
        times.append((checker.backend.timings['step_s'] - before) / checker.env.controller.substeps * 1e3)
    return float(np.median(times))


def benchmark(checker, sweeps=3, samples=1000, plans=10, seed=0):
    """Cost of the robot's collision geometry: physics while holding and while an
    arm swings into the torso and back; planning collision checks and RRT."""
    model, arm = checker.model, checker.spec.arms['right']
    checker.reset()
    checker.step(count=5)
    hold = physics_ms(checker, [{}] * 75)
    ready = checker.arm_q('right')
    target, _ = checker.colliding_target('right', ready)
    path = []
    for _ in range(sweeps):
        for start, goal in ((ready, target), (target, ready)):
            path += [{'right': (start + (goal - start) * k / 50)[None]} for k in range(1, 51)]
    sweep = physics_ms(checker, path)
    planner = model.planners['right']
    rng = np.random.default_rng(seed)
    configurations = []
    for _ in range(samples):
        q = model.q.copy()
        for name in checker.spec.arms:
            indices = model.arm_indices(name)
            q[indices] = rng.uniform(*model.limits[indices].T)
        configurations.append(q)
    started = time.perf_counter()
    for q in configurations:
        planner.robot.set_qpos(q, True)
        planner.check_for_self_collision()
    check_us = (time.perf_counter() - started) / samples * 1e6
    model.set_state()
    free = [q for q in configurations if not model.self_collisions(q)]
    times = []
    for start, goal in zip(free[:plans], free[plans:2 * plans]):
        model.set_state(dict(zip(model.joints, start)))
        started = time.perf_counter()
        model.plan_rrt('right', None, goals=[goal], planning_time=2.)
        times.append(time.perf_counter() - started)
    checker.reset()
    return {'physics_hold_ms': hold, 'physics_self_contact_ms': sweep, 'self_collision_check_us': check_us,
            'rrt_median_s': float(np.median(times))}


def _pose_error(a, b):
    from table_1000.robots.kinematics import pose_error
    return pose_error(a, b)
