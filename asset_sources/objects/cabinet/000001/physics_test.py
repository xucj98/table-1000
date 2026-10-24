"""Force-driven tests for the native 208 mm wooden two-drawer rack.

All loads are applied external forces, not measured reactions. Importing this
module starts no application. Mass, contact, damping and geometry are unchanged.
"""

import json
import math
from pathlib import Path
from contextlib import contextmanager

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.physics.simulation import rotation
from table_1000.physics.testing import TestContext


TRAVEL = .208
HOUSING = 'cabinet.housing'
DRAWERS = ('cabinet.drawer1', 'cabinet.drawer2')
JOINTS = ('drawer1_open', 'drawer2_open')
KNOB = (0, -.025, 0)


def _geometry(path):
    model = json.loads((Path(path).parent / 'model.json').read_text())
    nodes = {n['name']: n for n in model['nodes']}
    transforms = {}

    def world(name):
        if name not in transforms:
            n = nodes[name]
            transforms[name] = (world(n['parent']) if n['parent'] else np.eye(4)) @ np.asarray(n['matrix'])
        return transforms[name]

    points = {name: [] for name in (HOUSING, *DRAWERS)}
    for n in nodes.values():
        if n.get('role') == 'collision':
            vertices = np.c_[n['vertices'], np.ones(len(n['vertices']))]
            local = np.linalg.inv(world(n['body'])) @ world(n['name'])
            points[n['body']].append((vertices @ local.T)[:, :3])
    return {name: np.unique(np.concatenate(v), axis=0) for name, v in points.items()}


class _Record:
    """Read every native replica once per sample, then expose scalar columns."""

    def __init__(self, ctx, instances):
        from pxr import UsdPhysics
        self.ctx, self.instances = ctx, list(instances)
        self.geometry = _geometry(ctx.asset_path)
        self.frames = {}
        self.key, self.cache = None, {}
        for instance in self.instances:
            joints = []
            for name in JOINTS:
                joint = UsdPhysics.PrismaticJoint(instance.joint(name))
                assert abs(joint.GetLowerLimitAttr().Get()) < 1e-8
                assert abs(joint.GetUpperLimitAttr().Get() - TRAVEL) < 1e-7
                endpoints = []
                for i in (0, 1):
                    target = str(getattr(joint, f'GetBody{i}Rel')().GetTargets()[0])
                    body = ctx.session.stage.GetPrimAtPath(target).GetCustomDataByKey('table1000:name')
                    q = getattr(joint, f'GetLocalRot{i}Attr')().Get()
                    endpoints.append((instance.indices[body],
                                      np.asarray(getattr(joint, f'GetLocalPos{i}Attr')().Get()),
                                      rotation([q.GetReal(), *q.GetImaginary()])))
                joints.append((endpoints, 'XYZ'.index(joint.GetAxisAttr().Get())))
            self.frames[instance.name] = joints
            for e in range(ctx.num_envs):
                for name, unit in [('q1', 'm'), ('q2', 'm'), ('v1', 'm/s'), ('v2', 'm/s'),
                                   ('anchor_error', 'm'), ('angle_error', 'rad'),
                                   ('root_x', 'm'), ('root_y', 'm'), ('root_z', 'm'),
                                   ('tilt', 'rad'), ('root_speed', 'm/s'),
                                   ('speed', 'm/s'), ('spin', 'rad/s'), ('min_z', 'm')]:
                    col = f'e{e}.{instance.name}.{name}'
                    ctx.observe(col, lambda e=e, n=instance.name, k=name: self.state(n)[k][e], unit=unit)

    def state(self, name):
        key = self.ctx.session.steps
        if key != self.key:
            self.key, self.cache = key, {}
        if name in self.cache:
            return self.cache[name]
        if name not in self.ctx.session.instances:
            return {k: np.zeros(self.ctx.num_envs) for k in
                    ('q1', 'q2', 'v1', 'v2', 'anchor_error', 'angle_error',
                     'root_x', 'root_y', 'root_z', 'tilt', 'root_speed', 'speed', 'spin', 'min_z')}
        instance = self.ctx.session.instances[name]
        p, r, com = instance.environment_states()
        velocity = instance.view.view.get_velocities().detach().cpu().numpy().reshape(self.ctx.num_envs, -1, 6)
        root = instance.indices[HOUSING]
        data = {f'root_{a}': p[:, root, i] for i, a in enumerate('xyz')}
        data.update(tilt=np.arccos(np.clip(r[:, root, 2, 2], -1, 1)),
                    root_speed=np.linalg.norm(velocity[:, root, :3], axis=1),
                    speed=np.linalg.norm(velocity[..., :3], axis=2).max(axis=1),
                    spin=np.linalg.norm(velocity[..., 3:], axis=2).max(axis=1))
        anchor_errors, angle_errors = [], []
        for j, (endpoints, axis_index) in enumerate(self.frames[name], 1):
            anchors = []
            for i, offset, local_r in endpoints:
                a = p[:, i] + np.einsum('eij,j->ei', r[:, i], offset)
                rr = r[:, i] @ local_r
                v = velocity[:, i, :3] + np.cross(velocity[:, i, 3:], a-com[:, i])
                anchors.append((a, rr, v))
            a, b = anchors
            axis = a[1][:, :, axis_index]
            delta = b[0]-a[0]
            q = np.einsum('ei,ei->e', delta, axis)
            data[f'q{j}'] = q
            data[f'v{j}'] = np.einsum('ei,ei->e', b[2]-a[2], axis)
            anchor_errors.append(np.linalg.norm(delta-q[:, None]*axis, axis=1))
            relative = np.swapaxes(a[1], 1, 2) @ b[1]
            angle_errors.append(np.linalg.norm(Rotation.from_matrix(relative).as_rotvec(), axis=1))
        data['anchor_error'] = np.max(anchor_errors, axis=0)
        data['angle_error'] = np.max(angle_errors, axis=0)
        minima = []
        for body, points in self.geometry.items():
            i = instance.indices[body]
            minima.append(p[:, i, 2] + np.einsum('ei,vi->ev', r[:, i, 2, :], points).min(axis=1))
        data['min_z'] = np.min(minima, axis=0)
        self.cache[name] = data
        return data

    def series(self, name='asset', environment=0):
        prefix = f'e{environment}.{name}.'
        return {k[len(prefix):]: np.array([row[k] for row in self.ctx.rows])
                for k in self.ctx.rows[0] if k.startswith(prefix)}


def _view(ctx, *, wide=False):
    ctx.camera(position=(1.22 if wide else .96, -1.65, 1.05),
               target=(0, -.08, .22), focal_length=28)


def _force(ctx, name, instance, drawer, frames):
    ctx.force(name, instance.body(DRAWERS[drawer-1]), frame='body', point_frame='body',
              keyframes=[{'time': t, 'force': (0, -f, 0), 'point': KNOB} for t, f in frames])


def _integrity(ctx, record, instances=None, *, ground=False):
    out = []
    for instance in instances or record.instances:
        for e in range(ctx.num_envs):
            s = record.series(instance.name, e)
            assert max(s['anchor_error']) < .0005, (instance.name, e, 'anchor', max(s['anchor_error']))
            assert max(s['angle_error']) < math.radians(.5), (instance.name, e, 'axis')
            for j in (1, 2):
                assert min(s[f'q{j}']) > -.0005 and max(s[f'q{j}']) < TRAVEL+.0005, (instance.name, e, j, 'limit')
            if ground:
                assert min(s['min_z']) > -.002, (instance.name, e, 'ground', min(s['min_z']))
            assert np.isfinite(np.array(list(s.values()))).all()
            out.append({'instance': instance.name, 'environment': e,
                        'anchor_error_m': float(max(s['anchor_error'])),
                        'angle_error_rad': float(max(s['angle_error'])),
                        'min_collision_z_m': float(min(s['min_z']))})
    return out


def _slide(ctx, drawer):
    ctx.camera(position=(1.25, -2.12, 1.30),
               target=(0, -.08, .22), focal_length=28)
    rec = _Record(ctx, [ctx.asset])
    _force(ctx, 'pull', ctx.asset, drawer,
           [(0, 0), (.4, 0), (1.6, .8), (1.8, .8), (1.804, 0),
            (2.6, 0), (2.604, -1.4), (3.4, -1.4), (3.404, 0), (4, 0)])
    ctx.plots('pull.force.y', f'e0.asset.q{drawer}', f'e0.asset.v{drawer}',
              f'e0.asset.q{3-drawer}', 'e0.asset.anchor_error')
    with ctx.fixed(ctx.asset.body(HOUSING)):
        ctx.run(4)
        t = np.array([r['time'] for r in ctx.rows])
        results = []
        for e in range(ctx.num_envs):
            s = rec.series(environment=e)
            q, v = s[f'q{drawer}'], s[f'v{drawer}']
            onset = np.flatnonzero((t <= 1.8) & (q > .0005) & (v > .002))
            assert len(onset), (e, 'no onset')
            load = -ctx.rows[onset[0]]['pull.force.y']
            assert .35 <= load <= .8, (e, 'onset force', load)
            peak = max(q[t < 2.6])
            assert .005 < peak < .190, (e, 'slide travel', peak)
            release = np.flatnonzero(t >= 1.804-1e-8)[0]
            coast = np.flatnonzero((t >= 1.804) & (t <= 2.6))
            stopped = [i for i in coast if np.max(np.abs(v[i:coast[-1]+1])) < .002]
            assert stopped, (e, 'did not stop')
            stop_time = t[stopped[0]]-t[release]
            distance = max(q[coast])-q[release]
            assert stop_time < .5 and distance < .025, (e, stop_time, distance)
            assert abs(q[-1]) < .001, (e, 'did not close', q[-1])
            idle = max(abs(s[f'q{3-drawer}']))
            assert idle < .0002, (e, 'idle moved', idle)
            results.append({'environment': e, 'onset_force_N': load,
                            'onset_time_s': float(t[onset[0]]), 'peak_q_m': float(peak),
                            'stop_time_s': float(stop_time), 'stop_distance_m': float(distance),
                            'idle_max_q_m': float(idle), 'final_q_m': float(q[-1])})
        integrity = _integrity(ctx, rec)
    ctx.metric('onset_force', max(r['onset_force_N'] for r in results), unit='N', label='最大环境起滑外载荷')
    ctx.metric('stop_distance', max(r['stop_distance_m'] for r in results)*1000, unit='mm', label='最大撤力后停止距离')
    ctx.metric('stop_time', max(r['stop_time_s'] for r in results), unit='s', label='最大撤力后停止时间')
    ctx.metric('idle_drift', max(r['idle_max_q_m'] for r in results)*1000, unit='mm', label='闲置抽屉最大位移')
    ctx.experiment_details = {'contract': 'cabinet-000001-v1', 'drawer': drawer,
                              'force_point_body_m': KNOB, 'force_is_applied_not_reaction': True,
                              'per_environment': results, 'integrity': integrity}


def drawer1_slide(ctx: TestContext):
    _slide(ctx, 1)


def drawer2_slide(ctx: TestContext):
    _slide(ctx, 2)


@contextmanager
def _fixed_housing_base(ctx):
    """Static test-only world joint, declared before initialization.

    Moving grips require six coordinates; a stationary housing can instead be
    the native fixed base. Its original solver/self-collision attributes remain.
    The temporary joint lives inside the instance and is removed at teardown.
    """
    from pxr import UsdPhysics, PhysxSchema
    assert not ctx.initialized
    instance = ctx.asset
    housing = ctx.session.stage.GetPrimAtPath(instance.body(HOUSING).path)
    source = PhysxSchema.PhysxArticulationAPI(housing)
    position_iterations = source.GetSolverPositionIterationCountAttr().Get()
    velocity_iterations = source.GetSolverVelocityIterationCountAttr().Get()
    self_collision = source.GetEnabledSelfCollisionsAttr().Get()
    anchor = UsdPhysics.FixedJoint.Define(ctx.session.stage, instance.path+'/TestHousingAnchor')
    anchor.CreateBody1Rel().SetTargets([housing.GetPath()])
    UsdPhysics.ArticulationRootAPI.Apply(anchor.GetPrim())
    api = PhysxSchema.PhysxArticulationAPI.Apply(anchor.GetPrim())
    api.CreateSolverPositionIterationCountAttr(position_iterations)
    api.CreateSolverVelocityIterationCountAttr(velocity_iterations)
    api.CreateEnabledSelfCollisionsAttr(self_collision)
    housing.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    instance.fixture_root = str(anchor.GetPath())
    yield


def travel_stops(ctx: TestContext):
    _view(ctx)
    ctx.camera(position=(1.25, -2.25, 1.35), target=(0, -.12, .20))
    rec = _Record(ctx, [ctx.asset])
    for j, offset in ((1, 0), (2, 3.8)):
        schedule = [(0, 0), (.2+offset, 0), (.204+offset, 1.4),
                    (1.8+offset, 1.4), (1.804+offset, 0), (2.1+offset, 0),
                    (2.104+offset, -1.4), (3.7+offset, -1.4), (3.704+offset, 0)]
        if j == 1:
            schedule += [(7.8, 0), (7.804, 1.4), (8.2, 1.4), (8.204, 0)]
        schedule += [(8.6, 0)]
        _force(ctx, f'load{j}', ctx.asset, j, schedule)
    ctx.plots('load1.force.y', 'e0.asset.q1', 'e0.asset.v1',
              'load2.force.y', 'e0.asset.q2', 'e0.asset.anchor_error')
    with _fixed_housing_base(ctx):
        ctx.run(8.6)
        t = np.array([row['time'] for row in ctx.rows])
        results = []
        for e in range(ctx.num_envs):
            s = rec.series(environment=e)
            for j, offset in ((1, 0), (2, 3.8)):
                q = s[f'q{j}']
                open_dwell = (t >= 1.6+offset) & (t <= 1.8+offset)
                closed_dwell = (t >= 3.5+offset) & (t <= 3.7+offset)
                assert max(abs(q[open_dwell]-TRAVEL)) < .001, (e, j, 'open dwell')
                assert max(abs(q[closed_dwell])) < .001, (e, j, 'closed dwell')
                release = (t >= 1.804+offset) & (t <= 2.1+offset)
                rebound = max(q[release])-min(q[release])
                assert rebound < .002, (e, j, 'rebound', rebound)
                idle_phase = (t >= .2+offset) & (t <= 3.7+offset)
                idle_q = s[f'q{3-j}'][idle_phase]
                idle_drift = float(max(idle_q)-min(idle_q))
                results.append({'environment': e, 'drawer': j,
                                'max_q_m': float(max(q)), 'min_q_m': float(min(q)),
                                'loaded_open_dwell_s': float(sum(open_dwell)*ctx.dt),
                                'loaded_closed_dwell_s': float(sum(closed_dwell)*ctx.dt),
                                'idle_q_range_m': idle_drift,
                                'release_rebound_m': float(rebound)})
            assert s['q1'][np.argmin(abs(t-8.2))] > .005, (e, 'reverse jam')
        integrity = _integrity(ctx, rec)
    ctx.metric('max_travel', max(r['max_q_m'] for r in results)*1000, unit='mm', label='双抽屉最大拉出行程')
    ctx.metric('max_overshoot', max(max(r['max_q_m']-TRAVEL, -r['min_q_m'], 0) for r in results)*1000,
               unit='mm', label='双端最大越限')
    ctx.metric('max_rebound', max(r['release_rebound_m'] for r in results)*1000, unit='mm', label='限位撤力最大回弹')
    ctx.metric('idle_drift', max(r['idle_q_range_m'] for r in results)*1000, unit='mm', label='限位持续加载闲置抽屉最大位移范围')
    ctx.experiment_details = {'contract': 'cabinet-000001-v1', 'load_N': 1.4,
                              'force_point_body_m': KNOB, 'per_environment': results,
                              'dt_s': ctx.dt,
                              'fixture': 'native fixed-base world joint; no six-axis proxy links',
                              'original_six_axis_fixture_idle_drift_failed': True,
                              'integrity': integrity}
    assert max(r['idle_q_range_m'] for r in results) < .0002, ('idle drift', results)


def free_housing_pull(ctx: TestContext):
    ctx.initial(position=(0, 0, .002))
    ctx.ground(static_friction=.55, dynamic_friction=.40, contact_offset=.0001)
    ctx.camera(position=(2.1, -3.4, 1.9), target=(-.2, -.55, .2))
    for body in ctx.asset.body_names:
        ctx.observe_body(ctx.asset.body(body))
    rec = _Record(ctx, [ctx.asset])
    _force(ctx, 'pull', ctx.asset, 1,
           [(0, 0), (.4, 0), (.6, 1), (3, 1), (3.4, 32), (3.8, 32), (3.804, 0), (5.2, 0)])
    ctx.plots('pull.force.y', 'e0.asset.q1', 'e0.asset.q2',
              'e0.asset.root_y', 'e0.asset.tilt', 'e0.asset.min_z')
    ctx.run(5.2)
    t = np.array([row['time'] for row in ctx.rows])
    results = []
    instance = ctx.asset
    centers = np.array([instance.body(n).state['com'] for n in instance.body_names])
    total_mass = float(sum(instance.mass))
    for e in range(ctx.num_envs):
        s = rec.series(environment=e)
        low = t <= 3+1e-8
        low_motion = np.hypot(s['root_x'][low]-s['root_x'][0], s['root_y'][low]-s['root_y'][0]).max()
        assert max(s['q1'][low]) > .205, (e, 'low load did not reach limit')
        assert low_motion < .005, (e, 'low-load root motion', low_motion)
        movement = np.hypot(s['root_x']-s['root_x'][0], s['root_y']-s['root_y'][0]).max()
        assert movement > .01 or max(s['tilt']) > math.radians(2), (e, 'no root response')
        assert max(s['speed']) < 3 and max(s['spin']) < 10, (e, 'unbounded response')
        results.append({'environment': e, 'low_load_root_motion_m': float(low_motion),
                        'root_max_horizontal_motion_m': float(movement),
                        'root_max_tilt_rad': float(max(s['tilt'])),
                        'final_root_speed_m_s': float(s['root_speed'][-1]),
                        'idle_max_q_m': float(max(s['q2']))})
    integrity = _integrity(ctx, rec, ground=True)
    ctx.metric('root_displacement', max(r['root_max_horizontal_motion_m'] for r in results)*1000,
               unit='mm', label='自由柜体最大水平位移')
    ctx.metric('root_tilt', max(r['root_max_tilt_rad'] for r in results)*180/math.pi,
               unit='deg', label='自由柜体最大倾角')
    ctx.experiment_details = {'contract': 'cabinet-000001-v1', 'fixture': None,
                              'total_mass_kg': total_mass, 'static_slide_estimate_N': total_mass*9.81*.55,
                              'final_com_env0_m': np.average(centers, axis=0, weights=instance.mass).tolist(),
                              'handle_initial_height_m': .1989,
                              'per_environment': results, 'integrity': integrity}


def gravity_tilt(ctx: TestContext):
    from pxr import UsdPhysics
    instances = [ctx.asset, ctx.spawn(ctx.asset_path, name='lower', position=(.6, 0, .3))]
    ctx.initial(instances[0], position=(-.6, 0, .3))
    ctx.initial(instances[1], position=(.6, 0, .3))
    rec = _Record(ctx, instances)
    # Test-only idle constraints are authored before initialization. The other
    # drawer remains a passive native slider throughout the tilt trajectory.
    for instance, idle in zip(instances, (2, 1)):
        joint = UsdPhysics.PrismaticJoint(instance.joint(JOINTS[idle-1]))
        joint.CreateLowerLimitAttr(0)
        joint.CreateUpperLimitAttr(0)
    grips = [ctx.grip(i.body(HOUSING), name=f'hold_{i.name}') for i in instances]
    ctx.camera(position=(2.2, -3.2, 1.9), target=(0, -.05, .40))
    for instance in instances:
        for body in instance.body_names:
            ctx.observe_body(instance.body(body))
    ctx.plots('e0.asset.tilt', 'e0.asset.q1', 'e0.asset.q2',
              'e0.lower.tilt', 'e0.lower.q2', 'e0.lower.q1')
    for i in instances:
        ctx.observe(f'{i.name}.axial_gravity',
                    lambda name=i.name: .35*9.81*math.sin(rec.state(name)['tilt'][0]), unit='N')
    ctx.run(.4)
    a = math.radians(35)
    motions = [g.move_to(rotation=(math.cos(a/2), math.sin(a/2), 0, 0), duration=6, wait=False) for g in grips]
    ctx.wait(*motions)
    ctx.run(.4)
    results = []
    for instance, drawer in zip(instances, (1, 2)):
        for e in range(ctx.num_envs):
            s = rec.series(instance.name, e)
            onset = np.flatnonzero((s[f'q{drawer}'] > .0005) & (s[f'v{drawer}'] > .002))
            assert len(onset), (instance.name, e, 'no tilt onset')
            angle = float(s['tilt'][onset[0]])
            assert math.radians(3) < angle < math.radians(25), (instance.name, e, angle)
            assert max(s[f'q{drawer}']) > .005
            assert max(abs(s[f'q{3-drawer}'])) < .0002, (instance.name, e, 'locked idle')
            assert abs(s['tilt'][-1]-a) < math.radians(.5)
            results.append({'instance': instance.name, 'environment': e, 'drawer': drawer,
                            'onset_angle_rad': angle, 'axial_gravity_at_onset_N': .35*9.81*math.sin(angle),
                            'final_q_m': float(s[f'q{drawer}'][-1])})
    integrity = _integrity(ctx, rec)
    for drawer in (1, 2):
        ctx.metric(f'drawer{drawer}_onset_angle',
                   max(r['onset_angle_rad'] for r in results if r['drawer'] == drawer)*180/math.pi,
                   unit='deg', label=f'抽屉{drawer}最大环境实际重力起滑角')
    ctx.experiment_details = {'contract': 'cabinet-000001-v1', 'ideal_effort_angle_rad': math.asin(.5/(.35*9.81)),
                              'comparison': 'one explicitly locked idle drawer in each rack; other drawer passive',
                              'per_environment': results, 'integrity': integrity,
                              'grip_wrench_not_used_as_friction_measurement': True}


def rest_drop_multi(ctx: TestContext):
    instances = [ctx.asset, ctx.spawn(ctx.asset_path, name='half'), ctx.spawn(ctx.asset_path, name='open')]
    for i, (instance, q) in enumerate(zip(instances, (0, TRAVEL/2, TRAVEL))):
        ctx.initial(instance, position=((i-1)*1.05, 0, .01), linear_velocity=(.02, 0, 0),
                    joints={j: q for j in JOINTS})
    ctx.ground(static_friction=.55, dynamic_friction=.40, contact_offset=.0001)
    ctx.camera(position=(2.2, -3.5, 1.9), target=(0, -.04, .2))
    for instance in instances:
        for body in instance.body_names:
            ctx.observe_body(instance.body(body))
    rec = _Record(ctx, instances)
    ctx.plots('e0.asset.root_z', 'e0.half.root_z', 'e0.open.root_z',
              'e0.open.tilt', 'e0.open.q1', 'e0.open.min_z')
    ctx.run(3)
    results = []
    for instance in instances:
        for e in range(ctx.num_envs):
            s = rec.series(instance.name, e)
            assert max(s['speed'][-26:]) < .02, (instance.name, e, 'not settled', max(s['speed'][-26:]))
            assert max(s['spin'][-26:]) < .1, (instance.name, e, 'not angularly settled')
            results.append({'instance': instance.name, 'environment': e,
                            'final_max_body_speed_m_s': float(max(s['speed'][-26:])),
                            'max_tilt_rad': float(max(s['tilt'])), 'final_q_m': [float(s[f'q{j}'][-1]) for j in (1, 2)]})
    integrity = _integrity(ctx, rec, ground=True)
    ctx.metric('min_collision_z', min(r['min_collision_z_m'] for r in integrity)*1000, unit='mm', label='最低碰撞顶点高度')
    ctx.metric('settled_speed', max(r['final_max_body_speed_m_s'] for r in results)*1000, unit='mm/s', label='落座后最大速度')
    ctx.experiment_details = {'contract': 'cabinet-000001-v1', 'drop_height_m': .01,
                              'instances_per_environment': 3, 'initial_q_m': [0, TRAVEL/2, TRAVEL],
                              'per_environment': results, 'integrity': integrity}


def lifecycle_reset_reload(ctx: TestContext):
    """Two true stopped-stage removals, preserving a native sibling tree."""
    from pxr import Sdf, Usd, UsdPhysics
    from isaacsim.core.simulation_manager import SimulationManager
    import torch

    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, 0, .6), joints={JOINTS[0]: .03, JOINTS[1]: .08})
    retained = ctx.spawn(ctx.asset_path, name='retained', position=(-1.1, 0, .6))
    ctx.initial(retained, position=(-1.1, 0, .6), linear_velocity=(.01, 0, 0),
                joints={JOINTS[0]: .04, JOINTS[1]: .06})
    rec = _Record(ctx, [ctx.asset, retained])
    ctx.observe('loaded_present', lambda: float('asset' in ctx.session.instances), unit='1')
    ctx.camera(position=(1.3, -3, 1.7), target=(-.45, 0, .8))
    ctx.plots('loaded_present', 'pull.force.y', 'e0.asset.q1', 'e0.asset.v1',
              'e0.retained.root_x', 'e0.retained.q1')
    ctx.ensure_initialized()
    stage, session = ctx.session.stage, ctx.session
    identifier = stage.GetRootLayer().identifier
    subscriptions = session.prepare_subscription, session.flush_subscription

    def snapshot(instance):
        view = instance.articulation.view
        return {k: getattr(view, f'get_{k}')().clone() for k in
                ('root_transforms', 'root_velocities', 'dof_positions', 'dof_velocities')}

    def restore(instance, saved):
        view = instance.articulation.view
        for k, value in saved.items():
            indices = torch.arange(ctx.num_envs, device=value.device, dtype=torch.int32)
            getattr(view, f'set_{k}')(value, indices)
        SimulationManager.get_physics_sim_view().update_articulations_kinematic()
        ctx.world.forward()
        session.refresh()
        rec.key = None
        read = snapshot(instance)
        per_env = [max(float((read[k][e]-saved[k][e]).abs().max().item()) for k in saved)
                   for e in range(ctx.num_envs)]
        assert max(per_env) < 1e-5, ('lifecycle restore', per_env)
        return per_env

    initial = snapshot(ctx.asset)
    events, loaded_phases = [], []

    def loaded_phase():
        start = session.time
        before = rec.state('asset')['q1'].copy()
        ctx.actions.pop('pull', None)
        _force(ctx, 'pull', ctx.asset, 1,
               [(start, 0), (start+.1, 0), (start+.4, .8), (start+.6, .8),
                (start+.604, 0), (start+.8, 0)])
        ctx.run(.8)
        change = rec.state('asset')['q1']-before
        assert min(change) > .005, ('reloaded force response', change)
        loaded_phases.append({'start_s': start, 'q1_change_m': change.tolist()})

    loaded_phase()
    retained_before = snapshot(retained)
    session.active = False
    ctx.world.stop()
    ctx.world.reset()
    session.initialize()
    reset_error = restore(ctx.asset, initial)
    retained_error = restore(retained, retained_before)
    events.append({'kind': 'native_stop_reset', 'time_s': session.time,
                   'initial_state_error_per_environment': reset_error,
                   'retained_state_error_per_environment': retained_error})
    loaded_phase()
    for cycle in range(2):
        old = ctx.asset
        before = snapshot(retained)
        session.active = False
        ctx.world.stop()
        paths = [session.replica_path(old.path, e) for e in range(ctx.num_envs)]
        for path in paths:
            assert stage.GetPrimAtPath(path)
            stage.RemovePrim(path)
        session.unregister(old.path)
        ctx.actions.pop('pull')
        if ctx.app:
            for _ in range(5):
                ctx.app.update()
        assert all(not stage.GetPrimAtPath(p) for p in paths)
        ctx.world.reset()
        session.initialize()
        removed_error = restore(retained, before)
        rec.key = None
        absent_time = session.time
        ctx.run(.2)
        after = snapshot(retained)
        dx = (after['root_transforms'][:, :3]-before['root_transforms'][:, :3]).cpu().numpy()
        assert np.max(abs(dx-np.array([.002, 0, 0]))) < 1e-5, ('retained motion', dx)
        session.active = False
        ctx.world.stop()
        replacement = ctx.spawn(ctx.asset_path, name='asset', position=(0, 0, .6))
        ctx.asset = replacement
        for e in range(1, ctx.num_envs):
            assert Sdf.CopySpec(stage.GetRootLayer(), replacement.path,
                                stage.GetRootLayer(), session.replica_path(replacement.path, e))
        ctx.world.reset()
        session.initialize()
        fresh_error = restore(replacement, initial)
        sibling_error = restore(retained, after)
        counts = []
        for e in range(ctx.num_envs):
            tree = list(Usd.PrimRange(stage.GetPrimAtPath(session.replica_path(replacement.path, e))))
            count = [sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in tree),
                     sum(p.IsA(UsdPhysics.Joint) for p in tree)]
            assert count == [3, 2], (e, count)
            counts.append(count)
        assert replacement is not old
        assert session.prepare_subscription is subscriptions[0] and session.flush_subscription is subscriptions[1]
        assert stage.GetRootLayer().identifier == identifier and ctx.session is session
        events.append({'kind': 'remove_reference_reload', 'cycle': cycle+1, 'absent_start_s': absent_time,
                       'reload_time_s': session.time, 'all_replica_prims_removed': True,
                       'new_reference_object': True, 'body_joint_counts_per_environment': counts,
                       'fresh_state_error_per_environment': fresh_error,
                       'retained_state_error_per_environment': sibling_error,
                       'removal_state_error_per_environment': removed_error,
                       'retained_displacement_while_absent_m': dx.tolist()})
        loaded_phase()
    for e in range(ctx.num_envs):
        idle = rec.series('retained', e)
        assert max(idle['q1'])-min(idle['q1']) < .0002
        assert max(idle['q2'])-min(idle['q2']) < .0002
    integrity = _integrity(ctx, rec)
    ctx.metric('reset_error', max(reset_error), unit='1', label='原生停止复位最大状态分量误差')
    ctx.metric('reload_error', max(max(e['fresh_state_error_per_environment']) for e in events if 'cycle' in e),
               unit='1', label='真实移除重载最大初态误差')
    ctx.experiment_details = {'contract': 'cabinet-000001-v1', 'events': events,
                              'loaded_phases': loaded_phases, 'integrity': integrity,
                              'same_stage_session_callbacks': True,
                              'state_restore_only_at_stopped_initialization_boundaries': True,
                              'gravity_m_s2': [0, 0, 0], 'no_behavior_or_GUI_claim': True,
                              'exclude_from_steady_state_benchmark': True}


TESTS = {'drawer1_slide': drawer1_slide, 'drawer2_slide': drawer2_slide,
         'travel_stops': travel_stops, 'free_housing_pull': free_housing_pull,
         'gravity_tilt': gravity_tilt, 'rest_drop_multi': rest_drop_multi,
         'lifecycle_reset_reload': lifecycle_reset_reload}
