"""Rigid remote: face/back/side contact, placement, hole and whole-body loads.

SI units, wxyz rotations. Recorded forces are applied loads, not reactions.
The static pin is an ideal experiment fixture, not an asset component.
"""
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from table_1000.physics.testing import TestContext
from table_1000.physics.test import ground_clearance

BODY = 'remote_control.body'
MASS_KG = .060
BACK = (1, 0, 0, 0)
FACE = (0, 1, 0, 0)
SIDE = (math.sqrt(.5), math.sqrt(.5), 0, 0)


def _col(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def _body(ctx, instance):
    b = instance.body(BODY)
    ctx.observe_body(b)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(b.state['velocity'][:3]), unit='m/s')
    if not hasattr(ctx, '_remote_histories'):
        ctx._remote_histories = {}
    history = ctx._remote_histories[instance.name] = []
    def sample_environments():
        positions, rotations, _ = instance.environment_states()
        velocities = instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, 1, 6)[:, 0]
        history.append((ctx.session.time, positions[:, 0].copy(), rotations[:, 0].copy(), velocities.copy()))
        return float(np.linalg.norm(velocities[:, :3], axis=1).max())
    ctx.observe(instance.name+'.all_env_max_speed', sample_environments, unit='m/s')
    return b


def _vertices(ctx):
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    worlds, points = {}, []
    for node in model['nodes']:
        worlds[node['name']] = worlds.get(node['parent'], np.eye(4)) @ np.array(node['matrix'])
        if node.get('role') == 'collision':
            local = np.linalg.inv(worlds[BODY]) @ worlds[node['name']]
            points.extend((np.c_[node['vertices'], np.ones(len(node['vertices']))] @ local.T)[:, :3])
    return np.array(points)


def _height(ctx, q, clearance):
    r = Rotation.from_quat(np.array(q)[[1, 2, 3, 0]]).as_matrix()
    return float(clearance-(_vertices(ctx) @ r.T)[:, 2].min())


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.00005)


def _stable(ctx, names):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        positions = np.array([[r[_col(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert max(r[name+'.speed'] for r in tail) < .01, name+' speed'
        assert np.linalg.norm(np.ptp(positions, axis=0)) < .001, name+' settling'


def _contact(ctx):
    result = ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()),
                              ctx.rows, ctx.session.instances, 0)
    assert result['maximum_geometry_penetration_m'] < .002, result
    return result


def _replicas(ctx, *, stable=False, contact=False):
    """Apply the original stability/contact bounds to every environment."""
    report = {}
    vertices = _vertices(ctx) if contact else None
    for name, history in ctx._remote_histories.items():
        times = np.array([h[0] for h in history])
        positions = np.stack([h[1] for h in history])
        velocities = np.stack([h[3] for h in history])
        tail = times >= ctx.session.time-.5-1e-8
        speed = np.linalg.norm(velocities[tail, :, :3], axis=2).max(axis=0)
        drift = np.linalg.norm(np.ptp(positions[tail], axis=0), axis=1)
        if stable:
            assert np.all(speed < .01), (name, speed)
            assert np.all(drift < .001), (name, drift)
        entry = {'tail_max_speed_m_s': speed.tolist(), 'tail_position_range_m': drift.tolist(),
                 'final_positions_m': positions[-1].tolist()}
        if contact:
            minimum = np.full(ctx.num_envs, np.inf)
            for _, p, r, _ in history:
                z = np.einsum('ej,pj->ep', r[:, 2, :], vertices) + p[:, 2, None]
                minimum = np.minimum(minimum, z.min(axis=1))
            overlap = np.maximum(0, -minimum)
            assert np.all(overlap < .002), (name, overlap)
            entry['maximum_geometry_penetration_m'] = overlap.tolist()
        report[name] = entry
    ctx.experiment_details['per_environment'] = report
    ctx.metric('environments_checked', ctx.num_envs, unit='1', label='独立检查环境数')
    if contact:
        ctx.metric('maximum_penetration', max(max(v['maximum_geometry_penetration_m']) for v in report.values())*1000,
                   unit='mm', label='全部环境最大穿地')
    return report


def slide_faces(ctx: TestContext):
    _ground(ctx)
    ctx.initial(position=(0, -.12, _height(ctx, BACK, 0)), rotation=BACK)
    face = ctx.spawn(ctx.asset_path, name='face', position=(0, 0, _height(ctx, FACE, 0)), rotation=FACE)
    side = ctx.spawn(ctx.asset_path, name='side', position=(0, .12, _height(ctx, SIDE, 0)), rotation=SIDE)
    bodies = [_body(ctx, i) for i in (ctx.asset, face, side)]
    for name, b in zip(('back_push', 'face_push', 'side_push'), bodies):
        ctx.force(name, b, keyframes=[{'time': 0, 'force': (0, 0, 0)},
            {'time': .5, 'force': (0, 0, 0)}, {'time': 1., 'force': (.32, 0, 0)},
            {'time': 1.08, 'force': (.32, 0, 0)}, {'time': 1.12, 'force': (0, 0, 0)}])
    ctx.camera(position=(.38, -.50, .48), target=(.06, 0, .01))
    ctx.video_annotation = lambda: 'Lower to upper: back / button face / side; equal COM +X ramp then release'
    ctx.plots('back_push.force.x', *[_col(n, 'position', 'x') for n in ('asset', 'face', 'side')],
              *[n+'.speed' for n in ('asset', 'face', 'side')])
    ctx.run(3.2)
    _stable(ctx, ('asset', 'face', 'side'))
    measurements = {}
    for name, load in zip(('asset', 'face', 'side'), ('back_push', 'face_push', 'side_push')):
        start = min(ctx.rows, key=lambda r: abs(r['time']-.5))
        release = min(ctx.rows, key=lambda r: abs(r['time']-1.12))
        shift = ctx.rows[-1][_col(name, 'position', 'x')]-start[_col(name, 'position', 'x')]
        assert shift > .005, (name, shift)
        onset = next(r for r in ctx.rows if r['time'] >= .5 and r[_col(name, 'position', 'x')]-start[_col(name, 'position', 'x')] > .0005)
        measurements[name] = {'travel_m': shift, 'onset_applied_N': onset[load+'.force.x'],
                             'release_stop_distance_m': ctx.rows[-1][_col(name, 'position', 'x')]-release[_col(name, 'position', 'x')]}
        ctx.metric(name+'_onset', measurements[name]['onset_applied_N'], unit='N', label=name+' 起滑载荷')
        ctx.metric(name+'_stop_distance', measurements[name]['release_stop_distance_m']*1000,
                   unit='mm', label=name+' 撤力停止距离')
    ctx.experiment_details = {'contract': 'back/face/side equal COM push and release; free rotations',
        'measurements': measurements, 'geometry_contact': _contact(ctx),
        'boundary': 'effective support responses, not isolated friction measurement; side may tip'}
    _replicas(ctx, stable=True, contact=True)
    for name, history in ctx._remote_histories.items():
        start = min(history, key=lambda h: abs(h[0]-.5))
        travel = history[-1][1][:, 0]-start[1][:, 0]
        assert np.all(travel > .005), (name, travel)
        ctx.experiment_details['per_environment'][name]['travel_m'] = travel.tolist()


def drop_orientations(ctx: TestContext):
    # 4 ms exceeded the fixed 2 mm geometric overlap contract at impact.
    ctx.simulation(dt=.002)
    _ground(ctx)
    ctx.initial(position=(-.13, 0, _height(ctx, BACK, .03)), rotation=BACK)
    face = ctx.spawn(ctx.asset_path, name='face', position=(0, 0, _height(ctx, FACE, .03)), rotation=FACE)
    side = ctx.spawn(ctx.asset_path, name='side', position=(.13, 0, _height(ctx, SIDE, .03)), rotation=SIDE)
    for i in (ctx.asset, face, side):
        _body(ctx, i)
    ctx.camera(position=(.35, -.48, .36), target=(0, 0, .025))
    ctx.video_annotation = lambda: '30 mm lowest-point placement: back / button face / side; no fixture'
    ctx.plots(*[_col(n, 'position', 'z') for n in ('asset', 'face', 'side')],
              *[n+'.speed' for n in ('asset', 'face', 'side')])
    ctx.run(3.2)
    _stable(ctx, ('asset', 'face', 'side'))
    ctx.experiment_details = {'release_clearance_m': .03, 'geometry_contact': _contact(ctx),
                              'boundary': 'rigid placement only; no damage, button action or battery motion'}
    _replicas(ctx, stable=True, contact=True)


def flip_release(ctx: TestContext):
    # GPU 4 ms penetrated 2.423 mm: retain the 2 mm bound with finer steps.
    if ctx.physics == 'gpu':
        ctx.simulation(dt=.002)
    _ground(ctx)
    ctx.initial(position=(-.07, 0, _height(ctx, BACK, 0)), rotation=BACK)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(.07, 0, _height(ctx, BACK, 0)))
    b = _body(ctx, ctx.asset); _body(ctx, idle)
    ctx.observe('asset.face_normal_z', lambda: b.state['rotation'][2, 2], unit='1')
    ctx.torque('roll', b, keyframes=[{'time': 0, 'torque': (0, 0, 0)},
        {'time': .5, 'torque': (0, 0, 0)}, {'time': .504, 'torque': (.010, 0, 0)},
        {'time': .624, 'torque': (.010, 0, 0)}, {'time': .628, 'torque': (0, 0, 0)}])
    ctx.camera(position=(.26, -.40, .28), target=(0, 0, .015))
    ctx.video_annotation = lambda: 'Left: +X roll torque then free release | right: untouched back-supported control'
    ctx.plots('roll.torque.x', 'asset.face_normal_z', _col('asset', 'angular_velocity', 'x'), 'asset.speed', 'idle.speed')
    ctx.run(3.2)
    assert min(r['asset.face_normal_z'] for r in ctx.rows) < -.5
    assert idle.body(BODY).state['rotation'][2, 2] > .98
    _stable(ctx, ('asset', 'idle'))
    ctx.experiment_details = {'contract': 'whole-body roll under torque then release; idle control', 'geometry_contact': _contact(ctx)}
    _replicas(ctx, stable=True, contact=True)
    normals = np.stack([h[2][:, 2, 2] for h in ctx._remote_histories['asset']])
    assert np.all(normals.min(axis=0) < -.5), normals.min(axis=0)
    assert np.all(ctx._remote_histories['idle'][-1][2][:, 2, 2] > .98)
    ctx.experiment_details['minimum_face_normal_z_per_environment'] = normals.min(axis=0).tolist()
    ctx.metric('minimum_face_normal_z', normals.min(), unit='1', label='加载面法线Z最小值')


def ring_contact(ctx: TestContext):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, -.055, .04))
    free = ctx.spawn(ctx.asset_path, name='free', position=(0, .055, .04))
    bodies = [_body(ctx, i) for i in (ctx.asset, free)]
    pin = UsdGeom.Cylinder.Define(ctx.session.stage, ctx.session.environment(0)+'/FixturePin')
    pin.CreateRadiusAttr(.00065); pin.CreateHeightAttr(.08); pin.CreateAxisAttr('Z')
    pin.AddTranslateOp().Set(Gf.Vec3d(.0445, -.065, .046))
    pin.CreateDisplayColorAttr([Gf.Vec3f(.7, .25, .08)])
    UsdPhysics.CollisionAPI.Apply(pin.GetPrim())
    api = PhysxSchema.PhysxCollisionAPI.Apply(pin.GetPrim())
    api.CreateContactOffsetAttr(.00005); api.CreateRestOffsetAttr(0)
    material = UsdShade.Material.Define(ctx.session.stage, '/World/PinMaterial')
    m = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    m.CreateStaticFrictionAttr(.25); m.CreateDynamicFrictionAttr(.18); m.CreateRestitutionAttr(0)
    UsdShade.MaterialBindingAPI.Apply(pin.GetPrim()).Bind(material, materialPurpose='physics')
    for name, b in zip(('pinned_load', 'free_load'), bodies):
        ctx.force(name, b, keyframes=[{'time': 0, 'force': (0, 0, 0)},
            {'time': .2, 'force': (0, 0, 0)}, {'time': .204, 'force': (.03, 0, 0)},
            {'time': .604, 'force': (.03, 0, 0)}, {'time': .608, 'force': (0, 0, 0)}])
    def hole_error():
        state = bodies[0].state
        hole = state['position'] + state['rotation'] @ np.array([.0445, -.010, .006])
        return np.linalg.norm(hole[:2]-np.array([.0445, -.065]))
    ctx.observe('pinned.hole_axis_offset', hole_error, unit='m')
    ctx.camera(position=(.42, -.46, .50), target=(.105, 0, .046))
    ctx.video_annotation = lambda: 'Lower: fixed ring around static 1.3 mm pin | upper: free; same COM load, zero gravity'
    ctx.plots('pinned_load.force.x', 'pinned.hole_axis_offset',
              _col('asset', 'angular_velocity', 'z'), _col('free', 'position', 'x'))
    ctx.run(1.6)
    maximum = max(r['pinned.hole_axis_offset'] for r in ctx.rows)
    assert maximum < .002, maximum
    assert bodies[1].state['position'][0] > .05
    ctx.experiment_details = {'contract': 'fixed hole contact vs unpinned free rigid body',
        'pin_radius_m': .00065, 'pin_height_m': .08, 'maximum_hole_axis_offset_m': maximum,
        'boundary': 'ideal static straight pin, zero gravity; no activity of ring, no hook retention or pull-off rating'}
    _replicas(ctx)
    offsets = np.stack([np.linalg.norm((p+np.einsum('eij,j->ei', r, [.0445, -.010, .006]))[:, :2]-[.0445, -.065], axis=1)
                        for _, p, r, _ in ctx._remote_histories['asset']]).max(axis=0)
    free_travel = ctx._remote_histories['free'][-1][1][:, 0]
    assert np.all(offsets < .002), offsets
    assert np.all(free_travel > .05), free_travel
    ctx.experiment_details['hole_offset_per_environment_m'] = offsets.tolist()
    ctx.experiment_details['free_travel_per_environment_m'] = free_travel.tolist()
    ctx.metric('maximum_hole_offset', offsets.max()*1000, unit='mm', label='全部环境最大孔心偏移')
    ctx.metric('minimum_free_travel', free_travel.min()*1000, unit='mm', label='全部环境自由件最小位移')


def free_eccentric(ctx: TestContext):
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.06, -.10, .06))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(-.06, 0, .06))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.06, .10, .06))
    bodies = [_body(ctx, i) for i in (ctx.asset, offset, idle)]
    for name, b in zip(('com_push', 'offset_push'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
            {'time': .404, 'force': (.06, 0, 0)}, {'time': .604, 'force': (.06, 0, 0)},
            {'time': .608, 'force': (0, 0, 0)}]
        if name == 'offset_push':
            frames[0]['point'] = (.0445, -.010, .006)
        ctx.force(name, b, point_frame='body', keyframes=frames)
    ctx.camera(position=(.34, -.47, .44), target=(.025, 0, .06))
    ctx.video_annotation = lambda: 'Zero gravity | lower: COM | middle: fixed-ring point | upper: untouched; equal impulse'
    ctx.plots('com_push.force.x', _col('asset', 'linear_velocity', 'x'),
              _col('offset', 'linear_velocity', 'x'), _col('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': list(map(float, errors)),
                              'contract': 'equal COM/ring-point impulses; idle control; one rigid body',
                              'asset_pose_writes_after_initialization': False}
    _replicas(ctx)
    all_errors = []
    for name in ('asset', 'offset'):
        velocity = ctx._remote_histories[name][-1][3]
        error = np.abs(MASS_KG*velocity[:, 0]-impulse)/impulse
        assert np.all(error < .01), (name, error)
        all_errors.extend(error.tolist())
        angular = np.linalg.norm(velocity[:, 3:], axis=1)
        if name == 'asset':
            assert np.all(angular < .01), angular
        else:
            assert np.all(angular > .1), angular
    assert np.all(np.linalg.norm(ctx._remote_histories['idle'][-1][3], axis=1) < 1e-6)
    ctx.experiment_details['momentum_errors_all_environments'] = all_errors
    ctx.metric('maximum_momentum_error', max(all_errors)*100, unit='%', label='全部环境最大线动量误差')
    ctx.metric('applied_impulse', impulse, unit='N·s', label='施加冲量')


TESTS = {'slide_faces': slide_faces, 'drop_orientations': drop_orientations,
         'flip_release': flip_release, 'ring_contact': ring_contact, 'free_eccentric': free_eccentric}
