"""Native rigid eraser experiments, SI units and wxyz rotations.

Paper/rubber are uncalibrated contact estimates. End-standing rubber needs a
rotation-only test guide for sliding: free upright response is tested separately.
No asset pose writes during integration; forces are applied loads, not reactions.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from table_1000.physics.testing import TestContext

BODY = 'eraser.body'
MASS_KG = .014
END_UP = (math.sqrt(.5), 0, math.sqrt(.5), 0)


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(f'{instance.name}.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    # Read every replica in environment-local coordinates, once per step.
    cached = {}
    def states():
        if cached.get('step') != ctx.session.steps:
            positions, rotations, _ = instance.environment_states()
            velocities = instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, 1, 6)
            cached.update(step=ctx.session.steps, position=positions[:, 0],
                          quaternion=Rotation.from_matrix(rotations[:, 0]).as_quat()[:, [3, 0, 1, 2]],
                          linear_velocity=velocities[:, 0, :3], angular_velocity=velocities[:, 0, 3:])
        return cached
    for env in range(ctx.num_envs):
        prefix = f'env{env}.{instance.name}.{BODY}.'
        for field, axes, unit in [('position', 'xyz', 'm'), ('quaternion', 'wxyz', '1'),
                                  ('linear_velocity', 'xyz', 'm/s'), ('angular_velocity', 'xyz', 'rad/s')]:
            for index, axis in enumerate(axes):
                ctx.observe(prefix+field+'.'+axis,
                            lambda e=env, f=field, i=index: states()[f][e, i], unit=unit)
        ctx.observe(f'env{env}.{instance.name}.speed',
                    lambda e=env: np.linalg.norm(states()['linear_velocity'][e]), unit='m/s')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.00005)


def _column(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def _environment_rows(ctx):
    """Map each replica's observations to the original measurement names."""
    for env in range(ctx.num_envs):
        prefix = f'env{env}.'
        yield [{**row, **{key[len(prefix):]: value for key, value in row.items() if key.startswith(prefix)}}
               for row in ctx.rows]


def _stable(ctx, names, speed=.01):
    for env, rows in enumerate(_environment_rows(ctx)):
        tail = [r for r in rows if r['time'] >= ctx.session.time-.5-1e-8]
        for name in names:
            positions = np.array([[r[_column(name, 'position', a)] for a in 'xyz'] for r in tail])
            assert max(r[f'{name}.speed'] for r in tail) < speed, f'env{env} {name}: unsettled velocity'
            assert np.linalg.norm(np.ptp(positions, axis=0)) < .001, f'env{env} {name}: unsettled position'


def _contact(ctx):
    from table_1000.physics.test import ground_clearance
    import json
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    clearances = [ground_clearance(model, rows, ctx.session.instances, 0) for rows in _environment_rows(ctx)]
    for env, value in enumerate(clearances):
        assert value['maximum_geometry_penetration_m'] < .002, (env, value, 'all_environments', clearances)
    maximum = max(value['maximum_geometry_penetration_m'] for value in clearances)
    ctx.metric('maximum_penetration', maximum*1000, unit='mm', label='各环境最大几何穿地')
    return {'per_environment': clearances, 'maximum_geometry_penetration_m': maximum}


def _rotation(row, name):
    q = [row[_column(name, 'quaternion', a)] for a in 'xyzw']
    return Rotation.from_quat(q).as_matrix()


def contact_slide(ctx: TestContext):
    if ctx.physics == 'gpu':
        ctx.simulation(dt=.001)
    _ground(ctx)
    ctx.initial(position=(0, -.055, .00002))
    rubber = ctx.spawn(ctx.asset_path, name='rubber', position=(0, .015, .03000), rotation=END_UP)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .085, .03000), rotation=END_UP)
    bodies = [_body(ctx, a) for a in (ctx.asset, rubber, idle)]
    ctx.camera(position=(.54, -.70, .48), target=(.12, .015, .025))
    ctx.video_annotation = lambda: 'Lower: free PAPER flat | middle: RUBBER end, rotation guided | upper: idle, free | release 1.34 s'
    for label, body in zip(('paper_push', 'rubber_push'), bodies):
        ctx.force(label, body, keyframes=[
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .5, 'force': (0, 0, 0)},
            {'time': 1.3, 'force': (.10, 0, 0)},
            {'time': 1.34, 'force': (0, 0, 0)},
        ])
    ctx.plots('paper_push.force.x', _column('asset', 'position', 'x'),
              _column('rubber', 'position', 'x'), _column('asset', 'linear_velocity', 'x'),
              _column('rubber', 'linear_velocity', 'x'))
    guide = ctx.grip(bodies[1], name='rubber_rotation_guide',
                     gain_reference_dt=.004 if ctx.physics == 'gpu' else None)
    guide.axes(linear=(0, 0, 0), angular=True)
    ctx.run(3)
    guide.release()
    environments = []
    for env, rows in enumerate(_environment_rows(ctx)):
        onset = {}
        for name, force in [('asset', 'paper_push'), ('rubber', 'rubber_push')]:
            column = _column(name, 'position', 'x')
            start = rows[0][column]
            row = next(r for r in rows if r[column]-start >= .0005)
            onset[name] = {'time_s': row['time'], 'applied_force_N': row[force+'.force.x']}
            assert rows[-1][column]-start > .005, (env, name)
        assert onset['rubber']['applied_force_N']-onset['asset']['applied_force_N'] > .01, (env, onset)
        assert _rotation(rows[-1], 'asset')[2, 2] > .98
        assert -_rotation(rows[-1], 'rubber')[2, 0] > .98
        positions = np.array([[r[_column('idle', 'position', a)] for a in 'xy'] for r in rows])
        cold_drift = float(np.max(np.linalg.norm(positions-positions[0], axis=1)))
        settled = positions[np.array([r['time'] >= .5-1e-8 for r in rows])]
        control_drift = float(np.max(np.linalg.norm(settled-settled[0], axis=1)))
        assert -_rotation(rows[-1], 'idle')[2, 0] > math.cos(math.radians(5))
        release = min(rows, key=lambda r: abs(r['time']-1.34))
        environments.append({'onset': onset, 'control_drift_after_settling_m': control_drift,
            'control_cold_start_max_horizontal_drift_m': cold_drift,
            'stopping_distance_m': {n: rows[-1][_column(n, 'position', 'x')]-release[_column(n, 'position', 'x')]
                                    for n in ('asset', 'rubber')}})
    for env, measured in enumerate(environments):
        assert measured['control_drift_after_settling_m'] < .0005, (
            env, measured['control_drift_after_settling_m'],
            'all_environment_control_drift_m', [e['control_drift_after_settling_m'] for e in environments])
    _stable(ctx, ('asset', 'rubber', 'idle'), speed=.005)
    for name, label in [('asset', '纸套平放'), ('rubber', '露胶端导向')]:
        ctx.metric(name+'_onset', environments[0]['onset'][name]['applied_force_N'], unit='N', label=label+'起滑载荷')
        ctx.metric(name+'_stop', environments[0]['stopping_distance_m'][name]*1000, unit='mm', label=label+'撤力停止距离')
    ctx.metric('control_drift', max(e['control_drift_after_settling_m'] for e in environments)*1000,
               unit='mm', label='各环境闲置实例沉降后最大漂移')
    ctx.experiment_details = {
        'contract': 'equal COM ramp and withdrawal, paper flat versus rotation-guided rubber end; free idle control',
        'onset_definition_m': .0005, 'release_time_s': 1.34, 'control_observation_start_s': .5,
        'per_environment': environments, 'geometry_contact': _contact(ctx),
        'fixture': 'predeclared native tensor grip; rotation controlled, X/Y/Z translation axes disabled; released after observation',
        'nominal_average_combined_static_friction': {'paper': .425, 'rubber': .625},
        'nominal_average_combined_dynamic_friction': {'paper': .325, 'rubber': .50},
        'boundary': 'effective supported/guided response, not a pure material coefficient measurement; 0.2 mm paper skin can allow mixed contact',
    }


def free_tip(ctx: TestContext):
    # Approved GPU contact-step exception; CPU and non-contact tests stay 4 ms.
    if ctx.physics == 'gpu':
        ctx.simulation(dt=.001)
    _ground(ctx)
    ctx.initial(position=(.045, 0, .03000), rotation=END_UP)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.045, 0, .03000), rotation=END_UP)
    loaded, control = [_body(ctx, a) for a in (ctx.asset, idle)]
    for label, body in [('loaded', loaded), ('control', control)]:
        ctx.observe(label+'.angle', lambda b=body: math.acos(float(np.clip(-b.state['rotation'][2, 0], -1, 1))), unit='rad')
    ctx.camera(position=(.24, -.32, .22), target=(0, .035, .025))
    ctx.video_annotation = lambda: 'Both RUBBER-end standing, no guide | right: high +Y push | left: idle | release 1.04 s'
    ctx.force('push', loaded, point_frame='body', keyframes=[
        {'time': 0, 'force': (0, 0, 0), 'point': (-.025, 0, .005)},
        {'time': .4, 'force': (0, 0, 0)},
        {'time': .8, 'force': (0, .04, 0)},
        {'time': 1, 'force': (0, .04, 0)},
        {'time': 1.04, 'force': (0, 0, 0)},
    ])
    ctx.plots('push.force.y', 'loaded.angle', 'control.angle', 'asset.speed')
    ctx.run(3)
    environments = []
    for env, rows in enumerate(_environment_rows(ctx)):
        angles = {name: [math.acos(float(np.clip(-_rotation(row, name)[2, 0], -1, 1))) for row in rows]
                  for name in ('asset', 'idle')}
        assert max(angles['asset']) > math.pi/4, env
        assert max(angles['idle']) < math.radians(5), env
        environments.append({'peak_loaded_tilt_rad': max(angles['asset']), 'peak_idle_tilt_rad': max(angles['idle'])})
    _stable(ctx, ('asset', 'idle'))
    ctx.metric('peak_tilt', math.degrees(environments[0]['peak_loaded_tilt_rad']), unit='deg', label='受力实例峰值倾角')
    ctx.metric('control_tilt', math.degrees(max(e['peak_idle_tilt_rad'] for e in environments)), unit='deg', label='各环境闲置实例最大倾角')
    ctx.experiment_details = {'contract': 'free end-standing loaded/control, 0.04 N high push then withdrawal',
                              'application_point_body_m': [-.025, 0, .005], 'release_time_s': 1.04,
                              'per_environment': environments, 'geometry_contact': _contact(ctx),
                              'fixture': None, 'asset_pose_writes_after_initialization': False}


def drop_orientations(ctx: TestContext):
    # Approved GPU contact-step exception; CPU and non-contact tests stay 4 ms.
    if ctx.physics == 'gpu':
        ctx.simulation(dt=.001)
    _ground(ctx)
    ctx.initial(position=(-.09, 0, .05))
    back = ctx.spawn(ctx.asset_path, name='back', position=(0, 0, .06), rotation=(0, 1, 0, 0))
    end = ctx.spawn(ctx.asset_path, name='end', position=(.09, 0, .08), rotation=END_UP)
    for a in (ctx.asset, back, end):
        _body(ctx, a)
    ctx.camera(position=(.40, -.56, .38), target=(.015, -.015, .035))
    ctx.video_annotation = lambda: 'Left: paper bottom | centre: paper top | right: rubber end | 50 mm lowest-point release'
    ctx.plots(*[_column(n, 'position', 'z') for n in ('asset', 'back', 'end')],
              *[n+'.speed' for n in ('asset', 'back', 'end')])
    ctx.run(3)
    _stable(ctx, ('asset', 'back', 'end'))
    clearance = _contact(ctx)
    ctx.experiment_details = {'contract': '50 mm lowest-collision-point gravity drop in three orientations',
                              'release_clearance_m': .05, 'geometry_contact': clearance,
                              'boundary': 'rigid zero-restitution proxy, not calibrated impact, paper deformation or rubber compression'}


def free_impulse(ctx: TestContext):
    ctx.simulation(dt=ctx.dt, gravity=(0, 0, 0))
    ctx.initial(position=(-.06, -.05, .08))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(-.06, 0, .08))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.06, .05, .08))
    bodies = [_body(ctx, a) for a in (ctx.asset, offset, idle)]
    for name, body in zip(('com_push', 'offset_push'), bodies):
        frames = [
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .4, 'force': (0, 0, 0)},
            {'time': .404, 'force': (.014, 0, 0)},
            {'time': .604, 'force': (.014, 0, 0)},
            {'time': .608, 'force': (0, 0, 0)},
        ]
        if name == 'offset_push':
            frames[0]['point'] = (0, .005, .005)
        ctx.force(name, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.22, -.32, .29), target=(.015, 0, .08))
    ctx.video_annotation = lambda: 'Zero gravity | lower: COM impulse | middle: offset impulse | upper: idle'
    ctx.plots('com_push.force.x', _column('asset', 'linear_velocity', 'x'),
              _column('offset', 'linear_velocity', 'x'), _column('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    environments = []
    for env, rows in enumerate(_environment_rows(ctx)):
        row = rows[-1]
        velocity = lambda name, field: np.array([row[_column(name, field, a)] for a in 'xyz'])
        errors = [abs(MASS_KG*velocity(n, 'linear_velocity')[0]-impulse)/impulse for n in ('asset', 'offset')]
        assert max(errors) < .01, (env, errors)
        assert np.linalg.norm(velocity('asset', 'angular_velocity')) < .01
        assert np.linalg.norm(velocity('offset', 'angular_velocity')) > .1
        assert np.linalg.norm(np.r_[velocity('idle', 'linear_velocity'), velocity('idle', 'angular_velocity')]) < 1e-6
        environments.append({'relative_linear_momentum_errors': errors,
                             'offset_angular_speed_rad_s': float(np.linalg.norm(velocity('offset', 'angular_velocity')))})
    ctx.metric('momentum_error', max(max(e['relative_linear_momentum_errors']) for e in environments)*100,
               unit='%', label='各环境最大线动量相对误差')
    ctx.metric('offset_spin', environments[0]['offset_angular_speed_rad_s'], unit='rad/s', label='偏心受力末态角速')
    ctx.experiment_details = {'contract': 'equal world +X impulses, offset point follows body; untouched control',
                              'applied_impulse_N_s': impulse, 'per_environment': environments,
                              'gravity_m_s2': [0, 0, 0], 'ground': None,
                              'asset_pose_writes_after_initialization': False}


def guide_release(ctx: TestContext):
    """Lock rotation only, then tensor-release without a pose/velocity jump."""
    ctx.simulation(dt=ctx.dt, gravity=(0, 0, 0))
    ctx.initial(position=(0, -.05, .08))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .05, .08))
    body = _body(ctx, ctx.asset)
    _body(ctx, idle)
    guide = ctx.grip(body, name='release_guide')
    guide.axes(linear=(0, 0, 0), angular=True)
    ctx.torque('twist', body, keyframes=[
        {'time': 0, 'torque': (0, 0, 0)},
        {'time': .2, 'torque': (0, 0, .00001)},
        {'time': .8, 'torque': (0, 0, .00001)},
        {'time': .84, 'torque': (0, 0, 0)},
    ])
    ctx.camera(position=(.22, -.32, .28), target=(0, 0, .08))
    ctx.video_annotation = lambda: 'Zero gravity | lower: rotation guide released at 0.40 s | upper: idle | applied torque, no pose animation'
    ctx.plots('twist.torque.z', _column('asset', 'angular_velocity', 'z'),
              'release_guide.torque.z', _column('idle', 'angular_velocity', 'z'))
    ctx.run(.4)
    def state_arrays():
        poses = ctx.asset.view.view.get_transforms().detach().cpu().numpy().copy()
        velocities = ctx.asset.view.view.get_velocities().detach().cpu().numpy().copy()
        return poses, velocities
    before = state_arrays()
    guide.release()
    after = state_arrays()
    jumps = [float(np.max(np.abs(a-b))) for a, b in zip(before, after)]
    assert max(jumps) < 1e-6, jumps
    ctx.run(.8)
    per_environment = []
    for env, rows in enumerate(_environment_rows(ctx)):
        angular = np.array([[r[_column('asset', 'angular_velocity', a)] for a in 'xyz'] for r in rows])
        held = [i for i, r in enumerate(rows) if .3 <= r['time'] <= .4+1e-8]
        peak_held = float(np.linalg.norm(angular[held], axis=1).max())
        final_spin = float(np.linalg.norm(angular[-1]))
        assert peak_held < .01, (env, peak_held)
        assert final_spin > .1, (env, final_spin)
        assert max(r['idle.speed'] for r in rows) < 1e-6, env
        assert max(np.linalg.norm([r[_column('idle', 'angular_velocity', a)] for a in 'xyz']) for r in rows) < 1e-6
        per_environment.append({'held_angular_speed_rad_s': peak_held, 'released_final_angular_speed_rad_s': final_spin})
    ctx.metric('release_jump', max(jumps), unit='1', label='释放瞬间状态最大数值差（位姿/速度分量）')
    ctx.metric('released_spin', per_environment[0]['released_final_angular_speed_rad_s'], unit='rad/s', label='导向释放后末态角速')
    ctx.experiment_details = {'release_time_s': .4, 'release_pose_max_component_jump': jumps[0],
        'release_velocity_max_component_jump': jumps[1], 'per_environment': per_environment,
        'fixture': 'rotation-only tensor grip; no translation control, no USD joint deletion',
        'applied_torque_N_m': .00001, 'gravity_m_s2': [0, 0, 0]}


def stop_reset(ctx: TestContext):
    """Reset a freely moving native rigid body; inspect every replica."""
    ctx.simulation(dt=ctx.dt, gravity=(0, 0, 0))
    ctx.initial(position=(-.03, -.045, .08), linear_velocity=(.05, 0, 0), angular_velocity=(0, 0, .4))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.03, .045, .08))
    _body(ctx, ctx.asset)
    _body(ctx, idle)
    ctx.camera(position=(.22, -.32, .28), target=(0, 0, .08))
    ctx.video_annotation = lambda: 'Reset check | lower: free translating/spinning body | upper: idle | explicit stop/reset at 0.40 s'
    ctx.plots(_column('asset', 'position', 'x'), _column('asset', 'linear_velocity', 'x'),
              _column('asset', 'angular_velocity', 'z'), 'idle.speed')
    ctx.ensure_initialized()
    saved = {n: a.snapshot() for n, a in ctx.session.instances.items()}
    stage = ctx.session.stage
    from pxr import UsdGeom
    rigid_path = ctx.asset.body(BODY).path
    def descendants():
        cache = UsdGeom.XformCache()
        inverse = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(rigid_path)).GetInverse()
        return {str(p.GetPath()): np.asarray(cache.GetLocalToWorldTransform(p)*inverse)
                for p in stage.Traverse() if str(p.GetPath()).startswith(rigid_path+'/')
                and p.GetCustomDataByKey('table1000:role') in ('part', 'visual', 'collision')}
    relative = descendants()
    ctx.run(.4)
    current = descendants()
    relative_error = max(float(np.max(np.abs(relative[p]-current[p]))) for p in relative)
    assert relative_error < 1e-6
    ctx.session.active = False
    ctx.world.stop()
    ctx.world.reset()
    ctx.session.initialize()
    for name, state in saved.items():
        ctx.session.instances[name].restore(state)
    ctx.world.forward()
    ctx.session.refresh()
    errors = []
    for name, instance in ctx.session.instances.items():
        pose = instance.view.view.get_transforms().detach().cpu().numpy().reshape(ctx.num_envs, 1, 7).copy()
        pose[:, :, :3] -= ctx.session.offsets[:, None, :]
        velocity = instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, 1, 6)
        positions, quaternions, initial_velocity = saved[name]
        expected = np.c_[positions, quaternions[:, [1, 2, 3, 0]]]
        for env in range(ctx.num_envs):
            error = float(max(np.max(np.abs(pose[env]-expected)), np.max(np.abs(velocity[env]-initial_velocity))))
            assert error < 1e-6, (name, env, error)
            errors.append({'instance': name, 'environment': env, 'max_state_component_error': error})
    ctx.run(.4)
    for env, rows in enumerate(_environment_rows(ctx)):
        final = rows[-1]
        # initial(linear_velocity) is the asset-origin velocity; state stores
        # COM velocity including omega cross its offset from that origin.
        final_velocity = np.array([final[_column('asset', 'linear_velocity', a)] for a in 'xyz'])
        expected_velocity = saved[ctx.asset.name][2][0, :3]
        assert np.linalg.norm(final_velocity-expected_velocity) < 1e-5, (env, final_velocity, expected_velocity)
        assert max(r['idle.speed'] for r in rows) < 1e-6
    ctx.metric('reset_error', max(e['max_state_component_error'] for e in errors), unit='1', label='各环境复位状态最大分量误差')
    ctx.metric('integrity_error', relative_error, unit='1', label='整体后代相对变换最大分量误差')
    ctx.experiment_details = {'reset_at_timeline_s': .4, 'per_environment': errors,
        'descendant_nodes_checked': len(relative), 'descendant_relative_transform_max_error': relative_error,
        'boundary': 'explicit reset intentionally restores initial state; no pose writes used to produce loaded motion',
        'fresh_context': 'runner creates a new stage/context per test; teardown/reload evidence requires separate verification'}





def unload_reload(ctx: TestContext):
    """Two real stop/unload/reload cycles in one stage; remaining body continues.

    Asset structure changes occur only while the timeline is stopped. State
    restoration belongs to lifecycle initialization, never the free-motion run.
    No behavior script is present, so no behavior subscription is claimed.
    """
    from pxr import Sdf, UsdPhysics
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.05, -.05, .10))
    retained = ctx.spawn(ctx.asset_path, name='retained', position=(-.05, .05, .10))
    ctx.initial(retained, position=(-.05, .05, .10), linear_velocity=(.025, 0, 0))
    _body(ctx, retained)
    ctx.observe('loaded_present', lambda: float('asset' in ctx.session.instances), unit='1')
    ctx.camera(position=(.24, -.36, .28), target=(0, 0, .10))
    ctx.video_annotation = lambda: 'Same stage, two real unload/reload cycles | lower: unloaded/reloaded | upper: retained, free translation'
    ctx.plots('loaded_present', _column('retained', 'position', 'x'),
              _column('retained', 'linear_velocity', 'x'))
    ctx.run(.2)
    stage, session = ctx.session.stage, ctx.session
    original_stage = stage.GetRootLayer().identifier
    original_session = session
    subscriptions = (session.prepare_subscription, session.flush_subscription)
    events = []

    def native_state(instance):
        return (instance.view.view.get_transforms().clone(), instance.view.view.get_velocities().clone())

    def restore_retained(saved):
        import torch
        view = retained.view.view
        indices = torch.arange(ctx.num_envs, device=saved[0].device, dtype=torch.int32)
        view.set_transforms(saved[0], indices)
        view.set_velocities(saved[1], indices)
        ctx.world.forward()
        session.refresh()
        read = native_state(retained)
        return max(float((a-b).abs().max().item()) for a,b in zip(read,saved))

    for cycle in range(2):
        old = ctx.asset
        before = native_state(retained)
        session.active = False
        ctx.world.stop()
        paths = [session.replica_path(old.path, e) for e in range(ctx.num_envs)]
        for path in paths:
            assert stage.GetPrimAtPath(path)
            stage.RemovePrim(path)
        session.unregister(old.path)
        if ctx.app:
            for _ in range(5): ctx.app.update()
        assert 'asset' not in session.instances
        assert not any(stage.GetPrimAtPath(path) for path in paths)
        ctx.world.reset()
        session.initialize()
        removal_error = restore_retained(before)
        assert removal_error < 1e-6
        t = session.time
        ctx.run(.2)
        after_absent = native_state(retained)
        displacement = (after_absent[0][:,:3]-before[0][:,:3]).detach().cpu().numpy()
        assert np.max(np.abs(displacement-np.array([.005, 0, 0]))) < 1e-5
        assert np.max(np.abs((after_absent[1]-before[1]).detach().cpu().numpy())) < 1e-6
        session.active = False
        ctx.world.stop()
        replacement = ctx.spawn(ctx.asset_path, name='asset', position=(-.05, -.05, .10))
        ctx.asset = replacement
        for e in range(1, ctx.num_envs):
            assert Sdf.CopySpec(stage.GetRootLayer(), replacement.path,
                                stage.GetRootLayer(), session.replica_path(replacement.path, e))
        position, quaternion = ctx.usd_body_pose(replacement.body(BODY))
        ctx.world.reset()
        session.initialize()
        reload_error = restore_retained(after_absent)
        positions, rotations, _ = replacement.environment_states()
        expected_rotation = Rotation.from_quat(quaternion[[1,2,3,0]]).as_matrix()
        loaded_error = max(float(np.max(np.abs(positions[:,0]-position))),
                           float(np.max(np.abs(rotations[:,0]-expected_rotation))),
                           float(replacement.view.view.get_velocities().abs().max().item()))
        assert max(reload_error, loaded_error) < 1e-6
        assert old is not replacement and set(session.instances) == {'asset', 'retained'}
        counts = []
        for e in range(ctx.num_envs):
            prim = stage.GetPrimAtPath(session.environment(e))
            from pxr import Usd
            count = sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(prim))
            assert count == 2, (e, count)
            counts.append(count)
        assert stage.GetRootLayer().identifier == original_stage and session is original_session
        assert session.prepare_subscription is subscriptions[0] and session.flush_subscription is subscriptions[1]
        ctx.run(.2)
        loaded_final = replacement.environment_states()[0][:,0]
        assert np.max(np.abs(loaded_final-position)) < 1e-6
        events.append({'cycle':cycle+1, 'unload_time_s':t, 'reload_time_s':t+.2,
                       'removed_all_environment_prims':True, 'new_asset_object':True,
                       'retained_restore_max_component_error':max(removal_error,reload_error),
                       'fresh_loaded_max_component_error':loaded_error,
                       'retained_displacement_while_unloaded_m':displacement.tolist(),
                       'rigid_bodies_per_environment_after_reload':counts})
    ctx.metric('reload_error', max(e['fresh_loaded_max_component_error'] for e in events), unit='1', label='卸载重载初态最大分量误差')
    ctx.metric('retained_state_error', max(e['retained_restore_max_component_error'] for e in events), unit='1', label='保留实例跨生命周期状态恢复误差')
    ctx.experiment_details = {'cycles':events, 'same_stage_and_session':True,
        'no_behavior_script':True, 'dt_s':ctx.dt,
        'boundary':'real stopped-timeline removal/reload, no fresh TestContext; explicit retained-state restoration only at lifecycle boundaries; not behavior/GUI lifecycle or continuous runtime topology mutation'}


TESTS = {'contact_slide': contact_slide, 'free_tip': free_tip,
         'drop_orientations': drop_orientations, 'free_impulse': free_impulse,
         'guide_release': guide_release, 'stop_reset': stop_reset, 'unload_reload': unload_reload}
