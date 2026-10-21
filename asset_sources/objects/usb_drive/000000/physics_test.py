"""Fixed USB connector: native contact, planar guides, drops and free loads.

SI units, wxyz rotations. Loads are applied forces, not contact reactions.
Geometry, materials, thresholds and the approved 2 ms drop exception persist.
"""
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from table_1000.physics.testing import TestContext

BODY = 'usb_drive.body'
MASS_KG = .012
UPRIGHT = (math.sqrt(.5), 0, math.sqrt(.5), 0)


def _vertices(ctx):
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    worlds, groups = {}, {}
    for n in model['nodes']:
        worlds[n['name']] = worlds.get(n['parent'], np.eye(4)) @ np.array(n['matrix'])
        if n.get('role') == 'collision':
            local = np.linalg.inv(worlds[BODY]) @ worlds[n['name']]
            groups[n['parent'].rsplit('.', 1)[-1]] = (np.c_[n['vertices'], np.ones(len(n['vertices']))] @ local.T)[:, :3]
    return groups


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    cache, vertices = {}, _vertices(ctx)

    def states():
        if cache.get('step') != ctx.session.steps:
            positions, rotations, _ = instance.environment_states()
            velocities = instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, -1, 6)
            i = instance.indices[BODY]
            cache.update(step=ctx.session.steps, positions=positions[:, i], rotations=rotations[:, i], velocities=velocities[:, i])
        return cache

    for env in range(ctx.num_envs):
        prefix = f'{instance.name}.env{env}'
        for j, axis in enumerate('xyz'):
            ctx.observe(prefix+'.position.'+axis, lambda e=env, j=j: states()['positions'][e, j], unit='m')
            ctx.observe(prefix+'.velocity.'+axis, lambda e=env, j=j: states()['velocities'][e, j], unit='m/s')
        ctx.observe(prefix+'.speed', lambda e=env: np.linalg.norm(states()['velocities'][e, :3]), unit='m/s')
        ctx.observe(prefix+'.spin', lambda e=env: np.linalg.norm(states()['velocities'][e, 3:]), unit='rad/s')
        ctx.observe(prefix+'.tilt', lambda e=env: math.acos(float(np.clip(-states()['rotations'][e, 2, 0], -1, 1))), unit='rad')
        ctx.observe(prefix+'.normal_z', lambda e=env: states()['rotations'][e, 2, 2], unit='1')
        for part, points in vertices.items():
            ctx.observe(prefix+'.'+part+'_clearance', lambda e=env, p=points: float(
                (p @ states()['rotations'][e].T)[:, 2].min()+states()['positions'][e, 2]), unit='m')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.0001)


def _stable(ctx, names, speed=.01):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    results = {}
    for name in names:
        for env in range(ctx.num_envs):
            prefix = f'{name}.env{env}'
            positions = [[r[prefix+'.position.'+a] for a in 'xyz'] for r in tail]
            peak = max(r[prefix+'.speed'] for r in tail)
            span = float(np.linalg.norm(np.ptp(positions, axis=0)))
            assert peak < speed, (prefix, 'unsettled velocity', peak)
            assert span < .001, (prefix, 'unsettled position', span)
            results[prefix] = {'tail_max_speed_m_s': peak, 'tail_position_extent_m': span}
    return results


def _contact(ctx):
    results = {}
    for name in ctx.session.instances:
        for env in range(ctx.num_envs):
            prefix = f'{name}.env{env}'
            low = min(r[prefix+'.'+part+'_clearance'] for r in ctx.rows for part in ('housing', 'connector'))
            penetration = max(0., -low)
            results[prefix] = {'maximum_geometry_penetration_m': penetration}
            assert penetration < .002, (prefix, penetration)
    ctx.metric('maximum_penetration', max(v['maximum_geometry_penetration_m'] for v in results.values())*1000,
               unit='mm', label='全部环境最大几何穿地')
    return results


def _planar_guide(ctx, bodies):
    """Declare native YZ-plane D6 guides before replication/initialization.

    World X translation and Y/Z rotation locked; Y/Z translation and X roll
    free. No vertical holding force or in-plane tip drive. No runtime USD edit.
    """
    from pxr import Gf, UsdPhysics
    assert not ctx.initialized
    for body in bodies:
        position, q = ctx.body_pose(body)
        path = ctx.session.environment(0)+'/Fixtures/planar_'+body.instance.name
        joint = UsdPhysics.Joint.Define(ctx.session.stage, path)
        joint.CreateBody1Rel().SetTargets([body.path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, position)))
        joint.CreateLocalRot0Attr(Gf.Quatf(1))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot1Attr(Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, -q[1:]))))
        for axis in ('transX', 'transY', 'transZ', 'rotX', 'rotY', 'rotZ'):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            locked = axis in ('transX', 'rotY', 'rotZ')
            limit.CreateLowAttr(0 if locked else -float('inf'))
            limit.CreateHighAttr(0 if locked else float('inf'))


def support_response(ctx: TestContext):
    _ground(ctx)
    ctx.initial(position=(-.10, -.04, .0002))
    tip = ctx.spawn(ctx.asset_path, name='connector', position=(0, -.04, .0302), rotation=UPRIGHT)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(.10, -.04, .0302), rotation=UPRIGHT)
    bodies = [_body(ctx, a) for a in (ctx.asset, tip, idle)]
    _planar_guide(ctx, bodies[1:])
    ctx.camera(position=(.29, -.42, .32), target=(0, .02, .025))
    ctx.video_annotation = lambda: 'Left: housing | centre/right: planar-guided connector; centre loaded, right idle'
    for name, body in zip(('housing_push', 'connector_push'), bodies):
        ctx.force(name, body, keyframes=[
            {'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
            {'time': .9, 'force': (0, .065, 0)}, {'time': 1., 'force': (0, .065, 0)},
            {'time': 1.04, 'force': (0, 0, 0)}])
    ctx.plots('housing_push.force.y', 'asset.env0.position.y', 'connector.env0.tilt',
              'idle.env0.tilt', 'connector.env0.connector_clearance', 'asset.speed', 'connector.speed')
    ctx.run(3.2)
    settling = _stable(ctx, ('asset', 'connector', 'idle'), speed=.005)
    release = min(ctx.rows, key=lambda r: abs(r['time']-1.04))
    before = min(ctx.rows, key=lambda r: abs(r['time']-.3))
    measurements = []
    for env in range(ctx.num_envs):
        shell, connector, control = (f'{n}.env{env}' for n in ('asset', 'connector', 'idle'))
        shift = ctx.rows[-1][shell+'.position.y']-ctx.rows[0][shell+'.position.y']
        tilt = max(r[connector+'.tilt'] for r in ctx.rows)
        idle_tilt = max(r[control+'.tilt'] for r in ctx.rows)
        assert shift > .005, (env, shift)
        assert ctx.rows[-1][shell+'.normal_z'] > .98
        assert tilt > math.pi/4, (env, tilt)
        assert idle_tilt < math.radians(5), (env, idle_tilt)
        support = {p: before[connector+'.'+p+'_clearance'] for p in ('housing', 'connector')}
        assert abs(support['connector']) < .0005 and support['housing'] > .01, (env, support)
        measurements.append({'environment': env, 'housing_slide_m': shift,
            'housing_stop_distance_m': ctx.rows[-1][shell+'.position.y']-release[shell+'.position.y'],
            'peak_connector_tilt_rad': tilt, 'idle_peak_tilt_rad': idle_tilt,
            'preload_connector_support_clearance_m': support})
    ctx.experiment_details = {'per_environment': measurements, 'settling': settling,
        'fixture': 'predeclared native D6 planar guides; world X / rotY / rotZ locked, Y / Z / rotX free',
        'release_time_s': 1.04, 'geometry_contact': _contact(ctx),
        'boundary': 'not free connector-end standing or isolated material coefficient measurement'}
    ctx.metric('housing_slide', measurements[0]['housing_slide_m']*1000, unit='mm', label='首环境壳面滑移')
    ctx.metric('housing_stop_distance', measurements[0]['housing_stop_distance_m']*1000, unit='mm', label='首环境撤力停止距离')
    ctx.metric('minimum_loaded_tilt', math.degrees(min(v['peak_connector_tilt_rad'] for v in measurements)), unit='deg', label='全部环境最小受力倾倒峰值')
    ctx.metric('maximum_idle_tilt', math.degrees(max(v['idle_peak_tilt_rad'] for v in measurements)), unit='deg', label='全部环境最大闲置倾角')


def drop_orientations(ctx: TestContext):
    """Approved 2 ms exception: the historical 4 ms overlap exceeded 2 mm."""
    ctx.simulation(dt=.002)
    _ground(ctx)
    q = Rotation.from_euler('y', 25, degrees=True).as_quat()[[3, 0, 1, 2]]
    rotations = [(1, 0, 0, 0), q, UPRIGHT]
    vertices = np.concatenate(list(_vertices(ctx).values()))
    heights = [.03-(vertices @ Rotation.from_quat(np.array(r)[[1, 2, 3, 0]]).as_matrix().T)[:, 2].min() for r in rotations]
    ctx.initial(position=(-.10, 0, heights[0]), rotation=rotations[0])
    tilted = ctx.spawn(ctx.asset_path, name='tilted', position=(0, 0, heights[1]), rotation=rotations[1])
    tip = ctx.spawn(ctx.asset_path, name='connector', position=(.10, 0, heights[2]), rotation=rotations[2])
    for instance in (ctx.asset, tilted, tip):
        _body(ctx, instance)
    ctx.camera(position=(.30, -.40, .27), target=(0, 0, .025))
    ctx.video_annotation = lambda: '30 mm release | left housing | centre 25 deg | right connector end | dt 2 ms'
    ctx.plots(*[n+'.env0.position.z' for n in ('asset', 'tilted', 'connector')],
              *[n+'.speed' for n in ('asset', 'tilted', 'connector')])
    ctx.run(3.2)
    settling = _stable(ctx, ('asset', 'tilted', 'connector'))
    ctx.experiment_details = {'release_clearance_m': .03, 'initial_origin_z_m': list(map(float, heights)),
                              'geometry_contact': _contact(ctx), 'settling': settling,
                              'boundary': 'rigid placement, not impact damage or connector deformation'}
    ctx.metric('tail_max_speed', max(v['tail_max_speed_m_s'] for v in settling.values())*1000, unit='mm/s', label='全部环境最大末段速度')


def free_eccentric(ctx: TestContext):
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.06, -.075, .06))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(-.06, 0, .06))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.06, .075, .06))
    bodies = [_body(ctx, a) for a in (ctx.asset, offset, idle)]
    for name, body in zip(('com_push', 'offset_push'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
                  {'time': .404, 'force': (.012, 0, 0)}, {'time': .604, 'force': (.012, 0, 0)},
                  {'time': .608, 'force': (0, 0, 0)}]
        if name == 'offset_push':
            frames[0]['point'] = (0, .007, .005)
        ctx.force(name, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.29, -.40, .35), target=(.015, 0, .06))
    ctx.video_annotation = lambda: 'Zero gravity | lower COM | middle eccentric | upper idle | same +X impulse'
    ctx.plots('com_push.force.x', 'asset.env0.velocity.x', 'offset.env0.velocity.x', 'offset.env0.spin')
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    report = []
    for env in range(ctx.num_envs):
        final = ctx.rows[-1]
        velocities = [np.array([final[f'{n}.env{env}.velocity.{a}'] for a in 'xyz']) for n in ('asset', 'offset', 'idle')]
        errors = [abs(MASS_KG*v[0]-impulse)/impulse for v in velocities[:2]]
        assert max(errors) < .01, (env, errors)
        assert final[f'asset.env{env}.spin'] < .01
        assert final[f'offset.env{env}.spin'] > .1
        assert max(r[f'idle.env{env}.speed'] for r in ctx.rows) < 1e-6
        report.append({'environment': env, 'relative_linear_momentum_errors': list(map(float, errors)),
                       'final_velocities_m_s': [v.tolist() for v in velocities],
                       'offset_final_spin_rad_s': final[f'offset.env{env}.spin']})
    ctx.experiment_details = {'applied_impulse_N_s': impulse, 'per_environment': report,
                              'asset_pose_writes_after_initialization': False, 'ground': None}
    ctx.metric('maximum_momentum_error', max(max(v['relative_linear_momentum_errors']) for v in report)*100, unit='%', label='全部环境最大线动量相对误差')
    ctx.metric('minimum_offset_spin', min(v['offset_final_spin_rad_s'] for v in report), unit='rad/s', label='全部环境最小偏心转速')


def lifecycle_reset_reload(ctx: TestContext):
    """Native stop/reset/reload audit; state writes occur only during reset."""
    import torch
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils
    stage_asset = Usd.Stage.Open(str(ctx.asset_path))
    rigids = [p for p in stage_asset.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    colliders = [p for p in stage_asset.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(rigids) == 1 and len(colliders) == 2
    assert not any(p.IsA(UsdPhysics.Joint) for p in stage_asset.Traverse())
    assert abs(UsdPhysics.MassAPI(rigids[0]).GetMassAttr().Get()-MASS_KG) < 1e-8
    materials = {}
    for p in colliders:
        material, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial('physics')
        api = UsdPhysics.MaterialAPI(material)
        coefficients = [api.GetStaticFrictionAttr().Get(), api.GetDynamicFrictionAttr().Get()]
        part = p.GetParent().GetName()
        assert np.allclose(coefficients, {'housing': [.40, .30], 'connector': [.25, .18]}[part])
        materials[part] = coefficients
    assert not UsdUtils.ComputeAllDependencies(str(ctx.asset_path))[2]
    assert UsdGeom.GetStageMetersPerUnit(stage_asset) == 1 and UsdGeom.GetStageUpAxis(stage_asset) == 'Z'
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.04, -.04, .08))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.04, .04, .08))
    ctx.camera(position=(.24, -.32, .29), target=(.0, 0, .08))
    ctx.ensure_initialized()
    stage = ctx.session.stage
    stage_id = stage.GetRootLayer().identifier
    original_render = ctx.render
    ctx.render = False

    def snapshot():
        return {n: (a.view.view.get_transforms().clone(), a.view.view.get_velocities().clone())
                for n, a in ctx.session.instances.items()}

    def restore(saved):
        for n, (poses, speeds) in saved.items():
            a = ctx.session.instances[n]
            ids = torch.arange(len(poses), dtype=torch.int32, device=poses.device)
            a.view.view.set_transforms(poses.clone(), ids)
            a.view.view.set_velocities(speeds.clone(), ids)
        ctx.world.forward()
        ctx.session.refresh()

    def difference(a, b):
        return max(float(torch.max(torch.abs(a[n][i]-b[n][i])).cpu()) for n in a for i in (0, 1))

    def configure():
        ctx.rows.clear(); ctx.observers.clear(); ctx.units.clear()
        ctx.actions.clear(); ctx.action_values.clear()
        ctx.session.time = 0.
        ctx.session.steps = ctx.session.callback_steps = 0
        ctx.session.error = None
        for a in (ctx.asset, idle):
            _body(ctx, a)
        ctx.force('push', ctx.asset.body(BODY), keyframes=[
            {'time': 0, 'force': (0, 0, 0)}, {'time': .2, 'force': (0, 0, 0)},
            {'time': .204, 'force': (.012, 0, 0)}, {'time': .404, 'force': (.012, 0, 0)},
            {'time': .408, 'force': (0, 0, 0)}])
        ctx.plots('push.force.x', 'asset.env0.velocity.x', 'idle.env0.speed')

    def checks():
        impulse = sum(r['push.force.x'] for r in ctx.rows)*ctx.dt
        result = []
        for e in range(ctx.num_envs):
            velocity = ctx.rows[-1][f'asset.env{e}.velocity.x']
            error = abs(MASS_KG*velocity-impulse)/impulse
            idle_speed = max(r[f'idle.env{e}.speed'] for r in ctx.rows)
            assert error < .01 and idle_speed < 1e-6, (e, error, idle_speed)
            assert ctx.rows[-1][f'asset.env{e}.spin'] < .01
            result.append({'environment': e, 'relative_momentum_error': error, 'idle_max_speed_m_s': idle_speed})
        assert ctx.session.callback_steps == ctx.session.steps
        return result

    def relative_transforms():
        cache = UsdGeom.XformCache()
        matrices = {}
        for a in ctx.session.instances.values():
            for e in range(ctx.num_envs):
                root = ctx.session.replica_path(a.paths[BODY], e)
                inv = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(root)).GetInverse()
                for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                    if prim.GetCustomDataByKey('table1000:role') in ('part', 'visual', 'collision'):
                        matrices[str(prim.GetPath())] = np.asarray(cache.GetLocalToWorldTransform(prim)*inv)
        return matrices

    saved = snapshot(); before = relative_transforms()
    configure(); ctx.run(1.2); first_checks = checks(); first = snapshot()
    first_rows = list(ctx.rows)
    after = relative_transforms()
    relative_error = max(float(np.max(np.abs(before[k]-after[k]))) for k in before)
    assert relative_error < 1e-6
    ctx.session.active = False
    ctx.world._disable_app_control_on_stop_handle = True
    ctx.world.stop(); ctx.world.reset(); ctx.session.initialize(); restore(saved)
    reset_error = difference(saved, snapshot()); assert reset_error < 1e-6
    configure(); ctx.run(1.2); reset_checks = checks()
    repeat_error = difference(first, snapshot())
    trace_error = max(abs(a[k]-b[k]) for a, b in zip(first_rows, ctx.rows) for k in a)
    assert len(first_rows) == len(ctx.rows)
    ctx.session.active = False
    ctx.world._disable_app_control_on_stop_handle = True
    ctx.world.stop()
    paths = {n: [ctx.session.replica_path(a.path, e) for e in range(ctx.num_envs)]
             for n, a in ctx.session.instances.items()}
    for path in paths['asset']:
        stage.RemovePrim(path)
    assert all(not stage.GetPrimAtPath(p) for p in paths['asset'])
    assert all(stage.GetPrimAtPath(p) for p in paths['idle'])
    ctx.session.unregister(ctx.asset.path)
    for path in paths['idle']:
        stage.RemovePrim(path)
    ctx.session.unregister(idle.path)
    assert not ctx.session.instances
    for n, group in paths.items():
        for path in group:
            UsdGeom.Xform.Define(stage, path).GetPrim().GetReferences().AddReference(str(ctx.asset_path))
        ctx.session.register(group[0], n)
    ctx.asset, idle = (ctx.session.instances[n] for n in ('asset', 'idle'))
    ctx.world.reset(); ctx.session.initialize(); restore(saved)
    reload_error = difference(saved, snapshot()); assert reload_error < 1e-6
    for a in (ctx.asset, idle):
        assert a.body_names == [BODY] and not a.joint_names
        assert len(a.view.view.get_transforms()) == ctx.num_envs
    configure()
    ctx.render = original_render
    ctx.experiment_started = None; ctx.integrating_seconds = 0.
    ctx.video_annotation = lambda: 'Native reloaded segment | loaded and idle | stop/reset/reload checked in every environment'
    ctx.run(1.2); reload_checks = checks()
    ctx.experiment_details = {'lifecycle': {
        'same_stage': stage.GetRootLayer().identifier == stage_id,
        'environments_checked': ctx.num_envs, 'reset_state_max_scalar_error': reset_error,
        'repeat_final_max_scalar_error': repeat_error, 'repeat_trace_max_scalar_error': trace_error,
        'reload_state_max_scalar_error': reload_error, 'removal_preserved_sibling': True,
        'descendant_relative_transform_error': relative_error, 'descendant_nodes_checked': len(before),
        'first_checks': first_checks, 'reset_checks': reset_checks, 'reload_checks': reload_checks,
        'state_writes': 'only native stopped/reset initialization; no pose writes while integrating',
        'media_scope': 'final reloaded segment; first/reset segments numerically checked',
        'performance_scope': 'lifecycle diagnostic excluded from steady-state benchmark'},
        'usd_reopen': {'rigid_bodies': 1, 'colliders': 2, 'joints': 0, 'mass_kg': MASS_KG,
                       'materials': materials, 'unresolved_dependencies': []}}
    ctx.metric('reset_error', reset_error, unit='1', label='全部环境复位最大状态误差')
    ctx.metric('reload_error', reload_error, unit='1', label='全部环境重载最大状态误差')
    ctx.metric('relative_transform_error', relative_error, unit='1', label='部件相对变换最大误差')


TESTS = {'support_response': support_response, 'drop_orientations': drop_orientations,
         'free_eccentric': free_eccentric, 'lifecycle_reset_reload': lifecycle_reset_reload}
