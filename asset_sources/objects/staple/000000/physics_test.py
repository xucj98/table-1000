"""Native rigid-body checks for the complete staple strip.

All dimensions and loads are SI units.  These experiments assess rigid-strip
contact and numerical margins; they do not model individual staple separation,
wire bending, paper penetration, or actual stapling.
"""
import math
import json
import numpy as np

from table_1000.physics.testing import TestContext

BODY = 'staple.strip'
SQRT_HALF = math.sqrt(.5)


def _states(ctx, instance):
    key = (ctx.session.steps, ctx.session.time)
    cache = getattr(instance, '_staple_observation', None)
    if cache is None or cache[0] != key:
        pose = instance.view.view.get_transforms().detach().cpu().numpy().reshape(ctx.num_envs, 1, 7)[:, 0]
        vel = instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, 1, 6)[:, 0]
        cache = (key, pose[:, :3]-ctx.session.offsets, pose[:, [6, 3, 4, 5]], vel)
        instance._staple_observation = cache
    return cache[1:]


def _prefix(env, name):
    return name if env == 0 else f'env{env}.{name}'


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    for env in range(ctx.num_envs):
        name = _prefix(env, instance.name)
        ctx.observe(name+'.speed', lambda e=env: np.linalg.norm(_states(ctx, instance)[2][e, :3]), unit='m/s')
        ctx.observe(name+'.angular_speed', lambda e=env: np.linalg.norm(_states(ctx, instance)[2][e, 3:]), unit='rad/s')
        if env:
            for field, index, axes, unit in [('position', 0, 'xyz', 'm'), ('quaternion', 1, 'wxyz', '1')]:
                for i, axis in enumerate(axes):
                    ctx.observe(f'{name}.{BODY}.{field}.{axis}', lambda e=env, j=i, k=index: _states(ctx, instance)[k][e, j], unit=unit)
    return body


def _ground(ctx):
    ctx.ground(static_friction=.35, dynamic_friction=.25, restitution=.05, contact_offset=.00025)


def _contact(ctx):
    from table_1000.physics.test import ground_clearance
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    results = []
    for env in range(ctx.num_envs):
        prefix = f'env{env}.'
        rows = ctx.rows if env == 0 else [
            {'time': row['time'], **{k[len(prefix):]: v for k, v in row.items() if k.startswith(prefix)}}
            for row in ctx.rows]
        results.append(ground_clearance(model, rows, ctx.session.instances, 0))
    peak = max(x['maximum_geometry_penetration_m'] for x in results)
    ctx.metric('penetration', peak, unit='m', label='Maximum sampled ground penetration, all environments')
    assert peak < .000255, results
    return {'per_environment': results, 'maximum_geometry_penetration_m': peak}


def _settled(ctx, names, speed=.005):
    tail = [row for row in ctx.rows if row['time'] >= ctx.session.time-.25-1e-9]
    peaks = []
    for env in range(ctx.num_envs):
        for original in names:
            name = _prefix(env, original)
            peak = max(row[f'{name}.speed'] for row in tail)
            peaks.append(peak)
            assert peak < speed, f'{name}: residual linear motion'
            assert max(row[f'{name}.angular_speed'] for row in tail) < speed/.01, f'{name}: residual angular motion'
    ctx.metric('settled_speed', max(peaks), unit='m/s', label='Maximum tail speed, all environments')


def orientation_settle(ctx: TestContext):
    """Compare feet-down, crown-down and side placement after a low release."""
    _ground(ctx)
    feet = ctx.asset
    crown = ctx.spawn(ctx.asset_path, name='crown', position=(.04, 0, .00635), rotation=(0, 1, 0, 0))
    side = ctx.spawn(ctx.asset_path, name='side', position=(.08, 0, .01310), rotation=(SQRT_HALF, 0, SQRT_HALF, 0))
    ctx.initial(position=(0, 0, .00035))
    ctx.initial(crown, position=(.04, 0, .00635), rotation=(0, 1, 0, 0))
    ctx.initial(side, position=(.08, 0, .01310), rotation=(SQRT_HALF, 0, SQRT_HALF, 0))
    for instance in (feet, crown, side):
        _body(ctx, instance)
    ctx.camera(position=(.16, -.22, .12), target=(.04, 0, .003))
    ctx.video_annotation = lambda: 'feet down / crown down / side; 7 native box collision proxies'
    ctx.plots('asset.speed', 'crown.speed', 'side.speed', 'asset.angular_speed', 'crown.angular_speed', 'side.angular_speed')
    ctx.run(1.2)
    _settled(ctx, ('asset', 'crown', 'side'))
    ctx.experiment_details = {
        'geometry_contact': _contact(ctx),
        'contract': 'three initial orientations settle under gravity on a steel-like ground plane',
        'orientations': {'feet_down': 'identity', 'crown_down': '180 deg about X', 'side': '90 deg about Y'},
        'ground': {'static_friction': .35, 'dynamic_friction': .25, 'restitution': .05},
        'boundary': 'rigid strip contact only; no individual staple or wire deformation',
    }


def low_drop(ctx: TestContext):
    """Low-height release in the three reference orientations."""
    if ctx.physics == 'gpu':
        ctx.simulation(dt=.0005)  # 4 ms GPU drop exceeds the half-wire penetration limit.
    _ground(ctx)
    instances = [
        ctx.asset,
        ctx.spawn(ctx.asset_path, name='crown', position=(.04, 0, .036), rotation=(0, 1, 0, 0)),
        ctx.spawn(ctx.asset_path, name='side', position=(.08, 0, .04275), rotation=(SQRT_HALF, 0, SQRT_HALF, 0)),
    ]
    for instance in instances:
        _body(ctx, instance)
    ctx.initial(position=(0, 0, .030))
    ctx.initial(instances[1], position=(.04, 0, .036), rotation=(0, 1, 0, 0))
    ctx.initial(instances[2], position=(.08, 0, .04275), rotation=(SQRT_HALF, 0, SQRT_HALF, 0))
    ctx.camera(position=(.184, -.264, .155), target=(.04, 0, .024))
    ctx.video_annotation = lambda: '30 mm lowest-point release; feet / crown / side'
    ctx.plots(*[f'{name}.speed' for name in ('asset', 'crown', 'side')],
              *[f'{name}.{BODY}.position.z' for name in ('asset', 'crown', 'side')])
    ctx.run(1.2)
    _settled(ctx, ('asset', 'crown', 'side'))
    ctx.experiment_details = {
        'contract': 'low-height gravity drop, lowest point 30 mm above ground',
        'release_clearance_m': .030, 'geometry_contact': _contact(ctx),
        'boundary': 'impact response uses zero/low restitution proxy; not a calibrated drop rating',
    }


def push_release(ctx: TestContext):
    """Apply and remove a horizontal load, with an untouched control strip."""
    _ground(ctx)
    control = ctx.spawn(ctx.asset_path, name='control', position=(0, .030, .00035))
    pushed = ctx.asset
    ctx.initial(position=(0, 0, .00035))
    ctx.initial(control, position=(0, .030, .00035))
    pushed_body, control_body = _body(ctx, pushed), _body(ctx, control)
    ctx.force('push', pushed_body, keyframes=[
        {'time': 0, 'force': (0, 0, 0)},
        {'time': .4, 'force': (0, 0, 0)},
        {'time': .404, 'force': (.008, 0, 0)},
        {'time': .484, 'force': (.008, 0, 0)},
        {'time': .488, 'force': (0, 0, 0)},
    ])
    ctx.camera(position=(.10, -.12, .09), target=(.02, .015, .003))
    ctx.video_annotation = lambda: '8 mN push then withdrawal; control strip at +Y'
    ctx.plots('push.force.x', 'asset.staple.strip.position.x', 'control.staple.strip.position.x',
              'asset.speed', 'control.speed')
    ctx.run(2.0)
    displacement = pushed_body.state['position'][0] - control_body.state['position'][0]
    displacements = (_states(ctx, pushed)[0][:, 0]-_states(ctx, control)[0][:, 0]).tolist()
    assert min(displacements) > .001, displacements
    ctx.metric('slide', displacement, unit='m', label='Loaded strip travel relative to idle')
    assert np.max(np.abs(_states(ctx, control)[0][:, 0])) < .000255
    _settled(ctx, ('asset', 'control'))
    assert abs(control_body.state['position'][0]) < .000255
    ctx.experiment_details = {
        'geometry_contact': _contact(ctx),
        'contract': '8 mN horizontal pulse with 4 ms ramps and 80 ms plateau; free control strip',
        'load_N': .008, 'load_interval_s': [.404, .488], 'relative_displacement_m': float(displacement), 'per_environment_displacement_m': displacements,
        'boundary': 'effective rigid-body sliding check; not a paper-feed or stapling-force measurement',
    }


def u_probe(ctx: TestContext):
    """A static 6 x 2 mm gauge lies inside the U; a free strip hits its side.

    A native D6 test guide leaves lateral translation free and locks other
    axes. The static gauge supplies collision contact; no asset pose is driven.
    """
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    ctx.simulation(gravity=(0, 0, 0))
    stage = ctx.session.stage
    gauge = UsdGeom.Cube.Define(stage, ctx.session.environment(0)+'/Fixtures/slot_gauge')
    gauge.CreateSizeAttr(1)
    xform = UsdGeom.Xformable(gauge)
    xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, .013))
    xform.AddScaleOp().Set(Gf.Vec3f(.010, .006, .002))
    gauge.CreateDisplayColorAttr([Gf.Vec3f(.8, .3, .08)])
    UsdPhysics.CollisionAPI.Apply(gauge.GetPrim())
    collision = PhysxSchema.PhysxCollisionAPI.Apply(gauge.GetPrim())
    collision.CreateContactOffsetAttr(.00005)
    collision.CreateRestOffsetAttr(0)
    material = UsdShade.Material.Define(stage, ctx.session.environment(0)+'/Fixtures/gauge_material')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.35)
    api.CreateDynamicFrictionAttr(.25)
    api.CreateRestitutionAttr(0)
    UsdShade.MaterialBindingAPI.Apply(gauge.GetPrim()).Bind(material, materialPurpose='physics')
    ctx.initial(position=(0, 0, .016), rotation=(0, 1, 0, 0))
    body = _body(ctx, ctx.asset)
    ctx.force('probe_load', body, keyframes=[
        {'time': 0, 'force': (0, 0, 0)},
        {'time': .2, 'force': (0, 0, 0)},
        {'time': .204, 'force': (0, .001, 0)},
        {'time': .5, 'force': (0, .001, 0)},
        {'time': .504, 'force': (0, 0, 0)},
    ])
    ctx.camera(position=(.045, -.060, .055), target=(0, 0, .013))
    ctx.video_annotation = lambda: 'Orange: static 6 x 2 mm gauge inside U | 1 mN +Y load then release'
    ctx.plots('probe_load.force.y', 'asset.staple.strip.position.y', 'asset.speed')
    # Ideal test guide: lock X/Z and rotations, leave Y translation free.
    guide = UsdPhysics.Joint.Define(stage, ctx.session.environment(0)+'/Fixtures/probe_guide')
    guide.CreateBody1Rel().SetTargets([body.path])
    guide.CreateLocalPos0Attr(Gf.Vec3f(0, 0, .016))
    guide.CreateLocalRot0Attr(Gf.Quatf(0, Gf.Vec3f(1, 0, 0)))
    guide.CreateLocalPos1Attr(Gf.Vec3f(0))
    guide.CreateLocalRot1Attr(Gf.Quatf(1))
    for axis in ('transX', 'transZ', 'rotX', 'rotY', 'rotZ'):
        limit = UsdPhysics.LimitAPI.Apply(guide.GetPrim(), axis)
        limit.CreateLowAttr(0)
        limit.CreateHighAttr(0)
    ctx.run(.8)
    peaks = [max(row[_prefix(e, 'asset')+'.'+BODY+'.position.y'] for row in ctx.rows) for e in range(ctx.num_envs)]
    assert all(.0025 < peak < .003095 for peak in peaks), peaks
    peak = peaks[0]
    ctx.metric('probe_travel', peak, unit='m', label='U-slot gauge contact travel')
    ctx.metric('probe_environment_spread', max(peaks)-min(peaks), unit='m', label='Contact travel spread across environments')
    ctx.experiment_details = {
        'contract': 'crown-down strip moves in zero gravity until a leg contacts the internal static gauge; withdraw load',
        'gauge_dimensions_m': [.010, .006, .002], 'gauge_center_m': [0, 0, .013],
        'fixture_guide': 'native D6 locks X/Z/rotation; Y free; no fixture reaction sampled',
        'initial_origin_m': [0, 0, .016], 'nominal_lateral_clearance_m': .00284,
        'contact_envelope_m': .0003,
        'accepted_peak_interval_m': [.0025, .003095],
        'peak_lateral_travel_m': peak, 'per_environment_peak_travel_m': peaks, 'applied_load_N': .001,
        'boundary': 'ideal gauge validates the U void and leg contact, not an insertion path or real stapler loading',
    }


def free_impulse(ctx: TestContext):
    """Equal COM/off-centre pulses test mass, free motion and instance isolation."""
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, -.03, .03))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(0, 0, .03))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .03, .03))
    bodies = [_body(ctx, a) for a in (ctx.asset, offset, idle)]
    for label, body in zip(('com', 'offset'), bodies):
        frames = [
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .2, 'force': (0, 0, 0)},
            {'time': .204, 'force': (.001, 0, 0)},
            {'time': .284, 'force': (.001, 0, 0)},
            {'time': .288, 'force': (0, 0, 0)},
        ]
        if label == 'offset':
            frames[0]['point'] = (0, .003, .003)
        ctx.force(label, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.10, -.14, .13), target=(.01, 0, .03))
    ctx.video_annotation = lambda: 'Zero gravity | COM / offset impulse / idle | all 50 staples form one rigid body'
    ctx.plots('com.force.x', 'asset.staple.strip.linear_velocity.x',
              'offset.staple.strip.linear_velocity.x', 'offset.angular_speed', 'idle.speed')
    ctx.run(.8)
    impulse = sum(row['com.force.x'] for row in ctx.rows)*ctx.dt
    velocities = [_states(ctx, b.instance)[2] for b in bodies]
    errors = [abs(.002*v[:, 0]-impulse)/impulse for v in velocities[:2]]
    assert np.max(errors) < .01, errors
    assert np.max(np.linalg.norm(velocities[0][:, 3:], axis=1)) < .001
    assert np.min(np.linalg.norm(velocities[1][:, 3:], axis=1)) > .1
    assert np.max(np.linalg.norm(velocities[2], axis=1)) < 1e-6
    errors = np.asarray(errors).tolist()
    ctx.metric('momentum_error', np.max(errors), unit='1', label='Maximum relative momentum error, all environments')
    ctx.metric('offset_spin', np.linalg.norm(velocities[1][0, 3:]), unit='rad/s', label='Offset-loaded strip angular speed')
    ctx.experiment_details = {
        'contract': 'equal 1 mN world-X COM/offset pulses and untouched free instance',
        'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
        'offset_body_point_m': [0, .003, .003], 'asset_pose_writes_after_initialization': False,
        'boundary': 'single rigid strip only; inertia uses uniform collision-volume estimate',
    }

def stop_reset(ctx: TestContext):
    """Two free instances; restore tensors only at an explicit reset boundary."""
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, -.03, .03), linear_velocity=(.02, 0, 0))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .03, .03))
    for instance in (ctx.asset, idle):
        _body(ctx, instance)
    ctx.camera(position=(.10, -.14, .13), target=(.01, 0, .03))
    ctx.video_annotation = lambda: 'Two free rigid strips | explicit stop/reset at 0.4 s'
    ctx.plots('asset.staple.strip.position.x', 'idle.staple.strip.position.x', 'asset.speed', 'idle.speed')
    ctx.ensure_initialized()
    saved = {name: instance.snapshot() for name, instance in ctx.session.instances.items()}
    ctx.run(.4)
    first = _states(ctx, ctx.asset)[0].copy()
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
        instance._staple_observation = None
        pos, quat, vel = _states(ctx, instance)
        positions, quaternions, velocity = saved[name]
        error = max(np.max(abs(pos-positions)), np.max(abs(quat-quaternions)), np.max(abs(vel-velocity)))
        assert error < 1e-6, (name, error)
        errors.append(float(error))
    ctx.run(.4)
    repeat_error = float(np.max(abs(_states(ctx, ctx.asset)[0]-first)))
    assert repeat_error < 1e-6, repeat_error
    assert np.max(np.linalg.norm(_states(ctx, idle)[2], axis=1)) < 1e-6
    ctx.metric('reset_error', max(errors), unit='1', label='Maximum reset component error, all environments')
    ctx.metric('repeat_error', repeat_error, unit='m', label='Repeated free-motion end-position error')
    ctx.experiment_details = {'contract': 'same-stage stop/reset, free moving and idle instances',
        'reset_at_timeline_s': .4, 'reset_errors': errors, 'repeat_position_error_m': repeat_error,
        'scope': 'lifecycle diagnostic excluded from steady-state benchmark; no behavior script'}



TESTS = {'orientation_settle': orientation_settle, 'low_drop': low_drop,
         'push_release': push_release, 'u_probe': u_probe, 'free_impulse': free_impulse, 'stop_reset': stop_reset}
