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


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(f'{instance.name}.speed', lambda: float(np.linalg.norm(body.state['velocity'][:3])), unit='m/s')
    ctx.observe(f'{instance.name}.angular_speed', lambda: float(np.linalg.norm(body.state['velocity'][3:])), unit='rad/s')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.35, dynamic_friction=.25, restitution=.05, contact_offset=.00025)


def _contact(ctx):
    from table_1000.physics.test import ground_clearance
    result = ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()),
                              ctx.rows, ctx.session.instances, 0)
    assert result['maximum_geometry_penetration_m'] < .000255, result
    return result


def _settled(ctx, names, speed=.005):
    tail = [row for row in ctx.rows if row['time'] >= ctx.session.time - .25 - 1e-9]
    for name in names:
        assert max(row[f'{name}.speed'] for row in tail) < speed, f'{name}: residual linear motion'
        assert max(row[f'{name}.angular_speed'] for row in tail) < speed / .01, f'{name}: residual angular motion'


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
    ctx.camera(position=(.16, -.22, .13), target=(.04, 0, .005))
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
    assert displacement > .001, 'push did not produce measurable strip translation'
    _settled(ctx, ('asset', 'control'))
    assert abs(control_body.state['position'][0]) < .000255
    ctx.experiment_details = {
        'geometry_contact': _contact(ctx),
        'contract': '8 mN horizontal pulse with 4 ms ramps and 80 ms plateau; free control strip',
        'load_N': .008, 'load_interval_s': [.404, .488], 'relative_displacement_m': float(displacement),
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
    gauge = UsdGeom.Cube.Define(stage, '/World/Fixtures/slot_gauge')
    gauge.CreateSizeAttr(1)
    xform = UsdGeom.Xformable(gauge)
    xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, .013))
    xform.AddScaleOp().Set(Gf.Vec3f(.010, .006, .002))
    gauge.CreateDisplayColorAttr([Gf.Vec3f(.8, .3, .08)])
    UsdPhysics.CollisionAPI.Apply(gauge.GetPrim())
    collision = PhysxSchema.PhysxCollisionAPI.Apply(gauge.GetPrim())
    collision.CreateContactOffsetAttr(.00005)
    collision.CreateRestOffsetAttr(0)
    material = UsdShade.Material.Define(stage, '/World/Fixtures/gauge_material')
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
    ctx.ensure_initialized()
    guide = UsdPhysics.Joint.Define(stage, '/World/Fixtures/probe_guide')
    guide.CreateBody1Rel().SetTargets([body.path])
    guide.CreateLocalPos0Attr(Gf.Vec3f(0, 0, .016))
    guide.CreateLocalRot0Attr(Gf.Quatf(0, Gf.Vec3f(1, 0, 0)))
    guide.CreateLocalPos1Attr(Gf.Vec3f(0))
    guide.CreateLocalRot1Attr(Gf.Quatf(1))
    for axis in ('transX', 'transZ', 'rotX', 'rotY', 'rotZ'):
        limit = UsdPhysics.LimitAPI.Apply(guide.GetPrim(), axis)
        limit.CreateLowAttr(0)
        limit.CreateHighAttr(0)
    try:
        ctx.run(.8)
    finally:
        stage.RemovePrim(guide.GetPath())
    peak = max(r['asset.staple.strip.position.y'] for r in ctx.rows)
    assert .0025 < peak < .003095, ('U-slot gauge side contact', peak)
    ctx.experiment_details = {
        'contract': 'crown-down strip moves in zero gravity until a leg contacts the internal static gauge; withdraw load',
        'gauge_dimensions_m': [.010, .006, .002], 'gauge_center_m': [0, 0, .013],
        'fixture_guide': 'native D6 locks X/Z/rotation; Y free; no fixture reaction sampled',
        'initial_origin_m': [0, 0, .016], 'nominal_lateral_clearance_m': .00284,
        'contact_envelope_m': .0003,
        'accepted_peak_interval_m': [.0025, .003095],
        'peak_lateral_travel_m': peak, 'applied_load_N': .001,
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
    errors = [abs(.002*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01, errors
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .001
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {
        'contract': 'equal 1 mN world-X COM/offset pulses and untouched free instance',
        'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
        'offset_body_point_m': [0, .003, .003], 'asset_pose_writes_after_initialization': False,
        'boundary': 'single rigid strip only; inertia uses uniform collision-volume estimate',
    }


TESTS = {'orientation_settle': orientation_settle, 'low_drop': low_drop,
         'push_release': push_release, 'u_probe': u_probe, 'free_impulse': free_impulse}
