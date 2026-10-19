"""Single rigid basket experiments; SI units, ideal test-only contact rods."""
import json
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'file_basket.shell'
MASS_KG = .280
# Rest the sloping long side on floor edge and rim, rather than an unstable
# exact 90-degree rim-only balance. Derived from the delivered geometry.
SIDE_ANGLE = math.atan2(.093, .121-.1031)
SIDE = (math.cos(SIDE_ANGLE/2), math.sin(SIDE_ANGLE/2), 0, 0)
SIDE_HEIGHT = .10124175926599437


def column(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def observe(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    ctx.observe(instance.name+'.tilt', lambda: math.acos(float(np.clip(body.state['rotation'][2, 2], -1, 1))), unit='rad')
    return body


def ground(ctx):
    ctx.ground(static_friction=.4, dynamic_friction=.3, contact_offset=.0001)


def stable(ctx, names):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        assert max(r[name+'.speed'] for r in tail) < .01, name+' velocity'
        p = np.array([[r[column(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert np.linalg.norm(np.ptp(p, axis=0)) < .001, name+' drift'


def clearance(ctx):
    from table_1000.physics.test import ground_clearance
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    result = ground_clearance(model, ctx.rows, ctx.session.instances, 0)
    assert result['maximum_geometry_penetration_m'] < .002, result
    return result


def slide_compare(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(0, -.325, .00002))
    side = ctx.spawn(ctx.asset_path, name='side', position=(0, .325, SIDE_HEIGHT+.00002), rotation=SIDE)
    bodies = [observe(ctx, a) for a in (ctx.asset, side)]
    ctx.camera(position=(1.65, -1.9, 1.65), target=(.13, 0, .05))
    ctx.video_annotation = lambda: 'Bottom / long-side support | equal COM ramp 0-2 N | release 1.24 s'
    for label, body in zip(('bottom_push', 'side_push'), bodies):
        ctx.force(label, body, keyframes=[{'time': 0, 'force': (0, 0, 0)},
            {'time': .5, 'force': (0, 0, 0)}, {'time': 1.2, 'force': (2, 0, 0)},
            {'time': 1.24, 'force': (0, 0, 0)}])
    ctx.plots('bottom_push.force.x', column('asset', 'position', 'x'), column('side', 'position', 'x'),
              'asset.speed', 'side.speed')
    ctx.run(3)
    stable(ctx, ('asset', 'side'))
    loaded = [r for r in ctx.rows if r['time'] >= .5-1e-8]
    assert max(abs(r['side.tilt']-SIDE_ANGLE) for r in loaded) < math.radians(5)
    assert max(r['asset.tilt'] for r in loaded) < math.radians(5)
    onset, stop = {}, {}
    release = min(ctx.rows, key=lambda r: abs(r['time']-1.24))
    for name, force in [('asset', 'bottom_push'), ('side', 'side_push')]:
        key = column(name, 'position', 'x')
        start = ctx.rows[0][key]
        assert ctx.rows[-1][key]-start > .005
        row = next(r for r in ctx.rows if r[key]-start >= .0005)
        onset[name] = {'time_s': row['time'], 'applied_force_N': row[force+'.force.x']}
        stop[name] = ctx.rows[-1][key]-release[key]
    ctx.experiment_details = {'onset_0_5mm': onset, 'release_time_s': 1.24,
        'stopping_distance_m': stop, 'geometry_contact': clearance(ctx),
        'boundary': 'equal PP material; support-dependent response, not measured material friction'}


def drop_orientations(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(0, -.325, .05))
    side = ctx.spawn(ctx.asset_path, name='side', position=(0, .325, SIDE_HEIGHT+.05), rotation=SIDE)
    for a in (ctx.asset, side):
        observe(ctx, a)
    ctx.camera(position=(1.6, -1.9, 1.6), target=(0, 0, .05))
    ctx.video_annotation = lambda: 'Bottom / long-side support | 50 mm minimum-point gravity drop'
    ctx.plots(column('asset', 'position', 'z'), column('side', 'position', 'z'), 'asset.speed', 'side.speed')
    ctx.run(3)
    stable(ctx, ('asset', 'side'))
    assert abs(ctx.rows[-1]['side.tilt']-SIDE_ANGLE) < math.radians(5)
    assert ctx.rows[-1]['asset.tilt'] < math.radians(5)
    ctx.experiment_details = {'release_clearance_m': .05, 'geometry_contact': clearance(ctx),
        'boundary': 'rigid zero-restitution engineering contact; no impact or PP flex calibration'}


def handle_contact(ctx: TestContext):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    ground(ctx)
    ctx.initial(position=(0, -.325, .00002))
    eccentric = ctx.spawn(ctx.asset_path, name='eccentric', position=(0, .325, .00002))
    for a in (ctx.asset, eccentric):
        observe(ctx, a)
    rods = []
    stage = ctx.session.stage
    material = UsdShade.Material.Define(stage, '/World/Fixtures/RodMaterial')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.4); api.CreateDynamicFrictionAttr(.3); api.CreateRestitutionAttr(0)
    # 24 mm width and 6 mm height fit in 90 x 20.86 mm end handles.
    # Length 60 mm crosses each 2.2 mm end wall; only rod surfaces touch asset.
    for i, (x, y) in enumerate([(-.157, -.325), (.157, -.325), (.157, .325)]):
        rod = UsdGeom.Cube.Define(stage, f'/World/Fixtures/Rod{i}')
        rod.CreateSizeAttr(1)
        rod.CreateDisplayColorAttr([Gf.Vec3f(.15, .38, .85)])
        op = rod.AddTranslateOp(); op.Set(Gf.Vec3d(x, y, .084))
        rod.AddScaleOp().Set(Gf.Vec3f(.060, .024, .006))
        api = UsdPhysics.RigidBodyAPI.Apply(rod.GetPrim()); api.CreateKinematicEnabledAttr(True)
        UsdPhysics.CollisionAPI.Apply(rod.GetPrim())
        api = PhysxSchema.PhysxCollisionAPI.Apply(rod.GetPrim())
        api.CreateContactOffsetAttr(.0001); api.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(rod.GetPrim()).Bind(material, materialPurpose='physics')
        rods.append((op, x, y))
    def lift(t):
        return .060*min(1, max(0, (t-.5)/.5), max(0, (1.75-t)/.5))
    old = ctx.session.actions
    def actions(t, dt):
        old(t, dt)
        for op, x, y in rods:
            op.Set(Gf.Vec3d(x, y, .084+lift(t)))
    ctx.session.actions = actions
    ctx.observe('fixture.lift', lambda: lift(ctx.session.time), unit='m')
    ctx.camera(position=(1.5, -1.7, 1.4), target=(0, 0, .08))
    ctx.video_annotation = lambda: 'Blue contact rods: lower BOTH end holes / upper ONE end hole | rise, hold, lower'
    ctx.plots('fixture.lift', column('asset', 'position', 'z'), column('eccentric', 'position', 'z'),
              'asset.tilt', 'eccentric.tilt')
    ctx.run(3)
    lifted = [r for r in ctx.rows if 1 <= r['time'] <= 1.25]
    assert min(r[column('asset', 'position', 'z')] for r in lifted) > .03
    assert max(r['asset.tilt'] for r in lifted) < math.radians(5)
    assert max(r['eccentric.tilt'] for r in ctx.rows) > math.radians(10)
    assert all(np.isfinite(list(r.values())).all() for r in ctx.rows)
    ctx.experiment_details = {'fixture': 'three native kinematic contact rods, no asset constraint or pose writes',
        'rod_size_m': [.060, .024, .006], 'rod_start_z_m': .084, 'lift_m': .060,
        'symmetric_min_height_m': min(r[column('asset', 'position', 'z')] for r in lifted),
        'symmetric_max_tilt_rad': max(r['asset.tilt'] for r in lifted),
        'eccentric_peak_tilt_rad': max(r['eccentric.tilt'] for r in ctx.rows),
        'geometry_contact': clearance(ctx), 'boundary': 'unloaded basket and ideal contact fixture, not books or robot grasp; fixture reaction not sampled'}


def free_impulse(ctx: TestContext):
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, -.45, .1))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(0, 0, .1))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .45, .1))
    bodies = [observe(ctx, a) for a in (ctx.asset, offset, idle)]
    for name, body in zip(('com_push', 'offset_push'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
            {'time': .404, 'force': (.28, 0, 0)}, {'time': .604, 'force': (.28, 0, 0)},
            {'time': .608, 'force': (0, 0, 0)}]
        if name == 'offset_push':
            frames[0]['point'] = (0, .08, .05)
        ctx.force(name, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(1.8, -2.1, 1.7), target=(.06, 0, .12))
    ctx.video_annotation = lambda: 'Zero gravity | COM impulse / eccentric impulse / idle | no ground or fixtures'
    ctx.plots('com_push.force.x', column('asset', 'linear_velocity', 'x'),
              column('offset', 'linear_velocity', 'x'), column('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
        'boundary': 'rigid unloaded basket, no gravity or ground; geometry-derived inertia'}


TESTS = {'slide_compare': slide_compare, 'drop_orientations': drop_orientations,
         'handle_contact': handle_contact, 'free_impulse': free_impulse}
