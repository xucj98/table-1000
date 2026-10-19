"""Native rigid-roll tests; SI units, estimated materials, no adhesion or peel.

Ideal pins are test fixtures only. Asset poses are written only at initialization.
"""
import json
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'tape.roll'
MASS_KG = .050
# Rest one polygon face on the floor. Exact vertex-down is an unstable balance
# and produces spontaneous rolling during central pin insertion.
_HALF_SECTOR = math.pi/32
UPRIGHT = (math.sqrt(.5)*math.cos(_HALF_SECTOR), math.sqrt(.5)*math.cos(_HALF_SECTOR),
           -math.sqrt(.5)*math.sin(_HALF_SECTOR), math.sqrt(.5)*math.sin(_HALF_SECTOR))
RADIUS_M = .0305
AXIS_HEIGHT_M = RADIUS_M*math.cos(math.pi/16) + .0001


def col(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def observe(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: float(np.linalg.norm(body.state['velocity'][:3])), unit='m/s')
    ctx.observe(instance.name+'.spin', lambda: float(np.linalg.norm(body.state['velocity'][3:])), unit='rad/s')
    ctx.observe(instance.name+'.axis_vertical', lambda: abs(float(body.state['rotation'][2, 2])), unit='1')
    return body


def ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.28, restitution=0, contact_offset=.0001)


def settled(ctx, names):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        assert max(r[name+'.speed'] for r in tail) < .01, name+' linear velocity'
        assert max(r[name+'.spin'] for r in tail) < .15, name+' angular velocity'
        points = [[r[col(name, 'position', axis)] for axis in 'xyz'] for r in tail]
        assert np.linalg.norm(np.ptp(points, axis=0)) < .001, name+' drift'


def clearance(ctx):
    from table_1000.physics.test import ground_clearance
    result = ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()),
                              ctx.rows, ctx.session.instances, 0)
    assert result['maximum_geometry_penetration_m'] < .0015, result
    return result


def roll_decay(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(0, -.08, AXIS_HEIGHT_M), rotation=UPRIGHT)
    flat = ctx.spawn(ctx.asset_path, name='flat', position=(0, .06, .00002))
    rolling, sliding = [observe(ctx, a) for a in (ctx.asset, flat)]
    for label, body in [('roll_push', rolling), ('flat_push', sliding)]:
        ctx.force(label, body, keyframes=[
            {'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
            {'time': .404, 'force': (.12, 0, 0)}, {'time': .704, 'force': (.12, 0, 0)},
            {'time': .708, 'force': (0, 0, 0)}])
    ctx.camera(position=(.65, -.70, .46), target=(.20, 0, .025))
    ctx.video_annotation = lambda: 'Lower: upright roll | upper: flat | equal 0.12 N COM pulse, release 0.708 s'
    ctx.plots('roll_push.force.x', col('asset', 'position', 'x'), col('flat', 'position', 'x'),
              col('asset', 'angular_velocity', 'y'), 'asset.spin', 'flat.spin')
    ctx.run(4)
    peak = max(r['asset.spin'] for r in ctx.rows)
    displacement = ctx.rows[-1][col('asset', 'position', 'x')]-ctx.rows[0][col('asset', 'position', 'x')]
    flat_drift = max(abs(r[col('flat', 'position', 'x')]-ctx.rows[0][col('flat', 'position', 'x')]) for r in ctx.rows)
    assert peak > 1
    assert displacement > .01
    assert flat_drift < .001
    settled(ctx, ('asset', 'flat'))
    ctx.experiment_details = {'contract': 'equal low COM load: upright rolling versus flat static-friction support; then withdrawal',
        'pulse_N': .12, 'withdrawal_s': .708, 'peak_spin_rad_s': peak,
        'travel_m': displacement, 'flat_drift_m': flat_drift, 'geometry_contact': clearance(ctx),
        'boundary': '16-sided circumference causes contact energy loss; no calibrated rolling resistance or bearing drag; zero artificial damping'}


def tip_over(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(-.06, 0, AXIS_HEIGHT_M), rotation=UPRIGHT)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(.06, 0, AXIS_HEIGHT_M), rotation=UPRIGHT)
    loaded, control = [observe(ctx, a) for a in (ctx.asset, idle)]
    ctx.force('side_push', loaded, point_frame='body', keyframes=[
        {'time': 0, 'force': (0, 0, 0), 'point': (0, .026, .0102)},
        {'time': .4, 'force': (0, 0, 0)}, {'time': .404, 'force': (0, .18, 0)},
        {'time': .652, 'force': (0, .18, 0)}, {'time': .656, 'force': (0, 0, 0)}])
    ctx.camera(position=(.28, -.38, .26), target=(0, .025, .02))
    ctx.video_annotation = lambda: 'Left: upper-edge axial push 0.18 N | right: idle standing | release 0.656 s'
    ctx.plots('side_push.force.y', 'asset.axis_vertical', 'idle.axis_vertical',
              col('asset', 'position', 'y'), 'asset.spin')
    ctx.run(4)
    assert max(r['asset.axis_vertical'] for r in ctx.rows) > math.cos(math.radians(10))
    assert max(r['idle.axis_vertical'] for r in ctx.rows) < math.sin(math.radians(5))
    settled(ctx, ('asset', 'idle'))
    ctx.experiment_details = {'contract': 'free standing roll versus idle standing control; push parallel to bore axis at upper edge, then withdrawal',
        'load_N': .18, 'application_point_body_m': [0, .026, .0102], 'withdrawal_s': .656,
        'geometry_contact': clearance(ctx), 'boundary': 'whole rigid roll; no winding deformation or adhesion'}


def bore_contact(ctx: TestContext):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    ground(ctx)
    ctx.initial(position=(0, 0, AXIS_HEIGHT_M), rotation=UPRIGHT)
    body = observe(ctx, ctx.asset)
    stage = ctx.session.stage
    # Explicit convex cylinder avoids a backend-specific implicit primitive proxy.
    pin = UsdGeom.Mesh.Define(stage, '/World/Fixtures/core_pin')
    segments = 32
    points = [( .014*math.cos(i*math.tau/segments), y,
                .014*math.sin(i*math.tau/segments))
              for y in (-.030, .030) for i in range(segments)]
    faces = [list(reversed(range(segments))), list(range(segments, 2*segments))]
    faces += [[i, (i+1)%segments, (i+1)%segments+segments, i+segments] for i in range(segments)]
    pin.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    pin.CreateFaceVertexCountsAttr([len(f) for f in faces])
    pin.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    pin.CreateSubdivisionSchemeAttr('none')
    UsdPhysics.MeshCollisionAPI.Apply(pin.GetPrim()).CreateApproximationAttr('convexHull')
    pin.CreateDisplayColorAttr([Gf.Vec3f(.10, .35, .85)])
    op = pin.AddTranslateOp()
    op.Set(Gf.Vec3d(0, -.09, AXIS_HEIGHT_M))
    UsdPhysics.RigidBodyAPI.Apply(pin.GetPrim()).CreateKinematicEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(pin.GetPrim())
    api = PhysxSchema.PhysxCollisionAPI.Apply(pin.GetPrim())
    api.CreateContactOffsetAttr(.0001)
    api.CreateRestOffsetAttr(0)
    material = UsdShade.Material.Define(stage, '/World/Fixtures/pin_material')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.30)
    api.CreateDynamicFrictionAttr(.22)
    api.CreateRestitutionAttr(0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr('average')
    UsdShade.MaterialBindingAPI.Apply(pin.GetPrim()).Bind(material, materialPurpose='physics')

    def pin_position(t):
        # Enter centrally, push the inner wall sideways, then withdraw axially.
        x = .04*min(1, max(0, (t-1.6)/.8))
        y = -.09+.09*min(1, max(0, (t-.4)/.8))+.09*min(1, max(0, (t-2.4)/.8))
        return x, y, AXIS_HEIGHT_M

    original_actions = ctx.session.actions
    def actions(t, dt):
        original_actions(t, dt)
        op.Set(Gf.Vec3d(*pin_position(t)))
    ctx.session.actions = actions
    ctx.observe('fixture.pin_x', lambda: pin_position(ctx.session.time)[0], unit='m')
    ctx.observe('fixture.pin_y', lambda: pin_position(ctx.session.time)[1], unit='m')
    ctx.camera(position=(.22, -.32, .23), target=(.015, 0, .025))
    ctx.video_annotation = lambda: 'Blue 28 mm pin: central entry, inner-wall push, axial withdrawal | free roll'
    ctx.plots('fixture.pin_x', 'fixture.pin_y', col('asset', 'position', 'x'),
              col('asset', 'position', 'y'), 'asset.spin')
    ctx.run(4)
    central = [r for r in ctx.rows if .4 <= r['time'] <= 1.2]
    central_drift = max(abs(r[col('asset', 'position', 'x')]) for r in central)
    assert central_drift < .001
    pushed = [r for r in ctx.rows if 2.2 <= r['time'] <= 2.4]
    assert min(r[col('asset', 'position', 'x')] for r in pushed) > .025
    settled(ctx, ('asset',))
    ctx.experiment_details = {'contract': 'ideal pin clears central hole, then inner wall transmits lateral displacement; pin withdraws with asset unrestrained',
        'fixture': 'kinematic cylinder, radius .014 m, length .060 m, axis Y; no asset joint',
        'central_entry_s': [.4, 1.2], 'lateral_push_s': [1.6, 2.4], 'withdrawal_s': [2.4, 3.2],
        'central_entry_drift_m': central_drift, 'geometry_contact': clearance(ctx),
        'boundary': 'fixture reaction not sampled; not a spindle, adhesion, or unwinding model'}


def drop_orientations(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(0, -.08, .05))
    side = ctx.spawn(ctx.asset_path, name='side', position=(0, .08, AXIS_HEIGHT_M-.0001+.05), rotation=UPRIGHT)
    for instance in (ctx.asset, side):
        observe(ctx, instance)
    ctx.camera(position=(.40, -.55, .36), target=(0, 0, .055))
    ctx.video_annotation = lambda: '50 mm lowest-point release: flat / standing | independent free instances'
    ctx.plots(col('asset', 'position', 'z'), col('side', 'position', 'z'), 'asset.spin', 'side.spin')
    ctx.run(3)
    settled(ctx, ('asset', 'side'))
    ctx.experiment_details = {'contract': '50 mm minimum-point gravity release in flat and standing orientations',
        'release_clearance_m': .05, 'geometry_contact': clearance(ctx),
        'boundary': 'rigid zero-restitution engineering impact; no compression, peel or damage calibration'}


def free_impulse(ctx: TestContext):
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.06, -.08, .08))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(-.06, 0, .08))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.06, .08, .08))
    bodies = [observe(ctx, a) for a in (ctx.asset, offset, idle)]
    for label, body in zip(('com_push', 'offset_push'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
                  {'time': .404, 'force': (.06, 0, 0)}, {'time': .604, 'force': (.06, 0, 0)},
                  {'time': .608, 'force': (0, 0, 0)}]
        if label == 'offset_push':
            frames[0]['point'] = (0, .015, .0102)
        ctx.force(label, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.28, -.38, .30), target=(0, 0, .08))
    ctx.video_annotation = lambda: 'Zero gravity: COM impulse / eccentric impulse / idle | free whole rolls'
    ctx.plots('com_push.force.x', col('asset', 'linear_velocity', 'x'),
              col('offset', 'linear_velocity', 'x'), col('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'contract': 'equal COM/eccentric external impulses and idle control, gravity off',
        'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
        'boundary': 'single rigid roll integrity; collision-derived inertia, not measured winding/core density distribution'}


TESTS = {'roll_decay': roll_decay, 'tip_over': tip_over, 'bore_contact': bore_contact,
         'drop_orientations': drop_orientations, 'free_impulse': free_impulse}
