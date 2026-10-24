"""Single-ID closed notebook experiments; native contact, SI units and wxyz.

Accessories stay rigid. Edge-standing contacts can involve cover rims/strap;
these are effective orientation comparisons, not isolated paper friction tests.
"""
from contextlib import contextmanager
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'notebook.closed'
MASS_KG = .640
WIDTH, LENGTH, HEIGHT = .210, .297, .012
# Use the +Y short edge, away from the -Y bookmark on larger variants.
EDGE = (math.sqrt(.5), -math.sqrt(.5), 0, 0)


def _col(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.00005)


def _camera(ctx, *, position, target):
    scale = max(1., LENGTH/.143)
    position = np.asarray(target)+(np.asarray(position)-np.asarray(target))*scale
    ctx.camera(position=tuple(position), target=target)


def _stable(ctx, names):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        points = np.array([[r[_col(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert max(r[name+'.speed'] for r in tail) < .01, name+' not stopped'
        assert np.linalg.norm(np.ptp(points, axis=0)) < .001, name+' drifting'


def _contact(ctx):
    import json
    from table_1000.physics.test import ground_clearance
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    report = ground_clearance(model, ctx.rows, ctx.session.instances, 0)
    assert report['maximum_geometry_penetration_m'] < .002, report
    return report


def _ramp(ctx, label, body, force, *, point=None, release=1.34):
    start = {'time': 0, 'force': (0, 0, 0)}
    if point is not None:
        start['point'] = point
    ctx.force(label, body, point_frame='body', keyframes=[start,
        {'time': .5, 'force': (0, 0, 0)},
        {'time': 1.3, 'force': force},
        {'time': release, 'force': (0, 0, 0)}])


@contextmanager
def _rotation_guide(ctx, body):
    """Test fixture: native D6 locks rotation, leaves translation free."""
    from pxr import Gf, UsdPhysics
    ctx.ensure_initialized()
    state = body.state
    joint = UsdPhysics.Joint.Define(ctx.session.stage, '/World/Fixtures/edge_rotation_guide')
    joint.CreateBody1Rel().SetTargets([body.path])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*state['position']))
    q = state['quaternion']
    joint.CreateLocalRot0Attr(Gf.Quatf(q[0], Gf.Vec3f(*q[1:])))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    for axis in ('rotX', 'rotY', 'rotZ'):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr(0)
        limit.CreateHighAttr(0)
    try:
        yield
    finally:
        ctx.session.stage.RemovePrim(joint.GetPath())


def contact_slide(ctx: TestContext):
    _ground(ctx)
    gap = LENGTH + .04
    ctx.initial(position=(0, -gap, .00002))
    back = ctx.spawn(ctx.asset_path, name='back', position=(0, 0, HEIGHT+.00002), rotation=(0, 1, 0, 0))
    edge = ctx.spawn(ctx.asset_path, name='edge', position=(0, gap, LENGTH/2+.00002), rotation=EDGE)
    bodies = [_body(ctx, a) for a in (ctx.asset, back, edge)]
    for name, body in zip(('front_push', 'back_push', 'edge_push'), bodies):
        _ramp(ctx, name, body, (.8*MASS_KG*9.81, 0, 0))
    _camera(ctx, position=(.85, -1.05, .9), target=(.15, 0, LENGTH/4))
    ctx.video_annotation = lambda: 'Bottom/front | middle/back | upper/edge (rotation guide) | equal COM force, release 1.34 s'
    ctx.plots('front_push.force.x', *[_col(n, 'position', 'x') for n in ('asset', 'back', 'edge')],
              *[_col(n, 'linear_velocity', 'x') for n in ('asset', 'back', 'edge')])
    with _rotation_guide(ctx, bodies[2]):
        ctx.run(3)
    onset, stopping = {}, {}
    release = min(ctx.rows, key=lambda r: abs(r['time']-1.34))
    for name, label in zip(('asset', 'back', 'edge'), ('front_push', 'back_push', 'edge_push')):
        col = _col(name, 'position', 'x')
        row = next(r for r in ctx.rows if r[col]-ctx.rows[0][col] >= .0005)
        onset[name] = {'time_s': row['time'], 'applied_force_N': row[label+'.force.x']}
        assert ctx.rows[-1][col]-ctx.rows[0][col] > .005
        stopping[name] = ctx.rows[-1][col]-release[col]
    _stable(ctx, ('asset', 'back', 'edge'))
    ctx.experiment_details = {'onset_definition_m': .0005, 'onset': onset, 'stopping_distance_m': stopping,
        'release_time_s': 1.34, 'geometry_contact': _contact(ctx),
        'fixture': 'edge rotation-only native D6; all translations free',
        'boundary': 'orientation/contact geometry comparison; strap or cover rims can support the book; no pure paper friction claim'}


def drop_orientations(ctx: TestContext):
    _ground(ctx)
    gap = WIDTH+.05
    ctx.initial(position=(-gap, 0, .05))
    back = ctx.spawn(ctx.asset_path, name='back', position=(0, 0, HEIGHT+.05), rotation=(0, 1, 0, 0))
    edge = ctx.spawn(ctx.asset_path, name='edge', position=(gap, 0, LENGTH/2+.05), rotation=EDGE)
    for a in (ctx.asset, back, edge):
        _body(ctx, a)
    _camera(ctx, position=(.7, -.95, .7), target=(0, 0, .045))
    ctx.video_annotation = lambda: 'Front / back / short edge | 50 mm lowest-point gravity drop | no fixtures'
    ctx.plots(*[_col(n, 'position', 'z') for n in ('asset', 'back', 'edge')],
              *[n+'.speed' for n in ('asset', 'back', 'edge')])
    ctx.run(3)
    _stable(ctx, ('asset', 'back', 'edge'))
    ctx.experiment_details = {'release_clearance_m': .05, 'geometry_contact': _contact(ctx),
                              'boundary': 'rigid zero-restitution estimate, not calibrated impact or deformation'}


def free_tip(ctx: TestContext):
    # The 4 ms tip exceeded 2 mm geometric ground penetration.
    ctx.simulation(dt=.002)
    _ground(ctx)
    ctx.initial(position=(WIDTH*.8, 0, LENGTH/2+.00002), rotation=EDGE)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-WIDTH*.8, 0, LENGTH/2+.00002), rotation=EDGE)
    loaded, control = [_body(ctx, a) for a in (ctx.asset, idle)]
    for name, body in (('loaded', loaded), ('control', control)):
        ctx.observe(name+'.angle', lambda b=body: math.acos(float(np.clip(-b.state['rotation'][2, 1], -1, 1))), unit='rad')
    point = (0, -LENGTH*.4, HEIGHT/2)
    ctx.force('push', loaded, point_frame='body', keyframes=[
        {'time': 0, 'force': (0, 0, 0), 'point': point},
        {'time': .4, 'force': (0, 0, 0)},
        {'time': .8, 'force': (0, .25*MASS_KG*9.81, 0)},
        {'time': 1, 'force': (0, .25*MASS_KG*9.81, 0)},
        {'time': 1.04, 'force': (0, 0, 0)}])
    _camera(ctx, position=(.65, -.85, .6), target=(0, .02, LENGTH/4))
    ctx.video_annotation = lambda: 'Right: high +Y push | left: idle | both free edge-standing | release 1.04 s'
    ctx.plots('push.force.y', 'loaded.angle', 'control.angle', 'asset.speed')
    ctx.run(3)
    assert max(r['loaded.angle'] for r in ctx.rows) > math.pi/4
    assert max(r['control.angle'] for r in ctx.rows if r['time'] >= .5) < math.radians(5)
    _stable(ctx, ('asset', 'idle'))
    ctx.experiment_details = {'application_point_body_m': point, 'release_time_s': 1.04,
                              'peak_tilt_rad': max(r['loaded.angle'] for r in ctx.rows),
                              'geometry_contact': _contact(ctx), 'fixture': None}


def stack_slide(ctx: TestContext):
    # 4/2 ms stacks exceeded 10 mm/s residual speed; 1 ms passed diagnostics.
    ctx.simulation(dt=.001)
    _ground(ctx)
    gap = LENGTH+.08
    ctx.initial(position=(0, -gap/2, .00002))
    instances = [ctx.asset]
    for name, x, y, z in [('middle', .03*WIDTH, -gap/2, HEIGHT+.00012),
                          ('top', .06*WIDTH, -gap/2, 2*HEIGHT+.00022),
                          ('base_idle', 0, gap/2, .00002),
                          ('middle_idle', .03*WIDTH, gap/2, HEIGHT+.00012),
                          ('top_idle', .06*WIDTH, gap/2, 2*HEIGHT+.00022)]:
        instances.append(ctx.spawn(ctx.asset_path, name=name, position=(x, y, z)))
    bodies = [_body(ctx, a) for a in instances]
    _ramp(ctx, 'top_push', bodies[2], (.8*MASS_KG*9.81, 0, 0))
    _camera(ctx, position=(.85, -1.05, .85), target=(.1, 0, HEIGHT*1.5))
    ctx.video_annotation = lambda: 'Lower: loaded top | upper: idle stack | both bottom books fixed | same ID, rigid accessories'
    ctx.observe('loaded.relative_x', lambda: bodies[2].state['position'][0]-bodies[0].state['position'][0], unit='m')
    ctx.plots('top_push.force.x', 'loaded.relative_x', 'top.speed', 'top_idle.speed',
              _col('top', 'position', 'z'), _col('top_idle', 'position', 'z'))
    with ctx.fixed(bodies[0]), ctx.fixed(bodies[3]):
        ctx.run(4)
    assert ctx.rows[-1]['loaded.relative_x']-ctx.rows[0]['loaded.relative_x'] > .005
    relative_drift = {}
    for name in ('middle_idle', 'top_idle'):
        points = np.array([[r[_col(name, 'position', a)]-r[_col('base_idle', 'position', a)] for a in 'xy']
                           for r in ctx.rows if r['time'] >= .5])
        relative_drift[name] = float(np.max(np.linalg.norm(points-points[0], axis=1)))
        assert relative_drift[name] < .002
    _stable(ctx, [a.name for a in instances])
    ctx.experiment_details = {'instances': 6, 'relative_control_drift_m': relative_drift,
        'release_time_s': 1.34, 'geometry_contact': _contact(ctx),
        'fixture': 'both bottom books fixed; upper books dynamically free',
        'boundary': 'same-ID offset-stack sliding only; no whole-stack transport or mixed-ID acceptance'}


def free_impulse(ctx: TestContext):
    ctx.simulation(dt=ctx.dt, gravity=(0, 0, 0))
    gap = math.hypot(WIDTH+.026, LENGTH+.07)+.08
    ctx.initial(position=(-.06, -gap, .15))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(-.06, 0, .15))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.06, gap, .15))
    bodies = [_body(ctx, a) for a in (ctx.asset, offset, idle)]
    for label, body in zip(('com_push', 'offset_push'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
                  {'time': .404, 'force': (MASS_KG, 0, 0)},
                  {'time': .604, 'force': (MASS_KG, 0, 0)}, {'time': .608, 'force': (0, 0, 0)}]
        if label == 'offset_push':
            frames[0]['point'] = (0, LENGTH*.25, HEIGHT/2)
        ctx.force(label, body, point_frame='body', keyframes=frames)
    _camera(ctx, position=(.85, -1.05, .85), target=(.04, 0, .15))
    ctx.video_annotation = lambda: 'Zero gravity | COM impulse / offset impulse / idle | no fixtures or pose writes'
    ctx.plots('com_push.force.x', _col('asset', 'linear_velocity', 'x'),
              _col('offset', 'linear_velocity', 'x'), _col('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'applied_impulse_N_s': impulse, 'relative_momentum_errors': errors,
                              'ground': None, 'gravity_m_s2': [0, 0, 0]}


TESTS = {'contact_slide': contact_slide, 'drop_orientations': drop_orientations,
         'free_tip': free_tip, 'stack_slide': stack_slide, 'free_impulse': free_impulse}
