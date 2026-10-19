"""Native rigid eraser experiments, SI units and wxyz rotations.

Paper/rubber are uncalibrated contact estimates. End-standing rubber needs a
rotation-only test guide for sliding: free upright response is tested separately.
No asset pose writes during integration; forces are applied loads, not reactions.
"""
from contextlib import contextmanager
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'eraser.body'
MASS_KG = .014
END_UP = (math.sqrt(.5), 0, math.sqrt(.5), 0)


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(f'{instance.name}.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.00005)


def _column(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def _stable(ctx, names, speed=.01):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        positions = np.array([[r[_column(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert max(r[f'{name}.speed'] for r in tail) < speed, f'{name}: unsettled velocity'
        assert np.linalg.norm(np.ptp(positions, axis=0)) < .001, f'{name}: unsettled position'


def _contact(ctx):
    from table_1000.physics.test import ground_clearance
    import json
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    clearance = ground_clearance(model, ctx.rows, ctx.session.instances, 0)
    assert clearance['maximum_geometry_penetration_m'] < .002
    return clearance


@contextmanager
def _rotation_guide(ctx, body):
    """Test-only native D6: rotation locked at initial pose, translation free."""
    from pxr import Gf, UsdPhysics
    ctx.ensure_initialized()
    state = body.state
    path = '/World/Fixtures/rubber_rotation_guide'
    joint = UsdPhysics.Joint.Define(ctx.session.stage, path)
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
        ctx.session.stage.RemovePrim(path)


def contact_slide(ctx: TestContext):
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
    with _rotation_guide(ctx, bodies[1]):
        ctx.run(3)
    onset = {}
    for name, force in [('asset', 'paper_push'), ('rubber', 'rubber_push')]:
        column = _column(name, 'position', 'x')
        start = ctx.rows[0][column]
        row = next(r for r in ctx.rows if r[column]-start >= .0005)
        onset[name] = {'time_s': row['time'], 'applied_force_N': row[force+'.force.x']}
        assert ctx.rows[-1][column]-start > .005
    assert onset['rubber']['applied_force_N']-onset['asset']['applied_force_N'] > .01
    assert bodies[0].state['rotation'][2, 2] > .98
    assert -bodies[1].state['rotation'][2, 0] > .98
    # Compare the idle control during loading, after the common 0.5 s settling
    # interval. Report cold-start settling separately; it is not load response.
    positions = np.array([[r[_column('idle', 'position', a)] for a in 'xy'] for r in ctx.rows])
    initial_settling_drift = float(np.max(np.linalg.norm(positions-positions[0], axis=1)))
    settled = positions[np.array([r['time'] >= .5-1e-8 for r in ctx.rows])]
    control_drift = float(np.max(np.linalg.norm(settled-settled[0], axis=1)))
    assert control_drift < .0005
    assert -bodies[2].state['rotation'][2, 0] > math.cos(math.radians(5))
    _stable(ctx, ('asset', 'rubber', 'idle'), speed=.005)
    release = min(ctx.rows, key=lambda r: abs(r['time']-1.34))
    ctx.experiment_details = {
        'contract': 'equal COM ramp and withdrawal, paper flat versus rotation-guided rubber end; free idle control',
        'onset_definition_m': .0005, 'onset': onset, 'release_time_s': 1.34,
        'geometry_contact': _contact(ctx),
        'control_observation_start_s': .5, 'control_drift_after_settling_m': control_drift,
        'control_cold_start_max_horizontal_drift_m': initial_settling_drift,
        'stopping_distance_m': {n: ctx.rows[-1][_column(n, 'position', 'x')]-release[_column(n, 'position', 'x')] for n in ('asset', 'rubber')},
        'fixture': 'native D6 locks rubber rotation only; X/Y/Z translation free; no fixture force sampled',
        'nominal_average_combined_static_friction': {'paper': .425, 'rubber': .625},
        'nominal_average_combined_dynamic_friction': {'paper': .325, 'rubber': .50},
        'boundary': 'effective supported/guided response, not a pure material coefficient measurement; 0.2 mm paper skin can allow mixed contact',
    }


def free_tip(ctx: TestContext):
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
    assert max(r['loaded.angle'] for r in ctx.rows) > math.pi/4
    assert max(r['control.angle'] for r in ctx.rows) < math.radians(5)
    _stable(ctx, ('asset', 'idle'))
    ctx.experiment_details = {'contract': 'free end-standing loaded/control, 0.04 N high push then withdrawal',
                              'application_point_body_m': [-.025, 0, .005], 'release_time_s': 1.04,
                              'peak_tilt_rad': max(r['loaded.angle'] for r in ctx.rows),
                              'geometry_contact': _contact(ctx),
                              'fixture': None, 'asset_pose_writes_after_initialization': False}


def drop_orientations(ctx: TestContext):
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
    ctx.simulation(gravity=(0, 0, 0))
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
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'contract': 'equal world +X impulses, offset point follows body; untouched control',
                              'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
                              'gravity_m_s2': [0, 0, 0], 'ground': None,
                              'asset_pose_writes_after_initialization': False}


TESTS = {'contact_slide': contact_slide, 'free_tip': free_tip,
         'drop_orientations': drop_orientations, 'free_impulse': free_impulse}
