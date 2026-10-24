"""Native rigid-body experiments; SI units, wxyz rotations, no pose animation.

Materials and mass are engineering estimates. Recorded forces are applied test
loads, not measured contact reactions. Ground is an ideal uniform test surface.
"""
import math
import numpy as np

from table_1000.physics.testing import TestContext

BODY = 'board_eraser.body'
MASS_KG = .050


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(f'{instance.name}.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.0001)


def _column(instance, field, axis):
    return f'{instance}.{BODY}.{field}.{axis}'


def _stable(ctx, names, speed=.01):
    for name in names:
        tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
        positions = np.array([[r[_column(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert max(r[f'{name}.speed'] for r in tail) < speed, f'{name}: unsettled velocity'
        assert np.linalg.norm(np.ptp(positions, axis=0)) < .001, f'{name}: unsettled position'


def face_friction(ctx: TestContext):
    """Compare contact faces under equal forces, plus an untouched control."""
    _ground(ctx)
    ctx.initial(position=(0, -.10, .0002))
    shell = ctx.spawn(ctx.asset_path, name='shell', position=(0, .0, .0252), rotation=(0, 1, 0, 0))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .10, .0002))
    bodies = [_body(ctx, a) for a in (ctx.asset, shell, idle)]
    ctx.camera(position=(.85, -1.0, .85), target=(.25, 0, .015))
    ctx.video_annotation = lambda: 'Lower: FELT | middle: SHELL | upper: idle | same COM force; release 1.84 s'
    for label, body in zip(('felt_push', 'shell_push'), bodies):
        ctx.force(label, body, keyframes=[
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .5, 'force': (.10, 0, 0)},
            {'time': 1., 'force': (.10, 0, 0)},
            {'time': 1.8, 'force': (.30, 0, 0)},
            {'time': 1.84, 'force': (0, 0, 0)},
        ])
    ctx.plots('felt_push.force.x', _column('asset', 'position', 'x'),
              _column('shell', 'position', 'x'), _column('asset', 'linear_velocity', 'x'),
              _column('shell', 'linear_velocity', 'x'))
    ctx.run(3.6)
    onset = {}
    for name, force in [('asset', 'felt_push'), ('shell', 'shell_push')]:
        column = _column(name, 'position', 'x')
        start = ctx.rows[0][column]
        row = next(r for r in ctx.rows if r[column]-start >= .0005)
        onset[name] = {'time_s': row['time'], 'applied_force_N': row[force+'.force.x']}
        assert ctx.rows[-1][column]-start > .005
        assert bodies[0 if name == 'asset' else 1].state['rotation'][2, 2] * (1 if name == 'asset' else -1) > .98
    assert onset['asset']['applied_force_N']-onset['shell']['applied_force_N'] > .04
    idle_positions = np.array([[r[_column('idle', 'position', a)] for a in 'xy'] for r in ctx.rows])
    assert np.max(np.linalg.norm(idle_positions-idle_positions[0], axis=1)) < .0005
    _stable(ctx, ('asset', 'shell', 'idle'), speed=.005)
    release = min(ctx.rows, key=lambda r: abs(r['time']-1.84))
    ctx.experiment_details = {
        'contract': 'equal COM ramp/withdrawal on felt and shell, untouched third instance',
        'onset_definition_m': .0005, 'onset': onset,
        'release_time_s': 1.84,
        'stopping_distance_m': {n: ctx.rows[-1][_column(n, 'position', 'x')]-release[_column(n, 'position', 'x')] for n in ('asset', 'shell')},
        'nominal_average_combined_static_friction': {'felt': .55, 'shell': .375},
        'nominal_average_combined_dynamic_friction': {'felt': .425, 'shell': .275},
        'boundary': 'effective face/ideal-ground comparison, not calibrated felt friction or wiping',
    }


def eccentric_tip(ctx: TestContext):
    """High lateral load tips a side-standing body; identical control is idle."""
    _ground(ctx)
    q = (math.sqrt(.5), math.sqrt(.5), 0, 0)
    ctx.initial(position=(.09, 0, .0277), rotation=q)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.09, 0, .0277), rotation=q)
    body, control = [_body(ctx, a) for a in (ctx.asset, idle)]
    for name, b in [('loaded', body), ('control', control)]:
        ctx.observe(name+'.angle', lambda b=b: math.acos(float(np.clip(-b.state['rotation'][1, 2], -1, 1))), unit='rad')
    ctx.camera(position=(.55, -.75, .65), target=(0, .18, .02))
    ctx.video_annotation = lambda: 'Right: high lateral force on side-standing eraser | left: unloaded control'
    ctx.force('push', body, point_frame='body', keyframes=[
        {'time': 0, 'force': (0, 0, 0), 'point': (0, .022, .0145)},
        {'time': .4, 'force': (0, 0, 0)},
        {'time': .8, 'force': (0, .30, 0)},
        {'time': 1.1, 'force': (0, .30, 0)},
        {'time': 1.12, 'force': (0, 0, 0)},
    ])
    ctx.plots('push.force.y', 'loaded.angle', 'control.angle', 'asset.speed')
    ctx.run(4)
    assert max(r['loaded.angle'] for r in ctx.rows) > math.pi/4
    assert max(r['control.angle'] for r in ctx.rows) < math.radians(5)
    _stable(ctx, ('asset', 'idle'))
    ctx.experiment_details = {'contract': 'side-standing loaded/control, 0.30 N high push then withdrawal',
                              'application_point_body_m': [0, .022, .0145], 'release_time_s': 1.12,
                              'peak_tilt_rad': max(r['loaded.angle'] for r in ctx.rows)}


def drop_faces(ctx: TestContext):
    """Felt, shell and side releases, each 50 mm above its contact face."""
    _ground(ctx)
    ctx.initial(position=(-.16, 0, .05))
    shell = ctx.spawn(ctx.asset_path, name='shell', position=(0, 0, .075), rotation=(0, 1, 0, 0))
    side = ctx.spawn(ctx.asset_path, name='side', position=(.16, 0, .0775),
                     rotation=(math.sqrt(.5), math.sqrt(.5), 0, 0))
    for a in (ctx.asset, shell, side):
        _body(ctx, a)
    ctx.camera(position=(.50, -.65, .40), target=(0, 0, .035))
    ctx.video_annotation = lambda: 'Left: felt | centre: shell | right: side | 50 mm lowest-point release'
    ctx.plots(*[_column(n, 'position', 'z') for n in ('asset', 'shell', 'side')],
              *[n+'.speed' for n in ('asset', 'shell', 'side')])
    ctx.run(4)
    _stable(ctx, ('asset', 'shell', 'side'))
    from table_1000.physics.test import ground_clearance
    import json
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    clearance = ground_clearance(model, ctx.rows, ctx.session.instances, 0)
    assert clearance['maximum_geometry_penetration_m'] < .002
    ctx.experiment_details = {'contract': '50 mm lowest-collision-point gravity drop in three orientations',
                              'release_clearance_m': .05, 'geometry_contact': clearance,
                              'boundary': 'rigid zero-restitution proxy, not calibrated impact or pad compression'}


def free_eccentric(ctx: TestContext):
    """COM and offset impulses distinguish translation from torque in free space."""
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.16, -.12, .10))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(-.16, 0, .10))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.16, .12, .10))
    bodies = [_body(ctx, a) for a in (ctx.asset, offset, idle)]
    for name, body in zip(('com_push', 'offset_push'), bodies):
        frames = [
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .4, 'force': (0, 0, 0)},
            {'time': .404, 'force': (.08, 0, 0)},
            {'time': .604, 'force': (.08, 0, 0)},
            {'time': .608, 'force': (0, 0, 0)},
        ]
        if name == 'offset_push':
            frames[0]['point'] = (0, .020, .0125)
        ctx.force(name, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.52, -.65, .55), target=(-.05, 0, .10))
    ctx.video_annotation = lambda: 'Zero gravity | lower: COM impulse | middle: offset impulse | upper: untouched'
    ctx.plots('com_push.force.x', _column('asset', 'linear_velocity', 'x'),
              _column('offset', 'linear_velocity', 'x'), _column('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'contract': 'equal world +X impulses, offset point follows the body; idle control',
                              'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
                              'gravity_m_s2': [0, 0, 0], 'ground': None,
                              'asset_pose_writes_after_initialization': False}


TESTS = {'face_friction': face_friction, 'eccentric_tip': eccentric_tip,
         'drop_faces': drop_faces, 'free_eccentric': free_eccentric}
