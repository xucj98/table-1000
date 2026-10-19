"""Rigid screwdriver: support, push direction, rolling and mass distribution.

SI units; wxyz rotations. Loads are applied external loads, not contact forces.
All bodies remain free after initialization; there are no hidden fixtures.
"""
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from table_1000.physics.testing import TestContext
from table_1000.physics.test import ground_clearance

BODY = 'screwdriver.body'
MASS_KG = .038


def _col(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


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


def _observe(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: float(np.linalg.norm(body.state['velocity'][:3])), unit='m/s')
    ctx.observe(instance.name+'.roll', lambda: math.atan2(body.state['rotation'][2, 1], body.state['rotation'][2, 2]), unit='rad')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.00005)


def _stable(ctx, names):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        positions = np.array([[r[_col(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert max(r[name+'.speed'] for r in tail) < .01, name+' speed'
        assert np.linalg.norm(np.ptp(positions, axis=0)) < .001, name+' settling'


def _contact(ctx):
    report = ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()),
                              ctx.rows, ctx.session.instances, 0)
    assert report['maximum_geometry_penetration_m'] < .002, report
    return report


def axial_lateral(ctx: TestContext):
    _ground(ctx)
    height = _height(ctx, (1, 0, 0, 0), 0)
    ctx.initial(position=(-.10, -.11, height))
    lateral = ctx.spawn(ctx.asset_path, name='lateral', position=(-.10, 0, height))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(.20, .30, height))
    for instance in (ctx.asset, lateral, idle):
        _observe(ctx, instance)
    for name, instance, direction in (('axial', ctx.asset, (1, 0, 0)), ('lateral', lateral, (0, 1, 0))):
        force = tuple(.24*v for v in direction)
        ctx.force(name, instance.body(BODY), keyframes=[{'time': 0, 'force': (0, 0, 0)},
            {'time': .4, 'force': (0, 0, 0)}, {'time': .8, 'force': force},
            {'time': .86, 'force': force}, {'time': .9, 'force': (0, 0, 0)}])
    ctx.camera(position=(.45, -.58, .6), target=(0, .20, .015), focal_length=24)
    ctx.video_annotation = lambda: 'Lower: axial COM push | middle: lateral COM push | upper: idle; release 0.90s'
    ctx.plots('axial.force.x', 'lateral.force.y', _col('asset', 'position', 'x'),
              _col('lateral', 'position', 'y'), 'lateral.roll', 'asset.speed', 'lateral.speed')
    ctx.run(3.2)
    _stable(ctx, ('asset', 'lateral', 'idle'))
    displacements = {}
    for name, axis in (('asset', 'x'), ('lateral', 'y')):
        key = _col(name, 'position', axis)
        displacements[name] = ctx.rows[-1][key]-ctx.rows[0][key]
        assert displacements[name] > .003, displacements
    idle_shift = np.linalg.norm(idle.body(BODY).state['position'][:2]-np.array([.20, .30]))
    assert idle_shift < .001
    ctx.experiment_details = {'displacements_m': displacements, 'idle_xy_shift_m': float(idle_shift),
        'geometry_contact': _contact(ctx), 'boundary': 'support response includes rolling, mixed contacts and rib faceting; not a friction calibration'}


def support_drop(ctx: TestContext):
    # Narrow end impacts exceeded the fixed 2 mm overlap limit at 4/2 ms.
    ctx.simulation(dt=.001)
    _ground(ctx)
    # Slight 3 degree lean removes the exact, unstable vertical equilibrium.
    q_grip = tuple(Rotation.from_euler('y', -87, degrees=True).as_quat()[[3, 0, 1, 2]])
    q_shaft = tuple(Rotation.from_euler('y', 87, degrees=True).as_quat()[[3, 0, 1, 2]])
    ctx.initial(position=(-.30, 0, _height(ctx, q_grip, .03)), rotation=q_grip)
    shaft = ctx.spawn(ctx.asset_path, name='shaft', position=(0, 0, _height(ctx, q_shaft, .03)), rotation=q_shaft)
    horizontal = ctx.spawn(ctx.asset_path, name='horizontal', position=(.30, 0, _height(ctx, (1, 0, 0, 0), .03)))
    for instance in (ctx.asset, shaft, horizontal):
        _observe(ctx, instance)
    ctx.camera(position=(.5, -.75, .5), target=(0, 0, .045), focal_length=24)
    ctx.video_annotation = lambda: '30mm lowest-point drop | grip-end / shaft-end (3deg lean) / horizontal | rigid placement'
    ctx.plots(*[_col(n, 'position', 'z') for n in ('asset', 'shaft', 'horizontal')],
              *[n+'.speed' for n in ('asset', 'shaft', 'horizontal')])
    ctx.run(3.2)
    _stable(ctx, ('asset', 'shaft', 'horizontal'))
    ctx.experiment_details = {'clearance_m': .03, 'vertical_lean_degrees': 3,
        'geometry_contact': _contact(ctx), 'boundary': 'no damage, shaft bending, screws or precise impact prediction'}


def eccentric_impulse(ctx: TestContext):
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.30, 0, .06))
    grip = ctx.spawn(ctx.asset_path, name='grip', position=(-.10, 0, .06))
    shaft = ctx.spawn(ctx.asset_path, name='shaft', position=(.10, 0, .06))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(.30, 0, .06))
    instances = (ctx.asset, grip, shaft, idle)
    for instance in instances:
        _observe(ctx, instance)
    for name, instance, point in (('com', ctx.asset, None), ('grip', grip, (-.05, 0, .011)),
                                   ('shaft', shaft, (.06, 0, .011))):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .2, 'force': (0, 0, 0)},
            {'time': .204, 'force': (0, .006, 0)}, {'time': .404, 'force': (0, .006, 0)},
            {'time': .408, 'force': (0, 0, 0)}]
        if point is not None:
            frames[0]['point'] = point
        ctx.force(name, instance.body(BODY), point_frame='body', keyframes=frames)
    ctx.camera(position=(.48, -.75, .65), target=(0, .02, .06), focal_length=24)
    ctx.video_annotation = lambda: 'Zero gravity | equal impulses at COM / grip / shaft; right idle control; no world anchor'
    ctx.plots('com.force.y', *[_col(n, 'linear_velocity', 'y') for n in ('asset', 'grip', 'shaft')],
              *[_col(n, 'angular_velocity', 'z') for n in ('asset', 'grip', 'shaft')])
    ctx.run(1.2)
    impulse = sum(r['com.force.y'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*a.body(BODY).state['velocity'][1]-impulse)/impulse for a in instances[:3]]
    assert max(errors) < .01, errors
    assert grip.body(BODY).state['velocity'][5] < -.1
    assert shaft.body(BODY).state['velocity'][5] > .1
    assert np.linalg.norm(ctx.asset.body(BODY).state['velocity'][3:]) < .01
    assert np.linalg.norm(idle.body(BODY).state['velocity']) < 1e-6
    ctx.experiment_details = {'external_impulse_N_s': impulse,
        'relative_linear_momentum_errors': list(map(float, errors)),
        'engine_com_m': ctx.asset.local_com.tolist(), 'engine_inertia_kg_m2': ctx.asset.local_inertia.tolist(),
        'mass_model': '38g composite cylinders: plastic grip/neck, visible steel, embedded tang',
        'reaction_forces': 'not sampled; forces are external loads'}


def roll_release(ctx: TestContext):
    _ground(ctx)
    height = _height(ctx, (1, 0, 0, 0), 0)
    ctx.initial(position=(-.10, -.10, height), angular_velocity=(8, 0, 0))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.10, .10, height))
    for instance in (ctx.asset, idle):
        _observe(ctx, instance)
    ctx.camera(position=(.28, -.43, .30), target=(-.05, -.03, .015), focal_length=24)
    ctx.video_annotation = lambda: 'Lower: initial +X spin 8 rad/s; then free gravity/contact | upper: untouched'
    ctx.plots('asset.roll', _col('asset', 'angular_velocity', 'x'), 'asset.speed', 'idle.speed')
    ctx.run(3.2)
    _stable(ctx, ('asset', 'idle'))
    assert max(abs(r['asset.roll']) for r in ctx.rows if r['time'] <= .5) > .1
    ctx.experiment_details = {'initial_spin_rad_s': 8, 'geometry_contact': _contact(ctx),
        'boundary': 'faceted grip/ribs dissipate spin via contact; no calibrated rolling resistance or damping'}


TESTS = {'axial_lateral': axial_lateral, 'support_drop': support_drop,
         'eccentric_impulse': eccentric_impulse, 'roll_release': roll_release}
