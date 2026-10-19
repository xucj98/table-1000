"""Fixed-connector USB drive: support, release, drop and free-body loads.

SI units; wxyz quaternions. Forces are imposed loads, not contact reactions.
The ideal ground and uncalibrated materials do not represent a USB socket.
"""
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.physics.testing import TestContext
from table_1000.physics.test import ground_clearance

BODY = 'usb_drive.body'
MASS_KG = .012
UPRIGHT = (math.sqrt(.5), 0, math.sqrt(.5), 0)


def _column(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def _body(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.0001)


def _stable(ctx, names, speed=.01):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        positions = np.array([[r[_column(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert max(r[name+'.speed'] for r in tail) < speed, f'{name}: unsettled velocity'
        assert np.linalg.norm(np.ptp(positions, axis=0)) < .001, f'{name}: unsettled position'


def _vertices(ctx):
    """Collision vertices in body coordinates, grouped by semantic material."""
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    worlds, groups = {}, {}
    for node in model['nodes']:
        worlds[node['name']] = worlds.get(node['parent'], np.eye(4)) @ np.array(node['matrix'])
        if node.get('role') == 'collision':
            local = np.linalg.inv(worlds[BODY]) @ worlds[node['name']]
            groups[node['parent'].rsplit('.', 1)[-1]] = (np.c_[node['vertices'], np.ones(len(node['vertices']))] @ local.T)[:, :3]
    return groups


def _contact(ctx):
    result = ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()),
                              ctx.rows, ctx.session.instances, 0)
    assert result['maximum_geometry_penetration_m'] < .002
    return result


from contextlib import contextmanager


@contextmanager
def _planar_guide(ctx, bodies):
    """Ideal YZ-plane fixture: suppress weak-axis rocking, allow Y/Z and X-roll.

    Native world D6 joints carry out-of-plane reactions only. They do not hold
    the drive upright, drive it, or prevent in-plane tipping/sliding.
    """
    from pxr import Gf, UsdPhysics
    ctx.ensure_initialized()
    paths = []
    for body in bodies:
        state = body.state
        path = '/World/Fixtures/planar_'+body.instance.name
        paths.append(path)
        joint = UsdPhysics.Joint.Define(ctx.session.stage, path)
        joint.CreateBody1Rel().SetTargets([body.path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*state['position']))
        joint.CreateLocalRot0Attr(Gf.Quatf(1))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        q = state['quaternion']
        joint.CreateLocalRot1Attr(Gf.Quatf(float(q[0]), Gf.Vec3f(*(-q[1:]))))
        for axis in ('transX', 'transY', 'transZ', 'rotX', 'rotY', 'rotZ'):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            locked = axis in ('transX', 'rotY', 'rotZ')
            limit.CreateLowAttr(0 if locked else -float('inf'))
            limit.CreateHighAttr(0 if locked else float('inf'))
    try:
        yield
    finally:
        for path in paths:
            ctx.session.stage.RemovePrim(path)


def support_response(ctx: TestContext):
    """Equal COM loads: broad housing slides, connector support tips."""
    _ground(ctx)
    ctx.initial(position=(-.10, -.04, .0002))
    tip = ctx.spawn(ctx.asset_path, name='connector', position=(0, -.04, .0302), rotation=UPRIGHT)
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(.10, -.04, .0302), rotation=UPRIGHT)
    bodies = [_body(ctx, a) for a in (ctx.asset, tip, idle)]
    for label, body in zip(('connector', 'idle'), bodies[1:]):
        ctx.observe(label+'.tilt', lambda b=body: math.acos(float(np.clip(-b.state['rotation'][2, 0], -1, 1))), unit='rad')
    ctx.camera(position=(.29, -.42, .32), target=(0, .02, .025))
    ctx.video_annotation = lambda: 'Left: housing | centre/right: planar-guided connectors; centre loaded, right idle'
    for name, body in zip(('housing_push', 'connector_push'), bodies):
        ctx.force(name, body, keyframes=[
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .4, 'force': (0, 0, 0)},
            {'time': .9, 'force': (0, .065, 0)},
            {'time': 1., 'force': (0, .065, 0)},
            {'time': 1.04, 'force': (0, 0, 0)},
        ])
    # Sample actual collision support just before loading; distinguish materials
    # by geometry, without claiming measured reaction force or friction.
    vertices = _vertices(ctx)
    for part, points in vertices.items():
        ctx.observe('connector.'+part+'_clearance',
                    lambda p=points: (p @ bodies[1].state['rotation'].T)[:, 2].min()+bodies[1].state['position'][2], unit='m')
    ctx.plots('housing_push.force.y', _column('asset', 'position', 'y'),
              'connector.tilt', 'idle.tilt', 'connector.connector_clearance', 'asset.speed', 'connector.speed')
    with _planar_guide(ctx, bodies[1:]):
        ctx.run(3.2)
    shell_shift = ctx.rows[-1][_column('asset', 'position', 'y')]-ctx.rows[0][_column('asset', 'position', 'y')]
    assert shell_shift > .005
    assert bodies[0].state['rotation'][2, 2] > .98
    assert max(r['connector.tilt'] for r in ctx.rows) > math.pi/4
    assert max(r['idle.tilt'] for r in ctx.rows) < math.radians(5)
    before = min(ctx.rows, key=lambda r: abs(r['time']-.3))
    support = {part: before['connector.'+part+'_clearance'] for part in vertices}
    assert abs(support['connector']) < .0005 and support['housing'] > .01
    _stable(ctx, ('asset', 'connector', 'idle'), speed=.005)
    release = min(ctx.rows, key=lambda r: abs(r['time']-1.04))
    ctx.experiment_details = {
        'contract': 'equal COM ramps: housing sliding / planar-guided connector tipping / planar-guided idle',
        'fixture': 'native D6 world guide locks world X translation and Y/Z rotations; Y/Z translation and X rotation free',
        'release_time_s': 1.04, 'housing_slide_m': shell_shift,
        'housing_stop_distance_m': ctx.rows[-1][_column('asset', 'position', 'y')]-release[_column('asset', 'position', 'y')],
        'peak_connector_tilt_rad': max(r['connector.tilt'] for r in ctx.rows),
        'preload_connector_support_clearance_m': support, 'geometry_contact': _contact(ctx),
        'boundary': 'planar fixture suppresses weak-axis rocking; not free connector-end standing or isolated material coefficient measurement',
    }


def drop_orientations(ctx: TestContext):
    """30 mm releases at 2 ms: 4 ms exceeded the 2 mm overlap contract."""
    ctx.simulation(dt=.002)
    _ground(ctx)
    q = Rotation.from_euler('y', 25, degrees=True).as_quat()[[3, 0, 1, 2]]
    rotations = [(1, 0, 0, 0), q, UPRIGHT]
    vertices = np.concatenate(list(_vertices(ctx).values()))
    offsets = [.03-(vertices @ Rotation.from_quat(np.array(r)[[1, 2, 3, 0]]).as_matrix().T)[:, 2].min() for r in rotations]
    ctx.initial(position=(-.10, 0, offsets[0]), rotation=rotations[0])
    tilted = ctx.spawn(ctx.asset_path, name='tilted', position=(0, 0, offsets[1]), rotation=rotations[1])
    tip = ctx.spawn(ctx.asset_path, name='connector', position=(.10, 0, offsets[2]), rotation=rotations[2])
    for instance in (ctx.asset, tilted, tip):
        _body(ctx, instance)
    ctx.camera(position=(.30, -.40, .27), target=(0, 0, .025))
    ctx.video_annotation = lambda: '30 mm lowest-point drop | left: housing | centre: 25 deg eccentric | right: connector end'
    ctx.plots(*[_column(n, 'position', 'z') for n in ('asset', 'tilted', 'connector')],
              *[n+'.speed' for n in ('asset', 'tilted', 'connector')])
    ctx.run(3.2)
    _stable(ctx, ('asset', 'tilted', 'connector'))
    ctx.experiment_details = {'contract': 'housing / 25 degree eccentric / connector-end gravity releases',
                              'release_clearance_m': .03, 'initial_origin_z_m': list(map(float, offsets)),
                              'geometry_contact': _contact(ctx),
                              'boundary': 'rigid low-height placement; not impact damage or connector deformation'}


def free_eccentric(ctx: TestContext):
    """Equal COM and eccentric impulses; the idle instance stays still."""
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(-.06, -.075, .06))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(-.06, 0, .06))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(-.06, .075, .06))
    bodies = [_body(ctx, a) for a in (ctx.asset, offset, idle)]
    for name, body in zip(('com_push', 'offset_push'), bodies):
        frames = [
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .4, 'force': (0, 0, 0)},
            {'time': .404, 'force': (.012, 0, 0)},
            {'time': .604, 'force': (.012, 0, 0)},
            {'time': .608, 'force': (0, 0, 0)},
        ]
        if name == 'offset_push':
            frames[0]['point'] = (0, .007, .005)
        ctx.force(name, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.29, -.40, .35), target=(.015, 0, .06))
    ctx.video_annotation = lambda: 'Zero gravity | lower: COM | middle: eccentric | upper: untouched | same +X impulse'
    ctx.plots('com_push.force.x', _column('asset', 'linear_velocity', 'x'),
              _column('offset', 'linear_velocity', 'x'), _column('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'contract': 'equal +X impulses; eccentric point follows body; idle control',
                              'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': list(map(float, errors)),
                              'asset_pose_writes_after_initialization': False, 'ground': None}


TESTS = {'support_response': support_response, 'drop_orientations': drop_orientations,
         'free_eccentric': free_eccentric}
