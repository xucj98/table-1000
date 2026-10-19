"""Rigid contact and approved signed magnetic model experiments.

SI units, wxyz quaternions. Ground is an unmarked ideal test surface. Forces
are applied external loads, not contact reactions. No pose writes after start.
"""
import json
import math
from contextlib import ExitStack
import numpy as np

from table_1000.physics.testing import TestContext

BODY = 'magnet.body'
MASS_KG = .008
HEIGHT_M = .00745


def _column(instance, field, axis):
    return f'{instance}.{BODY}.{field}.{axis}'


def _body(ctx, instance, name=BODY):
    body = instance.body(name)
    ctx.observe_body(body)
    ctx.observe(f'{instance.name}.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    return body


def _ground(ctx):
    ctx.ground(static_friction=.40, dynamic_friction=.30, restitution=0, contact_offset=.0001)


def _settled(ctx, names, speed):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.4-1e-8]
    for name in names:
        positions = np.array([[r[_column(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert max(r[f'{name}.speed'] for r in tail) < speed, f'{name}: final velocity'
        assert np.linalg.norm(np.ptp(positions, axis=0)) < .0005, f'{name}: final drift'


def _plane_penetration(ctx, names, *, normal=(0, 0, 1), origin=(0, 0, 0)):
    """Every recorded hull vertex versus the steel surface plane.

    Using the whole plane is conservative for the finite fixture: points
    outside its rectangle receive no exemption in this low-speed check.
    """
    from scipy.spatial.transform import Rotation
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    transforms, vertices = {}, []
    for node in model['nodes']:
        transforms[node['name']] = transforms.get(node['parent'], np.eye(4))@np.array(node['matrix'])
        if node.get('role') == 'collision':
            local = np.linalg.inv(transforms[BODY])@transforms[node['name']]
            vertices.extend((np.c_[node['vertices'], np.ones(len(node['vertices']))]@local.T)[:, :3])
    worst = {'maximum_geometry_penetration_m': 0., 'time_s': 0., 'instance': None,
             'sampling_interval_s': ctx.dt,
             'method': 'all collision hull vertices at every integration sample versus conservative steel surface plane'}
    for name in names:
        prefix = name+'.'+BODY+'.'
        positions = np.array([[r[prefix+'position.'+a] for a in 'xyz'] for r in ctx.rows])
        quaternions = np.array([[r[prefix+'quaternion.'+a] for a in 'xyzw'] for r in ctx.rows])
        rotations = Rotation.from_quat(quaternions).as_matrix()
        points = np.einsum('nij,vj->nvi', rotations, np.array(vertices))+positions[:, None, :]
        gaps = (points-np.array(origin))@np.array(normal)
        index = np.unravel_index(np.argmin(gaps), gaps.shape)
        depth = max(0., -float(gaps[index]))
        if depth > worst['maximum_geometry_penetration_m']:
            worst.update(maximum_geometry_penetration_m=depth, time_s=ctx.rows[index[0]]['time'], instance=name)
    return worst


def contact_faces(ctx: TestContext):
    """Base and inverted cap receive identical ramps; third instance is idle."""
    _ground(ctx)
    ctx.initial(position=(0, -.050, .0002))
    cap = ctx.spawn(ctx.asset_path, name='cap', position=(0, 0, HEIGHT_M+.0002), rotation=(0, 1, 0, 0))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .050, .0002))
    bodies = [_body(ctx, a) for a in (ctx.asset, cap, idle)]
    for label, body in zip(('base_push', 'cap_push'), bodies):
        ctx.force(label, body, keyframes=[
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .4, 'force': (.018, 0, 0)},
            {'time': .8, 'force': (.018, 0, 0)},
            {'time': 1.04, 'force': (.045, 0, 0)},
            {'time': 1.12, 'force': (.045, 0, 0)},
            {'time': 1.16, 'force': (0, 0, 0)},
        ])
    ctx.camera(position=(.25, -.32, .28), target=(.065, 0, .005))
    ctx.video_annotation = lambda: 'Lower: BASE | middle: inverted CAP | upper: idle | unmarked ground'
    ctx.plots('base_push.force.x', _column('asset', 'position', 'x'),
              _column('cap', 'position', 'x'), _column('asset', 'linear_velocity', 'x'),
              _column('cap', 'linear_velocity', 'x'))
    ctx.run(2.8)
    onset = {}
    for name, force, body, sign in [('asset', 'base_push', bodies[0], 1), ('cap', 'cap_push', bodies[1], -1)]:
        col = _column(name, 'position', 'x')
        start = ctx.rows[0][col]
        row = next(r for r in ctx.rows if r[col]-start >= .0002)
        onset[name] = {'time_s': row['time'], 'applied_force_N': row[force+'.force.x']}
        assert ctx.rows[-1][col]-start > .002
        assert sign*body.state['rotation'][2, 2] > .98, f'{name}: unintended tipping'
    idle_xy = np.array([[r[_column('idle', 'position', a)] for a in 'xy'] for r in ctx.rows])
    assert np.max(np.linalg.norm(idle_xy-idle_xy[0], axis=1)) < .0002
    _settled(ctx, ('asset', 'cap', 'idle'), speed=.005)
    release = min(ctx.rows, key=lambda r: abs(r['time']-1.16))
    ctx.experiment_details = {'contract': 'base/cap equal COM force ramp, withdrawal, untouched control',
        'magnetic_target': False, 'onset_displacement_m': .0002, 'onset': onset, 'release_time_s': 1.16,
        'stopping_distances_m': {n: ctx.rows[-1][_column(n, 'position', 'x')]-release[_column(n, 'position', 'x')] for n in ('asset', 'cap')},
        'boundary': 'ordinary estimated contact on unmarked ground, no magnetic validation'}


def free_impulses(ctx: TestContext):
    """Equal impulses at COM/offset; untouched third instance, no gravity."""
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, -.050, .03))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(0, 0, .03))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .050, .03))
    bodies = [_body(ctx, a) for a in (ctx.asset, offset, idle)]
    for label, body in zip(('com_push', 'offset_push'), bodies):
        frames = [
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .2, 'force': (0, 0, 0)},
            {'time': .204, 'force': (.0008, 0, 0)},
            {'time': .404, 'force': (.0008, 0, 0)},
            {'time': .408, 'force': (0, 0, 0)},
        ]
        if label == 'offset_push':
            frames[0]['point'] = (0, .006, .003)
        ctx.force(label, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.15, -.24, .20), target=(.008, 0, .03))
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
    ctx.experiment_details = {'contract': 'equal free impulses, COM/offset, idle control',
        'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
        'offset_point_body_m': [0, .006, .003], 'gravity_m_s2': [0, 0, 0], 'ground': None,
        'asset_pose_writes_after_initialization': False,
        'boundary': 'no magnetic target; magnetic interactions require separate experiments'}


def drop_faces(ctx: TestContext):
    """20 mm release at 2 ms: the 4 ms diagnostic exceeds 1 mm penetration."""
    ctx.simulation(dt=.002)
    _ground(ctx)
    ctx.initial(position=(-.05, 0, .020))
    cap = ctx.spawn(ctx.asset_path, name='cap', position=(0, 0, .020+HEIGHT_M), rotation=(0, 1, 0, 0))
    side = ctx.spawn(ctx.asset_path, name='side', position=(.05, 0, .020+.0125),
                     rotation=(math.sqrt(.5), math.sqrt(.5), 0, 0))
    for a in (ctx.asset, cap, side):
        _body(ctx, a)
    ctx.camera(position=(.15, -.23, .16), target=(0, 0, .018))
    ctx.video_annotation = lambda: 'Left: BASE | centre: CAP | right: rim | 20 mm lowest-point release'
    ctx.plots(*[_column(n, 'position', 'z') for n in ('asset', 'cap', 'side')],
              *[n+'.speed' for n in ('asset', 'cap', 'side')])
    ctx.run(3.2)
    _settled(ctx, ('asset', 'cap', 'side'), speed=.01)
    from table_1000.physics.test import ground_clearance
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    clearance = ground_clearance(model, ctx.rows, ctx.session.instances, 0)
    assert clearance['maximum_geometry_penetration_m'] < .001
    ctx.experiment_details = {'contract': 'three-orientation 20 mm low release on unmarked ground',
        'release_clearance_m': .020, 'geometry_contact': clearance,
        'boundary': 'rigid estimated contact; not calibrated impact or magnetic holding'}


TESTS = {'contact_faces': contact_faces, 'free_impulses': free_impulses, 'drop_faces': drop_faces}


def _magnetic(ctx, instance, *, steel=False):
    # Resolve friction anchors at the contact margin scale. The default
    # 25 mm merge distance exceeds these small magnetic contact patches.
    # This scene setting is explicit and never imposed by asset behavior.
    from pxr import PhysxSchema, UsdPhysics
    scene = next(p for p in ctx.session.stage.Traverse() if p.IsA(UsdPhysics.Scene))
    PhysxSchema.PhysxSceneAPI.Apply(scene).CreateFrictionCorrelationDistanceAttr(.0001)
    body = _body(ctx, instance)
    for i, axis in enumerate('xyz'):
        ctx.observe(f'{instance.name}.magnetic_force.{axis}',
                    lambda i=i: instance.behavior().last_force[i], unit='N')
        ctx.observe(f'{instance.name}.magnetic_torque.{axis}',
                    lambda i=i: instance.behavior().last_torque[i], unit='N·m')
    ctx.observe(f'{instance.name}.magnetic_pairs', lambda: len(instance.behavior().last_pairs), unit='1')
    if steel:
        # No marked face => present=0 and zero diagnostic fields. Unmarked
        # controls still record native pose; no face is inferred from visuals.
        ctx.observe(instance.name+'.steel.present', lambda: instance.behavior().last_steel_diagnostic is not None, unit='1')
        ctx.observe(instance.name+'.steel.angular_factor',
            lambda: (instance.behavior().last_steel_diagnostic or {}).get('angular_factor', 0.), unit='1')
        for field, key, unit in [('gap', 'gaps_m', 'm'), ('force', 'forces_N', 'N'),
                                 ('edge', 'edge_factors', '1'), ('cutoff', 'cutoff_factors', '1')]:
            for i in range(9):
                ctx.observe(f'{instance.name}.steel.{field}.{i}',
                    lambda i=i, key=key: (instance.behavior().last_steel_diagnostic or {}).get(key, [0.]*9)[i], unit=unit)
    return body


def _steel(ctx, name, *, position=(0, 0, 0), rotation=(1, 0, 0, 0),
           half=(.075, .075), marked=True, dynamic=False):
    """Finite ideal fixture, local XY surface at z=0; no magnetic force here."""
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade
    stage = ctx.session.stage
    path = '/World/Fixtures/'+name
    root = UsdGeom.Xform.Define(stage, path)
    root.AddTranslateOp().Set(Gf.Vec3d(*position))
    root.AddOrientOp().Set(Gf.Quatf(rotation[0], Gf.Vec3f(*rotation[1:])))
    if marked:
        root.GetPrim().CreateAttribute('table1000:magnet:steelHalfExtents', Sdf.ValueTypeNames.Double2).Set(Gf.Vec2d(*half))
    cube = UsdGeom.Cube.Define(stage, path+'/surface')
    cube.CreateSizeAttr(1)
    cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, -.002))
    cube.AddScaleOp().Set(Gf.Vec3f(2*half[0], 2*half[1], .004))
    cube.CreateDisplayColorAttr([Gf.Vec3f(.20, .24, .28)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    collision = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    collision.CreateContactOffsetAttr(.0001); collision.CreateRestOffsetAttr(0)
    material = UsdShade.Material.Define(stage, path+'/material')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.40); api.CreateDynamicFrictionAttr(.30); api.CreateRestitutionAttr(0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr('average')
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, materialPurpose='physics')
    if dynamic:
        prim = root.GetPrim()
        prim.SetCustomDataByKey('table1000:name', name)
        prim.SetCustomDataByKey('table1000:role', 'rigid')
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.050)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        api.CreateLinearDampingAttr(0); api.CreateAngularDampingAttr(0)
        api.CreateSolverPositionIterationCountAttr(16); api.CreateSolverVelocityIterationCountAttr(4)
        api.CreateEnableCCDAttr(True)
        return ctx.session.register(path, name=name)
    return root.GetPrim()


def _center(body):
    return body.state['position']+body.state['rotation']@np.array([0., 0., .002])


def _total_momentum(instances):
    linear = np.zeros(3); angular = np.zeros(3)
    for instance in instances:
        for name in instance.body_names:
            body = instance.body(name); state = body.state
            i = instance.indices[name]
            momentum = instance.mass[i]*state['velocity'][:3]
            inertia = state['rotation']@instance.local_inertia[i]@state['rotation'].T
            linear += momentum
            angular += np.cross(state['com'], momentum)+inertia@state['velocity'][3:]
    return linear, angular


def steel_distance(ctx: TestContext):
    """Native fixed measurement fixtures only; gaps and controls differ."""
    ctx.simulation(gravity=(0, 0, 0))
    gaps = [.0002, .001, .002, .004, .006, .008, .010]
    instances = []
    for i, gap in enumerate(gaps + [.0002, .0002]):
        x, y = (i % 5-2)*.060, (i//5)*.060
        back, marked = i == 7, i != 8
        position = (x, y, gap+(HEIGHT_M if back else 0))
        q = (0, 1, 0, 0) if back else (1, 0, 0, 0)
        if i == 0:
            ctx.initial(position=position, rotation=q); instance = ctx.asset
        else:
            instance = ctx.spawn(ctx.asset_path, name=f'g{i}', position=position, rotation=q)
        _steel(ctx, f'plate{i}', position=(x, y, 0), half=(.023, .023), marked=marked)
        _magnetic(ctx, instance, steel=True)
        instances.append(instance)
    ctx.camera(position=(.32, -.48, .32), target=(0, .030, .008))
    ctx.video_annotation = lambda: 'Fixed measurement fixtures | gaps .2/1/2/4/6/8/10 mm | back / unmarked controls'
    ctx.plots('asset.magnetic_force.z', 'g2.magnetic_force.z', 'g4.magnetic_force.z',
              'g5.magnetic_force.z', 'g7.magnetic_force.z', 'g8.magnetic_force.z')
    with ExitStack() as stack:
        for a in instances:
            stack.enter_context(ctx.fixed(a.body(BODY)))
        ctx.run(.8)
        measurements = []
        for i, a in enumerate(instances):
            actual = -float(a.behavior().last_force[2])
            gap = gaps[i] if i < 7 else .0002
            expected = 1/(1+gap/.002)**2 if i < 5 else 0.
            assert abs(actual-expected) < max(1e-6, .01*expected), f'gap/control {i}: {actual} vs {expected}'
            assert a.behavior().callback_count == ctx.session.steps
            measurements.append({'instance': a.name, 'gap_m': gap, 'force_N': actual,
                                 'expected_force_N': expected, 'back': i == 7, 'marked': i != 8})
    ctx.experiment_details = {'contract': 'distance law, back-face and unmarked controls; native fixed measurement fixtures',
        'measurements': measurements, 'boundary': 'magnetic force output, not fixture/contact reaction measurement'}


def steel_capture(ctx: TestContext):
    """Three free magnets: marked face, marked back, unmarked face."""
    ctx.simulation(dt=.002)
    instances = []
    for i, (x, back, marked) in enumerate([(-.06, False, True), (0, True, True), (.06, False, False)]):
        q = (0, 1, 0, 0) if back else (1, 0, 0, 0)
        pos = (x, 0, .002+(HEIGHT_M if back else 0))
        if i == 0:
            ctx.initial(position=pos); a = ctx.asset
        else:
            a = ctx.spawn(ctx.asset_path, name=f'control{i}', position=pos, rotation=q)
        _steel(ctx, f'plate{i}', position=(x, 0, 0), half=(.025, .025), marked=marked)
        _magnetic(ctx, a, steel=True); instances.append(a)
    ctx.camera(position=(.22, -.28, .17), target=(0, 0, .008))
    ctx.video_annotation = lambda: 'Left: active face | centre: back | right: unmarked | gravity and no pose control'
    ctx.plots('asset.magnetic_force.z', _column('asset', 'position', 'z'),
              'control1.magnetic_force.z', 'control2.magnetic_force.z')
    ctx.run(2)
    contact = _plane_penetration(ctx, ('asset', 'control1', 'control2'))
    assert contact['maximum_geometry_penetration_m'] <= .0005, contact
    assert max(abs(r['control1.magnetic_force.z']) for r in ctx.rows) < 1e-9
    assert max(abs(r['control2.magnetic_force.z']) for r in ctx.rows) < 1e-9
    tail = [r for r in ctx.rows if r['time'] >= 1.6]
    assert max(abs(r[_column('asset', 'position', 'z')]) for r in tail) < .0003
    assert min(r[_column('asset', 'position', 'z')] for r in ctx.rows) >= -.0005
    _settled(ctx, ('asset', 'control1', 'control2'), .005)
    ctx.experiment_details = {'contract': '2 mm marked-face/back/unmarked gravity release; withdrawal is release of initial support',
        'target': 'finite ideal steel faces; controls still experience native gravity/contact',
        'minimum_active_base_height_m': min(r[_column('asset', 'position', 'z')] for r in ctx.rows),
        'geometry_contact': contact}


def steel_shear(ctx: TestContext):
    """Vertical steel, weak shear holds; stronger downward force slides."""
    ctx.simulation(dt=.002)
    q = (math.sqrt(.5), 0, math.sqrt(.5), 0)
    _steel(ctx, 'vertical', position=(0, 0, .03), rotation=q)
    ctx.initial(position=(.0002, 0, .03), rotation=q)
    body = _magnetic(ctx, ctx.asset, steel=True)
    ctx.force('shear', body, keyframes=[
        {'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, -.10)},
        {'time': .8, 'force': (0, 0, -.10)}, {'time': .84, 'force': (0, 0, 0)},
        {'time': 1., 'force': (0, 0, 0)}, {'time': 1.2, 'force': (0, 0, -.50)},
        {'time': 1.32, 'force': (0, 0, -.50)}, {'time': 1.36, 'force': (0, 0, 0)},
    ])
    ctx.camera(position=(.22, -.30, .16), target=(0, 0, .015))
    ctx.video_annotation = lambda: 'Vertical steel | weak 0.10 N | strong ramp towards 0.50 N; withdraw at 3 mm slip'
    ctx.plots('shear.force.z', 'asset.magnetic_force.x', _column('asset', 'position', 'z'),
              _column('asset', 'position', 'x'))
    ctx.run(1.)
    strong_start = body.state['position'][2]
    while ctx.session.time < 1.36 and strong_start-body.state['position'][2] < .003:
        ctx.step()
    withdrawal = ctx.session.time
    ctx.actions.pop('shear')
    ctx.run(round((math.ceil((withdrawal+.6)/.04)*.04-withdrawal)/ctx.dt)*ctx.dt)
    weak = [r for r in ctx.rows if .3 <= r['time'] <= .8]
    assert np.ptp([r[_column('asset', 'position', 'z')] for r in weak]) < .0005
    strong = [r for r in ctx.rows if 1. <= r['time'] <= withdrawal]
    assert max(r[_column('asset', 'position', 'z')] for r in strong)-min(r[_column('asset', 'position', 'z')] for r in strong) > .002
    assert min(r[_column('asset', 'position', 'x')] for r in ctx.rows) >= -.0005
    contact = _plane_penetration(ctx, ('asset',), normal=(1, 0, 0), origin=(0, 0, .03))
    assert contact['maximum_geometry_penetration_m'] <= .0005, contact
    ctx.experiment_details = {'contract': 'normal magnetic traction with native friction on a vertical plate',
        'weak_shear_extra_N': .10, 'strong_shear_extra_N': .50, 'gravity_load_N': MASS_KG*9.81,
        'strong_load_withdrawal_rule': '3 mm vertical slip or 1.36 s, whichever occurs first',
        'actual_strong_withdrawal_time_s': withdrawal,
        'actual_peak_applied_shear_N': max(abs(r['shear.force.z']) for r in ctx.rows),
        'weak_height_range_m': float(np.ptp([r[_column('asset', 'position', 'z')] for r in weak])),
        'geometry_contact': contact,
        'boundary': 'applied shear is not a measured contact reaction'}


def _pair_test(ctx, mode, peer_path=None):
    ctx.simulation(dt=.001 if mode == 'tilted' else .002 if mode == 'attract' else .004, gravity=(0, 0, 0))
    ctx.initial(position=(0, 0, .02))
    distance = .040 if mode == 'range' else .015 if mode == 'tilted' else .012
    angle = math.radians(30) if mode == 'tilted' else math.pi if mode == 'repel' else 0
    q = (math.cos(angle/2), math.sin(angle/2), 0, 0)
    from table_1000.physics.simulation import rotation as qmatrix
    center2 = np.array([0., 0., .022+distance])
    pos2 = center2-qmatrix(q)@np.array([0., 0., .002])
    peer = ctx.spawn(peer_path or ctx.asset_path, name='peer', position=pos2, rotation=q)
    b1, b2 = [_magnetic(ctx, a) for a in (ctx.asset, peer)]
    ctx.observe('pair.distance', lambda: np.linalg.norm(_center(b2)-_center(b1)), unit='m')
    for i, axis in enumerate('xyz'):
        ctx.observe('total_linear_momentum.'+axis, lambda i=i: _total_momentum((ctx.asset, peer))[0][i], unit='kg·m/s')
        ctx.observe('total_angular_momentum.'+axis, lambda i=i: _total_momentum((ctx.asset, peer))[1][i], unit='kg·m²/s')
        ctx.observe('net_magnetic_force.'+axis,
                    lambda i=i: ctx.asset.behavior().last_force[i]+peer.behavior().last_force[i], unit='N')
    ctx.observe('peer.angle', lambda: math.acos(float(np.clip(b2.state['rotation'][2, 2], -1, 1))), unit='rad')
    if mode == 'repel':
        ctx.camera(position=(.60, -.975, .45), target=(0, 0, .028), focal_length=24)
    else:
        ctx.camera(position=(.12, -.18, .10), target=(0, 0, .04))
    ctx.video_annotation = lambda: f'Two free magnets | {mode} | explicit axial polarity, no gravity or fixtures'
    ctx.plots('pair.distance', 'asset.magnetic_force.z', 'peer.magnetic_force.z', 'peer.angle',
              'total_linear_momentum.z', 'total_angular_momentum.x')
    ctx.run(1.2)
    rows = ctx.rows
    assert min(r['pair.distance'] for r in rows) >= .003
    residual = max(np.linalg.norm([r['net_magnetic_force.'+a] for a in 'xyz']) for r in rows)
    assert residual < 1e-6
    linear_error = max(np.linalg.norm([r['total_linear_momentum.'+a] for a in 'xyz']) for r in rows)
    angular_error = max(np.linalg.norm([r['total_angular_momentum.'+a] for a in 'xyz']) for r in rows)
    assert linear_error < 1e-6
    assert angular_error < 5e-7
    if mode == 'attract':
        assert rows[-1]['pair.distance'] < .008
        assert np.ptp([r['pair.distance'] for r in rows if r['time'] >= .8]) < .0005
    elif mode == 'repel':
        assert rows[-1]['pair.distance'] > distance+.002
    elif mode == 'range':
        assert max(abs(r['pair.distance']-distance) for r in rows) < 1e-6
        assert max(abs(r['asset.magnetic_force.z']) for r in rows) == 0
    else:
        assert max(abs(r['peer.angle']-angle) for r in rows) > math.radians(5)
    for a in (ctx.asset, peer):
        assert a.behavior().callback_count == ctx.session.steps
        assert sum(p['applied_by_this_instance'] for p in a.behavior().last_pairs) <= 1
    ctx.experiment_details = {'contract': 'free signed dipole pair '+mode,
        'initial_center_distance_m': distance, 'minimum_center_distance_m': min(r['pair.distance'] for r in rows),
        'maximum_net_magnetic_force_N': float(residual), 'linear_momentum_drift_kg_m_s': float(linear_error),
        'angular_momentum_drift_kg_m2_s': float(angular_error), 'initial_tilt_rad': angle}


def mutual_attract(ctx):
    _pair_test(ctx, 'attract')


def mutual_repel(ctx):
    _pair_test(ctx, 'repel')


def mutual_range(ctx):
    _pair_test(ctx, 'range')


def mutual_tilt(ctx):
    _pair_test(ctx, 'tilted')


TESTS.update(steel_distance=steel_distance, steel_capture=steel_capture, steel_shear=steel_shear,
             mutual_attract=mutual_attract, mutual_repel=mutual_repel,
             mutual_range=mutual_range, mutual_tilt=mutual_tilt)


def _return_with_native_fixture(ctx, body, goal, *, duration=.8):
    """Move only a native D6 fixture target; release without touching body state."""
    from pxr import Gf
    from scipy.spatial.transform import Rotation, Slerp
    start = body.state['position'].copy()
    q = body.state['quaternion'].copy()
    slerp = Slerp([0, 1], Rotation.from_quat([q[[1, 2, 3, 0]], [0, 0, 0, 1]]))
    with ctx.spring(body, position=start, rotation=q, linear_stiffness=1000, linear_damping=6,
                    angular_stiffness=.01, angular_damping=.0002) as joint:
        for i in range(round(duration/ctx.dt)):
            u = (i+1)/round(duration/ctx.dt)
            # Smooth velocity at both ends; this is an actuator command.
            u = u*u*(3-2*u)
            target = (1-u)*start+u*np.asarray(goal)
            quat = slerp([u]).as_quat()[0][[3, 0, 1, 2]]
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(*target))
            joint.GetLocalRot0Attr().Set(Gf.Quatf(quat[0], Gf.Vec3f(*quat[1:])))
            ctx.step()
        ctx.run(.2)
        release = {'time_s': ctx.session.time, 'position_m': body.state['position'].tolist(),
                   'velocity_m_s_rad_s': body.state['velocity'].tolist()}
    return release


def steel_pull_peel(ctx: TestContext):
    """COM/edge equal ramps; detach, actuator return, free recapture."""
    ctx.simulation(dt=.002, gravity=(0, 0, 0))
    ctx.initial(position=(0, -.035, .0002))
    edge = ctx.spawn(ctx.asset_path, name='edge', position=(0, .035, .0002))
    _steel(ctx, 'com_plate', position=(0, -.035, 0), half=(.025, .025))
    _steel(ctx, 'edge_plate', position=(0, .035, 0), half=(.025, .025))
    bodies = [_magnetic(ctx, a, steel=True) for a in (ctx.asset, edge)]
    for label, body in zip(('com_pull', 'edge_pull'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .3, 'force': (0, 0, 0)},
                  {'time': 1.1, 'force': (0, 0, 1.5)}, {'time': 1.104, 'force': (0, 0, 0)}]
        if label == 'edge_pull':
            frames[0]['point'] = (.010, 0, HEIGHT_M)
        ctx.force(label, body, keyframes=frames, point_frame='body')
    ctx.observe('edge.angle', lambda: math.acos(float(np.clip(bodies[1].state['rotation'][2, 2], -1, 1))), unit='rad')
    ctx.camera(position=(.21, -.28, .17), target=(0, 0, .020))
    phase = {'name': 'external pull'}
    ctx.video_annotation = lambda: 'Lower: COM | upper: edge pull | '+phase['name']
    ctx.plots('com_pull.force.z', 'edge_pull.force.z', 'asset.magnetic_force.z', 'edge.magnetic_force.z',
              _column('asset', 'position', 'z'), _column('edge', 'position', 'z'), 'edge.angle')
    detached = {}
    with ExitStack() as holds:
        while ctx.session.time < 1.12 and len(detached) < 2:
            ctx.step()
            for a, body, label in zip((ctx.asset, edge), bodies, ('com_pull', 'edge_pull')):
                if a.name not in detached and body.state['position'][2] > .025 and np.linalg.norm(a.behavior().last_force) < 1e-8:
                    detached[a.name] = {'time_s': ctx.session.time,
                        'applied_force_N': float(ctx.rows[-1][label+'.force.z']), 'height_m': float(body.state['position'][2])}
                    ctx.actions.pop(label, None)
                    holds.enter_context(ctx.spring(body, position=body.state['position'], rotation=body.state['quaternion'],
                        linear_stiffness=1000, linear_damping=6, angular_stiffness=.01, angular_damping=.0002))
    assert len(detached) == 2
    assert max(r['edge.angle'] for r in ctx.rows) > math.radians(15)
    ctx.actions.pop('com_pull', None); ctx.actions.pop('edge_pull', None)
    phase['name'] = 'native fixture return (not free magnetic motion)'
    # Catch both before returning either, preventing free flight off camera.
    with ctx.spring(bodies[1], position=bodies[1].state['position'], rotation=bodies[1].state['quaternion'],
                    linear_stiffness=1000, linear_damping=6, angular_stiffness=.01, angular_damping=.0002):
        release_com = _return_with_native_fixture(ctx, bodies[0], (0, -.035, .0025))
    with ctx.spring(bodies[0], position=bodies[0].state['position'], rotation=bodies[0].state['quaternion'],
                    linear_stiffness=1000, linear_damping=6, angular_stiffness=.01, angular_damping=.0002):
        release_edge = _return_with_native_fixture(ctx, bodies[1], (0, .035, .0025))
    phase['name'] = 'fixtures released; free recapture'
    # Finish on a 25 FPS frame boundary, including the final physical sample.
    ctx.run(round((math.ceil((ctx.session.time+.8)/.04)*.04-ctx.session.time)/ctx.dt)*ctx.dt)
    for body in bodies:
        assert abs(body.state['position'][2]) < .0003
    _settled(ctx, ('asset', 'edge'), .005)
    contact = _plane_penetration(ctx, ('asset', 'edge'))
    assert contact['maximum_geometry_penetration_m'] <= .0005, contact
    ctx.experiment_details = {'contract': 'equal COM/edge pull ramps, zero magnetic force after separation, native fixture return, free recapture',
        'detachments': detached, 'native_fixture_release': [release_com, release_edge],
        'peak_edge_tilt_rad': max(r['edge.angle'] for r in ctx.rows),
        'geometry_contact': contact,
        'fixture_position_writes': 'only native joint targets; never dynamic body poses or velocities'}


def mutual_pull_recapture(ctx: TestContext):
    """Clamp one magnet; pull the mate beyond cutoff, return and release."""
    ctx.simulation(dt=.002, gravity=(0, 0, 0))
    ctx.initial(position=(0, 0, .020))
    peer = ctx.spawn(ctx.asset_path, name='peer', position=(0, 0, .02765))
    b1, b2 = [_magnetic(ctx, a) for a in (ctx.asset, peer)]
    ctx.observe('pair.distance', lambda: np.linalg.norm(_center(b2)-_center(b1)), unit='m')
    ctx.force('pull', b2, keyframes=[{'time': 0, 'force': (0, 0, 0)},
        {'time': .3, 'force': (0, 0, 0)}, {'time': .9, 'force': (0, 0, .5)},
        {'time': .904, 'force': (0, 0, 0)}])
    ctx.camera(position=(.13, -.20, .11), target=(0, 0, .045))
    phase = {'name': 'pull; lower magnet clamped'}
    ctx.video_annotation = lambda: phase['name']
    ctx.plots('pull.force.z', 'peer.magnetic_force.z', 'pair.distance', _column('peer', 'linear_velocity', 'z'))
    with ctx.fixed(b1):
        while ctx.session.time < 1. and np.linalg.norm(_center(b2)-_center(b1)) < .040:
            ctx.step()
        detached_distance = float(np.linalg.norm(_center(b2)-_center(b1)))
        assert detached_distance > .035 and np.linalg.norm(peer.behavior().last_force) < 1e-8
        ctx.actions.pop('pull', None)
        phase['name'] = 'native actuator return; lower magnet clamped'
        release = _return_with_native_fixture(ctx, b2, (0, 0, .032))
        assert np.linalg.norm(_center(b2)-_center(b1)) > .009
        phase['name'] = 'upper magnet freely recaptures; lower magnet clamped'
        # Finish on a 25 FPS boundary; this adds at most one frame interval.
        ctx.run(round((math.ceil((ctx.session.time+.8)/.04)*.04-ctx.session.time)/ctx.dt)*ctx.dt)
        assert np.linalg.norm(_center(b2)-_center(b1)) < .008
    ctx.experiment_details = {'contract': 'mutual pull-off/cutoff and repeat capture with one explicit fixed measurement clamp',
        'detached_center_distance_m': detached_distance, 'native_fixture_release': release,
        'final_center_distance_m': float(np.linalg.norm(_center(b2)-_center(b1)))}


def dynamic_steel_reaction(ctx: TestContext):
    """Free finite steel and magnet; total momentum includes spin and orbit."""
    ctx.simulation(dt=.002, gravity=(0, 0, 0))
    # The default 25 mm friction-anchor merge distance is larger than this
    # contact patch. It creates a spurious couple for the free plate/magnet.
    # Keep native patch friction and both materials; resolve its anchors at
    # the 0.1 mm contact scale instead. This is an explicit scene requirement.
    from pxr import PhysxSchema, UsdPhysics
    scene = next(p for p in ctx.session.stage.Traverse() if p.IsA(UsdPhysics.Scene))
    PhysxSchema.PhysxSceneAPI.Apply(scene).CreateFrictionCorrelationDistanceAttr(.0001)
    ctx.initial(position=(.006, 0, .002))
    steel = _steel(ctx, 'free_steel', half=(.030, .030), dynamic=True)
    _magnetic(ctx, ctx.asset, steel=True)
    _body(ctx, steel, 'free_steel')
    for i, axis in enumerate('xyz'):
        for kind, index, unit in [('linear', 0, 'kg·m/s'), ('angular', 1, 'kg·m²/s')]:
            ctx.observe('total_'+kind+'.'+axis,
                lambda i=i, index=index: _total_momentum((ctx.asset, steel))[index][i], unit=unit)
    ctx.camera(position=(.16, -.22, .15), target=(0, 0, .005))
    ctx.video_annotation = lambda: 'Free magnet and 50 g finite steel | equal/opposite forces, no gravity'
    ctx.plots('asset.magnetic_force.z', _column('asset', 'position', 'z'),
              'free_steel.free_steel.position.z', 'total_linear.z', 'total_angular.y')
    ctx.run(1.2)
    linear = max(np.linalg.norm([r['total_linear.'+a] for a in 'xyz']) for r in ctx.rows)
    angular = max(np.linalg.norm([r['total_angular.'+a] for a in 'xyz']) for r in ctx.rows)
    assert linear < 1e-6 and angular < 5e-7
    assert max(abs(r['free_steel.free_steel.position.z']) for r in ctx.rows) > .0001
    ctx.experiment_details = {'contract': 'unclamped dynamic steel reaction; offset magnet; no gravity or external force',
        'steel_mass_kg': .050, 'linear_momentum_drift_kg_m_s': float(linear),
        'angular_momentum_drift_kg_m2_s': float(angular),
        'scene_friction_correlation_distance_m': .0001,
        'scene_friction_type': 'patch', 'contact_materials_changed': False}


def steel_two_magnets(ctx: TestContext):
    """Both force models coexist: two magnets on one explicit static plate."""
    ctx.simulation(dt=.002)
    ctx.initial(position=(-.015, 0, .002))
    peer = ctx.spawn(ctx.asset_path, name='peer', position=(.015, 0, .002))
    _steel(ctx, 'shared_steel', half=(.050, .035))
    for instance in (ctx.asset, peer):
        _magnetic(ctx, instance, steel=True)
        ctx.observe(instance.name+'.steel_force',
            lambda instance=instance: sum((instance.behavior().last_steel or {}).get('forces_N', [])), unit='N')
    ctx.observe('owned_pairs', lambda: sum(sum(p['applied_by_this_instance']
        for p in a.behavior().last_pairs) for a in (ctx.asset, peer)), unit='1')
    ctx.camera(position=(.14, -.20, .13), target=(0, 0, .008))
    ctx.video_annotation = lambda: 'One marked static steel plate | both magnets free | steel and dipole forces coexist'
    ctx.plots('asset.steel_force', 'peer.steel_force', 'owned_pairs',
              _column('asset', 'position', 'z'), _column('peer', 'position', 'z'))
    ctx.run(1.6)
    assert max(r['owned_pairs'] for r in ctx.rows) == 1
    for instance in (ctx.asset, peer):
        assert instance.behavior().last_steel['target'] == '/World/Fixtures/shared_steel'
        assert sum(instance.behavior().last_steel['forces_N']) > .9
        assert instance.behavior().callback_count == ctx.session.steps
        assert min(r[_column(instance.name, 'position', 'z')] for r in ctx.rows) >= -.0005
    _settled(ctx, ('asset', 'peer'), .005)
    contact = _plane_penetration(ctx, ('asset', 'peer'))
    assert contact['maximum_geometry_penetration_m'] <= .0005, contact
    ctx.experiment_details = {'contract': 'two free same-ID magnets on one explicitly marked static steel fixture',
        'maximum_owned_pairs': 1, 'steel_and_dipole_models_simultaneous': True,
        'geometry_contact': contact,
        'boundary': 'single-ID fixture test; not a cross-category assembly'}


def three_magnets(ctx: TestContext):
    """Three interacting magnets and an untouched fourth, no pair duplication."""
    ctx.simulation(dt=.002, gravity=(0, 0, 0))
    ctx.initial(position=(0, 0, .020))
    peers = [ctx.asset] + [ctx.spawn(ctx.asset_path, name=f'peer{i}', position=(0, 0, .020+i*.012)) for i in (1, 2)]
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .060, .032))
    for a in peers+[idle]:
        _magnetic(ctx, a)
    ctx.observe('owned_pairs', lambda: sum(sum(p['applied_by_this_instance'] for p in a.behavior().last_pairs) for a in peers), unit='1')
    for i, axis in enumerate('xyz'):
        ctx.observe('net_magnetic_force.'+axis, lambda i=i: sum(a.behavior().last_force[i] for a in peers), unit='N')
    ctx.camera(position=(.16, -.22, .13), target=(0, .020, .035))
    ctx.video_annotation = lambda: 'Three active magnets and a remote idle instance | each unordered pair once'
    ctx.plots('owned_pairs', 'asset.magnetic_pairs', 'peer1.magnetic_pairs', 'peer2.magnetic_pairs', 'idle.speed')
    ctx.run(1.2)
    assert max(r['owned_pairs'] for r in ctx.rows) == 3
    assert max(r['idle.magnetic_pairs'] for r in ctx.rows) == 0
    assert max(r['idle.speed'] for r in ctx.rows) < 1e-6
    assert max(np.linalg.norm([r['net_magnetic_force.'+a] for a in 'xyz']) for r in ctx.rows) < 1e-6
    for a in peers+[idle]:
        assert a.behavior().callback_count == ctx.session.steps
    ctx.experiment_details = {'contract': 'three active, one out-of-range idle; no duplicated unordered pairs',
        'maximum_owned_pairs': 3, 'idle_center_y_m': .060}


TESTS.update(steel_pull_peel=steel_pull_peel, mutual_pull_recapture=mutual_pull_recapture,
             dynamic_steel_reaction=dynamic_steel_reaction, three_magnets=three_magnets,
             steel_two_magnets=steel_two_magnets)


def _record_configuration(test):
    """Keep scene-level contact requirements in each official result."""
    def experiment(ctx):
        test(ctx)
        from pxr import PhysxSchema, UsdPhysics
        scene = next(p for p in ctx.session.stage.Traverse() if p.IsA(UsdPhysics.Scene))
        api = PhysxSchema.PhysxSceneAPI(scene)
        ctx.experiment_details['scene_parameters'] = {
            'friction_type': api.GetFrictionTypeAttr().Get(),
            'friction_correlation_distance_m': api.GetFrictionCorrelationDistanceAttr().Get(),
            'friction_offset_threshold_m': api.GetFrictionOffsetThresholdAttr().Get()}
        ctx.experiment_details['convex_cooking'] = {
            'hull_vertex_limit': 128, 'local_points_multiplier': 100,
            'inverse_local_scale': .01, 'world_geometry_changed': False}
    return experiment


TESTS = {name: _record_configuration(test) for name, test in TESTS.items()}
