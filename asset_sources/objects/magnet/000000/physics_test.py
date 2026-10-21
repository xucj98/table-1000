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


def _environment_states(ctx, instance):
    """Read each environment's own tensor state once per recorded sample."""
    from scipy.spatial.transform import Rotation
    key = (ctx.session.steps, ctx.session.callback_steps)
    if getattr(instance, '_magnet_sample_key', None) != key:
        positions, rotations, centers = instance.environment_states()
        count, bodies = positions.shape[:2]
        velocity = instance.view.view.get_velocities().detach().cpu().numpy().astype(float).reshape(count, bodies, 6)
        quaternions = Rotation.from_matrix(rotations.reshape(-1, 3, 3)).as_quat().reshape(count, bodies, 4)[..., [3, 0, 1, 2]]
        instance._magnet_sample = (positions, rotations, centers, velocity, quaternions)
        instance._magnet_sample_key = key
    return instance._magnet_sample


def _observe_environments(ctx, instance, name):
    i = instance.indices[name]
    for e in range(ctx.num_envs):
        prefix = f'env{e}.{instance.name}.{name}.'
        for field, index, axes, unit in [('position', 0, 'xyz', 'm'),
                ('quaternion', 4, 'wxyz', '1'), ('linear_velocity', 3, 'xyz', 'm/s'),
                ('angular_velocity', 3, 'xyz', 'rad/s')]:
            for j, axis in enumerate(axes):
                k = j+3 if field == 'angular_velocity' else j
                ctx.observe(prefix+field+'.'+axis,
                    lambda e=e, index=index, k=k: _environment_states(ctx, instance)[index][e, i, k], unit=unit)
        ctx.observe(f'env{e}.{instance.name}.speed',
            lambda e=e: np.linalg.norm(_environment_states(ctx, instance)[3][e, i, :3]), unit='m/s')


def _environment_diagnostic(instance, environment):
    diagnostics = instance.behavior().environment_diagnostics
    return diagnostics[environment] if diagnostics else {'force_N': np.zeros(3), 'torque_N_m': np.zeros(3), 'pairs': [], 'steel': None, 'steel_diagnostic': None}


def _environment_momentum(ctx, environment):
    linear, angular = np.zeros(3), np.zeros(3)
    for instance in ctx.session.instances.values():
        positions, rotations, centers, velocities, _ = _environment_states(ctx, instance)
        for i in range(len(instance.body_names)):
            p = instance.mass[i]*velocities[environment, i, :3]
            r = rotations[environment, i]
            linear += p
            angular += np.cross(centers[environment, i], p)+r@instance.local_inertia[i]@r.T@velocities[environment, i, 3:]
    return linear, angular


def _validate_environment(ctx, test, e):
    """Original physical criteria, evaluated on every replica's own records."""
    rows = [{k[len(f'env{e}.'):]: v for k, v in row.items() if k.startswith(f'env{e}.')} | {'time': row['time']} for row in ctx.rows]
    def values(instance, field):
        return np.array([[r[f'{instance}.{BODY}.{field}.{a}'] for a in ('wxyz' if field == 'quaternion' else 'xyz')] for r in rows])
    def speed(instance):
        return np.array([r[instance+'.speed'] for r in rows])
    def settled(names, limit):
        tail = np.array([r['time'] >= rows[-1]['time']-.4-1e-8 for r in rows])
        for name in names:
            assert speed(name)[tail].max() < limit, (e, name, 'final speed')
            assert np.linalg.norm(np.ptp(values(name, 'position')[tail], axis=0)) < .0005, (e, name, 'final drift')
    from scipy.spatial.transform import Rotation
    times = np.array([r['time'] for r in rows])
    tail = times >= times[-1]-.4-1e-8
    def centres(name):
        rotation = Rotation.from_quat(values(name, 'quaternion')[:, [1, 2, 3, 0]]).as_matrix()
        return values(name, 'position')+rotation@np.array([0, 0, .002])
    def net_force(names):
        force = sum(np.array([[r[n+'.magnetic_force.'+a] for a in 'xyz'] for r in rows]) for n in names)
        residual = float(np.linalg.norm(force, axis=1).max())
        assert residual < 1e-6, (e, 'net internal force', residual)
        return residual
    details = {'environment': e, 'recorded_samples': len(rows)}
    if test == 'contact_faces':
        for name, sign in [('asset', 1), ('cap', -1)]:
            assert values(name, 'position')[-1, 0]-values(name, 'position')[0, 0] > .002
            rotation = Rotation.from_quat(values(name, 'quaternion')[-1, [1, 2, 3, 0]]).as_matrix()
            assert sign*rotation[2, 2] > .98, (e, name, 'unintended tipping')
        xy = values('idle', 'position')[:, :2]
        assert np.linalg.norm(xy-xy[0], axis=1).max() < .0002
        settled(['asset', 'cap', 'idle'], .005)
    elif test == 'free_impulses':
        impulse = ctx.experiment_details['applied_impulse_N_s']
        for name in ['asset', 'offset']:
            assert abs(MASS_KG*values(name, 'linear_velocity')[-1, 0]-impulse)/impulse < .01
        assert np.linalg.norm(values('asset', 'angular_velocity')[-1]) < .01
        assert np.linalg.norm(values('offset', 'angular_velocity')[-1]) > .1
        assert max(np.linalg.norm(values('idle', field), axis=1).max() for field in ['linear_velocity', 'angular_velocity']) < 1e-6
    elif test == 'drop_faces':
        from table_1000.physics.test import ground_clearance
        model = json.loads((ctx.asset_path.parent/'model.json').read_text())
        contact = ground_clearance(model, rows, ['asset', 'cap', 'side'], 0)
        assert contact['maximum_geometry_penetration_m'] < .001, (e, contact)
        details['geometry_contact'] = contact
        settled(['asset', 'cap', 'side'], .01)
    elif test == 'steel_distance':
        for entry in ctx.experiment_details['measurements']:
            actual = -rows[-1][entry['instance']+'.magnetic_force.z']
            expected = entry['expected_force_N']
            assert abs(actual-expected) < max(1e-6, .01*expected), (e, entry['instance'], actual, expected)
    elif test == 'steel_capture':
        for name in ['control1', 'control2']:
            assert max(abs(r[name+'.magnetic_force.z']) for r in rows) < 1e-9
        assert np.abs(values('asset', 'position')[tail, 2]).max() < .0003
        settled(['asset', 'control1', 'control2'], .005)
    elif test == 'steel_shear':
        times = np.array([r['time'] for r in rows]);z = values('asset', 'position')[:, 2]
        assert np.ptp(z[(times >= .3) & (times <= .8)]) < .0005
        end = ctx.experiment_details['actual_strong_withdrawal_time_s']
        assert np.ptp(z[(times >= 1) & (times <= end)]) > .002
    elif test in ['mutual_attract', 'mutual_repel', 'mutual_range', 'mutual_tilt']:
        from scipy.spatial.transform import Rotation
        centres = []
        for name in ['asset', 'peer']:
            q = values(name, 'quaternion')[:, [1, 2, 3, 0]]
            r = Rotation.from_quat(q).as_matrix()
            centres.append(values(name, 'position')+r@np.array([0, 0, .002]))
        distance = np.linalg.norm(centres[1]-centres[0], axis=1)
        assert distance.min() >= .003
        if test == 'mutual_attract':
            assert distance[-1] < .008 and np.ptp(distance[tail]) < .0005
        elif test == 'mutual_repel':
            assert distance[-1] > .014
        elif test == 'mutual_range':
            assert np.abs(distance-.040).max() < 1e-6
            assert max(abs(row['asset.magnetic_force.z']) for row in rows) == 0
        else:
            angle = np.arccos(np.clip(Rotation.from_quat(values('peer', 'quaternion')[:, [1, 2, 3, 0]]).as_matrix()[:, 2, 2], -1, 1))
            assert np.abs(angle-math.radians(30)).max() > math.radians(5)
        linear = np.array([[r['total_linear.'+a] for a in 'xyz'] for r in rows])
        angular = np.array([[r['total_angular.'+a] for a in 'xyz'] for r in rows])
        details['maximum_net_magnetic_force_N'] = net_force(['asset', 'peer'])
        for name in ['asset', 'peer']:
            assert max(r[name+'.owned_pairs'] for r in rows) <= 1
        assert np.linalg.norm(linear, axis=1).max() < 1e-6
        assert np.linalg.norm(angular, axis=1).max() < 5e-7
        details.update(minimum_center_distance_m=float(distance.min()), final_center_distance_m=float(distance[-1]), angular_momentum_drift_kg_m2_s=float(np.linalg.norm(angular, axis=1).max()))
    elif test == 'dynamic_steel_reaction':
        linear = np.array([[r['total_linear.'+a] for a in 'xyz'] for r in rows])
        angular = np.array([[r['total_angular.'+a] for a in 'xyz'] for r in rows])
        assert np.linalg.norm(linear, axis=1).max() < 1e-6
        assert np.linalg.norm(angular, axis=1).max() < 5e-7
        assert max(abs(r['free_steel.free_steel.position.z']) for r in rows) > .0001
    elif test == 'three_magnets':
        assert max(r['idle.magnetic_pairs'] for r in rows) == 0
        assert speed('idle').max() < 1e-6
        assert max(sum(r[n+'.owned_pairs'] for n in ['asset', 'peer1', 'peer2']) for r in rows) == 3
        details['maximum_net_magnetic_force_N'] = net_force(['asset', 'peer1', 'peer2'])
    elif test == 'steel_two_magnets':
        assert max(r['asset.owned_pairs']+r['peer.owned_pairs'] for r in rows) == 1
        for name in ['asset', 'peer']:
            assert rows[-1][name+'.steel_force'] > .9
        settled(['asset', 'peer'], .005)
    elif test == 'steel_pull_peel':
        edge_rotation = Rotation.from_quat(values('edge', 'quaternion')[:, [1, 2, 3, 0]]).as_matrix()
        assert np.arccos(np.clip(edge_rotation[:, 2, 2], -1, 1)).max() > math.radians(15)
        details['detachments'] = {}
        for name in ['asset', 'edge']:
            force = np.array([[r[name+'.magnetic_force.'+a] for a in 'xyz'] for r in rows])
            detached = (times <= 1.12) & (values(name, 'position')[:, 2] > .025) & (np.linalg.norm(force, axis=1) < 1e-8)
            assert detached.any(), (e, name, 'did not detach before fixture return')
            details['detachments'][name] = float(times[detached][0])
            assert abs(values(name, 'position')[-1, 2]) < .0003
        settled(['asset', 'edge'], .005)
    elif test == 'mutual_pull_recapture':
        distance = np.linalg.norm(centres('peer')-centres('asset'), axis=1)
        force = np.array([[r['peer.magnetic_force.'+a] for a in 'xyz'] for r in rows])
        before_return = times <= ctx.experiment_details['native_fixture_release']['time_s']-1.+1e-8
        assert ((distance > .035) & (np.linalg.norm(force, axis=1) < 1e-8) & before_return).any(), (e, 'cutoff before return')
        release = np.argmin(np.abs(times-ctx.experiment_details['native_fixture_release']['time_s']))
        assert distance[release] > .009 and distance[-1] < .008
        details.update(detached_center_distance_m=float(distance[before_return].max()), final_center_distance_m=float(distance[-1]))
    if test in ['steel_capture', 'steel_shear', 'steel_pull_peel', 'steel_two_magnets']:
        normal, origin = ((1, 0, 0), (0, 0, .03)) if test == 'steel_shear' else ((0, 0, 1), (0, 0, 0))
        names = ['asset', 'control1', 'control2'] if test == 'steel_capture' else ['asset', 'edge'] if test == 'steel_pull_peel' else ['asset', 'peer'] if test == 'steel_two_magnets' else ['asset']
        saved = ctx.rows
        try:
            ctx.rows = rows
            contact = _plane_penetration(ctx, names, normal=normal, origin=origin)
        finally:
            ctx.rows = saved
        assert contact['maximum_geometry_penetration_m'] <= .0005, (e, contact)
        details['geometry_contact'] = contact
    return details


def _column(instance, field, axis):
    return f'{instance}.{BODY}.{field}.{axis}'


def _body(ctx, instance, name=BODY):
    body = instance.body(name)
    ctx.observe_body(body)
    _observe_environments(ctx, instance, name)
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
    """20 mm release at CPU 2 ms/GPU 1 ms, retaining the 1 mm limit."""
    ctx.simulation(dt=.001 if ctx.physics == 'gpu' else .002)
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
    clearance = ground_clearance(model, ctx.rows, ('asset', 'cap', 'side'), 0)
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
    for e in range(ctx.num_envs):
        for i, axis in enumerate('xyz'):
            ctx.observe(f'env{e}.{instance.name}.magnetic_force.{axis}',
                lambda e=e, i=i: _environment_diagnostic(instance, e)['force_N'][i], unit='N')
        ctx.observe(f'env{e}.{instance.name}.magnetic_pairs',
            lambda e=e: len(_environment_diagnostic(instance, e)['pairs']), unit='1')
        ctx.observe(f'env{e}.{instance.name}.owned_pairs',
            lambda e=e: sum(p['applied_by_this_instance'] for p in _environment_diagnostic(instance, e)['pairs']), unit='1')
        ctx.observe(f'env{e}.{instance.name}.steel_force',
            lambda e=e: sum((_environment_diagnostic(instance, e)['steel'] or {}).get('forces_N', [])), unit='N')

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
    path = ctx.session.environment(0)+'/Fixtures/'+name
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
    ctx.simulation(dt=.001 if mode == 'tilted' or mode == 'attract' and ctx.physics == 'gpu' else .002 if mode == 'attract' else .004, gravity=(0, 0, 0))
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


def _return_with_native_fixture(ctx, grip, goal, *, duration=.8):
    """Use a predeclared native tensor grip; release keeps body state."""
    grip.grasp()
    grip.move_to(position=goal, rotation=(1, 0, 0, 0), duration=duration)
    ctx.run(.2)
    body = grip.body
    release = {'time_s': ctx.session.time, 'position_m': body.state['position'].tolist(),
               'velocity_m_s_rad_s': body.state['velocity'].tolist()}
    grip.release()
    return release


def steel_pull_peel(ctx: TestContext):
    """COM/edge equal ramps; detach, actuator return, free recapture."""
    ctx.simulation(dt=.001 if ctx.physics == 'gpu' else .002, gravity=(0, 0, 0))
    ctx.initial(position=(0, -.035, .0002))
    edge = ctx.spawn(ctx.asset_path, name='edge', position=(0, .035, .0002))
    _steel(ctx, 'com_plate', position=(0, -.035, 0), half=(.025, .025))
    _steel(ctx, 'edge_plate', position=(0, .035, 0), half=(.025, .025))
    bodies = [_magnetic(ctx, a, steel=True) for a in (ctx.asset, edge)]
    guides = [ctx.grip(body, name='return_'+str(i)) for i, body in enumerate(bodies)]
    for guide in guides:
        guide.release()
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
    while ctx.session.time < 1.12 and len(detached) < 2:
        ctx.step()
        for a, body, label, guide in zip((ctx.asset, edge), bodies, ('com_pull', 'edge_pull'), guides):
            if a.name not in detached and body.state['position'][2] > .025 and np.linalg.norm(a.behavior().last_force) < 1e-8:
                detached[a.name] = {'time_s': ctx.session.time,
                    'applied_force_N': float(ctx.rows[-1][label+'.force.z']), 'height_m': float(body.state['position'][2])}
                ctx.actions.pop(label, None)
                guide.grasp()
    assert len(detached) == 2
    assert max(r['edge.angle'] for r in ctx.rows) > math.radians(15)
    ctx.actions.pop('com_pull', None); ctx.actions.pop('edge_pull', None)
    phase['name'] = 'native tensor grip return (not free magnetic motion)'
    release_com = _return_with_native_fixture(ctx, guides[0], (0, -.035, .0025))
    guides[0].grasp()
    release_edge = _return_with_native_fixture(ctx, guides[1], (0, .035, .0025))
    guides[0].release()
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
    guide = ctx.grip(b2, name='return_peer')
    guide.release()
    with ctx.fixed(b1):
        ctx.ensure_initialized()
        while ctx.session.time < 1. and np.linalg.norm(_center(b2)-_center(b1)) < .040:
            ctx.step()
        detached_distance = float(np.linalg.norm(_center(b2)-_center(b1)))
        assert detached_distance > .035 and np.linalg.norm(peer.behavior().last_force) < 1e-8
        ctx.actions.pop('pull', None)
        phase['name'] = 'native actuator return; lower magnet clamped'
        release = _return_with_native_fixture(ctx, guide, (0, 0, .032))
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
        assert instance.behavior().last_steel['target'] == ctx.session.environment(0)+'/Fixtures/shared_steel'
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


def environment_isolation(ctx: TestContext):
    """Different initial states per replica test force isolation, before motion."""
    import torch
    ctx.simulation(dt=.001 if ctx.physics == 'gpu' else .002, gravity=(0, 0, 0))
    ctx.initial(position=(-.06, 0, .02))
    peer = ctx.spawn(ctx.asset_path, name='peer', position=(-.06, 0, .032))
    steel_magnet = ctx.spawn(ctx.asset_path, name='steel_magnet', position=(.06, 0, .0002))
    _steel(ctx, 'isolation_plate', position=(.06, 0, 0), half=(.025, .025))
    for instance in [ctx.asset, peer, steel_magnet]:
        _magnetic(ctx, instance, steel=instance is steel_magnet)
    ctx.camera(position=(.20, -.27, .16), target=(0, 0, .018))
    ctx.video_annotation = lambda: 'Environment 0 shown | replicas have distinct initial gaps/polarities | initialization only'
    ctx.plots('asset.magnetic_force.z', 'peer.magnetic_force.z', 'steel_magnet.magnetic_force.z')
    ctx.ensure_initialized()
    configurations = []
    for e in range(ctx.num_envs):
        distance, inverted, gap, back = [(.012, False, .0002, False), (.018, False, .002, False),
            (.040, False, .010, False), (.012, True, .0002, True)][e % 4]
        configurations.append((distance, inverted, gap, back))
    # All heterogeneous transforms are initialization, before the first physical step.
    for instance in [peer, steel_magnet]:
        view = instance.view.view
        transforms = view.get_transforms().clone()
        for e, (distance, inverted, gap, back) in enumerate(configurations):
            if instance is peer:
                transforms[e, 2] = .022+distance+(.002 if inverted else -.002)
                transforms[e, 3:7] = torch.tensor([1, 0, 0, 0] if inverted else [0, 0, 0, 1], device=transforms.device)
            else:
                transforms[e, 2] = gap+(HEIGHT_M if back else 0)
                transforms[e, 3:7] = torch.tensor([1, 0, 0, 0] if back else [0, 0, 0, 1], device=transforms.device)
        view.set_transforms(transforms, torch.arange(ctx.num_envs, dtype=torch.int32, device=transforms.device))
    ctx.world.forward();ctx.session.refresh()
    ctx.step()
    checks = []
    for e, (distance, inverted, gap, back) in enumerate(configurations):
        softened = math.sqrt(distance*distance+.002**2)
        product = .035**2*(-1 if inverted else 1)
        expected_peer = 0. if distance >= .035 else 1e-7*product*(9*distance/softened**5-15*distance**3/softened**7)
        expected_steel = 0. if back or gap >= .008 else -1/(1+gap/.002)**2
        actual = [float(_environment_diagnostic(a, e)['force_N'][2]) for a in [ctx.asset, peer, steel_magnet]]
        for a, b in zip(actual, [-expected_peer, expected_peer, expected_steel]):
            assert abs(a-b) < max(1e-6, abs(b)*.01), (e, actual, expected_peer, expected_steel)
        assert abs(actual[0]+actual[1]) < 1e-6
        checks.append({'environment': e, 'initial_pair_distance_m': distance, 'peer_inverted': inverted,
            'initial_steel_gap_m': gap, 'steel_back': back, 'first_step_force_z_N': actual,
            'expected_peer_force_z_N': expected_peer, 'expected_steel_force_z_N': expected_steel})
    ctx.run(.039 if ctx.physics == 'gpu' else .038)
    for a in [ctx.asset, peer, steel_magnet]:
        assert a.behavior().callback_count == ctx.session.steps
    ctx.experiment_details = {'contract': 'heterogeneous initialization followed by per-environment force calculation; no state writes during motion',
        'environment_checks': checks, 'rendered_environment': 0, 'not_a_same_input_backend_comparison': True}
    ctx.metric('isolated_environments', ctx.num_envs, unit='1', label='异初态独立磁力环境数')


def behavior_lifecycle(ctx: TestContext):
    """Official components, same stage, stop/reset and real unload/reload."""
    import torch
    from pxr import Sdf, UsdPhysics
    from omni.kit.scripting import ScriptManager
    from table_1000.physics import simulation
    ctx.simulation(dt=.001 if ctx.physics == 'gpu' else .002, gravity=(0, 0, 0))
    ctx.initial(position=(0, 0, .020))
    peer = ctx.spawn(ctx.asset_path, name='peer', position=(0, 0, .032))
    steel = _steel(ctx, 'distant_steel', position=(1, 0, 0), half=(.025, .025), dynamic=True)
    for a in [ctx.asset, peer]: _magnetic(ctx, a)
    ctx.camera(position=(.12, -.18, .10), target=(0, 0, .03))
    ctx.video_annotation = lambda: 'Official components after same-stage stop/reset, unload and reload | final repeat shown'
    ctx.plots('asset.magnetic_force.z', 'peer.magnetic_force.z', _column('peer', 'position', 'z'))
    ctx.ensure_initialized()
    stage = ctx.session.stage
    stage_id = stage.GetRootLayer().identifier
    render = ctx.render
    ctx.render = False
    def updates():
        if ctx.app:
            for _ in range(5): ctx.app.update()
    def snapshot():
        return {n: (a.view.view.get_transforms().clone(), a.view.view.get_velocities().clone())
                for n,a in ctx.session.instances.items()}
    def restore(saved):
        for n,(pose,velocity) in saved.items():
            a = ctx.session.instances[n]
            indices = torch.arange(len(pose), dtype=torch.int32, device=pose.device)
            a.view.view.set_transforms(pose.clone(), indices)
            a.view.view.set_velocities(velocity.clone(), indices)
            a._magnet_sample_key = None
        ctx.world.forward();ctx.session.refresh()
        return max(float((x-y).abs().max().item()) for n,state in snapshot().items() for x,y in zip(state,saved[n]))
    def stop():
        ctx.session.active = False
        ctx.world._disable_app_control_on_stop_handle = True
        ctx.world.stop();updates()
    def start(saved):
        ctx.world.reset();ctx.session.initialize()
        error = restore(saved)
        assert error < 1e-6, error
        ctx.session.time = 0.;ctx.session.steps = ctx.session.callback_steps = 0
        ctx.session.error = None
        ctx.rows.clear();ctx.action_values.clear()
        ctx.experiment_started = None
        return error
    def captured():
        checks=[]
        for e in range(ctx.num_envs):
            positions=[]
            for a in [ctx.asset, ctx.session.instances['peer']]:
                pos,rot,_ = a.environment_states()
                positions.append(pos[e,0]+rot[e,0]@np.array([0,0,.002]))
            distance=float(np.linalg.norm(positions[1]-positions[0]));assert distance < .008,(e,distance)
            for a in [ctx.asset, ctx.session.instances['peer']]:
                b=a.behavior();assert b.error is None and b.callback_count == ctx.session.steps
                assert len(b.environment_diagnostics[e]['pairs']) == 1
            checks.append({'environment':e,'final_center_distance_m':distance})
        return checks
    saved=snapshot();initial_session=ctx.session
    ctx.run(.8);first=captured()
    a,b=ctx.asset.behavior(),peer.behavior()
    counts=[a.callback_count,b.callback_count]
    ctx.world.pause();updates();updates()
    assert [a.callback_count,b.callback_count] == counts
    ctx.world.play();updates()
    before=[a.callback_count,b.callback_count]
    ctx.step()
    assert [a.callback_count-before[0],b.callback_count-before[1]] == [1,1]
    stop()
    assert all(s.subscription is None and s.callback_count == 0 and not s.last_pairs for s in [a,b])
    reset_error=start(saved);ctx.run(.8);second=captured()
    retained={n:v for n,v in snapshot().items() if n!='peer'}
    stop()
    for e in range(ctx.num_envs): stage.RemovePrim(ctx.session.replica_path(peer.path,e))
    updates()
    assert b.session is None and b.instance is None and b.subscription is None
    assert peer.path not in ScriptManager.get_instance()._prim_to_scripts
    assert set(ctx.session.instances) == {'asset','distant_steel'}
    # Keep surviving bodies at their current state only at the stopped boundary.
    ctx.observers.clear();ctx.units.clear()
    _magnetic(ctx,ctx.asset)
    start(retained);ctx.run(.04)
    for e in range(ctx.num_envs):
        diag=_environment_diagnostic(ctx.asset,e)
        assert not diag['pairs'] and np.linalg.norm(diag['force_N']) == 0
    stop();old_session=ctx.session
    for e in range(ctx.num_envs): stage.RemovePrim(old_session.replica_path(ctx.asset.path,e))
    updates()
    assert a.session is None and a.instance is None and a.subscription is None
    assert not old_session.instances and old_session.key not in simulation._sessions
    assert old_session.prepare_subscription is None and old_session.flush_subscription is None
    # Reuse this same stage; new session and official scripts discover fresh views.
    ctx.session=simulation.session_for(stage)
    assert ctx.session is not old_session
    ctx.session.active=False;ctx.session.set_environments(ctx.num_envs)
    ctx.session.actions=ctx.apply_actions;ctx.session.fixtures=ctx.fixtures
    ctx.asset=ctx.spawn(ctx.asset_path,name='asset',position=(0,0,.020))
    peer=ctx.spawn(ctx.asset_path,name='peer',position=(0,0,.032))
    ctx.session.register(steel.path,name='distant_steel')
    for instance in [ctx.asset,peer]:
        for e in range(1,ctx.num_envs):
            assert Sdf.CopySpec(stage.GetRootLayer(),instance.path,stage.GetRootLayer(),ctx.session.replica_path(instance.path,e))
    updates();ctx.observers.clear();ctx.units.clear()
    for instance in [ctx.asset,peer]: _magnetic(ctx,instance)
    ctx.plots('asset.magnetic_force.z','peer.magnetic_force.z',_column('peer','position','z'))
    reload_error=start(saved)
    assert ctx.asset.behavior() is not a and peer.behavior() is not b
    ctx.render=render;ctx.images.clear();ctx.integrating_seconds=0.
    ctx.run(.8);third=captured()
    assert stage.GetRootLayer().identifier == stage_id
    body_counts=[]
    for e in range(ctx.num_envs):
        from pxr import Usd
        count=sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(stage.GetPrimAtPath(ctx.session.environment(e))))
        assert count == 3,(e,count);body_counts.append(count)
    ctx.experiment_details={'contract':'official scripting component lifecycle; same stage; no live topology edits',
        'first_capture':first,'reset_capture':second,'reloaded_capture':third,
        'reset_max_component_error':reset_error,'reload_max_component_error':reload_error,
        'pause_callbacks_unchanged':True,'resume_callbacks_per_step':[1,1],
        'unloaded_peer_residual_force_N':0.,'last_magnet_released_dynamic_target_and_callbacks':True,
        'new_script_objects_after_reload':True,'rigid_bodies_per_environment':body_counts,
        'state_writes':'only reset/initialization boundaries, never force-driven motion',
        'media_scope':'final reloaded .8s capture; prior segments numerically checked',
        'performance_scope':'lifecycle diagnostic; exclude from steady-state benchmark','gui_visible':'not tested'}
    ctx.metric('reset_error',reset_error,unit='1',label='复位状态最大分量误差')
    ctx.metric('reload_error',reload_error,unit='1',label='重载状态最大分量误差')


TESTS.update(environment_isolation=environment_isolation, behavior_lifecycle=behavior_lifecycle)

def _record_configuration(test):
    """Keep scene-level contact requirements in each official result."""
    def experiment(ctx):
        for e in range(ctx.num_envs):
            for j, kind, unit in [(0, 'linear', 'kg·m/s'), (1, 'angular', 'kg·m²/s')]:
                for i, axis in enumerate('xyz'):
                    ctx.observe(f'env{e}.total_{kind}.{axis}',
                        lambda e=e, j=j, i=i: _environment_momentum(ctx, e)[j][i], unit=unit)
        test(ctx)
        ctx.experiment_details['environment_checks'] = [_validate_environment(ctx, test.__name__, e) for e in range(ctx.num_envs)]
        for metric, key, factor, unit, label in [
                ('maximum_penetration', 'maximum_geometry_penetration_m', 1000, 'mm', '最大几何穿入'),
                ('shear_peak', 'actual_peak_applied_shear_N', 1, 'N', '实际峰值剪切外力'),
                ('angular_momentum_drift', 'angular_momentum_drift_kg_m2_s', 1, 'kg·m²/s', '角动量漂移'),
                ('minimum_pair_distance', 'minimum_center_distance_m', 1000, 'mm', '最小磁矩中心距')]:
            value = ctx.experiment_details.get(key, ctx.experiment_details.get('geometry_contact', {}).get(key))
            if value is not None:
                ctx.metric(metric, value*factor, unit=unit, label=label)
        for instance in ctx.session.instances.values():
            prim = ctx.session.stage.GetPrimAtPath(instance.path)
            if prim.GetAttribute('table1000:magnet:moment'):
                behavior = instance.behavior()
                assert behavior.error is None and behavior.callback_count == ctx.session.steps
        ctx.metric('validated_environments', ctx.num_envs, unit='1', label='逐项验证环境数')
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


TESTS = {name: test if name in ['behavior_lifecycle', 'environment_isolation'] else _record_configuration(test) for name, test in TESTS.items()}
