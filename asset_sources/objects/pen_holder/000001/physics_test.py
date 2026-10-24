"""Empty single-cell holder experiments; SI units and test-only ball probes."""
import json
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'pen_holder.container'
MASS_KG = .160


def column(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def environment_data(instance):
    # Reuse each tensor read across scalar observers in the same physical step.
    step = instance.session.steps
    cached = getattr(instance, '_holder_observations', None)
    if cached is None or cached[0] != step:
        positions, rotations, _ = instance.environment_states()
        velocities = instance.view.view.get_velocities().detach().cpu().numpy().reshape(instance.session.envs, -1, 6)
        cached = (step, positions, rotations, velocities)
        instance._holder_observations = cached
    return cached[1:]


def observe(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    ctx.observe(instance.name+'.tilt', lambda: math.acos(float(np.clip(body.state['rotation'][2, 2], -1, 1))), unit='rad')
    for e in range(ctx.num_envs):
        for axis, a in enumerate('xyz'):
            ctx.observe(f'{instance.name}.env{e}.position.{a}',
                lambda e=e, j=axis: environment_data(instance)[0][e,instance.indices[BODY],j], unit='m')
        ctx.observe(f'{instance.name}.env{e}.tilt',
            lambda e=e: math.acos(float(np.clip(environment_data(instance)[1][e,instance.indices[BODY],2,2],-1,1))), unit='rad')
        ctx.observe(f'{instance.name}.env{e}.speed',
            lambda e=e: np.linalg.norm(environment_data(instance)[2][e,instance.indices[BODY],:3]), unit='m/s')
    return body


def ground(ctx):
    if ctx.physics == 'gpu':
        ctx.simulation(dt=.001)
    ctx.ground(static_friction=.65, dynamic_friction=.50, contact_offset=.0001)


def stable(ctx, names):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        assert max(r[name+'.speed'] for r in tail) < .01, name+' velocity'
        p = np.array([[r[column(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert np.linalg.norm(np.ptp(p, axis=0)) < .001, name+' drift'
        for e in range(ctx.num_envs):
            assert max(r[f'{name}.env{e}.speed'] for r in tail) < .01
            positions = np.array([[r[f'{name}.env{e}.position.{a}'] for a in 'xyz'] for r in tail])
            assert np.linalg.norm(np.ptp(positions, axis=0)) < .001


def clearance(ctx):
    from table_1000.physics.test import ground_clearance
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    result = ground_clearance(model, ctx.rows, ctx.session.instances, 0)
    assert result['maximum_geometry_penetration_m'] < .002, result
    ctx.metric('ground_penetration', result['maximum_geometry_penetration_m']*1000, unit='mm', label='最大几何穿地')
    return result


def drop_orientations(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(-.18, 0, .05))
    side = ctx.spawn(ctx.asset_path, name='side', position=(.18, 0, .10),
                     rotation=(math.sqrt(.5), math.sqrt(.5), 0, 0))
    for instance in (ctx.asset, side):
        observe(ctx, instance)
    ctx.camera(position=(.55, -.8, .55), target=(0, 0, .05))
    ctx.video_annotation = lambda: 'Upright / side | 50 mm minimum-point drop | empty rigid holders'
    ctx.plots(column('asset', 'position', 'z'), column('side', 'position', 'z'), 'asset.speed', 'side.speed')
    ctx.run(3)
    stable(ctx, ('asset', 'side'))
    assert ctx.rows[-1]['asset.tilt'] < math.radians(5)
    assert abs(ctx.rows[-1]['side.tilt']-math.pi/2) < math.radians(5)
    for e in range(ctx.num_envs):
        assert ctx.rows[-1][f'asset.env{e}.tilt'] < math.radians(5)
        assert abs(ctx.rows[-1][f'side.env{e}.tilt']-math.pi/2) < math.radians(5)
    ctx.metric('upright_end_tilt', ctx.rows[-1]['asset.tilt']*180/math.pi, unit='deg', label='正立末倾角')
    ctx.metric('side_end_tilt', ctx.rows[-1]['side.tilt']*180/math.pi, unit='deg', label='侧倒末倾角')
    ctx.experiment_details = {'initial_minimum_clearance_m': .05, 'geometry_contact': clearance(ctx),
        'boundary': 'empty rigid holder; no plastic flex or calibrated impact response'}


def push_height(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(-.2, 0, .00002))
    high = ctx.spawn(ctx.asset_path, name='high', position=(.2, 0, .00002))
    for label, instance, height in [('low', ctx.asset, .010), ('high', high, .117)]:
        body = observe(ctx, instance)
        ctx.force(label+'_push', body, point_frame='body', keyframes=[
            {'time': 0, 'force': (0, 0, 0), 'point': (0, -.05, height)},
            {'time': .5, 'force': (0, 0, 0)}, {'time': 1.5, 'force': (0, 1.2, 0)},
            {'time': 1.8, 'force': (0, 1.2, 0)}, {'time': 1.804, 'force': (0, 0, 0)}])
    ctx.camera(position=(.85, -1.4, 1.2), target=(0, .40, .05))
    ctx.video_annotation = lambda: 'Low 10 mm / high 117 mm | same 0-1.2 N +Y ramp | release 1.804 s'
    ctx.plots('low_push.force.y', 'low_push.point.z', 'high_push.point.z',
              column('asset', 'position', 'y'), column('high', 'position', 'y'), 'asset.tilt', 'high.tilt')
    ctx.run(4)
    stable(ctx, ('asset', 'high'))
    assert max(r['asset.tilt'] for r in ctx.rows) < math.radians(10)
    assert max(r['high.tilt'] for r in ctx.rows) > math.radians(45)
    assert ctx.rows[-1][column('asset', 'position', 'y')] > .005
    for e in range(ctx.num_envs):
        assert max(r[f'asset.env{e}.tilt'] for r in ctx.rows) < math.radians(10)
        assert max(r[f'high.env{e}.tilt'] for r in ctx.rows) > math.radians(45)
        assert ctx.rows[-1][f'asset.env{e}.position.y']-ctx.rows[0][f'asset.env{e}.position.y'] > .005
    thresholds = {}
    for name, force in [('asset', 'low_push'), ('high', 'high_push')]:
        key = column(name, 'position', 'y')
        events = {}
        for event, predicate in [('slide_0_5mm', lambda r: r[key]-ctx.rows[0][key] >= .0005),
                                 ('tilt_5deg', lambda r: r[name+'.tilt'] >= math.radians(5)),
                                 ('tilt_45deg', lambda r: r[name+'.tilt'] >= math.radians(45))]:
            row = next((r for r in ctx.rows if predicate(r)), None)
            events[event] = None if row is None else {'time_s': row['time'], 'applied_force_N': row[force+'.force.y']}
        release = min(ctx.rows, key=lambda r: abs(r['time']-1.804))
        events['post_release_displacement_m'] = ctx.rows[-1][key]-release[key]
        thresholds[name] = events
    ctx.metric('low_slide_force', thresholds['asset']['slide_0_5mm']['applied_force_N'], unit='N', label='低位0.5mm起滑施加力')
    ctx.metric('high_tip_force', thresholds['high']['tilt_45deg']['applied_force_N'], unit='N', label='高位45度倾倒施加力')
    ctx.experiment_details = {'thresholds': thresholds, 'geometry_contact': clearance(ctx),
        'boundary': 'applied force thresholds on estimated rough table; not measured real tipping force; empty holders'}


def cavity_probes(ctx: TestContext):
    if ctx.physics == 'gpu':
        ctx.simulation(dt=.001)
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    stage = ctx.session.stage
    holder = observe(ctx, ctx.asset)
    root = ctx.session.environment(0) + '/Fixtures'
    material = UsdShade.Material.Define(stage, root+'/ProbeMaterial')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.9)
    api.CreateDynamicFrictionAttr(.8)
    api.CreateRestitutionAttr(0)
    # Inner planar faces: wall x +/-58mm, partitions at +/-19.333mm,
    # partition thickness 2mm; probes stay clear of rounded corners/notch.
    bounds = [(-.0475, .0475)]
    probes = []
    for i, (lo, hi) in enumerate(bounds):
        body = UsdGeom.Xform.Define(stage, root+f'/Probe{i}')
        body.AddTranslateOp().Set(Gf.Vec3d((lo+hi)/2, 0, .010))
        body.GetPrim().SetCustomDataByKey('table1000:name', f'probe{i}')
        body.GetPrim().SetCustomDataByKey('table1000:role', 'rigid')
        sphere = UsdGeom.Sphere.Define(stage, root+f'/Probe{i}/Sphere')
        sphere.CreateRadiusAttr(.004)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(.9, .18, .04)])
        p = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(.005)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
        api.CreateEnableCCDAttr(True)
        api.CreateAngularDampingAttr(20.)
        api.CreateSolverPositionIterationCountAttr(16)
        api.CreateSolverVelocityIterationCountAttr(4)
        api = PhysxSchema.PhysxCollisionAPI.Apply(p)
        api.CreateContactOffsetAttr(.002)
        api.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(p).Bind(material, materialPurpose='physics')
        instance = ctx.session.register(str(body.GetPath()), name=f'probe{i}')
        probes.append(instance.body(f'probe{i}'))
    def pose(view):
        return view.state['position']
    def velocity(view):
        return view.state['velocity']
    for i, probe in enumerate(probes):
        frames = [{'time': 0, 'force': (0, 0, 0)}]
        for start, force in [(1., (.012,0,0)), (1.6, (-.012,0,0)),
                             (2.2, (0,.012,0)), (2.8, (0,-.012,0)), (3.4, (0,0,0))]:
            frames.append({'time': start-.004, 'force': frames[-1]['force']})
            frames.append({'time': start, 'force': force})
        ctx.force(f'probe{i}_load', probe, keyframes=frames)
    # Declare all structure before initialization; the session owns replicas.
    with ctx.fixed(holder):
        for i, view in enumerate(probes):
            for axis, a in enumerate('xyz'):
                ctx.observe(f'probe{i}.position.{a}', lambda v=view, j=axis: pose(v)[j], unit='m')
            ctx.observe(f'probe{i}.speed', lambda v=view: np.linalg.norm(velocity(v)[:3]), unit='m/s')
            for e in range(ctx.num_envs):
                for j, a in enumerate('xyz'):
                    ctx.observe(f'probe{i}.env{e}.position.{a}',
                        lambda v=view, e=e, j=j: environment_data(v.instance)[0][e,0,j], unit='m')
                ctx.observe(f'probe{i}.env{e}.speed',
                    lambda v=view, e=e: np.linalg.norm(environment_data(v.instance)[2][e,0,:3]), unit='m/s')
        ctx.camera(position=(.08, -.12, .42), target=(0, 0, .035))
        ctx.video_annotation = lambda: 'Fixed holder | one 4 mm / 5 g ball | gravity, +X, -X, +Y, -Y, release'
        ctx.plots('probe0_load.force.x', 'probe0_load.force.y',
                  *[f'probe{i}.position.{a}' for a in 'xyz' for i in range(1)])
        ctx.run(4)
        for e in range(ctx.num_envs):
            for i, (lo, hi) in enumerate(bounds):
                xyz = np.array([[r[f'probe{i}.env{e}.position.{a}'] for a in 'xyz'] for r in ctx.rows])
                gaps = np.c_[xyz[:,0]-lo-.004, hi-xyz[:,0]-.004,
                             xyz[:,1]+.0475-.004, .0475-xyz[:,1]-.004, xyz[:,2]-.007]
                assert gaps.min() > -.001, (e,i,gaps.min())
                assert max(gaps.min(axis=0)) < .001, (e,i,gaps.min(axis=0))
                assert ctx.rows[-1][f'probe{i}.env{e}.speed'] < .01
        contacts = []
        for i, (lo, hi) in enumerate(bounds):
            xyz = np.array([[r[f'probe{i}.position.{a}'] for a in 'xyz'] for r in ctx.rows])
            gaps = np.c_[xyz[:, 0]-lo-.004, hi-xyz[:, 0]-.004,
                         xyz[:, 1]+.0475-.004, .0475-xyz[:, 1]-.004, xyz[:, 2]-.003-.004]
            assert gaps.min() > -.001, (i, gaps.min())
            # Each face must actually be reached, not merely avoid penetration.
            assert max(gaps.min(axis=0)) < .001, (i, gaps.min(axis=0))
            assert ctx.rows[-1][f'probe{i}.speed'] < .01
            contacts.append({'probe': i, 'minimum_surface_gaps_m': gaps.min(axis=0).tolist(),
                             'maximum_plane_penetration_m': max(0, float(-gaps.min()))})
        ctx.metric('probe_penetration', max(c['maximum_plane_penetration_m'] for c in contacts)*1000, unit='mm', label='探针最大穿透')
        ctx.metric('probe_end_speed', max(ctx.rows[-1][f'probe{i}.env{e}.speed'] for e in range(ctx.num_envs) for i in range(len(probes)))*1000, unit='mm/s', label='各环境探针末速')
        ctx.experiment_details = {'probe_radius_m': .004, 'probe_mass_kg': .005,
            'probe_initial_center_z_m': .010, 'probe_contact_offset_m': .002,
            'probe_rest_offset_m': 0, 'probe_lateral_force_N': .012,
            'probe_angular_damping_per_s': 20.,
            'fixture': 'initialization-time fixed fixture; registered environment-local rigid probes with ctx.force',
            'contacts': contacts, 'boundary': 'ideal balls in the single cell; no contact reaction sampled; not a pen-loaded combined asset'}


def free_impulse(ctx: TestContext):
    ctx.simulation(dt=ctx.dt, gravity=(0, 0, 0))
    ctx.initial(position=(0, -.2, .1))
    offset = ctx.spawn(ctx.asset_path, name='offset', position=(0, 0, .1))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .2, .1))
    bodies = [observe(ctx, instance) for instance in (ctx.asset, offset, idle)]
    for label, body in zip(('com_push', 'offset_push'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
                  {'time': .404, 'force': (MASS_KG, 0, 0)}, {'time': .604, 'force': (MASS_KG, 0, 0)},
                  {'time': .608, 'force': (0, 0, 0)}]
        if label == 'offset_push':
            frames[0]['point'] = (0, .03, .05)
        ctx.force(label, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.6, -.85, .65), target=(.07, 0, .12))
    ctx.video_annotation = lambda: 'Zero gravity | COM / eccentric / idle | same impulse | no ground or fixtures'
    ctx.plots('com_push.force.x', column('asset', 'linear_velocity', 'x'),
              column('offset', 'linear_velocity', 'x'), column('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*body.state['velocity'][0]-impulse)/impulse for body in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    for e in range(ctx.num_envs):
        velocities = [b.instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs,-1,6)[e,b.instance.indices[BODY]] for b in bodies]
        assert max(abs(MASS_KG*v[0]-impulse)/impulse for v in velocities[:2]) < .01
        assert np.linalg.norm(velocities[0][3:]) < .01
        assert np.linalg.norm(velocities[1][3:]) > .1
        assert np.linalg.norm(velocities[2]) < 1e-6
    ctx.metric('momentum_error', max(errors)*100, unit='%', label='线动量相对误差')
    ctx.metric('idle_speed', np.linalg.norm(bodies[2].state['velocity'][:3]), unit='m/s', label='闲置实例速度')
    ctx.experiment_details = {'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
        'boundary': 'empty rigid body; geometry-derived inertia; no ground or gravity'}


TESTS = {'drop_orientations': drop_orientations, 'push_height': push_height,
         'cavity_probes': cavity_probes, 'free_impulse': free_impulse}
