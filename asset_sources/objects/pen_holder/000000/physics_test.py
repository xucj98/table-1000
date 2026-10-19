"""Empty three-cell holder experiments; SI units and test-only ball probes."""
import json
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'pen_holder.container'
MASS_KG = .155


def column(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def observe(ctx, instance):
    body = instance.body(BODY)
    ctx.observe_body(body)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    ctx.observe(instance.name+'.tilt', lambda: math.acos(float(np.clip(body.state['rotation'][2, 2], -1, 1))), unit='rad')
    return body


def ground(ctx):
    ctx.ground(static_friction=.65, dynamic_friction=.50, contact_offset=.0001)


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


def drop_orientations(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(-.18, 0, .05))
    side = ctx.spawn(ctx.asset_path, name='side', position=(.18, 0, .09),
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
    ctx.experiment_details = {'initial_minimum_clearance_m': .05, 'geometry_contact': clearance(ctx),
        'boundary': 'empty rigid holder; no plastic flex or calibrated impact response'}


def push_height(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(-.2, 0, .00002))
    high = ctx.spawn(ctx.asset_path, name='high', position=(.2, 0, .00002))
    for label, instance, height in [('low', ctx.asset, .010), ('high', high, .090)]:
        body = observe(ctx, instance)
        ctx.force(label+'_push', body, point_frame='body', keyframes=[
            {'time': 0, 'force': (0, 0, 0), 'point': (0, -.04, height)},
            {'time': .5, 'force': (0, 0, 0)}, {'time': 1.5, 'force': (0, 1.25, 0)},
            {'time': 1.8, 'force': (0, 1.25, 0)}, {'time': 1.804, 'force': (0, 0, 0)}])
    ctx.camera(position=(.85, -1.4, 1.2), target=(0, .40, .05))
    ctx.video_annotation = lambda: 'Low 10 mm / high 90 mm | same 0-1.25 N +Y ramp | release 1.804 s'
    ctx.plots('low_push.force.y', 'low_push.point.z', 'high_push.point.z',
              column('asset', 'position', 'y'), column('high', 'position', 'y'), 'asset.tilt', 'high.tilt')
    ctx.run(4)
    stable(ctx, ('asset', 'high'))
    assert max(r['asset.tilt'] for r in ctx.rows) < math.radians(10)
    assert max(r['high.tilt'] for r in ctx.rows) > math.radians(45)
    assert ctx.rows[-1][column('asset', 'position', 'y')] > .005
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
    ctx.experiment_details = {'thresholds': thresholds, 'geometry_contact': clearance(ctx),
        'boundary': 'applied force thresholds on estimated rough table; not measured real tipping force; empty holders'}


def cavity_probes(ctx: TestContext):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from isaacsim.core.prims import RigidPrim
    stage = ctx.session.stage
    holder = observe(ctx, ctx.asset)
    material = UsdShade.Material.Define(stage, '/World/Fixtures/ProbeMaterial')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.1)
    api.CreateDynamicFrictionAttr(.1)
    api.CreateRestitutionAttr(0)
    # Inner planar faces: wall x +/-58mm, partitions at +/-19.333mm,
    # partition thickness 2mm; probes stay clear of rounded corners/notch.
    bounds = [(-.058, -.020333333333), (-.018333333333, .018333333333), (.020333333333, .058)]
    probes = []
    for i, (lo, hi) in enumerate(bounds):
        sphere = UsdGeom.Sphere.Define(stage, f'/World/Fixtures/Probe{i}')
        sphere.CreateRadiusAttr(.004)
        sphere.AddTranslateOp().Set(Gf.Vec3d((lo+hi)/2, 0, .010))
        sphere.CreateDisplayColorAttr([Gf.Vec3f(.9, .18, .04)])
        p = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.005)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
        api.CreateEnableCCDAttr(True)
        api.CreateSolverPositionIterationCountAttr(16)
        api.CreateSolverVelocityIterationCountAttr(4)
        api = PhysxSchema.PhysxCollisionAPI.Apply(p)
        api.CreateContactOffsetAttr(.002)
        api.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(p).Bind(material, materialPurpose='physics')
        view = RigidPrim(str(p.GetPath()), name=f'probe{i}')
        ctx.world.scene.add(view)
        probes.append(view)
    def pose(view):
        return np.asarray(view.get_world_poses()[0])[0]
    def velocity(view):
        return np.asarray(view.get_velocities())[0]
    def load(t):
        directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
        i = int((t-1)/.6)
        return .024*np.asarray(directions[i]) if 1 <= t < 3.4 else np.zeros(3)
    old = ctx.session.actions
    def actions(t, dt):
        old(t, dt)
        for view in probes:
            view.apply_forces_and_torques_at_pos(forces=load(t)[None, :], is_global=True)
    ctx.session.actions = actions
    # Initialize fixture views after native world initialization; no probe/asset
    # pose writes during the experiment. Benchmark warmup reset uses saved USD.
    with ctx.fixed(holder):
        for i, view in enumerate(probes):
            for axis, a in enumerate('xyz'):
                ctx.observe(f'probe{i}.position.{a}', lambda v=view, j=axis: pose(v)[j], unit='m')
            ctx.observe(f'probe{i}.speed', lambda v=view: np.linalg.norm(velocity(v)[:3]), unit='m/s')
        for j, a in enumerate('xyz'):
            ctx.observe('probe.applied_force.'+a, lambda j=j: load(max(0, ctx.session.time-ctx.dt))[j] if ctx.session.time else 0, unit='N')
        ctx.camera(position=(.24, -.32, .42), target=(0, 0, .035))
        ctx.video_annotation = lambda: 'Fixed holder | three 4 mm / 5 g balls | gravity, +X, -X, +Y, -Y, release'
        ctx.plots('probe.applied_force.x', 'probe.applied_force.y',
                  *[f'probe{i}.position.{a}' for a in 'xyz' for i in range(3)])
        ctx.run(4)
        contacts = []
        for i, (lo, hi) in enumerate(bounds):
            xyz = np.array([[r[f'probe{i}.position.{a}'] for a in 'xyz'] for r in ctx.rows])
            gaps = np.c_[xyz[:, 0]-lo-.004, hi-xyz[:, 0]-.004,
                         xyz[:, 1]+.038-.004, .038-xyz[:, 1]-.004, xyz[:, 2]-.003-.004]
            assert gaps.min() > -.001, (i, gaps.min())
            # Each face must actually be reached, not merely avoid penetration.
            assert max(gaps.min(axis=0)) < .001, (i, gaps.min(axis=0))
            assert ctx.rows[-1][f'probe{i}.speed'] < .01
            contacts.append({'probe': i, 'minimum_surface_gaps_m': gaps.min(axis=0).tolist(),
                             'maximum_plane_penetration_m': max(0, float(-gaps.min()))})
        ctx.experiment_details = {'probe_radius_m': .004, 'probe_mass_kg': .005,
            'probe_initial_center_z_m': .010, 'probe_contact_offset_m': .002,
            'probe_rest_offset_m': 0, 'probe_lateral_force_N': .024,
            'fixture': 'native fixed joint on holder; dynamic spheres loaded by official rigid-body force API',
            'contacts': contacts, 'boundary': 'ideal balls in each cell; no contact reaction sampled; not a pen-loaded combined asset'}


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
    ctx.experiment_details = {'applied_impulse_N_s': impulse, 'relative_linear_momentum_errors': errors,
        'boundary': 'empty rigid body; geometry-derived inertia; no ground or gravity'}


TESTS = {'drop_orientations': drop_orientations, 'push_height': push_height,
         'cavity_probes': cavity_probes, 'free_impulse': free_impulse}
