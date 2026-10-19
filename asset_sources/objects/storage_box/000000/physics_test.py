"""Empty rigid storage-box tests; SI units, explicit ideal fixtures.

Engineering parameters and acceptance contract are documented in README.
No pose writes after experiment initialization and no lid holding mechanism.
"""
import json
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'storage_box.container'
RADIUS = 0.075
HEIGHT = 0.09
MASS_KG = 0.112862
HAS_LID = False


def col(name, field, axis):
    return f'{name}.{BODY}.{field}.{axis}'


def observe(ctx, instance):
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    body = instance.body(BODY)
    ctx.observe(instance.name+'.speed', lambda: np.linalg.norm(body.state['velocity'][:3]), unit='m/s')
    ctx.observe(instance.name+'.tilt', lambda: math.acos(float(np.clip(body.state['rotation'][2, 2], -1, 1))), unit='rad')
    return body


def setup(ctx, instance, position, rotation=(1, 0, 0, 0)):
    # Lid moves aside only in initial state; it remains a simulated body.
    bodies = {'storage_box.lid': {'position': (position[0]+.13, position[1]-.15, .00525)}} if HAS_LID else {}
    ctx.initial(instance, position=position, rotation=rotation, bodies=bodies)
    return observe(ctx, instance)


def ground(ctx, rough=False):
    ctx.ground(static_friction=1.25 if rough else .65,
               dynamic_friction=.80 if rough else .50, contact_offset=.0001)


def stable(ctx, names):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-8]
    for name in names:
        assert max(r[name+'.speed'] for r in tail) < .01, name+' velocity'
        points = np.array([[r[col(name, 'position', a)] for a in 'xyz'] for r in tail])
        assert np.linalg.norm(np.ptp(points, axis=0)) < .001, name+' drift'


def clearance(ctx):
    from table_1000.physics.test import ground_clearance
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    result = ground_clearance(model, ctx.rows, ctx.session.instances, 0)
    assert result['maximum_geometry_penetration_m'] < .002, result
    return result


def drop_orientations(ctx: TestContext):
    ground(ctx)
    setup(ctx, ctx.asset, (-.18, 0, .02))
    side = ctx.spawn(ctx.asset_path, name='side')
    setup(ctx, side, (.18, 0, RADIUS+.02), (math.sqrt(.5), math.sqrt(.5), 0, 0))
    ctx.camera(position=(.55, -.8, .55), target=(0, 0, .04))
    ctx.video_annotation = lambda: 'Empty box | upright / side | 20 mm minimum-point drop'
    ctx.plots(col('asset', 'position', 'z'), col('side', 'position', 'z'), 'asset.speed', 'side.speed')
    ctx.run(3)
    stable(ctx, ('asset', 'side'))
    assert ctx.rows[-1]['asset.tilt'] < math.radians(5)
    assert abs(ctx.rows[-1]['side.tilt']-math.pi/2) < math.radians(5)
    ctx.experiment_details = {'initial_minimum_clearance_m': .02, 'geometry_contact': clearance(ctx),
        'boundary': 'empty rigid shell, lid separate where present; no calibrated impact response'}


def push_height(ctx: TestContext):
    ground(ctx, rough=True)
    setup(ctx, ctx.asset, (-.18, 0, .00002))
    high = ctx.spawn(ctx.asset_path, name='high')
    setup(ctx, high, (.18, 0, .00002))
    for label, instance, height in [('low', ctx.asset, .1*HEIGHT), ('high', high, .98*HEIGHT)]:
        ctx.force(label+'_push', instance.body(BODY), point_frame='body', keyframes=[
            {'time': 0, 'force': (0, 0, 0), 'point': (0, -RADIUS+.0008, height)},
            {'time': .5, 'force': (0, 0, 0)}, {'time': 1.5, 'force': (0, 1.2*MASS_KG*9.81, 0)},
            {'time': 1.504, 'force': (0, 0, 0)}])
    ctx.camera(position=(.8, -1.1, .9), target=(0, .18, .04))
    ctx.video_annotation = lambda: 'Low 0.1H / high 0.98H | 0-1.2mg ramp, release | rough ideal table'
    ctx.plots('low_push.force.y', 'low_push.point.z', 'high_push.point.z',
              col('asset', 'position', 'y'), col('high', 'position', 'y'), 'asset.tilt', 'high.tilt')
    ctx.run(4)
    stable(ctx, ('asset', 'high'))
    assert max(r['asset.tilt'] for r in ctx.rows) < math.radians(10)
    assert max(r['high.tilt'] for r in ctx.rows) > math.radians(45)
    assert ctx.rows[-1][col('asset', 'position', 'y')] > .005
    ctx.experiment_details = {'peak_force_N': 1.2*MASS_KG*9.81,
        'force_heights_m': [.1*HEIGHT, .98*HEIGHT], 'geometry_contact': clearance(ctx),
        'boundary': 'rough table fixture, not calibrated real tipping threshold; empty boxes'}


def cavity_probes(ctx: TestContext):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from isaacsim.core.prims import RigidPrim
    stage = ctx.session.stage
    holder = observe(ctx, ctx.asset)
    if HAS_LID:
        # Keep the independent lid clear of the open mouth.
        setup(ctx, ctx.asset, (0, 0, 0))
    material = UsdShade.Material.Define(stage, '/World/Fixtures/ProbeMaterial')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.1); api.CreateDynamicFrictionAttr(.1); api.CreateRestitutionAttr(0)
    angles = (np.arange(24)+.5)*math.tau/24
    normals = np.c_[np.cos(angles), np.sin(angles)]
    apothem = (RADIUS-.0016)*math.cos(math.pi/24)
    probes = []
    for i in range(5):
        direction = normals[(i%4)*6]
        rim = i == 4
        radial = (RADIUS-.00085)*math.cos(math.pi/24) if rim else apothem*.5
        sphere = UsdGeom.Sphere.Define(stage, f'/World/Fixtures/Probe{i}')
        sphere.CreateRadiusAttr(.004)
        sphere.AddTranslateOp().Set(Gf.Vec3d(*(radial*direction), HEIGHT+.008 if rim else .010))
        sphere.CreateDisplayColorAttr([Gf.Vec3f(.9, .18, .04)])
        p = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p); UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.005)
        api = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
        api.CreateEnableCCDAttr(True); api.CreateSolverPositionIterationCountAttr(16); api.CreateSolverVelocityIterationCountAttr(4)
        api = PhysxSchema.PhysxCollisionAPI.Apply(p)
        api.CreateContactOffsetAttr(.002); api.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(p).Bind(material, materialPurpose='physics')
        view = RigidPrim(str(p.GetPath()), name=f'probe{i}')
        ctx.world.scene.add(view); probes.append(view)
    def pose(view):
        return np.asarray(view.get_world_poses()[0])[0]
    loads = [np.zeros(3) for _ in probes]
    old = ctx.session.actions
    def actions(t, dt):
        old(t, dt)
        for i, view in enumerate(probes):
            force = np.zeros(3)
            if i < 4 and .5 <= t < 3:
                direction = normals[i*6]
                speed = np.dot(np.asarray(view.get_velocities())[0][:2], direction)
                force[:2] = direction*np.clip(.005*(.035-speed)/.05, -.004, .004)
            loads[i] = force
            view.apply_forces_and_torques_at_pos(forces=force[None, :], is_global=True)
    ctx.session.actions = actions
    with ctx.fixed(holder):
        for i, view in enumerate(probes):
            for j, axis in enumerate('xyz'):
                ctx.observe(f'probe{i}.position.{axis}', lambda v=view, j=j: pose(v)[j], unit='m')
                ctx.observe(f'probe{i}.applied_force.{axis}', lambda i=i,j=j: loads[i][j], unit='N')
        ctx.camera(position=(.23, -.32, .42), target=(0, 0, HEIGHT/2))
        ctx.video_annotation = lambda: 'Fixed open box | four inner balls + one rim ball | test-only speed-limited external forces'
        ctx.plots('probe0.applied_force.x', *[f'probe{i}.position.{a}' for i in range(5) for a in 'xz'])
        ctx.run(4)
        contacts = []
        for i in range(4):
            xyz = np.array([[r[f'probe{i}.position.{a}'] for a in 'xyz'] for r in ctx.rows])
            gaps = np.c_[apothem-xyz[:, :2]@normals.T-.004, xyz[:, 2]-.002-.004]
            assert gaps.min() > -.001, (i, gaps.min())
            assert gaps[:, i*6].min() < .001, (i, 'wall not reached')
            assert gaps[:, -1].min() < .001, (i, 'floor not reached')
            contacts.append({'probe': i, 'maximum_plane_penetration_m': max(0, float(-gaps.min()))})
        rim_z = np.array([r['probe4.position.z'] for r in ctx.rows if r['time'] >= 3.5])
        assert np.max(np.abs(rim_z-(HEIGHT+.004))) < .001, 'rim not supporting probe'
        ctx.experiment_details = {'probe_radius_m': .004, 'probe_mass_kg': .005,
            'contact_offset_m': .002, 'rest_offset_m': 0, 'target_speed_m_s': .035, 'force_cap_N': .004,
            'contacts': contacts, 'rim_support_error_m': float(np.max(np.abs(rim_z-(HEIGHT+.004)))),
            'boundary': 'ideal sphere fixtures, fixed box; no actual multi-object filling or contact-reaction measurement'}


def free_impulse(ctx: TestContext):
    ctx.simulation(dt=ctx.dt, gravity=(0, 0, 0))
    setup(ctx, ctx.asset, (0, -.2, .15))
    offset = ctx.spawn(ctx.asset_path, name='offset')
    idle = ctx.spawn(ctx.asset_path, name='idle')
    setup(ctx, offset, (0, 0, .15)); setup(ctx, idle, (0, .2, .15))
    bodies = [i.body(BODY) for i in (ctx.asset, offset, idle)]
    for label, body in zip(('com_push', 'offset_push'), bodies):
        frames = [{'time': 0, 'force': (0, 0, 0)}, {'time': .4, 'force': (0, 0, 0)},
                  {'time': .404, 'force': (MASS_KG, 0, 0)}, {'time': .604, 'force': (MASS_KG, 0, 0)},
                  {'time': .608, 'force': (0, 0, 0)}]
        if label == 'offset_push':
            frames[0]['point'] = (0, RADIUS*.5, HEIGHT*.5)
        ctx.force(label, body, point_frame='body', keyframes=frames)
    ctx.camera(position=(.9, -.7, .85), target=(.06, 0, .15))
    ctx.video_annotation = lambda: 'Zero gravity | COM / eccentric / idle | equal impulse | no ground'
    ctx.plots('com_push.force.x', col('asset', 'linear_velocity', 'x'),
              col('offset', 'linear_velocity', 'x'), col('offset', 'angular_velocity', 'z'))
    ctx.run(1.2)
    impulse = sum(r['com_push.force.x'] for r in ctx.rows)*ctx.dt
    errors = [abs(MASS_KG*b.state['velocity'][0]-impulse)/impulse for b in bodies[:2]]
    assert max(errors) < .01
    assert np.linalg.norm(bodies[0].state['velocity'][3:]) < .01
    assert np.linalg.norm(bodies[1].state['velocity'][3:]) > .1
    assert np.linalg.norm(bodies[2].state['velocity']) < 1e-6
    ctx.experiment_details = {'applied_impulse_N_s': impulse, 'relative_momentum_errors': errors,
        'boundary': 'empty rigid body with geometry-estimated inertia; independent lids remain free'}


TESTS = {'drop_orientations': drop_orientations, 'push_height': push_height,
         'cavity_probes': cavity_probes, 'free_impulse': free_impulse}
