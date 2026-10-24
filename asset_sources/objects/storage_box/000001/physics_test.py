"""Empty rigid storage-box tests; SI units, explicit ideal fixtures.

Engineering parameters and acceptance contract are documented in README.
No pose writes after experiment initialization and no lid holding mechanism.
"""
import json
import math
import numpy as np
from table_1000.physics.testing import TestContext

BODY = 'storage_box.container'
RADIUS = 0.045
HEIGHT = 0.05
MASS_KG = 0.038077
HAS_LID = True


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
    bodies = {'storage_box.lid': {'position': (position[0]+.13, position[1]-.15, .00525), 'rotation': (1, 0, 0, 0)}} if HAS_LID else {}
    ctx.initial(instance, position=position, rotation=rotation, bodies=bodies)
    return observe(ctx, instance)


def ground(ctx, rough=False):
    ctx.ground(static_friction=1.65 if rough else .65,
               dynamic_friction=1.40 if rough else .50, contact_offset=.0001)


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
    ctx.camera(position=(.65, -.85, .55), target=(.08, -.06, .025))
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
    ctx.camera(position=(.65, -.95, .75), target=(.08, .05, .025))
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
        ctx.camera(position=(3.07*RADIUS, -4.27*RADIUS, 5.6*RADIUS), target=(0, 0, HEIGHT/2))
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
    ctx.camera(position=(1, -1, .9), target=(.10, -.08, .10))
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


def observe_lid(ctx, instance):
    box = instance.body(BODY)
    lid = instance.body('storage_box.lid')
    def relative():
        return box.state['rotation'].T @ (lid.state['position']-box.state['position'])
    for i, axis in enumerate('xyz'):
        ctx.observe(instance.name+'.lid_relative.'+axis, lambda i=i: relative()[i], unit='m')
    ctx.observe(instance.name+'.lid_speed', lambda: np.linalg.norm(lid.state['velocity'][:3]), unit='m/s')
    return box, lid


def lid_seating(ctx: TestContext):
    ground(ctx)
    ctx.initial(position=(-.12, 0, .00002), bodies={
        'storage_box.lid': {'position': (-.12, 0, HEIGHT+.008)}})
    offset = ctx.spawn(ctx.asset_path, name='offset')
    ctx.initial(offset, position=(.12, 0, .00002), bodies={
        'storage_box.lid': {'position': (.132, 0, HEIGHT+.008)}})
    for instance in (ctx.asset, offset):
        observe(ctx, instance); observe_lid(ctx, instance)
    ctx.camera(position=(6*RADIUS, -10*RADIUS, 6.22*RADIUS), target=(0, 0, .7*HEIGHT))
    ctx.video_annotation = lambda: 'Gravity seating | aligned / 12 mm offset | no latch or attraction'
    ctx.plots('asset.lid_relative.z', 'asset.lid_relative.x', 'offset.lid_relative.z',
              'offset.lid_relative.x', 'asset.lid_speed', 'offset.lid_speed')
    with ctx.fixed(ctx.asset.body(BODY)), ctx.fixed(offset.body(BODY)):
        ctx.run(3)
    row = ctx.rows[-1]
    assert abs(row['asset.lid_relative.z']-HEIGHT) < .001
    assert math.hypot(row['asset.lid_relative.x'], row['asset.lid_relative.y']) < .001
    assert row['asset.lid_speed'] < .01
    ctx.experiment_details = {'aligned_final_plate_gap_m': row['asset.lid_relative.z']-HEIGHT,
        'offset_final_relative_m': [row['offset.lid_relative.'+a] for a in 'xyz'],
        'boundary': 'offset lid may tip, perch or slide through native contact; no automatic centering claim'}


def lid_lift_release(ctx: TestContext):
    ground(ctx)
    observe(ctx, ctx.asset)
    box, lid = observe_lid(ctx, ctx.asset)
    frames = [{'time': 0, 'force': (0, 0, 0)}]
    for start in (1., 2.5):
        frames.extend([{'time': start, 'force': (0, 0, 0)},
            {'time': start+.004, 'force': (0, 0, .30)},
            {'time': start+.120, 'force': (0, 0, .30)},
            {'time': start+.124, 'force': (0, 0, 0)}])
    ctx.force('lift', lid, keyframes=frames)
    ctx.camera(position=(.32, -.48, .30), target=(0, 0, .07))
    ctx.video_annotation = lambda: 'Fixed box | two 0.30 N upward pulses | release preserves velocity | gravity re-seating'
    ctx.plots('lift.force.z', 'asset.lid_relative.z', 'asset.lid_relative.x', 'asset.lid_speed')
    with ctx.fixed(box):
        ctx.run(4)
    cycles = []
    for lo, hi in [(1., 2.5), (2.5, 4.)]:
        rows = [r for r in ctx.rows if lo <= r['time'] <= hi+1e-8]
        peak = max(r['asset.lid_relative.z']-HEIGHT for r in rows)
        assert peak > .01
        assert abs(rows[-1]['asset.lid_relative.z']-HEIGHT) < .001
        assert rows[-1]['asset.lid_speed'] < .01
        cycles.append({'maximum_plate_gap_m': peak, 'final_plate_gap_m': rows[-1]['asset.lid_relative.z']-HEIGHT})
    ctx.experiment_details = {'cycles': cycles, 'boundary': 'pure vertical ideal pull; no grip adhesion, lateral re-placement or calibrated pull force'}


def lid_tilt_slide(ctx: TestContext):
    # Thin 2 mm plate impact needs a finer physical step; no hidden substeps.
    ctx.simulation(dt=min(ctx.dt, .002))
    ground(ctx)
    tilted = ctx.spawn(ctx.asset_path, name='tilted')
    inverted = ctx.spawn(ctx.asset_path, name='inverted')
    for instance, x, angle in [(ctx.asset, -.18, math.pi/3), (tilted, 0, 2*math.pi/3), (inverted, .18, math.pi)]:
        ctx.initial(instance, position=(x, 0, .16), rotation=(math.cos(angle/2), math.sin(angle/2), 0, 0))
        observe(ctx, instance); observe_lid(ctx, instance)
    ctx.camera(position=(.6, -.8, .55), target=(0, 0, .10))
    ctx.video_annotation = lambda: 'Fixed box at 60 / 120 / 180 degrees | gravity seats at 60, releases above 90 | no latch'
    ctx.plots('asset.lid_relative.z', 'tilted.lid_relative.z', 'tilted.lid_relative.y', 'inverted.lid_relative.z')
    with ctx.fixed(ctx.asset.body(BODY)), ctx.fixed(tilted.body(BODY)), ctx.fixed(inverted.body(BODY)):
        ctx.run(3)
    displacements = {}
    for name in ('asset', 'tilted', 'inverted'):
        first = np.array([ctx.rows[0][name+'.lid_relative.'+a] for a in 'xyz'])
        last = np.array([ctx.rows[-1][name+'.lid_relative.'+a] for a in 'xyz'])
        displacements[name] = float(np.linalg.norm(last-first))
    assert displacements['asset'] < .005
    assert displacements['tilted'] > .01
    assert displacements['inverted'] > .01
    ctx.experiment_details = {'lid_displacement_from_seat_m': displacements, 'geometry_contact': clearance(ctx),
        'boundary': 'initial fixed orientations; gravity presses plate into rim at 60 degrees, outward axial gravity above 90 degrees releases unlatched lid'}


TESTS.update(lid_seating=lid_seating, lid_lift_release=lid_lift_release, lid_tilt_slide=lid_tilt_slide)
