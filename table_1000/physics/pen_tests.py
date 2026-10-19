"""Experiments for pen base dynamics and native detent mechanisms.

Uses the common TestContext interface. Per-asset wrappers own masses, geometry
parameters and numerical steps; no asset-directory imports or simulator startup.
"""
import json
from math import cos, sin, pi
from table_1000.physics.simulation import rotation
import numpy as np
from scipy.spatial.transform import Rotation
from table_1000.physics.testing import TestContext
from table_1000.physics.test import ground_clearance
from table_1000.physics.joints import frames

def _observe(ctx, instance, is_click):
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    if is_click:
        ctx.observe_joint(instance, 'button_press')


def _points(ctx):
    model = json.loads((ctx.asset_path.parent/'model.json').read_text())
    worlds, points = {}, []
    for node in model['nodes']:
        world = worlds.get(node['parent'], np.eye(4)) @ np.array(node['matrix'])
        worlds[node['name']] = world
        if node.get('role') == 'collision':
            points.extend((np.c_[node['vertices'], np.ones(len(node['vertices']))] @ world.T)[:, :3])
    return np.array(points)


def _stable(ctx, instances):
    tail = [r for r in ctx.rows if r['time'] >= ctx.session.time-.5-1e-9]
    for instance in instances:
        for name in instance.body_names:
            prefix = instance.name+'.'+name
            velocity = np.array([[r[prefix+'.linear_velocity.'+a] for a in 'xyz'] for r in tail])
            position = np.array([[r[prefix+'.position.'+a] for a in 'xyz'] for r in tail])
            assert np.linalg.norm(velocity, axis=1).max() < .01, prefix+' did not settle'
            assert np.linalg.norm(np.ptp(position, axis=0)) < .001, prefix+' drifted'


def free_load(ctx: TestContext, masses, is_click):
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, -.035, .06))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .035, .06))
    instances = [ctx.asset, idle]
    if not is_click:
        # Start the independent cap completely clear of the nib, without a
        # fixture or artificial body/cap connection.
        from pxr import UsdGeom
        cache = UsdGeom.XformCache()
        for instance in instances:
            p = np.array(cache.GetLocalToWorldTransform(ctx.session.stage.GetPrimAtPath(instance.paths['cap'])).ExtractTranslation())
            ctx.initial(instance, position=(0, -.035 if instance is ctx.asset else .035, .06),
                        bodies={'cap': {'position': p+np.array([-.09, 0, 0])}})
    for instance in instances:
        _observe(ctx, instance, is_click)
    moving = 'pen.refill' if is_click else 'body'
    applied = .01 if is_click else masses[moving]*.5
    ctx.force('axial_load', ctx.asset.body(moving), keyframes=[
        {'time': 0, 'force': (0, 0, 0)},
        {'time': .2, 'force': (0, 0, 0)},
        {'time': .204, 'force': (-applied, 0, 0)},
        {'time': .284, 'force': (-applied, 0, 0)},
        {'time': .288, 'force': (0, 0, 0)},
    ])
    ctx.camera(position=(.16, -.36, .26), target=(-.06, 0, .06))
    ctx.video_annotation = lambda: 'Base only | lower: axial load then release | upper: idle | cap retention / click lock pending'
    ctx.plots('axial_load.force.x', 'asset.'+moving+'.position.x', 'asset.'+moving+'.linear_velocity.x')
    ctx.run(1.2)
    impulse = sum(r['axial_load.force.x'] for r in ctx.rows)*ctx.dt
    momentum = sum(masses[n]*ctx.asset.body(n).state['velocity'][0] for n in masses)
    error = abs(momentum-impulse)/abs(impulse)
    ctx.experiment_details = {'contract': 'free axial force / release / idle control with current base geometry',
        'mechanism_status': 'no cap retention or click lock', 'applied_impulse_N_s': impulse,
        'total_x_momentum_kg_m_s': float(momentum), 'relative_momentum_error': float(error),
        'geometry_pose_writes_after_initialization': False}
    assert error < .02, 'external impulse and total momentum disagree'
    assert max(np.linalg.norm(idle.body(n).state['velocity']) for n in masses) < 1e-5
    if is_click:
        shell = ctx.asset.body('pen.shell')
        assert shell.state['velocity'][0] < -.001, 'no reaction transmitted to free shell'
        from table_1000.physics.joints import frames
        q = frames(ctx.asset, 'button_press')['position']
        assert -.0031 <= q <= .0001, '3 mm slider travel violated'


def low_drop(ctx: TestContext, masses, is_click, dt, *, detent=False, click=False):
    # 4 ms diagnostic exceeded the fixed 2 mm geometric overlap contract.
    ctx.simulation(dt=dt)
    q = Rotation.from_euler('y', 20, degrees=True).as_quat()[[3, 0, 1, 2]]
    r = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    height = .020-float((_points(ctx) @ r.T)[:, 2].min())
    ctx.initial(position=(0, 0, height), rotation=tuple(q))
    ctx.ground(static_friction=.4, dynamic_friction=.3, contact_offset=.0001)
    _observe(ctx, ctx.asset, is_click)
    ctx.camera(position=(.22, -.42, .28), target=(0, 0, .045), focal_length=26)
    ctx.video_annotation = lambda: ('Native detent | ' if detent else 'Three-body click | ' if click else 'Base only | ') + 'lowest collision point 20 mm above ground | 20 deg pitch'
    ctx.plots(*['asset.'+name+'.position.z' for name in masses])
    ctx.run(3.2)
    contact = ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()), ctx.rows, ctx.session.instances, 0)
    ctx.experiment_details = {'contract': '20 mm low-height release, current geometry',
        'lowest_initial_clearance_m': .020, 'pitch_rad': float(np.deg2rad(20)),
        'initial_origin_height_m': height, 'geometry_contact': contact,
        'boundary': 'cap separation allowed; impact outcomes are not calibrated' if detent else 'cap separation allowed; does not verify cap holding or click state'}
    _stable(ctx, [ctx.asset])
    assert contact['maximum_geometry_penetration_m'] < .002, 'transient geometry penetration exceeds 2 mm'



def base_tests(*, masses, is_click, drop_dt):
    """Return independent force/release and low-drop experiment callables."""
    masses = dict(masses)

    def free(ctx):
        return free_load(ctx, masses, is_click)

    def drop(ctx):
        return low_drop(ctx, masses, is_click, drop_dt)

    tests = {'free_load': free, 'low_drop': drop}
    if is_click:
        tests['slider_limits'] = click_slider_limits
    return tests


def click_slider_limits(ctx: TestContext):
    """Load both existing slider stops twice; this adds no click mechanism."""
    ctx.simulation(gravity=(0, 0, 0))
    ctx.initial(position=(0, 0, .06), joints={'button_press': -.0015})
    _observe(ctx, ctx.asset, True)
    ctx.camera(position=(.12, -.38, .24), target=(0, 0, .06))
    phase = {'text': 'mid-stroke initial state'}
    ctx.video_annotation = lambda: 'Base 3 mm slider | ' + phase['text'] + ' | no return spring or lock'
    ctx.plots('asset.button_press.position', 'asset.button_press.velocity',
              'asset.pen.shell.position.x', 'asset.pen.refill.position.x')
    ctx.run(.2)
    records = []
    for cycle in range(2):
        for sign, target in [(-1, -.003), (1, 0.)]:
            start = ctx.session.time
            phase['text'] = f'cycle {cycle+1}: opposing 0.02 N loads toward {target*1000:g} mm stop'
            for name, direction in [('pen.refill', sign), ('pen.shell', -sign)]:
                ctx.force(f'cycle{cycle+1}_{sign}_{name}', ctx.asset.body(name), keyframes=[
                    {'time': start, 'force': (0, 0, 0)},
                    {'time': start+.004, 'force': (direction*.02, 0, 0)},
                    {'time': start+.396, 'force': (direction*.02, 0, 0)},
                    {'time': start+.4, 'force': (0, 0, 0)}])
            ctx.run(.4)
            loaded = frames(ctx.asset, 'button_press')['position']
            assert abs(loaded-target) < .0001, 'existing slider did not reach stop'
            phase['text'] = f'cycle {cycle+1}: both loads released'
            ctx.run(.2)
            records.append({'cycle': cycle+1, 'target_m': target,
                            'loaded_position_m': loaded,
                            'released_position_m': frames(ctx.asset, 'button_press')['position']})
    q = [row['asset.button_press.position'] for row in ctx.rows]
    assert min(q) >= -.0031 and max(q) <= .0001, '3 mm slider travel violated'
    ctx.experiment_details = {'contract': 'two bidirectional cycles against existing 3 mm travel stops',
        'equal_opposite_applied_load_N': .02, 'cycles': records,
        'minimum_position_m': min(q), 'maximum_position_m': max(q),
        'boundary': 'free shell, no gravity or fixtures; no return spring or bistable state added',
        'geometry_pose_writes_after_initialization': False}


def quaternion(angle, axis=(0,1,0)):
    axis = np.asarray(axis,dtype=float); axis /= np.linalg.norm(axis)
    return np.r_[cos(angle/2),axis*sin(angle/2)]


def _click_observe(ctx, instance):
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    for name in ['button_press', 'refill_slide']:
        ctx.observe_joint(instance, name)
    for name in ['extended', 'armed', 'toggle_count', 'callback_count']:
        ctx.observe(instance.name+'.click.'+name,
                    lambda n=name: getattr(instance.behavior(), n), unit='1')


def click_cycles(ctx, *, initial_extended=False):
    """Full press/hold/release, shallow press, then second full press/release."""
    ctx.simulation(dt=.001, gravity=(0, 0, 0))
    if initial_extended:
        ctx.session.stage.GetPrimAtPath(ctx.asset.path).GetAttribute('table1000:click:initialExtended').Set(True)
    ctx.initial(position=(0, 0, .05), joints={'refill_slide': -.003 if initial_extended else 0})
    _click_observe(ctx, ctx.asset)
    ctx.force('button_load', ctx.asset.body('pen.button'), keyframes=[
        {'time': 0, 'force': (0, 0, 0)}, {'time': .2, 'force': (0, 0, 0)},
        {'time': .5, 'force': (-1.2, 0, 0)}, {'time': 1., 'force': (-1.2, 0, 0)},
        {'time': 1.2, 'force': (0, 0, 0)}, {'time': 1.7, 'force': (0, 0, 0)},
        {'time': 1.9, 'force': (-.3, 0, 0)}, {'time': 2.1, 'force': (-.3, 0, 0)},
        {'time': 2.3, 'force': (0, 0, 0)}, {'time': 2.7, 'force': (0, 0, 0)},
        {'time': 3., 'force': (-1.2, 0, 0)}, {'time': 3.4, 'force': (-1.2, 0, 0)},
        {'time': 3.6, 'force': (0, 0, 0)}, {'time': 4.2, 'force': (0, 0, 0)}])
    ctx.camera(position=(.10, -.31, .20), target=(0, 0, .05), focal_length=38)
    ctx.video_annotation = lambda: 'Three-body click | full press/hold/release, shallow control, second full press | external button load'
    ctx.plots('button_load.force.x', 'asset.button_press.position', 'asset.refill_slide.position',
              'asset.click.toggle_count', 'asset.click.extended')
    with ctx.fixed(ctx.asset.body('pen.shell')):
        ctx.run(4.2)
    def at(t):
        return min(ctx.rows, key=lambda row: abs(row['time']-t))
    first = -.003 if not initial_extended else 0.
    original = -.003 if initial_extended else 0.
    for t, count, target in [(1., 0, original), (1.6, 1, first), (2.6, 1, first), (4.2, 2, original)]:
        row = at(t)
        assert row['asset.click.toggle_count'] == count, (t, row['asset.click.toggle_count'])
        assert abs(row['asset.refill_slide.position']-target) < .0001, (t, row['asset.refill_slide.position'])
    assert abs(at(4.2)['asset.button_press.position']) < .0001
    assert min(at(t)['asset.button_press.position'] for t in [2., 2.1]) > -.0027
    for joint in ['button_press', 'refill_slide']:
        values = [r['asset.'+joint+'.position'] for r in ctx.rows]
        assert min(values) >= -.0031 and max(values) <= .0001
    behavior = ctx.asset.behavior()
    assert behavior.error is None and behavior.callback_count == ctx.session.steps
    ctx.experiment_details = {'contract': 'two full press/release toggles, sustained hold no repeats and shallow-press control',
        'initial_extended': initial_extended, 'button_full_load_N': 1.2, 'shallow_load_N': .3,
        'checkpoints': [{k: at(t)[k] for k in ['time', 'asset.button_press.position', 'asset.refill_slide.position', 'asset.click.toggle_count']} for t in [1.,1.6,2.6,4.2]],
        'geometry_pose_writes_after_initialization': False,
        'boundary': 'ideal shell clamp; position-triggered approximation, uncalibrated gains, not a mechanical ratchet'}


def click_free(ctx, masses):
    ctx.simulation(dt=.001, gravity=(0, 0, 0))
    ctx.initial(position=(0, -.025, .05))
    idle = ctx.spawn(ctx.asset_path, name='idle', position=(0, .025, .05))
    for instance in [ctx.asset, idle]:
        _click_observe(ctx, instance)
    ctx.force('button_load', ctx.asset.body('pen.button'), keyframes=[
        {'time':0., 'force':(0,0,0)}, {'time':.2, 'force':(0,0,0)},
        {'time':.204, 'force':(-.02,0,0)}, {'time':.284, 'force':(-.02,0,0)},
        {'time':.288, 'force':(0,0,0)}, {'time':1.2, 'force':(0,0,0)}])
    ctx.camera(position=(.16, -.36, .26), target=(-.10, 0, .05), focal_length=26)
    ctx.video_annotation = lambda: 'Three-body click | lower free button 0.02 N, upper idle | reaction test below trigger load'
    ctx.plots('button_load.force.x', 'asset.pen.shell.linear_velocity.x',
              'asset.button_press.position', 'asset.refill_slide.position', 'asset.click.toggle_count')
    ctx.run(1.2)
    impulse = sum(r['button_load.force.x'] for r in ctx.rows)*ctx.dt
    momentum = sum(masses[n]*ctx.asset.body(n).state['velocity'][0] for n in masses)
    error = abs(momentum-impulse)/abs(impulse)
    assert error < .02
    assert ctx.asset.body('pen.shell').state['velocity'][0] < -.001
    assert max(np.linalg.norm(idle.body(n).state['velocity']) for n in masses) < 1e-5
    assert ctx.asset.behavior().toggle_count == 0 and idle.behavior().toggle_count == 0
    assert abs(frames(ctx.asset,'refill_slide')['position']) < .0001
    ctx.experiment_details = {'contract': 'free subtrigger button loading/release, native shell reaction, idle instance',
        'applied_impulse_N_s': impulse, 'total_x_momentum_kg_m_s': float(momentum),
        'relative_momentum_error': float(error), 'toggle_count': ctx.asset.behavior().toggle_count,
        'idle_toggle_count': idle.behavior().toggle_count, 'geometry_pose_writes_after_initialization': False}


def click_tests(*, masses, drop_dt):
    """Independent-button approved approximation, with per-ID masses and dt."""
    def cycles(ctx):
        return click_cycles(ctx)
    def extended(ctx):
        return click_cycles(ctx, initial_extended=True)
    def free(ctx):
        return click_free(ctx, masses)
    def drop(ctx):
        _click_observe(ctx, ctx.asset)
        low_drop(ctx, masses, True, drop_dt, click=True)
        ctx.experiment_details['click_final_extended'] = ctx.asset.behavior().extended
        ctx.experiment_details['click_toggle_count'] = ctx.asset.behavior().toggle_count
        ctx.experiment_details['boundary'] = '20 mm macro release; impact/false-trigger resilience not calibrated'
    return {'click_cycles': cycles, 'initial_extended': extended, 'free_press': free, 'low_drop': drop}


class Hand:
    """Native compliant grasp, with SI gains and explicit release."""
    def __init__(self,ctx,body,name):
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        ctx.ensure_initialized()
        self.ctx,self.body = ctx,body
        self.position = body.state['position'].copy()
        self.quaternion = body.state['quaternion'].copy()
        self.path = '/World/Fixtures/'+name
        hand = UsdGeom.Xform.Define(ctx.session.stage,self.path)
        self.ops = [hand.AddTranslateOp(),hand.AddOrientOp()]
        UsdPhysics.RigidBodyAPI.Apply(hand.GetPrim()).CreateKinematicEnabledAttr(True)
        PhysxSchema.PhysxRigidBodyAPI.Apply(hand.GetPrim()).CreateEnableCCDAttr(False)
        self.joint = UsdPhysics.Joint.Define(ctx.session.stage,self.path+'/grip')
        self.joint.CreateBody0Rel().SetTargets([self.path]);self.joint.CreateBody1Rel().SetTargets([body.path])
        for i,axis in enumerate(['transX','transY','transZ','rotX','rotY','rotZ']):
            drive=UsdPhysics.DriveAPI.Apply(self.joint.GetPrim(),axis)
            factor=1 if i<3 else pi/180
            drive.CreateTypeAttr('force');drive.CreateTargetPositionAttr(0)
            drive.CreateStiffnessAttr((2000 if i<3 else 20)*factor)
            drive.CreateDampingAttr((40 if i<3 else .4)*factor)
            drive.CreateMaxForceAttr(30 if i<3 else 1)
        self.move()

    def move(self,position=None,quaternion=None):
        from pxr import Gf
        if position is not None:self.position=np.asarray(position,dtype=float)
        if quaternion is not None:self.quaternion=np.asarray(quaternion,dtype=float)
        self.ops[0].Set(Gf.Vec3d(*self.position))
        self.ops[1].Set(Gf.Quatf(float(self.quaternion[0]),Gf.Vec3f(*map(float,self.quaternion[1:]))))

    def axial(self,stiffness=0,damping=0):
        from pxr import UsdPhysics
        drive=UsdPhysics.DriveAPI(self.joint.GetPrim(),'transX')
        drive.GetStiffnessAttr().Set(stiffness);drive.GetDampingAttr().Set(damping)

    def release(self,released=True):
        self.joint.GetJointEnabledAttr().Set(not released)



def cap_fit(instance, seat):
    """Return opening (m), transverse offset (m), and axis cosine (1)."""
    b, c = instance.body('body').state, instance.body('cap').state
    local = b['rotation'].T @ (c['position']-b['position'])
    return seat-local[0], float(np.linalg.norm(local[1:])), float(b['rotation'][:, 0] @ c['rotation'][:, 0])


def initial_cap(ctx, instance, position, opening=0, pose=(1, 0, 0, 0)):
    """Place cap and all its followers together, before physics starts."""
    from pxr import UsdGeom
    ctx.initial(instance, position=position, rotation=pose)
    cache = UsdGeom.XformCache()
    axis = rotation(pose)[:, 0]
    overrides = {name: {'position': np.array(cache.GetLocalToWorldTransform(
        ctx.session.stage.GetPrimAtPath(instance.paths[name])).ExtractTranslation())-axis*opening}
        for name in instance.body_names if name == 'cap' or name.startswith('cap_ball')}
    ctx.initial(instance, position=position, rotation=pose, bodies=overrides)


def observe_cap(ctx, instance, seat, count):
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    for index, (name, unit) in enumerate((('opening', 'm'), ('radial', 'm'), ('axis_cos', '1'))):
        ctx.observe(instance.name+'.'+name, lambda i=index: cap_fit(instance, seat)[i], unit=unit)
    for i in range(1, count+1):
        ctx.observe(instance.name+'.ball'+str(i), lambda i=i: frames(instance, 'cap.detent'+str(i))['position'], unit='m')



def _align_video(ctx):
    """Include the final state on the 25 FPS grid, at most 39 ms later."""
    stride = round(.04 / ctx.dt)
    steps = (-ctx.session.steps) % stride
    if steps:
        ctx.run(steps * ctx.dt)


def _cap_setup(ctx, detent, *, gravity=(0, 0, -9.81)):
    ctx.simulation(dt=detent['dt'], gravity=gravity)


def _cap_loads(ctx, bodies):
    values = {name: np.zeros(3) for name in bodies}
    original = ctx.session.actions

    def actions(t, dt):
        original(t, dt)
        for name, body in bodies.items():
            ctx.session.force(body, values[name])

    ctx.session.actions = actions
    for name in bodies:
        ctx.observe(name + '.applied_force_x', lambda n=name: values[n][0], unit='N')
    return values


def cap_pull(ctx, detent, protocol):
    """Tight/loose force ramp; catch only after 45 mm opening, then place."""
    _cap_setup(ctx, detent)
    seat, count = detent['seat'], detent['balls']
    loose = ctx.spawn(ctx.asset_path, name='loose')
    instances = [ctx.asset, loose]
    for instance, y, opening in zip(instances, [.035, -.035], [0, .003]):
        initial_cap(ctx, instance, (0, y, .06), opening=opening)
        observe_cap(ctx, instance, seat, count)
    ctx.ground(static_friction=.4, dynamic_friction=.3, contact_offset=.0001)
    guides = [Hand(ctx, a.body('cap'), 'pull_' + a.name) for a in instances]
    for hand in guides:
        hand.axial()
    forces = _cap_loads(ctx, {a.name: a.body('cap') for a in instances})
    ctx.camera(position=(-.04, -.46, .28), target=(-.04, 0, .03))
    ctx.video_annotation = lambda: 'Far: seated | near: 3 mm loose | applied axial force; catch after full separation'
    ctx.plots('asset.applied_force_x', 'loose.applied_force_x', 'asset.opening', 'loose.opening', 'asset.ball1')
    records = [None, None]
    with ctx.fixed(ctx.asset.body('body')), ctx.fixed(loose.body('body')):
        ctx.run(.4)
        initial_fits = [cap_fit(a, seat) for a in instances]
        weak_fits = None
        for k in range(round(8 / ctx.dt)):
            force = min(protocol['max_pull_N'] * k * ctx.dt / 6, protocol['max_pull_N'])
            for i, instance in enumerate(instances):
                if records[i] is None:
                    forces[instance.name] = np.array([-force, 0, 0])
            ctx.step()
            if weak_fits is None and force >= protocol['weak_pull_N']:
                weak_fits = cap_fit(ctx.asset, seat)
            for i, instance in enumerate(instances):
                fit = cap_fit(instance, seat)
                if records[i] is None and fit[0] > .045:
                    records[i] = {'time_s': ctx.session.time, 'last_applied_force_N': force, 'fit': fit}
                    forces[instance.name] = np.zeros(3)
                    guides[i].move(instance.body('cap').state['position'])
                    guides[i].axial(2000, 40)
            if all(records):
                break
        for name in forces:
            forces[name] = np.zeros(3)
        ctx.experiment_details = {'contract': 'tight/loose force ramp, catch after full separation and place',
            'settled_initial_fit': initial_fits, 'weak_pull_N': protocol['weak_pull_N'],
            'weak_pull_fit': weak_fits, 'separation': records,
            'full_separation_opening_m': .045,
            'guide': 'zero axial stiffness/damping while pulling; native D6 catch only after separation',
            'force_measurement': 'applied external load, not contact or constraint reaction'}
        assert all(records), 'a cap did not separate within the fixed force ceiling'
        assert abs(initial_fits[0][0]) < .0005
        assert weak_fits is not None and abs(weak_fits[0]) < .0005
        assert records[0]['last_applied_force_N'] > 3 * records[1]['last_applied_force_N']
        ctx.run(.4)
        # Place using a collision-free compliant hand; preserve body velocity
        # and pose when releasing the native grasp.
        starts = [h.body.state['position'].copy() for h in guides]
        cap_radius = protocol['cap_half_height_m']
        targets = [np.array([p[0], p[1], cap_radius + .0003]) for p in starts]
        for k in range(round(1.2 / ctx.dt)):
            fraction = (k + 1) * ctx.dt / 1.2
            fraction = fraction * fraction * (3 - 2 * fraction)
            for hand, start, target in zip(guides, starts, targets):
                hand.move(start + fraction * (target - start))
            ctx.step()
        ctx.run(.4)
        for hand in guides:
            hand.release()
        settling = protocol.get('placement_settle_s', 2.4)
        ctx.run(settling)
        _align_video(ctx)
        ctx.experiment_details['placement_settle_s'] = settling
        ctx.experiment_details['placed_positions_m'] = [a.body('cap').state['position'].tolist() for a in instances]
        ctx.experiment_details['placed_speed_m_s'] = [float(np.linalg.norm(a.body('cap').state['velocity'][:3])) for a in instances]
        assert max(ctx.experiment_details['placed_speed_m_s']) < .01
        _stable(ctx, instances)


def cap_close(ctx, detent, protocol):
    """Weak push must stop; a stronger push seats; all cap fixtures release."""
    _cap_setup(ctx, detent)
    seat, count = detent['seat'], detent['balls']
    initial_cap(ctx, ctx.asset, (0, 0, .06), opening=.003)
    observe_cap(ctx, ctx.asset, seat, count)
    guide = Hand(ctx, ctx.asset.body('cap'), 'close')
    guide.axial(0, 10)
    forces = _cap_loads(ctx, {'push': ctx.asset.body('cap')})
    ctx.camera(position=(-.03, -.30, .20), target=(-.025, 0, .06))
    ctx.video_annotation = lambda: 'Light +X push stops before ridge | ramp seats cap | cap guide and load release'
    ctx.plots('push.applied_force_x', 'asset.opening', 'asset.radial', 'asset.ball1')
    with ctx.fixed(ctx.asset.body('body')):
        forces['push'] = np.array([protocol['light_push_N'], 0, 0])
        ctx.run(2.)
        weak = cap_fit(ctx.asset, seat)
        tail = [r['asset.opening'] for r in ctx.rows if r['time'] >= ctx.session.time - .4 - 1e-9]
        drift = max(tail) - min(tail)
        seated = None
        for k in range(round(4 / ctx.dt)):
            force = protocol['light_push_N'] + (protocol['max_push_N'] - protocol['light_push_N']) * k * ctx.dt / 4
            forces['push'] = np.array([force, 0, 0])
            ctx.step()
            if cap_fit(ctx.asset, seat)[0] < .0003:
                seated = {'time_s': ctx.session.time, 'last_applied_force_N': force, 'fit': cap_fit(ctx.asset, seat)}
                break
        forces['push'] = np.zeros(3)
        guide.release()
        ctx.run(1.2)
        _align_video(ctx)
        final = cap_fit(ctx.asset, seat)
        ctx.experiment_details = {'contract': 'light push stops, strong push seats, cap load and guide release',
            'light_push_N': protocol['light_push_N'], 'weak_fit': weak,
            'weak_last_0_4s_opening_drift_m': drift, 'seated': seated,
            'released_final_fit': final, 'released_speed_m_s': float(np.linalg.norm(ctx.asset.body('cap').state['velocity'][:3])),
            'closing_guide_axial_damping_N_s_m': 10., 'force_measurement': 'applied external force'}
        assert .0008 < weak[0] < .0028 and drift < .00015, 'weak push did not stop before the ridge'
        assert seated is not None, 'strong push did not seat within the fixed force ceiling'
        assert abs(final[0]) < .0005 and final[1] < .0005 and final[2] > .98
        assert ctx.experiment_details['released_speed_m_s'] < .01


def cap_gravity(ctx, detent, protocol):
    """Only grasp bodies while turning cap-up to cap-down; loose control falls."""
    _cap_setup(ctx, detent)
    seat, count = detent['seat'], detent['balls']
    pose = quaternion(pi / 2)
    loose = ctx.spawn(ctx.asset_path, name='loose')
    instances = [ctx.asset, loose]
    for instance, x, opening in zip(instances, [-.11, .11], [0, .003]):
        initial_cap(ctx, instance, (x, 0, .25), opening=opening, pose=pose)
        observe_cap(ctx, instance, seat, count)
    ctx.ground(static_friction=.4, dynamic_friction=.3, contact_offset=.0001)
    hands = [Hand(ctx, a.body('body'), a.name + '_body') for a in instances]
    ctx.camera(position=(0, -1.12, .42), target=(0, 0, .18), focal_length=24)
    ctx.video_annotation = lambda: 'Left: seated | right: loose | compliant body grips only | rotate cap-up to cap-down'
    ctx.plots('asset.opening', 'loose.opening', 'asset.radial', 'asset.ball1')
    ctx.run(.6)
    for k in range(round(6 / ctx.dt)):
        for hand in hands:
            hand.move(quaternion=quaternion(pi / 2 - pi * (k + 1) * ctx.dt / 6))
        ctx.step()
    ctx.run(1.4)
    fits = [cap_fit(a, seat) for a in instances]
    extrema = {'maximum_abs_opening_m': max(abs(r['asset.opening']) for r in ctx.rows),
               'maximum_radial_m': max(r['asset.radial'] for r in ctx.rows),
               'minimum_axis_cosine': min(r['asset.axis_cos'] for r in ctx.rows)}
    ctx.experiment_details = {'contract': 'gravity retention throughout the turn with loose control', 'final_fit': fits,
        'tight_extrema': extrema,
        'fixtures': 'native compliant body grips only; no cap fixture or hidden coupling'}
    assert extrema['maximum_abs_opening_m'] < .0005 and extrema['maximum_radial_m'] < .0005 and extrema['minimum_axis_cosine'] > .98
    assert fits[1][0] > .03


def cap_free_reaction(ctx, detent, masses):
    """Load a seated free cap: the body must react through the native detent."""
    _cap_setup(ctx, detent, gravity=(0, 0, 0))
    seat, count = detent['seat'], detent['balls']
    idle = ctx.spawn(ctx.asset_path, name='idle')
    for instance, y in [(ctx.asset, -.035), (idle, .035)]:
        initial_cap(ctx, instance, (0, y, .06))
        observe_cap(ctx, instance, seat, count)
    masses = dict(masses)
    masses.update({f'cap_ball{i}': detent['mass'] for i in range(1, count + 1)})
    force = .02
    ctx.force('cap_load', ctx.asset.body('cap'), keyframes=[
        {'time': 0, 'force': (0, 0, 0)}, {'time': .2, 'force': (0, 0, 0)},
        {'time': .204, 'force': (-force, 0, 0)}, {'time': .284, 'force': (-force, 0, 0)},
        {'time': .288, 'force': (0, 0, 0)}])
    ctx.camera(position=(.05, -.60, .34), target=(-.10, 0, .06), focal_length=24)
    ctx.video_annotation = lambda: 'Lower: seated cap receives force then releases | upper: idle | no fixtures or gravity'
    ctx.plots('cap_load.force.x', 'asset.body.position.x', 'asset.cap.position.x', 'asset.opening')
    ctx.run(1.2)
    impulse = sum(r['cap_load.force.x'] for r in ctx.rows) * ctx.dt
    momentum = sum(masses[n] * ctx.asset.body(n).state['velocity'][0] for n in masses)
    idle_v = sum(masses[n] * idle.body(n).state['velocity'][:3] for n in masses) / sum(masses.values())
    error = abs(momentum - impulse) / abs(impulse)
    ctx.experiment_details = {'contract': 'free seated cap load and release, native body reaction, idle instance',
        'applied_impulse_N_s': impulse, 'total_x_momentum_kg_m_s': float(momentum),
        'relative_momentum_error': float(error), 'idle_center_of_mass_velocity_m_s': idle_v.tolist(),
        'body_reaction_velocity_x_m_s': float(ctx.asset.body('body').state['velocity'][0]),
        'final_fit': cap_fit(ctx.asset, seat), 'masses_kg': masses,
        'geometry_pose_writes_after_initialization': False}
    assert error < .02
    assert np.linalg.norm(idle_v) < 1e-5
    assert ctx.asset.body('body').state['velocity'][0] < -.001
    assert abs(cap_fit(ctx.asset, seat)[0]) < .0005


def detent_tests(*, masses, detent, protocol):
    """Native cap contracts; caller freezes per-ID parameters and load limits."""
    masses, detent, protocol = dict(masses), dict(detent), dict(protocol)

    def pull(ctx):
        return cap_pull(ctx, detent, protocol)

    def close(ctx):
        return cap_close(ctx, detent, protocol)

    def gravity(ctx):
        return cap_gravity(ctx, detent, protocol)

    def reaction(ctx):
        return cap_free_reaction(ctx, detent, masses)

    def drop(ctx):
        all_masses = {**masses, **{f'cap_ball{i}': detent['mass'] for i in range(1, detent['balls'] + 1)}}
        observe_cap(ctx, ctx.asset, detent['seat'], detent['balls'])
        low_drop(ctx, all_masses, False, detent['dt'], detent=True)
        ctx.plots(*ctx.plot_columns, 'asset.opening')
        ctx.experiment_details['final_cap_fit'] = cap_fit(ctx.asset, detent['seat'])
        ctx.experiment_details['maximum_abs_cap_opening_m'] = max(abs(r['asset.opening']) for r in ctx.rows)

    def cycles(ctx):
        return cap_cycles(ctx, detent, protocol)

    return {'pull_comparison': pull, 'close': close, 'gravity_rotation': gravity,
            'cycles': cycles, 'free_reaction': reaction, 'low_drop': drop}

def cap_cycles(ctx, detent, protocol):
    """Three physical detach/reinsert cycles without resetting the asset."""
    _cap_setup(ctx, detent)
    seat, count = detent['seat'], detent['balls']
    initial_cap(ctx, ctx.asset, (0, 0, .06))
    observe_cap(ctx, ctx.asset, seat, count)
    cap = ctx.asset.body('cap')
    guide = Hand(ctx, cap, 'cycle_cap')
    guide.axial()
    forces = _cap_loads(ctx, {'cycle': cap})
    ctx.camera(position=(-.04, -.34, .23), target=(-.04, 0, .06))
    state = {'cycle': 0, 'phase': 'settle'}
    closing_damping = protocol.get('closing_damping_N_s_m', 10.)
    insertion_ramp = protocol.get('insertion_ramp_s', 2.)
    insertion_timeout = protocol.get('insertion_timeout_s', 3.)
    ctx.video_annotation = lambda: f'Native detent | cycle {state["cycle"]}/3 | {state["phase"]} | no state reset'
    ctx.plots('cycle.applied_force_x', 'asset.opening', 'asset.radial', 'asset.ball1')
    records = []
    ctx.experiment_details = {'contract': 'three complete detach/reinsert cycles without resetting physics',
        'cycles': records, 'pose_writes_after_initialization': False,
        'closing_damping_N_s_m': closing_damping,
        'insertion_ramp_s': insertion_ramp, 'insertion_timeout_s': insertion_timeout,
        'fixture': 'native D6 hand; axial free for pull, viscous guide for close, full release after seating'}
    with ctx.fixed(ctx.asset.body('body')):
        ctx.run(.4)
        for cycle in range(3):
            state.update(cycle=cycle + 1, phase='pull')
            guide.move(cap.state['position'], cap.state['quaternion'])
            guide.axial()
            guide.release(False)
            record = {'index': cycle + 1, 'starting_fit': cap_fit(ctx.asset, seat)}
            records.append(record)
            separated = False
            for k in range(round(3 / ctx.dt)):
                force = min(protocol['max_pull_N'] * k * ctx.dt / 1.2, protocol['max_pull_N'])
                forces['cycle'] = np.array([-force, 0, 0])
                ctx.step()
                if cap_fit(ctx.asset, seat)[0] > .045:
                    record['separation_applied_force_N'] = force
                    record['separated_fit'] = cap_fit(ctx.asset, seat)
                    separated = True
                    break
            forces['cycle'] = np.zeros(3)
            assert separated, 'repeated pull did not completely separate within force ceiling'
            guide.move(cap.state['position'])
            guide.axial(2000, 40)
            ctx.run(.12)
            state['phase'] = 'return to 3 mm opening'
            start = cap.state['position'].copy()
            body = ctx.asset.body('body').state
            target = body['position'] + body['rotation'] @ np.array([seat - .003, 0, 0])
            for k in range(round(.6 / ctx.dt)):
                fraction = (k + 1) * ctx.dt / .6
                fraction = fraction * fraction * (3 - 2 * fraction)
                guide.move(start + fraction * (target - start))
                ctx.step()
            ctx.run(.16)
            state['phase'] = 'light push'
            guide.axial(0, closing_damping)
            forces['cycle'] = np.array([protocol['light_push_N'], 0, 0])
            ctx.run(.8)
            record['weak_fit'] = cap_fit(ctx.asset, seat)
            assert record['weak_fit'][0] > .0008, 'light repeated insertion bypassed the detent'
            state['phase'] = 'strong push'
            seated = False
            for k in range(round(insertion_timeout / ctx.dt)):
                force = protocol['light_push_N'] + (protocol['max_push_N'] - protocol['light_push_N']) * min(k * ctx.dt / insertion_ramp, 1.)
                forces['cycle'] = np.array([force, 0, 0])
                ctx.step()
                if cap_fit(ctx.asset, seat)[0] < .0003:
                    record['seating_applied_force_N'] = force
                    seated = True
                    break
            forces['cycle'] = np.zeros(3)
            guide.release()
            state['phase'] = 'all cap fixtures released'
            ctx.run(.6)
            record['released_fit'] = cap_fit(ctx.asset, seat)
            assert seated, 'repeated insertion did not seat within force ceiling'
            fit = record['released_fit']
            assert abs(fit[0]) < .0005 and fit[1] < .0005 and fit[2] > .98
        _align_video(ctx)
