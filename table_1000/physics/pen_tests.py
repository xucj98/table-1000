"""Experiments for pen base dynamics and native detent mechanisms.

Uses the common TestContext interface. Per-asset wrappers own masses, geometry
parameters and numerical steps; no asset-directory imports or simulator startup.
"""
import json
import re
from math import cos, sin, pi
from table_1000.physics.simulation import rotation
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from table_1000.physics.testing import TestContext
from table_1000.physics.test import ground_clearance
from table_1000.physics.joints import frames


def _environment_observers(ctx, instance):
    """Record all physical environments; the first observer refreshes one cache."""
    prefix = 'environment_refresh.'+instance.name
    if prefix in ctx.observers:
        return
    cache = {}
    def refresh():
        from table_1000.physics.simulation import rotation
        p, r, com = instance.environment_states()
        v = instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs,-1,6)
        cache.update(position=p, rotation=r, velocity=v)
        cache['quaternion'] = Rotation.from_matrix(r.reshape(-1,3,3)).as_quat().reshape(ctx.num_envs,-1,4)[...,[3,0,1,2]]
        return 0
    ctx.observe(prefix,refresh,unit='1')
    for e in range(ctx.num_envs):
        for name,i in instance.indices.items():
            for field,unit,axes in [('position','m','xyz'),('quaternion','1','wxyz'),('velocity','m/s','xyz')]:
                for j,axis in enumerate(axes):
                    ctx.observe(f'env{e}.{instance.name}.{name}.{field}.{axis}',
                        lambda e=e,i=i,field=field,j=j:cache[field][e,i,j],unit=unit)


def _body_positions(body):
    return body.instance.environment_states()[0][:,body.instance.indices[body.name]]


def _support_points(ctx, body):
    """Collision vertices expressed in the held body's coordinates."""
    model=json.loads((ctx.asset_path.parent/'model.json').read_text())
    worlds,owners,points={},{},[]
    for node in model['nodes']:
        name=node['name'];parent=node['parent']
        worlds[name]=worlds.get(parent,np.eye(4))@np.array(node['matrix'])
        owners[name]=name if node['rigid'] else owners.get(parent)
        if node.get('role')=='collision' and owners[name]==body.name:
            transform=np.linalg.inv(worlds[body.name])@worlds[name]
            points.extend((np.c_[node['vertices'],np.ones(len(node['vertices']))]@transform.T)[:,:3])
    return np.array(points)


def _support_height(ctx, body, rotations=None):
    """Ground-touching origin height from the actual authored collision hulls."""
    rotations = (body.instance.environment_states()[1][:,body.instance.indices[body.name]]
                 if rotations is None else rotations)
    return -np.einsum('eij,pj->epi', rotations, _support_points(ctx, body))[:,:,2].min(axis=1)


def _support_face(ctx, body):
    """Nearest axial roll with a finite support face beneath the cap COM.

    The compliant native grasp performs the rotation during its existing
    placement segment; this function never writes a body pose.
    """
    from scipy.spatial import ConvexHull
    points = _support_points(ctx, body)
    section = np.unique(points[:, 1:], axis=0)
    hull = section[ConvexHull(section).vertices]
    com = body.instance.local_com[body.instance.indices[body.name]]
    candidates = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        dy, dz = b-a
        for theta in (np.arctan2(-dz,dy), np.arctan2(-dz,dy)+pi):
            theta = (theta+pi) % (2*pi)-pi
            r = Rotation.from_rotvec([theta,0,0]).as_matrix()
            world = points @ r.T
            edge = np.c_[np.zeros(2), np.array([a,b])] @ r.T
            low, high = sorted(edge[:,1])
            center = r @ com
            if (high-low > 1e-6 and abs(edge[:,2].mean()-world[:,2].min()) < 1e-7
                    and low+1e-7 < center[1] < high-1e-7):
                candidates.append((abs(theta),theta,r,low,high,float(center[1])))
    _, theta, r, low, high, center = min(candidates,key=lambda item:(item[0],item[1]))
    return Rotation.from_matrix(r).as_quat()[[3,0,1,2]], {
        'roll_rad':float(theta), 'support_y_interval_m':[float(low),float(high)],
        'cap_com_y_m':center, 'authored_cap_collision_vertices':len(points)}


def cap_fits(instance, seat):
    p,r,_ = instance.environment_states()
    b,c = instance.indices['body'],instance.indices['cap']
    local = np.einsum('eji,ej->ei',r[:,b],p[:,c]-p[:,b])
    return np.c_[seat-local[:,0],np.linalg.norm(local[:,1:],axis=1),
                 np.einsum('ei,ei->e',r[:,b,:,0],r[:,c,:,0])]


def _checked(test, name, masses, *, click=False):
    def experiment(ctx):
        test(ctx)
        checks=[]
        for e in range(ctx.num_envs):
            prefix=f'env{e}.'
            rows=[{'time':r['time'],**{k[len(prefix):]:v for k,v in r.items() if k.startswith(prefix)}} for r in ctx.rows]
            last=rows[-1];entry={'environment':e,'samples':len(rows)}
            if name=='low_drop':
                contact=ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()),rows,ctx.session.instances,0)
                entry['geometry_contact']=contact
                assert contact['maximum_geometry_penetration_m']<.002,(e,contact)
            if name in ['low_drop','pull_comparison']:
                tail=[r for r in rows if r['time']>=rows[-1]['time']-.5-1e-9]
                maximum=span=0.
                for instance in ctx.session.instances.values():
                    for body in instance.body_names:
                        p=f'{instance.name}.{body}.'
                        maximum=max(maximum,max(np.linalg.norm([r[p+'velocity.'+a] for a in 'xyz']) for r in tail))
                        span=max(span,float(np.linalg.norm(np.ptp([[r[p+'position.'+a] for a in 'xyz'] for r in tail],axis=0))))
                entry.update(tail_max_speed_m_s=maximum,tail_position_span_m=span)
                assert maximum<.01 and span<.001,(e,entry)
            if name in ['free_press','free_reaction']:
                force='button_load.force.x' if click else 'cap_load.force.x'
                impulse=sum(r[force] for r in ctx.rows)*ctx.dt
                momentum=sum(m*last[f'asset.{n}.velocity.x'] for n,m in masses.items())
                error=abs(momentum-impulse)/abs(impulse)
                idle=max(np.linalg.norm([last[f'idle.{n}.velocity.{a}'] for a in 'xyz']) for n in masses)
                entry.update(relative_momentum_error=error,idle_max_speed_m_s=float(idle))
                assert error<.02 and idle<1e-5,(e,entry)
            if not click and name in ['close','gravity_rotation','free_reaction']:
                if name=='gravity_rotation':
                    opening=max(abs(r['asset.opening']) for r in rows)
                    radial=max(r['asset.radial'] for r in rows)
                    cosine=min(r['asset.axis_cos'] for r in rows)
                    assert last['loose.opening']>.03,(e,last['loose.opening'])
                else:
                    opening,radial,cosine=abs(last['asset.opening']),last['asset.radial'],last['asset.axis_cos']
                entry.update(tight_opening_m=opening,tight_radial_m=radial,tight_min_axis_cosine=cosine)
                assert opening<.0005 and radial<.0005 and cosine>.98,(e,entry)
            if click:
                for joint in ['button_press','refill_slide']:
                    values=[r[f'asset.{joint}.position'] for r in rows]
                    assert min(values)>=-.0031 and max(values)<=.0001,(e,joint,min(values),max(values))
                if name in ['click_cycles','initial_extended']:
                    initial=name=='initial_extended'
                    for t,count,target in [(1.,0,-.003 if initial else 0),(1.6,1,0 if initial else -.003),(2.6,1,0 if initial else -.003),(4.2,2,-.003 if initial else 0)]:
                        r=min(rows,key=lambda r:abs(r['time']-t))
                        assert r['asset.click.toggle_count']==count and abs(r['asset.refill_slide.position']-target)<.0001,(e,t,r)
                    assert abs(last['asset.button_press.position'])<.0001
                else:
                    assert last['asset.click.toggle_count']==0,(e,last['asset.click.toggle_count'])
                entry['final_extended']=bool(last['asset.click.extended'])
                entry['toggle_count']=int(last['asset.click.toggle_count'])
            checks.append(entry)
        ctx.experiment_details['environment_checks']=checks
        ctx.experiment_details['framework_contract']={'all_structures_declared_before_initialization':True,
            'runtime':'PhysX tensors, applied forces and native drive targets; no dynamic pose or USD physical writes',
            'cpu_evidence':'all affected experiments rerun; legacy framework results not reused',
            'dt_s':ctx.dt,'standard_dt_s':.004,'approved_step_exception':'previously approved pen 1ms retained',
            'gpu_drop_solver':'PGS with per-iteration external forces disabled' if name=='low_drop' and ctx.physics=='gpu' else None}
        ctx.metric('validated_environments',len(checks),unit='1',label='逐项验证物理环境数')
        if name=='low_drop':ctx.metric('penetration',max(c['geometry_contact']['maximum_geometry_penetration_m'] for c in checks)*1000,unit='mm',label='各环境最大瞬时几何穿地')
        if click:ctx.metric('toggle_count',max(c['toggle_count'] for c in checks),unit='1',label='各环境按放状态切换次数')
    return experiment

def _observe(ctx, instance, is_click):
    _environment_observers(ctx,instance)
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
    ctx.simulation(dt=dt,gpu_solver='PGS' if ctx.physics=='gpu' else None)
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
    _environment_observers(ctx,instance)
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    for name in ['button_press', 'refill_slide']:
        ctx.observe_joint(instance, name)
    for name in ['extended', 'armed', 'toggle_count', 'callback_count']:
        ctx.observe(instance.name+'.click.'+name,
                    lambda n=name: getattr(instance.behavior(), n), unit='1')
    for e in range(ctx.num_envs):
        for name,attribute in [('extended','extended_by_environment'),('armed','armed_by_environment'),('toggle_count','toggles_by_environment')]:
            ctx.observe(f'env{e}.{instance.name}.click.{name}',lambda e=e,a=attribute:getattr(instance.behavior(),a)[e],unit='1')
    for name in ['button_press','refill_slide']:
        def values(name=name):
            from table_1000.sim.isaaclab.asset_views import TensorView
            a=instance.articulation;v=TensorView(a.view,a.device)
            return v.get_dof_positions()[:,v.shared_metatype.dof_names.index(instance.joint(name).GetName())]
        for e in range(ctx.num_envs):
            ctx.observe(f'env{e}.{instance.name}.{name}.position',lambda e=e,f=values:f()[e],unit='m')


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
    return {n:_checked(t,n,masses,click=True) for n,t in {'click_cycles': cycles, 'initial_extended': extended, 'free_press': free, 'low_drop': drop}.items()}


class Hand:
    """Pen guide declared by ctx.grip; legacy SI gains applied via tensors.

    The guide has no geometry and never changes USD during integration. Targets,
    catches and release masks are independent in each replicated environment.
    """
    def __init__(self,ctx,body,name):
        self.ctx,self.body = ctx,body
        self.grip = ctx.grip(body, name=name)
        self.position,self.quaternion = map(np.copy,ctx.body_pose(body))
        self.position = np.tile(self.position,(ctx.num_envs,1))
        self.quaternion = np.tile(self.quaternion,(ctx.num_envs,1))
        self.k = np.tile([2000.,2000.,2000.,20.,20.,20.],(ctx.num_envs,1))
        self.c = np.tile([40.,40.,40.,.4,.4,.4],(ctx.num_envs,1))
        self.released = np.zeros(ctx.num_envs,dtype=bool)
        original = ctx.session.actions
        def actions(t,dt):
            original(t,dt)
            fixture = self.grip.fixture
            fixture.coordinates = np.array([fixture.pose_coordinates(p,q) for p,q in zip(self.position,self.quaternion)])
            fixture.velocity_targets = np.zeros_like(self.k)
            fixture.stiffness = np.where(self.released[:,None],0,self.k)
            fixture.damping = np.where(self.released[:,None],0,self.c)
            fixture.actuation = np.zeros_like(self.k)
            fixture.max_forces = np.tile([30.,30.,30.,1.,1.,1.],(ctx.num_envs,1))
            fixture.apply()
            self.grip.gains.update(pen_explicit_SI_gains=True,
                linear_stiffness_N_per_m=2000.,linear_damping_N_s_per_m=40.,
                angular_stiffness_N_m_per_rad=20.,angular_damping_N_m_s_per_rad=.4,
                axial_modes='free 0/0, viscous 0/10, caught 2000/40; no gravity feedforward',
                independently_controlled_environments=ctx.num_envs)
        ctx.session.actions = actions

    def move(self,position=None,quaternion=None,mask=None):
        mask = np.ones(self.ctx.num_envs,dtype=bool) if mask is None else mask
        if position is not None:self.position[mask]=np.broadcast_to(position,self.position.shape)[mask]
        if quaternion is not None:self.quaternion[mask]=np.broadcast_to(quaternion,self.quaternion.shape)[mask]
        self.grip.track(self.position[0],self.quaternion[0])

    def axial(self,stiffness=0,damping=0,mask=None):
        mask = np.ones(self.ctx.num_envs,dtype=bool) if mask is None else mask
        self.k[mask,0]=stiffness;self.c[mask,0]=damping

    def release(self,released=True):
        self.released[:]=released



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
    _environment_observers(ctx,instance)
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    for index, (name, unit) in enumerate((('opening', 'm'), ('radial', 'm'), ('axis_cos', '1'))):
        ctx.observe(instance.name+'.'+name, lambda i=index: cap_fit(instance, seat)[i], unit=unit)
    for i in range(1, count+1):
        ctx.observe(instance.name+'.ball'+str(i), lambda i=i: frames(instance, 'cap.detent'+str(i))['position'], unit='m')
    for e in range(ctx.num_envs):
        for i,(name,unit) in enumerate([('opening','m'),('radial','m'),('axis_cos','1')]):
            ctx.observe(f'env{e}.{instance.name}.{name}',lambda e=e,i=i:cap_fits(instance,seat)[e,i],unit=unit)



def _align_video(ctx):
    """Include the final state on the 25 FPS grid, at most 39 ms later."""
    stride = round(.04 / ctx.dt)
    steps = (-ctx.session.steps) % stride
    if steps:
        ctx.run(steps * ctx.dt)


def _cap_setup(ctx, detent, *, gravity=(0, 0, -9.81), gpu_solver=None):
    ctx.simulation(dt=detent['dt'], gravity=gravity,
                   gpu_solver=gpu_solver or detent.get('diagnostic_gpu_solver'))


def _cap_loads(ctx, bodies):
    values = {name: np.zeros((ctx.num_envs,3)) for name in bodies}
    original = ctx.session.actions

    def actions(t, dt):
        original(t, dt)
        for name, body in bodies.items():
            ctx.session.force(body, values[name])

    ctx.session.actions = actions
    for name in bodies:
        ctx.observe(name + '.applied_force_x', lambda n=name: np.asarray(values[n]).reshape(-1,3)[0,0], unit='N')
        for e in range(ctx.num_envs):
            ctx.observe(f'env{e}.{name}.applied_force_x',lambda n=name,e=e:np.broadcast_to(values[n],(ctx.num_envs,3))[e,0],unit='N')
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
    records = [[None for _ in range(ctx.num_envs)] for _ in instances]
    with ctx.fixed(ctx.asset.body('body')), ctx.fixed(loose.body('body')):
        ctx.run(.4)
        initial_fits = [cap_fits(a, seat) for a in instances]
        weak_fits = None
        for k in range(round(8 / ctx.dt)):
            force = min(protocol['max_pull_N'] * k * ctx.dt / 6, protocol['max_pull_N'])
            for i, instance in enumerate(instances):
                forces[instance.name] = np.array([[-force if r is None else 0,0,0] for r in records[i]])
            ctx.step()
            if weak_fits is None and force >= protocol['weak_pull_N']:
                weak_fits = cap_fits(ctx.asset, seat)
            for i, instance in enumerate(instances):
                fits = cap_fits(instance, seat)
                caught = np.array([r is None and fit[0]>.045 for r,fit in zip(records[i],fits)])
                for e in np.flatnonzero(caught):
                    records[i][e] = {'time_s': ctx.session.time, 'last_applied_force_N': force, 'fit': fits[e].tolist()}
                if caught.any():
                    forces[instance.name][caught] = 0
                    guides[i].move(_body_positions(instance.body('cap')),mask=caught)
                    guides[i].axial(2000, 40,mask=caught)
            if all(all(v is not None for v in row) for row in records):
                break
        for name in forces:
            forces[name] = np.zeros(3)
        ctx.experiment_details = {'contract': 'tight/loose force ramp, catch after full separation and place',
            'settled_initial_fit': [v.tolist() for v in initial_fits], 'weak_pull_N': protocol['weak_pull_N'],
            'weak_pull_fit': weak_fits.tolist(), 'separation': records,
            'full_separation_opening_m': .045,
            'guide': 'zero axial stiffness/damping while pulling; native D6 catch only after separation',
            'force_measurement': 'applied external load, not contact or constraint reaction'}
        assert all(all(v is not None for v in row) for row in records), 'a cap did not separate within the fixed force ceiling'
        assert np.max(np.abs(initial_fits[0][:,0])) < .0005
        assert weak_fits is not None and np.max(np.abs(weak_fits[:,0])) < .0005
        assert all(t['last_applied_force_N']>3*l['last_applied_force_N'] for t,l in zip(*records))
        ctx.run(.4)
        # Place using a collision-free compliant hand; preserve body velocity
        # and pose when releasing the native grasp.
        starts = [_body_positions(h.body) for h in guides]
        # A nominal cap radius plus clearance left the faceted cap suspended;
        # its tiny free fall initiated rolling. Place each actual hull on the
        # plane while held, then release unchanged poses and velocities.
        targets = [p.copy() for p in starts]
        face_placement = protocol.get('support_face_placement', False)
        orientations = []
        for hand,target in zip(guides,targets):
            if face_placement:
                q, details = _support_face(ctx,hand.body)
                orientations.append([Slerp([0,1],Rotation.from_quat(
                    np.stack([a,q])[:,[1,2,3,0]])) for a in hand.quaternion])
                target[:,2] = _support_height(ctx,hand.body,
                    np.tile(rotation(q),(ctx.num_envs,1,1)))
                ctx.experiment_details.setdefault('support_face_placement',[]).append(details)
            else:
                target[:,2] = _support_height(ctx,hand.body)
        for k in range(round(1.2 / ctx.dt)):
            fraction = (k + 1) * ctx.dt / 1.2
            fraction = fraction * fraction * (3 - 2 * fraction)
            for i,(hand, start, target) in enumerate(zip(guides, starts, targets)):
                hand.move(start + fraction * (target - start))
                if face_placement:
                    q = np.array([interpolate(fraction).as_quat()[[3,0,1,2]]
                                  for interpolate in orientations[i]])
                    hand.move(quaternion=q)
            ctx.step()
        ctx.run(.4)
        for hand in guides:
            hand.release()
        settling = protocol.get('placement_settle_s', 2.4)
        ctx.run(settling)
        _align_video(ctx)
        ctx.experiment_details['placement_settle_s'] = settling
        ctx.experiment_details['placement_target_origin_height_m']=[p[:,2].tolist() for p in targets]
        ctx.experiment_details['placed_positions_m'] = [a.body('cap').state['position'].tolist() for a in instances]
        ctx.experiment_details['placed_speed_m_s'] = [float(np.linalg.norm(a.body('cap').state['velocity'][:3])) for a in instances]
        assert max(ctx.experiment_details['placed_speed_m_s']) < .01
        ctx.metric('tight_pull',max(v['last_applied_force_N'] for v in records[0]),unit='N',label='各环境紧帽最大分离末载荷')
        ctx.metric('loose_pull',max(v['last_applied_force_N'] for v in records[1]),unit='N',label='各环境松帽最大分离末载荷')
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
        weak_all = cap_fits(ctx.asset, seat)
        weak = weak_all[0]
        tail = [r['asset.opening'] for r in ctx.rows if r['time'] >= ctx.session.time - .4 - 1e-9]
        drift = max(tail) - min(tail)
        seated = [None]*ctx.num_envs
        for k in range(round(4 / ctx.dt)):
            force = protocol['light_push_N'] + (protocol['max_push_N'] - protocol['light_push_N']) * k * ctx.dt / 4
            forces['push'] = np.array([[force if v is None else 0,0,0] for v in seated])
            ctx.step()
            fits = cap_fits(ctx.asset, seat)
            mask = np.array([v is None and f[0]<.0003 for v,f in zip(seated,fits)])
            for e in np.flatnonzero(mask):seated[e]={'time_s':ctx.session.time,'last_applied_force_N':force,'fit':fits[e].tolist()}
            guide.released[mask]=True
            if all(v is not None for v in seated):
                break
        forces['push'] = np.zeros(3)
        guide.release()
        ctx.run(1.2)
        _align_video(ctx)
        final = cap_fit(ctx.asset, seat)
        ctx.experiment_details = {'contract': 'light push stops, strong push seats, cap load and guide release',
            'light_push_N': protocol['light_push_N'], 'weak_fit': weak.tolist(), 'weak_fit_per_environment':weak_all.tolist(),
            'weak_last_0_4s_opening_drift_m': drift, 'seated': seated,
            'released_final_fit': final, 'released_speed_m_s': float(np.linalg.norm(ctx.asset.body('cap').state['velocity'][:3])),
            'closing_guide_axial_damping_N_s_m': 10., 'force_measurement': 'applied external force'}
        assert .0008 < weak[0] < .0028 and drift < .00015, 'weak push did not stop before the ridge'
        assert all(v is not None for v in seated), 'strong push did not seat within the fixed force ceiling'
        assert np.all((weak_all[:,0]>.0008)&(weak_all[:,0]<.0028))
        ctx.metric('seating_push',max(v['last_applied_force_N'] for v in seated),unit='N',label='各环境最大座合末载荷')
        ctx.metric('released_opening',float(np.max(np.abs(cap_fits(ctx.asset,seat)[:,0])))*1000,unit='mm',label='各环境撤力后最大开口')
        assert abs(final[0]) < .0005 and final[1] < .0005 and final[2] > .98
        assert ctx.experiment_details['released_speed_m_s'] < .01


def cap_gravity(ctx, detent, protocol):
    """Only grasp bodies while turning cap-up to cap-down; loose control falls."""
    _cap_setup(ctx, detent, gpu_solver=protocol.get('gpu_gravity_solver'))
    seat, count = detent['seat'], detent['balls']
    pose = quaternion(pi / 2)
    loose = ctx.spawn(ctx.asset_path, name='loose')
    instances = [ctx.asset, loose]
    for instance, x, opening in zip(instances, [-.11, .11], [0, .003]):
        initial_cap(ctx, instance, (x, 0, .25), opening=opening, pose=pose)
        observe_cap(ctx, instance, seat, count)
    ctx.ground(static_friction=.4, dynamic_friction=.3, contact_offset=.0001)
    hands = [Hand(ctx, a.body('body'), a.name + '_body') for a in instances]
    ctx.camera(position=(0, -1.60, .55), target=(0, 0, .20), focal_length=20)
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
        'approved_gpu_solver_exception': protocol.get('gpu_gravity_solver') if ctx.physics == 'gpu' else None,
        'fixtures': 'native compliant body grips only; no cap fixture or hidden coupling'}
    assert extrema['maximum_abs_opening_m'] < .0005 and extrema['maximum_radial_m'] < .0005 and extrema['minimum_axis_cosine'] > .98
    assert fits[1][0] > .03
    ctx.metric('held_opening',extrema['maximum_abs_opening_m']*1000,unit='mm',label='紧帽全程最大开口')


def cap_free_reaction(ctx, detent, masses, *, gpu_solver=None):
    """Load a seated free cap: the body must react through the native detent."""
    _cap_setup(ctx, detent, gravity=(0, 0, 0), gpu_solver=gpu_solver)
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
        'approved_gpu_solver_exception': gpu_solver if ctx.physics == 'gpu' else None,
        'applied_impulse_N_s': impulse, 'total_x_momentum_kg_m_s': float(momentum),
        'relative_momentum_error': float(error), 'idle_center_of_mass_velocity_m_s': idle_v.tolist(),
        'body_reaction_velocity_x_m_s': float(ctx.asset.body('body').state['velocity'][0]),
        'final_fit': cap_fit(ctx.asset, seat), 'masses_kg': masses,
        'geometry_pose_writes_after_initialization': False}
    assert error < .02
    assert np.linalg.norm(idle_v) < 1e-5
    assert ctx.asset.body('body').state['velocity'][0] < -.001
    assert abs(cap_fit(ctx.asset, seat)[0]) < .0005
    ctx.metric('momentum_error',error,unit='1',label='自由反作用动量相对误差')


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
        return cap_free_reaction(ctx, detent, masses, gpu_solver=protocol.get('gpu_free_reaction_solver'))

    def drop(ctx):
        all_masses = {**masses, **{f'cap_ball{i}': detent['mass'] for i in range(1, detent['balls'] + 1)}}
        observe_cap(ctx, ctx.asset, detent['seat'], detent['balls'])
        low_drop(ctx, all_masses, False, detent['dt'], detent=True)
        ctx.plots(*ctx.plot_columns, 'asset.opening')
        ctx.experiment_details['final_cap_fit'] = cap_fit(ctx.asset, detent['seat'])
        ctx.experiment_details['maximum_abs_cap_opening_m'] = max(abs(r['asset.opening']) for r in ctx.rows)

    def cycles(ctx):
        return cap_cycles(ctx, detent, protocol)

    all_masses={**masses,**{f'cap_ball{i}':detent['mass'] for i in range(1,detent['balls']+1)}}
    return {n:_checked(t,n,all_masses) for n,t in {'pull_comparison': pull, 'close': close, 'gravity_rotation': gravity,
            'cycles': cycles, 'free_reaction': reaction, 'low_drop': drop}.items()}

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
            guide.move(_body_positions(cap), cap.state['quaternion'])
            guide.axial()
            guide.release(False)
            record = {'index': cycle + 1, 'starting_fit': cap_fit(ctx.asset, seat), 'per_environment': [dict(environment=e) for e in range(ctx.num_envs)]}
            records.append(record)
            separated = np.zeros(ctx.num_envs,dtype=bool)
            for k in range(round(3 / ctx.dt)):
                force = min(protocol['max_pull_N'] * k * ctx.dt / 1.2, protocol['max_pull_N'])
                forces['cycle'] = np.c_[np.where(separated,0,-force),np.zeros((ctx.num_envs,2))]
                ctx.step()
                fits=cap_fits(ctx.asset,seat)
                mask=(fits[:,0]>.045)&~separated
                for e in np.flatnonzero(mask):record['per_environment'][e].update(separation_applied_force_N=force,separated_fit=fits[e].tolist())
                guide.move(_body_positions(cap),mask=mask)
                guide.axial(2000,40,mask=mask)
                separated |= mask
                if separated.all():
                    break
            forces['cycle'] = np.zeros(3)
            assert separated.all(), 'repeated pull did not completely separate within force ceiling'
            record.update({k:v for k,v in record['per_environment'][0].items() if k!='environment'})
            guide.move(_body_positions(cap))
            guide.axial(2000, 40)
            ctx.run(.12)
            state['phase'] = 'return to 3 mm opening'
            start = _body_positions(cap)
            positions,rotations,_=ctx.asset.environment_states()
            index=ctx.asset.indices['body']
            target = positions[:,index]+np.einsum('eij,j->ei',rotations[:,index],np.array([seat-.003,0,0]))
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
            weak=cap_fits(ctx.asset,seat)
            for e in range(ctx.num_envs):record['per_environment'][e]['weak_fit']=weak[e].tolist()
            assert np.all(weak[:,0]>.0008), 'light repeated insertion bypassed the detent'
            state['phase'] = 'strong push'
            seated = np.zeros(ctx.num_envs,dtype=bool)
            for k in range(round(insertion_timeout / ctx.dt)):
                force = protocol['light_push_N'] + (protocol['max_push_N'] - protocol['light_push_N']) * min(k * ctx.dt / insertion_ramp, 1.)
                forces['cycle'] = np.c_[np.where(seated,0,force),np.zeros((ctx.num_envs,2))]
                ctx.step()
                fits=cap_fits(ctx.asset,seat)
                mask=(fits[:,0]<.0003)&~seated
                for e in np.flatnonzero(mask):record['per_environment'][e]['seating_applied_force_N']=force
                guide.released[mask]=True
                seated |= mask
                if seated.all():
                    break
            forces['cycle'] = np.zeros(3)
            guide.release()
            state['phase'] = 'all cap fixtures released'
            ctx.run(.6)
            record['released_fit'] = cap_fit(ctx.asset, seat)
            assert seated.all(), 'repeated insertion did not seat within force ceiling'
            record['seating_applied_force_N']=record['per_environment'][0]['seating_applied_force_N']
            fits=cap_fits(ctx.asset,seat)
            for e in range(ctx.num_envs):record['per_environment'][e]['released_fit']=fits[e].tolist()
            fit = record['released_fit']
            assert abs(fit[0]) < .0005 and fit[1] < .0005 and fit[2] > .98
            assert np.all(np.abs(fits[:,0])<.0005) and np.all(fits[:,1]<.0005) and np.all(fits[:,2]>.98)
        _align_video(ctx)
        ctx.metric('cycles',3,unit='1',label='全部环境连续完整拔插次数')
