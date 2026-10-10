"""Marker cap experiments; equivalent fit parameters are not material calibration."""

from functools import partial
from math import cos, sin, pi
from table_1000.physics.testing import TestContext


def observe(ctx, pen):
    for name in ('body','cap'):
        ctx.observe_body(pen.body(name))
    for field,unit in [('engaged','1'),('opening','m'),('retention_force','N'),('radial_error','m')]:
        ctx.observe(f'{pen.name}.cap_fit.{field}',lambda field=field: pen.behavior().observations()[field],unit=unit)


def setup(ctx, *, opened=False):
    ctx.simulation(gravity=(0,0,0))
    ctx.initial(position=(0,0,.15),bodies={'cap':{'position':(-.052,0,.15)}} if opened else {})
    ctx.camera(position=(.15,-.4,.35) if opened else (.34,-.1,.23),
               target=(-.01,0,.15) if opened else (-.08,0,.15))
    observe(ctx,ctx.asset)


def pull(ctx: TestContext):
    setup(ctx)
    ctx.force('pull_cap',ctx.asset.body('cap'),keyframes=[
        {'time':0,'force':(0,0,0)},{'time':.8,'force':(-1,0,0)},{'time':1.2},
        {'time':1.212,'force':(-2.5,0,0)},{'time':1.216,'force':(0,0,0)}])
    ctx.plots('pull_cap.force.x','asset.cap.position.x','asset.cap.linear_velocity.x')
    with ctx.fixed(ctx.asset.body('body')):
        ctx.run(1.72)


def close(ctx: TestContext):
    setup(ctx,opened=True)
    ctx.force('push_cap',ctx.asset.body('cap'),keyframes=[
        {'time':0,'force':(0,0,0)},{'time':.8,'force':(.5,0,0)},{'time':1.2},
        {'time':1.4,'force':(2.5,0,0)},{'time':1.404},{'time':1.408,'force':(0,0,0)}])
    ctx.plots('push_cap.force.x','asset.cap.position.x','asset.cap.linear_velocity.x')
    with ctx.fixed(ctx.asset.body('body')):
        ctx.run(3)


def pressure_hold(ctx: TestContext):
    setup(ctx,opened=True)
    ctx.force('push_cap',ctx.asset.body('cap'),keyframes=[
        {'time':0,'force':(0,0,0)},{'time':.8,'force':(.5,0,0)},{'time':1.2},
        {'time':1.6,'force':(4.5,0,0)},{'time':2},{'time':2.1,'force':(0,0,0)}])
    ctx.plots('push_cap.force.x','asset.cap.position.x','asset.cap.linear_velocity.x')
    with ctx.fixed(ctx.asset.body('body')):
        ctx.run(3)


def drop(ctx: TestContext, *, height, dt):
    ctx.simulation(dt=dt)
    ctx.initial(position=(0,0,height),rotation=(cos(pi/8),0,sin(pi/8),0))
    ctx.ground()
    camera = { .2:((.18,-1.1,.3),(0,0,.12)), .5:((.3,-2.3,.6),(0,0,.27)),
                1:((.5,-4.2,1),(0,0,.54))}[height]
    ctx.camera(position=camera[0],target=camera[1])
    observe(ctx,ctx.asset)
    ctx.plots('asset.cap.position.z','asset.cap.linear_velocity.z','asset.cap_fit.opening')
    ctx.run(2)


def free_cap(ctx: TestContext):
    """G: already detached, no fixtures, off-axis force followed by free motion."""
    ctx.simulation(gravity=(0,0,0))
    ctx.initial(position=(0,0,.15),bodies={'cap':{'position':(-.092,0,.15)}})
    ctx.camera(position=(.15,-.45,.4),target=(-.025,.02,.15))
    observe(ctx,ctx.asset)
    ctx.force('off_axis',ctx.asset.body('cap'),frame='body',point_frame='body',keyframes=[
        {'time':0,'force':(0,0,0),'point':(-.015,0,0)},
        {'time':.08,'force':(0,.002,0)},{'time':.16,'force':(0,0,0)}])
    ctx.plots('off_axis.force.y','asset.cap.position.y','asset.cap.angular_velocity.z')
    ctx.run(1)
    assert not ctx.asset.behavior().observations()['engaged']



TESTS = {'pull':pull,'close':close,'pressure_hold':pressure_hold,
         'drop_0.2m_4ms':partial(drop,height=.2,dt=.004),'drop_0.5m_4ms':partial(drop,height=.5,dt=.004),
         'drop_1m_4ms':partial(drop,height=1,dt=.004),'drop_1m_2ms':partial(drop,height=1,dt=.002),
         'free_cap':free_cap}


def zero_fit_friction(ctx):
    """Scene-only overrides: zero friction at the fit, baseline ground friction."""
    from pxr import PhysxSchema, UsdPhysics
    prim = ctx.session.stage.GetPrimAtPath(ctx.asset.path)
    for name in ('static_retention_N', 'dynamic_retention_N'):
        prim.GetAttribute('table1000:capFit:' + name).Set(0.)
    material = ctx.session.stage.GetPrimAtPath(ctx.asset.path + '/Materials/plastic')
    api = UsdPhysics.MaterialAPI(material)
    api.GetStaticFrictionAttr().Set(0.); api.GetDynamicFrictionAttr().Set(0.)
    PhysxSchema.PhysxMaterialAPI(material).GetFrictionCombineModeAttr().Set('min')
    floor = ctx.session.stage.GetPrimAtPath('/World/GroundMaterial')
    PhysxSchema.PhysxMaterialAPI.Apply(floor).CreateFrictionCombineModeAttr('max')
    for name in ('static_retention_N', 'dynamic_retention_N', 'seating_barrier_N'):
        ctx.observe('fit_parameters.' + name, lambda name=name: ctx.asset.behavior().parameters[name], unit='N')


def drop_zero_friction(ctx, *, height, dt):
    ctx.simulation(dt=dt)
    ctx.initial(position=(0,0,height),rotation=(cos(pi/8),0,sin(pi/8),0))
    ctx.ground()
    zero_fit_friction(ctx)
    camera = {.2:((.18,-1.1,.3),(0,0,.12)), .5:((.3,-2.3,.6),(0,0,.27)),
              1:((.5,-4.2,1),(0,0,.54))}[height]
    ctx.camera(position=camera[0],target=camera[1],focal_length=8 if height==.2 else 24 if height==1 else 28)
    ctx.video_annotation = lambda: 'Fit friction 0; ground friction 0.35; seating barrier and normal contacts retained'
    observe(ctx,ctx.asset)
    ctx.plots('asset.cap.position.z','asset.cap.linear_velocity.z','asset.cap_fit.opening',
              'asset.cap_fit.engaged','asset.cap_fit.retention_force')
    ctx.run(2)
    from isaacsim.core.simulation_manager import SimulationManager
    import numpy as np
    engine = SimulationManager.get_physics_sim_view()
    shape_values = {name: np.unique(np.asarray(engine.create_rigid_body_view(ctx.asset.paths[name]).get_material_properties()).reshape(-1,3),axis=0).tolist()
                    for name in ctx.asset.body_names}
    assert all(all(row[:2] == [0.,0.] for row in values) for values in shape_values.values())
    assert ctx.asset.behavior().parameters['static_retention_N'] == ctx.asset.behavior().parameters['dynamic_retention_N'] == 0
    ctx.experiment_details = {'fit_parameters':dict(ctx.asset.behavior().parameters),
        'engine_shape_materials_static_dynamic_restitution':shape_values,
        'pair_friction':{'body_cap':0.,'body_ground':.35,'cap_ground':.35,
            'rule':'PhysX combine priority average < min < multiply < max; zero/min plastic with 0.35/max ground gives max(0,0.35)',
            'plastic':{'static':0.,'dynamic':0.,'combine':'min'},
            'ground':{'static':.35,'dynamic':.35,'combine':'max'}},
        'remaining_forces':['native normal collision and CCD contacts, unchanged geometry and margins',
            '3 N triangular seating barrier over the 0–2 mm opening interval',
            'unchanged radial/angular fit elasticity and damping, which may couple through rigid-body rotation',
            'gravity; no test force or fixture'],
        'camera_change':{'position_direction':'Original drop camera',
            'original_focal_length_mm':28,'reason':'Widened to include the complete cap trajectory and keep it clear of the caption' if height in (.2,1) else 'Original focal length'},
        'fully_disengaged':any(not bool(row['asset.cap_fit.engaged']) for row in ctx.rows[1:]),
        'first_disengagement_s':next((row['time'] for row in ctx.rows[1:] if not row['asset.cap_fit.engaged']),None),
        'final_opening_m':ctx.rows[-1]['asset.cap_fit.opening']}


def smooth_target(points, t):
    """Continuous targets with zero speed at waypoints, not body pose writes."""
    import numpy as np
    if t <= points[0][0]: return np.asarray(points[0][1],dtype=float)
    for (ta,a),(tb,b) in zip(points,points[1:]):
        if ta <= t < tb:
            u=(t-ta)/(tb-ta); u=u*u*(3-2*u)
            return np.asarray(a)*(1-u)+np.asarray(b)*u
    return np.asarray(points[-1][1],dtype=float)


def drive_translation(joint, target):
    import numpy as np
    from pxr import Sdf, UsdPhysics
    with Sdf.ChangeBlock():
        for axis, value in zip(('transX','transY','transZ'), target):
            value=float(np.float32(value))
            attr = UsdPhysics.DriveAPI.Get(joint.GetPrim(),axis).GetTargetPositionAttr()
            if attr.Get() != value: attr.Set(value)


def paired_body_number(pen):
    peer = pen.behavior().male
    return 0 if peer is None else 1 if peer.instance.name == 'asset' else 2


def exchanged_caps(ctx: TestContext):
    """One continuous native-fixture pull, exchange, close and released hold."""
    import numpy as np
    from contextlib import ExitStack
    ctx.simulation(gravity=(0,0,0))
    ctx.initial(position=(0,-.075,.15))
    other=ctx.spawn(ctx.asset_path,name='pen_b',position=(0,.075,.15))
    ctx.camera(position=(.18,-.6,.43),target=(-.035,0,.15),focal_length=24)
    for pen in (ctx.asset,other):
        observe(ctx,pen)
        ctx.observe(pen.name+'.paired_body',lambda pen=pen:paired_body_number(pen),unit='1')
    state={'phase':0,'gripped':1,'targets':[np.array([-.032,-.075,.15]),np.array([-.032,.075,.15])],
           'forces':[np.zeros(3),np.zeros(3)]}
    stages=['closed / cap fixtures attached','pulling both caps','fully separated / two routes',
            'exchanging caps','aligning opposite bodies','pressing caps onto opposite bodies',
            'seated / cap fixture hold','cap fixtures RELEASED / retention only']
    ctx.video_annotation=lambda: 'A started front (y=-0.075), B back | '+stages[state['phase'] if ctx.session.time else 0]
    ctx.observe('phase',lambda:state['phase'] if ctx.session.time else 0,unit='1')
    ctx.observe('cap_fixtures_active',lambda:state['gripped'],unit='1')
    paths=[]
    for y,z in ((-.075,.22),(.075,.08)):
        paths.append([(0,(-.032,y,.15)),(.5,(-.032,y,.15)),(1.5,(-.115,y,.15)),
            (2,(-.115,y,z)),(4,(-.115,-y,z)),(4.5,(-.115,-y,.15)),
            (6.5,(-.0316,-y,.15)),(7,(-.0316,-y,.15))])
    stiffness,damping=4000.,8.
    for i,pen in enumerate((ctx.asset,other)):
        for k,axis in enumerate('xyz'):
            ctx.observe(f'{pen.name}.fixture_target.{axis}',lambda i=i,k=k:state['targets'][i][k] if ctx.session.time else paths[i][0][1][k],unit='m')
            ctx.observe(f'{pen.name}.fixture_force_estimate.{axis}',lambda i=i,k=k:state['forces'][i][k] if ctx.session.time else 0.,unit='N')
    original=ctx.session.actions
    with ExitStack() as fixtures:
        for pen in (ctx.asset,other): fixtures.enter_context(ctx.fixed(pen.body('body')))
        with ExitStack() as grips:
            joints=[grips.enter_context(ctx.spring(pen.body('cap'),position=pen.body('cap').state['position'],
                linear_stiffness=stiffness,linear_damping=damping,angular_stiffness=1.,angular_damping=.02))
                for pen in (ctx.asset,other)]
            anchors=[pen.body('cap').state['position'].copy() for pen in (ctx.asset,other)]
            def actions(t,dt):
                original(t,dt)
                state['phase']=0 if t<.5 else 1 if t<1.5 else 2 if t<2 else 3 if t<4 else 4 if t<4.5 else 5 if t<6.5 else 6 if t<7 else 7
                for i,pen in enumerate((ctx.asset,other)):
                    target=smooth_target(paths[i],t);state['targets'][i]=target
                    if state['gripped']:
                        drive_translation(joints[i],target-anchors[i])
                        cap=pen.body('cap').state
                        origin_velocity=cap['velocity'][:3]-np.cross(cap['velocity'][3:],cap['com']-cap['position'])
                        state['forces'][i]=stiffness*(target-cap['position'])-damping*origin_velocity
                    else: state['forces'][i]=np.zeros(3)
            ctx.session.actions=actions
            ctx.plots('phase','cap_fixtures_active','asset.cap.position.x','asset.cap.position.y','asset.cap.position.z',
                'asset.fixture_force_estimate.x','pen_b.fixture_force_estimate.x',
                'asset.cap_fit.opening','pen_b.cap_fit.opening','asset.paired_body','pen_b.paired_body')
            ctx.run(7)
        state.update(gripped=0,phase=7)
        ctx.run(2)
    ctx.session.actions=original
    assert ctx.asset.behavior().male is other.behavior() and other.behavior().male is ctx.asset.behavior()
    assert all(abs(pen.behavior().observations()['opening']) <= .001 for pen in (ctx.asset,other))
    released_max={pen.name:max(abs(row[pen.name+'.cap_fit.opening']) for row in ctx.rows if row['time']>=7)
                  for pen in (ctx.asset,other)}
    assert all(value<=.001 for value in released_max.values())
    ctx.experiment_details={'fixture':'native USD generic joint with six force drives; world anchor fixed; translation drive targets changed smoothly',
        'linear_stiffness_N_per_m':stiffness,'linear_damping_N_s_per_m':damping,
        'angular_stiffness':1.,'angular_damping':.02,
        'fixture_force_columns':'estimated linear spring/damper force from target and sampled body-origin error; not a measured native constraint reaction',
        'cap_fixture_release_s':7.,'released_observation_seconds':2.,
        'maximum_abs_opening_after_cap_release_m':released_max,
        'pair_identity':{'cap_A_final_body':ctx.asset.behavior().male.instance.name,
                         'cap_B_final_body':other.behavior().male.instance.name},
        'final_opening_m':{pen.name:pen.behavior().observations()['opening'] for pen in (ctx.asset,other)}}


def loose_cap_rotation(ctx: TestContext):
    """Default fit, cap never gripped; body fixtures turn slowly under gravity."""
    import numpy as np
    from contextlib import ExitStack
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    ctx.simulation()
    ctx.initial(position=(0,-.075,.48))
    loose=ctx.spawn(ctx.asset_path,name='loose',position=(0,.075,.48))
    ctx.initial(loose,position=(0,.075,.48),bodies={'cap':{'position':(-.060,.075,.48)}})
    ctx.ground()
    ctx.camera(position=(.32,-1.35,.75),target=(-.02,0,.27),focal_length=18)
    state={'angle':0.}
    ctx.video_annotation=lambda:'Front: tight (0 mm) | Back: loose (28 mm, still over nib) | body drives only, caps never gripped'
    ctx.observe('body_target_angle',lambda:state['angle'] if ctx.session.time else 0.,unit='rad')
    for pen in (ctx.asset,loose):
        observe(ctx,pen)
        ctx.observe(pen.name+'.body_angle',lambda pen=pen:np.arctan2(-pen.body('body').state['rotation'][2,0],pen.body('body').state['rotation'][0,0]),unit='rad')
        ctx.observe(pen.name+'.paired_body',lambda pen=pen:paired_body_number(pen),unit='1')
    grippers=[]
    for pen,y in ((ctx.asset,-.075),(loose,.075)):
        grip=UsdGeom.Xform.Define(ctx.session.stage,'/World/Fixtures/body_grip_'+pen.name)
        transform=UsdGeom.Xformable(grip)
        transform.AddTranslateOp().Set(Gf.Vec3d(0,y,.48))
        orient=transform.AddOrientOp();orient.Set(Gf.Quatf(1))
        UsdPhysics.RigidBodyAPI.Apply(grip.GetPrim()).CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(grip.GetPrim()).CreateMassAttr(1)
        grippers.append((grip,orient))
        ctx.observe(pen.name+'.body_fixture_inverse_mass',lambda pen=pen:pen.body('body').state['inverse_mass'],unit='1/kg')
        ctx.observe(pen.name+'.body_predicted_gravity_z',lambda pen=pen:pen.body('body').state['gravity_acceleration'][2],unit='m/s²')
        ctx.observe(pen.name+'.cap_predicted_gravity_z',lambda pen=pen:pen.body('cap').state['gravity_acceleration'][2],unit='m/s²')
    original=ctx.session.actions
    ctx.ensure_initialized()
    with ExitStack() as fixtures:
        for pen,(grip,orient) in zip((ctx.asset,loose),grippers):
            joint=fixtures.enter_context(ctx.fixed(pen.body('body')))
            joint.CreateBody0Rel().SetTargets([grip.GetPath()])
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(0));joint.GetLocalRot0Attr().Set(Gf.Quatf(1))
        ctx.session.refresh()
        def actions(t,dt):
            original(t,dt)
            angle=float(smooth_target([(0,[0]),(4,[-pi/2]),(6,[-pi/2])],t)[0]);state['angle']=angle
            with Sdf.ChangeBlock():
                # Only native kinematic clamp targets change. Dynamic asset
                # poses are moved by their fixed constraints and PhysX.
                for grip,orient in grippers:
                    orient.Set(Gf.Quatf(cos(angle/2),Gf.Vec3f(0,sin(angle/2),0)))
        ctx.session.actions=actions
        ctx.plots('body_target_angle','asset.body_angle','loose.body_angle','asset.body.angular_velocity.y',
            'loose.body.angular_velocity.y','asset.cap_fit.opening','loose.cap_fit.opening',
            'asset.cap_fit.engaged','loose.cap_fit.engaged','asset.cap_fit.retention_force','loose.cap_fit.retention_force')
        ctx.run(6)
    ctx.session.actions=original
    ctx.experiment_details={'initial_opening_m':{'asset':0.,'loose':.028},
        'loose_geometry':'neck ends 26 mm from seat; nib spans opening 26–39 mm. Lip at 28 mm overlaps 11 mm of nib, 2 mm beyond behavior engagement interval.',
        'fit_parameters':{pen.name:dict(pen.behavior().parameters) for pen in (ctx.asset,loose)},
        'body_rotation':'native kinematic clamp orientation targets 0 to -pi/2 in 4 s, then held 2 s; asset bodies follow fixed joints',
        'kinematic_body_fixtures':2,'asset_pose_writes_during_experiment':False,
        'cap_test_forces_or_fixtures':False,
        'final_opening_m':{pen.name:pen.behavior().observations()['opening'] for pen in (ctx.asset,loose)},
        'tight_still_engaged':bool(ctx.asset.behavior().observations()['engaged']),
        'loose_final_engaged':bool(loose.behavior().observations()['engaged'])}


TESTS.update({'drop_zero_friction_0.2m_4ms':partial(drop_zero_friction,height=.2,dt=.004),
    'drop_zero_friction_0.5m_4ms':partial(drop_zero_friction,height=.5,dt=.004),
    'drop_zero_friction_1m_4ms':partial(drop_zero_friction,height=1,dt=.004),
    'drop_zero_friction_1m_2ms':partial(drop_zero_friction,height=1,dt=.002),
    'exchanged_caps':exchanged_caps,'loose_cap_rotation':loose_cap_rotation})
