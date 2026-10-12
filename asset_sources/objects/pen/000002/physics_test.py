"""Native detent acceptance through the standard asset test interface.

Actions retain the frozen Claude native-contact research protocols. Fixtures
are collision-free kinematic hands coupled by native D6 drives; they never
write dynamic asset poses or supply a hidden pen/cap connection. Springs and
masses are uncalibrated engineering proxies. Impact tests report outcomes.
"""
from contextlib import ExitStack
from math import cos, sin, pi
import numpy as np
from table_1000.physics.simulation import rotation

SEAT, BALL_RADIUS, BALL_X = -.032, .00865, -.0015


def quaternion(angle, axis=(0,1,0)):
    axis = np.asarray(axis,dtype=float); axis /= np.linalg.norm(axis)
    return np.r_[cos(angle/2),axis*sin(angle/2)]


def fit(body, cap):
    b,c = body.state,cap.state
    local = b['rotation'].T @ (c['position']-b['position'])
    return {'opening':SEAT-local[0], 'radial':np.linalg.norm(local[1:]),
            'axis_cos':b['rotation'][:,0] @ c['rotation'][:,0]}


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

    def release(self,released=True):
        self.joint.GetJointEnabledAttr().Set(not released)


def setup(ctx,dt=.004,height=0,ground=False,pose=(1,0,0,0),loose=0):
    from pxr import PhysxSchema, UsdPhysics, UsdGeom
    ctx.simulation(dt=dt)
    stage=ctx.session.stage
    scene=next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    api=PhysxSchema.PhysxSceneAPI.Apply(scene)
    api.CreateEnableStabilizationAttr(False);api.CreateBounceThresholdAttr(.05)
    api.CreateFrictionOffsetThresholdAttr(.001)
    ctx.initial(position=(0,0,height),rotation=pose)
    if ground:ctx.ground(static_friction=.5,dynamic_friction=.4,contact_offset=.002)
    if loose:
        # Authored initialization only, moving the entire cap assembly together.
        root=rotation(pose);cache=UsdGeom.XformCache()
        bodies={}
        for name in ['cap','cap_ball1','cap_ball2','cap_ball3']:
            transform=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(ctx.asset.paths[name]))
            bodies[name]={'position':np.asarray(transform.ExtractTranslation())-root[:,0]*loose}
        ctx.initial(position=(0,0,height),rotation=pose,bodies=bodies)
    ctx.camera(position=(-.025,-.36,.2),target=(-.03,.01,height),focal_length=24)
    ctx.input_state={'force':{},'phase':0}
    original=ctx.session.actions
    def control(t,dt):
        original(t,dt)
        for body,force in ctx.input_state['force'].values():
            ctx.session.force(body,force)
            ctx.action_values['test_force']={'force':force}
    ctx.session.actions=control
    ctx.observe('phase',lambda:ctx.input_state['phase'],unit='1')
    ctx.observe('applied_pull_N',lambda:sum(np.linalg.norm(v[1]) for v in ctx.input_state['force'].values()),unit='N')
    ctx.experiment_details={'mechanism':'native radial prismatic force drives and real ball/ridge contacts; no body/cap retention script or fixture',
        'asset_pose_writes_during_experiment':False,'ball_mass_note':'1 g per 0.8 mm ball is an inertia proxy, approximately 60x steel sphere mass',
        'external_substeps_per_step':1,'TGS_internal_position_iterations':32}
    return ctx.asset.body('body'),ctx.asset.body('cap')


def observe(ctx,instances=None,pairs=None):
    from pxr import PhysxSchema
    from isaacsim.core.prims import RigidPrim
    instances=instances or [ctx.asset]
    for instance in instances:
        for name in instance.body_names:
            PhysxSchema.PhysxContactReportAPI.Apply(ctx.session.stage.GetPrimAtPath(instance.paths[name])).CreateThresholdAttr(0)
            ctx.observe_body(instance.body(name))
    ctx.ensure_initialized()
    paths=[instance.paths[name] for instance in instances for name in instance.body_names]
    filters=paths+[str(p.GetPath()) for p in ctx.session.stage.Traverse() if p.GetName() in ['Ground','Block']]
    contacts=RigidPrim(paths,name='contacts',reset_xform_properties=False,track_contact_forces=True,
                       contact_filter_prim_paths_expr=[filters for _ in paths],max_contact_count=8192)
    contacts.initialize()
    cache={'time':None}
    def contact_values():
        if cache['time']!=ctx.session.time:
            forces,points,normals,distances,counts,starts=contacts.get_contact_force_data(dt=ctx.dt)
            active=np.concatenate([np.arange(int(start),int(start+count)) for start,count in zip(starts.ravel(),counts.ravel()) if count]) if np.any(counts) else []
            cache.update(time=ctx.session.time,force=float(np.max(np.abs(forces[active]))) if len(active) else 0,
                         separation=float(np.min(distances[active])) if len(active) else 0)
            if '/World/Block' in filters:
                matrix=contacts.get_contact_force_matrix(dt=ctx.dt)
                cache['block']=float(np.sum(np.linalg.norm(matrix[:,filters.index('/World/Block'),:],axis=1)))
        return cache
    ctx.observe('peak_normal_contact_N',lambda:contact_values()['force'],unit='N')
    ctx.observe('min_contact_separation_m',lambda:contact_values()['separation'],unit='m')
    if '/World/Block' in filters:ctx.observe('block_pen_contact_N',lambda:contact_values()['block'],unit='N')
    for name,(body,cap) in (pairs or {a.name:(a.body('body'),a.body('cap')) for a in instances}).items():
        for field,unit in [('opening','m'),('radial','m'),('axis_cos','1')]:
            ctx.observe(name+'.'+field,lambda b=body,c=cap,f=field:fit(b,c)[f],unit=unit)
    for instance in instances:
        cap=instance.body('cap')
        def travel(instance=instance,cap=cap):
            local=[cap.state['rotation'].T @ (instance.body(f'cap_ball{i}').state['position']-cap.state['position']) for i in range(1,4)]
            return [np.linalg.norm(v[1:])-BALL_RADIUS for v in local],max(abs(v[0]-BALL_X) for v in local)
        ctx.observe(instance.name+'.ball_travel_max_m',lambda t=travel:max(t()[0]),unit='m')
        ctx.observe(instance.name+'.ball_axis_error_m',lambda t=travel:t()[1],unit='m')
    ctx.plots(instances[0].name+'.opening','applied_pull_N','peak_normal_contact_N',instances[0].name+'.ball_travel_max_m')


def apply(ctx,body,force):
    ctx.input_state['force']={body.key:(body,np.asarray(force,dtype=float))}


def ramp(ctx,body,limit,seconds=0,escape=.004):
    start=ctx.session.time
    while ctx.session.time-start <= limit/4+seconds+ctx.dt:
        force=min(4*(ctx.session.time-start),limit)
        apply(ctx,body,(-force,0,0));ctx.step()
        if fit(ctx.asset.body('body'),body)['opening']>escape:
            ctx.input_state['force']={}
            return force
    ctx.input_state['force']={}
    return None


def move(ctx,hands,targets,seconds,dwell=.3):
    starts=[h.position.copy() for h in hands]
    for k in range(round(seconds/ctx.dt)):
        u=k*ctx.dt/seconds
        for hand,a,b in zip(hands,starts,targets):hand.move(a+(np.asarray(b)-a)*u)
        ctx.step()
    for hand,target in zip(hands,targets):hand.move(target)
    ctx.run(dwell)


def hold(ctx,dt=.004):
    body,cap=setup(ctx,dt);observe(ctx)
    with ctx.fixed(body):
        ctx.run(.4)
        ctx.input_state['phase']=1
        weak=ramp(ctx,cap,.5,1)
        assert weak is None,'0.5 N weak pull escaped'
        ctx.input_state['phase']=2
        strong=ramp(ctx,cap,4,escape=.010)
        assert strong is not None,'strong pull failed to cross the detent'
        ctx.experiment_details.update(weak_pull_N=.5,strong_ramp_escape_force_N=strong)
        ctx.run(.2)


def refit(ctx,dt=.004):
    body,cap=setup(ctx,dt);observe(ctx)
    hand=Hand(ctx,cap,'cap_hand')
    records=[]
    with ctx.fixed(body):
        ctx.run(.4)
        for cycle in range(3):
            hand.move(cap.state['position']);hand.release(False);start=hand.position.copy()
            ctx.input_state['phase']=cycle*3+1
            move(ctx,[hand],[start+[-.02,0,0]],1)
            removed=fit(body,cap)['opening']
            ctx.input_state['phase']=cycle*3+2
            move(ctx,[hand],[start+[.0015,0,0]],1.2)
            hand.release();ctx.run(.3)
            ctx.input_state['phase']=cycle*3+3
            assert ramp(ctx,cap,.5,1) is None
            opening=fit(body,cap)['opening']
            assert removed>.015 and abs(opening)<.001
            records.append({'removed_opening_m':removed,'weak_pull_final_opening_m':opening})
    ctx.experiment_details['cycles']=records


def free_cap(ctx):
    body,cap=setup(ctx,height=.3,ground=True);observe(ctx)
    ctx.camera(position=(.3,-.75,.4),target=(-.03,0,.15),focal_length=22)
    bh,ch=Hand(ctx,body,'body_hand'),Hand(ctx,cap,'cap_hand')
    ctx.run(.4);move(ctx,[ch],[ch.position+[-.06,0,0]],1.2)
    removed=fit(body,cap)['opening'];assert removed>.045
    bh.release();ch.release();ctx.input_state['phase']=1;ctx.run(.152)
    apply(ctx,body,(0,.03,0));ctx.run(.2);ctx.input_state['force']={};ctx.run(.648)
    ctx.experiment_details['removed_opening_m']=removed


def exchanged_caps(ctx):
    body,cap=setup(ctx)
    other=ctx.spawn(ctx.asset_path,name='pen_b',position=(0,.05,0))
    observe(ctx,[ctx.asset,other],{'a_to_b':(other.body('body'),cap),'b_to_a':(body,other.body('cap'))})
    ctx.plots('a_to_b.opening','b_to_a.opening','peak_normal_contact_N','asset.ball_travel_max_m')
    ctx.camera(position=(-.03,-.5,.28),target=(-.05,.025,0),focal_length=22)
    a,b=Hand(ctx,cap,'cap_a'),Hand(ctx,other.body('cap'),'cap_b')
    records=[]
    with ctx.fixed(body),ctx.fixed(other.body('body')):
        ctx.run(.5)
        for cycle in range(2):
            ya,yb=(0,.05) if cycle==0 else (.05,0)
            for hand in [a,b]:hand.move(hand.body.state['position']);hand.release(False)
            tracks=[[(SEAT,ya,0),(SEAT-.05,ya,0),(SEAT-.05,ya,.04),(SEAT-.05,yb,.04),(SEAT-.05,yb,0),(SEAT+.001,yb,0)],
                    [(SEAT,yb,0),(SEAT-.05,yb,0),(SEAT-.05,yb,-.04),(SEAT-.05,ya,-.04),(SEAT-.05,ya,0),(SEAT+.001,ya,0)]]
            for leg in range(1,6):
                ctx.input_state['phase']=cycle*6+leg
                move(ctx,[a,b],[track[leg] for track in tracks],1.2,dwell=0)
            ctx.run(.2);a.release();b.release();ctx.run(.4)
            ctx.input_state['force']={h.body.key:(h.body,np.array([-.5,0,0])) for h in [a,b]}
            ctx.run(1);ctx.input_state['force']={}
            pairs=[(other.body('body'),cap),(body,other.body('cap'))] if cycle==0 else [(body,cap),(other.body('body'),other.body('cap'))]
            fits=[fit(*pair) for pair in pairs]
            assert all(abs(f['opening'])<.0015 and f['radial']<.0008 and f['axis_cos']>.99 for f in fits)
            records.append(fits)
    ctx.experiment_details['actual_swaps']=records


def loose_cap_rotation(ctx,dt=.004):
    up=quaternion(pi/2)
    body,cap=setup(ctx,dt,height=.25,ground=True,pose=up)
    loose=ctx.spawn(ctx.asset_path,name='loose',position=(0,.06,.25),rotation=up)
    from pxr import UsdGeom
    cache=UsdGeom.XformCache();bodies={}
    for name in ['cap','cap_ball1','cap_ball2','cap_ball3']:
        transform=cache.GetLocalToWorldTransform(ctx.session.stage.GetPrimAtPath(loose.paths[name]))
        bodies[name]={'position':np.asarray(transform.ExtractTranslation())-rotation(up)[:,0]*.003}
    ctx.initial(loose,position=(0,.06,.25),rotation=up,bodies=bodies)
    observe(ctx,[ctx.asset,loose])
    ctx.camera(position=(.3,-.8,.5),target=(0,.025,.20),focal_length=22)
    hands=[Hand(ctx,body,'tight_hand'),Hand(ctx,loose.body('body'),'loose_hand')]
    ctx.observe('tip_angle_rad',lambda:ctx.input_state.get('angle',pi/2),unit='rad')
    ctx.plots('asset.opening','loose.opening','tip_angle_rad','peak_normal_contact_N')
    ctx.run(.6)
    for k in range(round(4/ctx.dt)):
        angle=pi/2-pi*k*ctx.dt/4;ctx.input_state['angle']=angle
        for hand in hands:hand.move(quaternion=quaternion(angle))
        ctx.step()
    for hand in hands:hand.move(quaternion=quaternion(-pi/2))
    ctx.run(1)
    result={'tight':fit(body,cap),'loose':fit(loose.body('body'),loose.body('cap'))}
    assert abs(result['tight']['opening'])<.0015 and result['loose']['opening']>.03
    ctx.experiment_details['final_fit']=result


def third_body_contact(ctx):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    from isaacsim.core.prims import RigidPrim
    body,cap=setup(ctx,height=.0101,ground=True)
    block=UsdGeom.Cube.Define(ctx.session.stage,'/World/Block');block.CreateSizeAttr(1)
    block.AddTranslateOp().Set(Gf.Vec3d(-.06,-.045,.016));block.AddScaleOp().Set(Gf.Vec3f(.03))
    block.CreateDisplayColorAttr([Gf.Vec3f(.3,.45,.7)])
    UsdPhysics.RigidBodyAPI.Apply(block.GetPrim());UsdPhysics.MassAPI.Apply(block.GetPrim()).CreateMassAttr(.05)
    UsdPhysics.CollisionAPI.Apply(block.GetPrim())
    api=PhysxSchema.PhysxRigidBodyAPI.Apply(block.GetPrim())
    api.CreateSolverPositionIterationCountAttr(32);api.CreateSolverVelocityIterationCountAttr(8)
    api.CreateSleepThresholdAttr(0);api.CreateStabilizationThresholdAttr(0)
    PhysxSchema.PhysxCollisionAPI.Apply(block.GetPrim()).CreateContactOffsetAttr(.002)
    material=UsdShade.Material.Define(ctx.session.stage,'/World/BlockMaterial')
    api=UsdPhysics.MaterialAPI.Apply(material.GetPrim());api.CreateStaticFrictionAttr(.5);api.CreateDynamicFrictionAttr(.4);api.CreateRestitutionAttr(0)
    UsdShade.MaterialBindingAPI.Apply(block.GetPrim()).Bind(material,materialPurpose='physics')
    observe(ctx)
    view=RigidPrim('/World/Block',name='block',reset_xform_properties=False);view.initialize()
    ctx.run(.3)
    view.set_velocities(np.array([[0,.5,0,0,0,0]],dtype=np.float32))
    ctx.run(1.5)
    peak=max(row['block_pen_contact_N'] for row in ctx.rows)
    assert peak>0,'third body did not contact the pen'
    ctx.plots('block_pen_contact_N','asset.opening','asset.body.position.y','peak_normal_contact_N')
    ctx.experiment_details.update(block_mass_kg=.05,initial_block_velocity_m_s=[0,.5,0],peak_block_pen_contact_N=peak)


def compliant_grasp(ctx):
    body,cap=setup(ctx);observe(ctx)
    bh,ch=Hand(ctx,body,'body_hand'),Hand(ctx,cap,'cap_hand')
    ctx.run(.5);move(ctx,[ch],[ch.position+[-.02,0,0]],1.5,dwell=1)
    assert fit(body,cap)['opening']>.01
    ctx.experiment_details['grasp_gains']={'linear_stiffness_N_m':2000,'linear_damping_N_s_m':40,'angular_stiffness_N_m_rad':20,'angular_damping_N_m_s_rad':.4,'maximum_force_N':30,'maximum_torque_N_m':1}


def drop(ctx,height,dt=.004):
    body,cap=setup(ctx,dt,height=height,ground=True,pose=quaternion(35*pi/180,(.3,1,.2)));observe(ctx)
    ctx.camera(position=(.2,-(.4+1.6*height),height/2+.25),target=(0,0,height/2),focal_length=17)
    ctx.plots('asset.opening','asset.body.position.z','min_contact_separation_m','peak_normal_contact_N')
    ctx.run(2)
    assert all(np.isfinite(value) for row in ctx.rows for value in row.values())
    assert 0<body.state['position'][2]<.025
    ctx.experiment_details.update(initial_origin_height_m=height,final_fit=fit(body,cap),
        interpretation='impact is a coarse test: detent release and temporary 4 ms penetration are reported; no height is required to detach')


def activity(ctx,instances=1):
    body,cap=setup(ctx)
    pens=[ctx.asset]
    if instances==2:pens.append(ctx.spawn(ctx.asset_path,name='pen_b',position=(0,.05,0)))
    observe(ctx,pens)
    hands=[Hand(ctx,p.body('cap'),f'cap_hand{i}') for i,p in enumerate(pens)]
    ctx.camera(position=(-.025,-.4,.22),target=(-.05,.025*(instances-1),0),focal_length=24)
    with ExitStack() as stack:
        for pen in pens:stack.enter_context(ctx.fixed(pen.body('body')))
        for cycle in range(3):
            for hand in hands:hand.move(hand.body.state['position']);hand.release(False)
            starts=[hand.position.copy() for hand in hands]
            move(ctx,hands,[p+[-.05,0,0] for p in starts],1,dwell=0)
            move(ctx,hands,[p+[.0015,0,0] for p in starts],1.2,dwell=.4)
            for hand in hands:hand.release()
            ctx.input_state['force']={h.body.key:(h.body,np.array([-.5,0,0])) for h in hands}
            ctx.run(.736 if cycle==0 else .732);ctx.input_state['force']={}
    assert abs(ctx.session.time-10)<1e-6


TESTS={'hold':hold,'refit':refit,'free_cap':free_cap,'exchanged_caps':exchanged_caps,
       'loose_cap_rotation':loose_cap_rotation,'third_body_contact':third_body_contact,'compliant_grasp':compliant_grasp,
       **{f'drop_{h:g}m_4ms':(lambda ctx,h=h:drop(ctx,h)) for h in [.1,.3,.6,1]},
       'hold_2ms':lambda ctx:hold(ctx,.002),'refit_2ms':lambda ctx:refit(ctx,.002),
       'loose_cap_rotation_2ms':lambda ctx:loose_cap_rotation(ctx,.002),
       'drop_1m_2ms':lambda ctx:drop(ctx,1,.002),
       'activity_single':activity,'activity_dual':lambda ctx:activity(ctx,2)}
