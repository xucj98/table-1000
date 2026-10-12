"""Native detent acceptance through the standard asset test interface.

Four scenes exercise the frozen Claude native-contact mechanism. Fixtures
are collision-free kinematic hands coupled by native D6 drives; they never
write dynamic asset poses or supply a hidden pen/cap connection. Springs and
masses are uncalibrated engineering proxies. Impact tests report outcomes.
"""
from math import cos, sin, pi
import numpy as np
from table_1000.physics.simulation import rotation

SEAT, BALL_RADIUS, BALL_X = -.032, .00865, -.0015
BODY_TIP_X, LOOSE_OPENING = -.071, .003


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

    def axial(self,stiffness=0,damping=0):
        from pxr import UsdPhysics
        drive=UsdPhysics.DriveAPI(self.joint.GetPrim(),'transX')
        drive.GetStiffnessAttr().Set(stiffness);drive.GetDampingAttr().Set(damping)

    def release(self,released=True):
        self.joint.GetJointEnabledAttr().Set(not released)


def setup(ctx,height=0,ground=False,pose=(1,0,0,0)):
    from pxr import PhysxSchema, UsdPhysics
    ctx.simulation(dt=.004)
    stage=ctx.session.stage
    scene=next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    api=PhysxSchema.PhysxSceneAPI.Apply(scene)
    api.CreateEnableStabilizationAttr(False);api.CreateBounceThresholdAttr(.05)
    api.CreateFrictionOffsetThresholdAttr(.001)
    ctx.initial(position=(0,0,height),rotation=pose)
    if ground:ctx.ground(static_friction=.5,dynamic_friction=.4,contact_offset=.002)
    ctx.camera(position=(-.025,-.36,.2),target=(-.03,.01,height),focal_length=24)
    ctx.input_state={'force':{},'phase':0}
    original=ctx.session.actions
    def control(t,dt):
        original(t,dt)
        for name,(body,force) in ctx.input_state['force'].items():
            ctx.session.force(body,force)
            ctx.action_values[name]={'force':force}
    ctx.session.actions=control
    ctx.observe('phase',lambda:ctx.input_state['phase'],unit='1')
    ctx.experiment_details={'mechanism':'native radial prismatic force drives and real ball/ridge contacts; no body/cap retention script or fixture',
        'asset_pose_writes_during_experiment':False,'force_direction':'world -X removes the cap; world +X closes it','ball_mass_note':'1 g per 0.8 mm ball is an inertia proxy, approximately 60x steel sphere mass',
        'external_substeps_per_step':1,'TGS_internal_position_iterations':32}
    return ctx.asset.body('body'),ctx.asset.body('cap')


def observe(ctx,instances=None):
    from pxr import PhysxSchema
    from isaacsim.core.prims import RigidPrim
    instances=instances or [ctx.asset]
    for instance in instances:
        for name in instance.body_names:
            PhysxSchema.PhysxContactReportAPI.Apply(ctx.session.stage.GetPrimAtPath(instance.paths[name])).CreateThresholdAttr(0)
            ctx.observe_body(instance.body(name))
    ctx.ensure_initialized()
    paths=[instance.paths[name] for instance in instances for name in instance.body_names]
    filters=paths+[str(p.GetPath()) for p in ctx.session.stage.Traverse() if p.GetName()=='Ground']
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
        return cache
    ctx.observe('peak_normal_contact_N',lambda:contact_values()['force'],unit='N')
    ctx.observe('min_contact_separation_m',lambda:contact_values()['separation'],unit='m')
    for name,(body,cap) in {a.name:(a.body('body'),a.body('cap')) for a in instances}.items():
        for field,unit in [('opening','m'),('radial','m'),('axis_cos','1')]:
            ctx.observe(name+'.'+field,lambda b=body,c=cap,f=field:fit(b,c)[f],unit=unit)
    for instance in instances:
        cap=instance.body('cap')
        def travel(instance=instance,cap=cap):
            local=[cap.state['rotation'].T @ (instance.body(f'cap_ball{i}').state['position']-cap.state['position']) for i in range(1,4)]
            return [np.linalg.norm(v[1:])-BALL_RADIUS for v in local],max(abs(v[0]-BALL_X) for v in local)
        ctx.observe(instance.name+'.ball_travel_max_m',lambda t=travel:max(t()[0]),unit='m')
        ctx.observe(instance.name+'.ball_axis_error_m',lambda t=travel:t()[1],unit='m')


def move(ctx,hands,targets,seconds,dwell=.3):
    starts=[h.position.copy() for h in hands]
    for k in range(round(seconds/ctx.dt)):
        u=k*ctx.dt/seconds
        for hand,a,b in zip(hands,starts,targets):hand.move(a+(np.asarray(b)-a)*u)
        ctx.step()
    for hand,target in zip(hands,targets):hand.move(target)
    ctx.run(dwell)


def initial_pen(ctx,instance,position,pose=(1,0,0,0),opening=0):
    from pxr import UsdGeom
    ctx.initial(instance,position=position,rotation=pose)
    if opening:
        cache=UsdGeom.XformCache();axis=rotation(pose)[:,0]
        bodies={name:{'position':np.asarray(cache.GetLocalToWorldTransform(
            ctx.session.stage.GetPrimAtPath(instance.paths[name])).ExtractTranslation())-axis*opening}
            for name in ['cap','cap_ball1','cap_ball2','cap_ball3']}
        ctx.initial(instance,position=position,rotation=pose,bodies=bodies)


def force_column(ctx,name):
    ctx.observe(name+'_N',lambda:np.linalg.norm(ctx.action_values.get(name,{}).get('force',np.zeros(3))),unit='N')


def pull_comparison(ctx):
    body,cap=setup(ctx,height=.055,ground=True)
    initial_pen(ctx,ctx.asset,(0,.035,.055))
    loose=ctx.spawn(ctx.asset_path,name='loose')
    initial_pen(ctx,loose,(0,-.035,.055),opening=LOOSE_OPENING)
    pens=[ctx.asset,loose];caps=[p.body('cap') for p in pens]
    observe(ctx,pens)
    names=['tight_pull','loose_pull']
    for name in names:force_column(ctx,name)
    ctx.plots('tight_pull_N','loose_pull_N','asset.cap.position.x','loose.cap.position.x')
    ctx.camera(position=(-.04,-.46,.31),target=(-.04,0,.025),focal_length=26)
    ctx.video_annotation=lambda:f'Tight: far row +Y | Loose: near row -Y | pull world -X | phase {ctx.input_state["phase"]}: 0 settle, 1 pull, 2 catch, 3 place, 4 released'
    hands=[Hand(ctx,caps[i],f'cap_guide_{i}') for i in range(2)]
    for hand in hands:hand.axial()
    records=[None,None]
    with ctx.fixed(body),ctx.fixed(loose.body('body')):
        ctx.run(.4);ctx.input_state['phase']=1;start=ctx.session.time
        while ctx.session.time-start<10 and any(r is None for r in records):
            force=min(.75*(ctx.session.time-start),5)
            ctx.input_state['force']={names[i]:(caps[i],np.array([-force,0,0])) for i in range(2) if records[i] is None}
            ctx.step()
            for i in range(2):
                if records[i] is not None:continue
                b,c=pens[i].body('body'),caps[i]
                # The mouth is the cap's most positive-X collision boundary.
                # Full separation requires it to pass the actual nib tip by 2 mm.
                gap=float((b.state['position']+b.state['rotation'][:,0]*BODY_TIP_X-c.state['position'])@b.state['rotation'][:,0])
                if gap>=.002:
                    records[i]={'fully_separated_time_s':ctx.session.time,'last_pull_N':force,'mouth_to_tip_gap_m':gap,
                                'opening_m':fit(b,c)['opening']}
                    ctx.input_state['force'].pop(names[i],None)
                    hands[i].move(c.state['position']);hands[i].axial(2000,40)
        ctx.input_state['force']={};assert all(records),'a cap did not fully separate within 5 N'
        ctx.input_state['phase']=2;ctx.run(.6)
        for hand in hands:hand.move(hand.body.state['position'])
        ctx.input_state['phase']=3
        targets=[np.array([h.position[0],h.position[1],.0105]) for h in hands]
        move(ctx,hands,targets,1.5,dwell=.4)
        for hand in hands:hand.release()
        ctx.input_state['phase']=4;ctx.run(.8)
    ctx.experiment_details.update(pulls=dict(zip(['tight','loose'],records)),loose_initial_opening_m=LOOSE_OPENING,
        cap_guide='native D6 lateral/rotational drives; axial stiffness and damping zero throughout force measurement, 2000 N/m and 40 N s/m only after complete separation',
        separation_geometry={'body_tip_local_x_m':BODY_TIP_X,'cap_mouth_local_x_m':0,'required_axial_gap_m':.002},
        placed_cap_positions_m=[c.state['position'].tolist() for c in caps])


def close(ctx):
    body,cap=setup(ctx,height=.04,ground=True)
    initial_pen(ctx,ctx.asset,(0,0,.04),opening=LOOSE_OPENING);observe(ctx)
    force_column(ctx,'push')
    ctx.plots('asset.cap.position.x','push_N')
    ctx.camera(position=(-.025,-.34,.23),target=(-.02,0,.04),focal_length=28)
    ctx.video_annotation=lambda:f'Close world +X | phase {ctx.input_state["phase"]}: 0 settle, 1 light <0.5 N, 2 increasing push, 3 external force removed'
    guide=Hand(ctx,cap,'closing_guide');guide.axial(0,40)
    with ctx.fixed(body):
        ctx.run(.4);ctx.input_state['phase']=1
        ctx.input_state['force']={'push':(cap,np.array([.3,0,0]))};ctx.run(1.6)
        weak=fit(body,cap);vx=float(cap.state['velocity'][0])
        tail=[row['asset.cap.position.x'] for row in ctx.rows if row['time']>=ctx.session.time-.4]
        drift=max(tail)-min(tail)
        # TGS can report a small end-of-step velocity while the contact pose
        # is stationary; verify both the velocity and 0.4 s position history.
        pose_v=(tail[-1]-tail[0])/.4
        assert .001<weak['opening']<LOOSE_OPENING-.0003 and abs(vx)<.01 and drift<.00015 and abs(pose_v)<.0004,'light push did not stop at the detent'
        ctx.input_state['phase']=2;start=ctx.session.time;seated_force=None
        while ctx.session.time-start<8:
            force=min(.3+.6*(ctx.session.time-start),5)
            ctx.input_state['force']={'push':(cap,np.array([force,0,0]))};ctx.step()
            if fit(body,cap)['opening']<.0002:
                seated_force=force;break
        assert seated_force is not None,'increasing push did not seat the cap'
        ctx.input_state['force']={};ctx.input_state['phase']=3;guide.release();ctx.run(1.2)
        final=fit(body,cap)
        assert abs(final['opening'])<.001 and np.linalg.norm(cap.state['velocity'][:3])<.01,'cap did not remain seated after force removal'
    ctx.experiment_details.update(light_push_N=.3,light_push_final_fit=weak,light_push_final_vx_m_s=vx,
        light_push_last_0_4s_x_drift_m=drift,light_push_pose_velocity_m_s=pose_v,seating_push_N=seated_force,final_fit=final,
        closing_guide='native D6 lateral/rotational guide; zero axial stiffness and 40 N s/m axial damping to limit closing speed; no static axial holding force; fully released after seating')


def collision_min_z(ctx,pose):
    import json
    model=json.loads((ctx.asset_path.parent/'model.json').read_text())
    worlds={};values=[];root=rotation(pose)
    for node in model['nodes']:
        world=worlds.get(node['parent'],np.eye(4))@np.asarray(node['matrix']);worlds[node['name']]=world
        if node.get('role')!='collision':continue
        if node.get('shape')=='SPHERE':values.append(float((root@world[:3,3])[2]-node['radius']))
        else:
            points=np.c_[node['vertices'],np.ones(len(node['vertices']))]@world.T
            values.append(float((points[:,:3]@root.T)[:,2].min()))
    return min(values)


def drop_heights(ctx):
    pose=quaternion(35*pi/180,(.3,1,.2))
    setup(ctx,ground=True,pose=pose)
    low=collision_min_z(ctx,pose);heights=[.2,.5,1.];positions=[-.25,0,.25]
    pens=[ctx.asset,ctx.spawn(ctx.asset_path,name='drop_0_5m'),ctx.spawn(ctx.asset_path,name='drop_1m')]
    for p,h,x in zip(pens,heights,positions):initial_pen(ctx,p,(x,0,h-low),pose)
    observe(ctx,pens)
    ctx.plots('asset.body.position.z','drop_0_5m.body.position.z','drop_1m.body.position.z')
    ctx.camera(position=(0,-2.2,.62),target=(0,0,.62),focal_length=17)
    ctx.video_annotation=lambda:'Left 0.2 m | Centre 0.5 m | Right 1.0 m | initial lowest collision point above the same ground | simultaneous zero-velocity release'
    ctx.run(3)
    final=[fit(p.body('body'),p.body('cap')) for p in pens]
    assert all(np.isfinite(v) for row in ctx.rows for v in row.values()),'non-finite dynamics'
    assert all(0<p.body('body').state['position'][2]<.025 for p in pens),'a pen body did not settle on the ground'
    ctx.experiment_details.update(initial_lowest_collision_height_m=dict(zip([p.name for p in pens],heights)),
        initial_model_collision_min_z_m=low,initial_origin_positions_m=[[x,0,h-low] for x,h in zip(positions,heights)],
        simultaneous_release=True,initial_velocity_m_s=[0,0,0],final_fit=dict(zip([p.name for p in pens],final)),
        interpretation='4 ms impacts are coarse tests; transient penetration and any physical cap release are reported, with no prescribed detachment outcome')


def gravity_rotation(ctx):
    up=quaternion(pi/2)
    body,cap=setup(ctx,height=.25,ground=True,pose=up)
    initial_pen(ctx,ctx.asset,(-.11,0,.25),up)
    loose=ctx.spawn(ctx.asset_path,name='loose')
    initial_pen(ctx,loose,(.11,0,.25),up,LOOSE_OPENING)
    observe(ctx,[ctx.asset,loose])
    ctx.camera(position=(0,-.88,.4),target=(0,0,.18),focal_length=20)
    hands=[Hand(ctx,body,'tight_body_hand'),Hand(ctx,loose.body('body'),'loose_body_hand')]
    ctx.input_state['angle_deg']=0
    ctx.observe('rotation_deg',lambda:ctx.input_state['angle_deg'],unit='deg')
    ctx.plots('asset.opening','loose.opening','rotation_deg')
    ctx.video_annotation=lambda:f'Left: tight | Right: loose 3 mm outside seat | body-only physical grips | turn {ctx.input_state["angle_deg"]:.1f} deg from tip/cap up'
    ctx.run(.6)
    for k in range(round(6/ctx.dt)):
        angle=pi/2-pi*(k+1)*ctx.dt/6;ctx.input_state['angle_deg']=180*(k+1)*ctx.dt/6
        for hand in hands:hand.move(quaternion=quaternion(angle))
        ctx.step()
    ctx.run(1.4)
    result={'tight':fit(body,cap),'loose':fit(loose.body('body'),loose.body('cap'))}
    assert abs(result['tight']['opening'])<.0015 and result['loose']['opening']>.04,'gravity rotation did not retain tight cap and separate loose cap'
    ctx.experiment_details.update(final_fit=result,rotation_duration_s=6,angular_speed_deg_s=30,
        loose_initial_opening_m=LOOSE_OPENING,fixtures='only native compliant body grips; no cap fixture or scripted coupling')


TESTS={'pull_comparison':pull_comparison,'close':close,'drop_heights':drop_heights,'gravity_rotation':gravity_rotation}
