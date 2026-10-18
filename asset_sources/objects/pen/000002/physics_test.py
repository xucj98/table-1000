"""Native detent acceptance through the standard asset test interface.

Four scenes exercise the frozen Claude native-contact mechanism. Grips and
locks hold bodies without geometry; they never write dynamic asset poses or
supply a hidden pen/cap connection. The pull is displacement-controlled and
reports the grip force; the close pushes with a force-limited grip. Springs
and masses are uncalibrated engineering proxies. Impact tests report outcomes.
"""
from math import cos, sin, pi
import numpy as np
from table_1000.physics.simulation import rotation

SEAT, BALL_RADIUS, BALL_X = -.032, .00865, -.0015
BODY_TIP_X, LOOSE_OPENING = -.071, .003
# Displacement-controlled pull, as on a tensile tester: 45 mm passes full
# separation (cap mouth 2 mm beyond the nib) from either initial opening.
PULL_SPEED, PULL_TRAVEL = .005, .045


def quaternion(angle, axis=(0,1,0)):
    axis = np.asarray(axis,dtype=float); axis /= np.linalg.norm(axis)
    return np.r_[cos(angle/2),axis*sin(angle/2)]


def fit(body, cap):
    b,c = body.state,cap.state
    local = b['rotation'].T @ (c['position']-b['position'])
    return {'opening':SEAT-local[0], 'radial':np.linalg.norm(local[1:]),
            'axis_cos':b['rotation'][:,0] @ c['rotation'][:,0]}


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
    ctx.input_state={'phase':0}
    ctx.observe('phase',lambda:ctx.input_state['phase'],unit='1')
    ctx.experiment_details={'mechanism':'native radial prismatic force drives and real ball/ridge contacts; no body/cap retention script or fixture',
        'asset_pose_writes_during_experiment':False,'force_direction':'world -X removes the cap; world +X closes it','ball_mass_note':'1 g per 0.8 mm ball is an inertia proxy, approximately 60x steel sphere mass',
        'external_substeps_per_step':1,'asset_position_iterations':32}
    return ctx.asset.body('body'),ctx.asset.body('cap')


def observe(ctx,instances=None):
    from pxr import PhysxSchema
    import carb
    from isaacsim.core.simulation_manager import SimulationManager
    carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
    instances=instances or [ctx.asset]
    for instance in instances:
        for name in instance.body_names:
            PhysxSchema.PhysxContactReportAPI.Apply(ctx.session.stage.GetPrimAtPath(instance.paths[name])).CreateThresholdAttr(0)
            ctx.observe_body(instance.body(name))
    paths=[instance.paths[name] for instance in instances for name in instance.body_names]
    filters=paths+[str(p.GetPath()) for p in ctx.session.stage.Traverse() if p.GetName()=='Ground']
    cache={'time':None, 'view':None}
    def contact_values():
        if cache['time'] is None:
            cache['view']=SimulationManager.get_physics_sim_view().create_rigid_contact_view(
                paths, [filters for _ in paths], max_contact_data_count=8192)
        if cache['time']!=ctx.session.time:
            forces,points,normals,distances,counts,starts=(value.detach().cpu().numpy()
                for value in cache['view'].get_contact_data(dt=ctx.dt))
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


def initial_pen(ctx,instance,position,pose=(1,0,0,0),opening=0):
    from pxr import UsdGeom
    ctx.initial(instance,position=position,rotation=pose)
    if opening:
        cache=UsdGeom.XformCache();axis=rotation(pose)[:,0]
        bodies={name:{'position':np.asarray(cache.GetLocalToWorldTransform(
            ctx.session.stage.GetPrimAtPath(instance.paths[name])).ExtractTranslation())-axis*opening}
            for name in ['cap','cap_ball1','cap_ball2','cap_ball3']}
        ctx.initial(instance,position=position,rotation=pose,bodies=bodies)


def pull_comparison(ctx):
    body,cap=setup(ctx,height=.055,ground=True)
    initial_pen(ctx,ctx.asset,(0,.035,.055))
    loose=ctx.spawn(ctx.asset_path,name='loose')
    initial_pen(ctx,loose,(0,-.035,.055),opening=LOOSE_OPENING)
    pens=[ctx.asset,loose];caps=[p.body('cap') for p in pens]
    observe(ctx,pens)
    grips=[ctx.grip(c,name=f'{n}_grip') for n,c in zip(['tight','loose'],caps)]
    for grip in grips:
        ctx.observe(grip.name+'.pull_N',lambda g=grip:-g.values()['force'][0],unit='N')
    ctx.plots('tight_grip.pull_N','loose_grip.pull_N','asset.cap.position.x','loose.cap.position.x')
    ctx.camera(position=(-.04,-.46,.31),target=(-.04,0,.025),focal_length=26)
    ctx.video_annotation=lambda:f'Tight: far row +Y | Loose: near row -Y | grips pull caps world -X at 5 mm/s | phase {ctx.input_state["phase"]}: 0 settle, 1 pull, 2 hold, 3 place, 4 released'
    records=[None,None];peaks=[(0.,0.),(0.,0.)]
    with ctx.fixed(body),ctx.fixed(loose.body('body')):
        ctx.run(.4);ctx.input_state['phase']=1
        starts=[c.state['position'].copy() for c in caps]
        motions=[g.follow(lambda t,p=g.position,q=g.quaternion:(p+[-PULL_SPEED*t,0,0],q),duration=PULL_TRAVEL/PULL_SPEED,wait=False)
                 for g in grips]
        while ctx.session.time<max(m.end for m in motions)-1e-9:
            ctx.step()
            for i,(b,c,g) in enumerate(zip([p.body('body') for p in pens],caps,grips)):
                pull=ctx.rows[-1][g.name+'.pull_N']
                if pull>peaks[i][0]:peaks[i]=(pull,float(starts[i][0]-c.state['position'][0]))
                if records[i] is not None:continue
                # The mouth is the cap's most positive-X collision boundary.
                # Full separation requires it to pass the actual nib tip by 2 mm.
                gap=float((b.state['position']+b.state['rotation'][:,0]*BODY_TIP_X-c.state['position'])@b.state['rotation'][:,0])
                if gap>=.002:
                    records[i]={'fully_separated_time_s':ctx.session.time,'mouth_to_tip_gap_m':gap,'opening_m':fit(b,c)['opening']}
        assert all(records),'a cap did not fully separate'
        for record,(peak,travel) in zip(records,peaks):
            record.update(peak_pull_N=peak,travel_at_peak_m=travel)
        ctx.input_state['phase']=2;ctx.run(.6)
        ctx.input_state['phase']=3
        ctx.wait(*[g.move_to(position=[c.state['position'][0],c.state['position'][1],.0105],duration=1.5,wait=False)
                   for g,c in zip(grips,caps)])
        ctx.run(.4)
        for grip in grips:grip.release()
        ctx.input_state['phase']=4;ctx.run(.8)
    ctx.experiment_details.update(pulls=dict(zip(['tight','loose'],records)),loose_initial_opening_m=LOOSE_OPENING,
        pull={'control':'displacement-controlled grip on each cap, world -X','speed_m_s':PULL_SPEED,'travel_m':PULL_TRAVEL,
              'force':'grip force along world -X, including cap inertia (negligible at this speed)'},
        separation_geometry={'body_tip_local_x_m':BODY_TIP_X,'cap_mouth_local_x_m':0,'required_axial_gap_m':.002},
        placed_cap_positions_m=[c.state['position'].tolist() for c in caps])


def close(ctx):
    body,cap=setup(ctx,height=.04,ground=True)
    initial_pen(ctx,ctx.asset,(0,0,.04),opening=LOOSE_OPENING);observe(ctx)
    grip=ctx.grip(cap,name='cap_grip')
    ctx.observe('push_N',lambda:grip.values()['force'][0],unit='N')
    ctx.plots('asset.cap.position.x','push_N')
    ctx.camera(position=(-.025,-.34,.23),target=(-.02,0,.04),focal_length=28)
    ctx.video_annotation=lambda:f'Close world +X | phase {ctx.input_state["phase"]}: 0 settle, 1 grip push limited to 0.3 N, 2 increasing limit, 3 released'
    with ctx.fixed(body):
        ctx.run(.4);ctx.input_state['phase']=1
        # Push along the pen axis toward the fully seated pose with at most
        # 0.3 N; the barrel guides the cap sideways, the grip holds its tilt.
        grip.axes(linear=(1,0,0))
        grip.limit(force=.3)
        seated=cap.state['position']+body.state['rotation'][:,0]*fit(body,cap)['opening']
        grip.move_to(position=seated,duration=.4);ctx.run(1.2)
        weak=fit(body,cap);vx=float(cap.state['velocity'][0])
        tail=[row['asset.cap.position.x'] for row in ctx.rows if row['time']>=ctx.session.time-.4]
        drift=max(tail)-min(tail)
        # TGS can report a small end-of-step velocity while the contact pose
        # is stationary; verify both the velocity and 0.4 s position history.
        pose_v=(tail[-1]-tail[0])/.4
        assert .001<weak['opening']<LOOSE_OPENING-.0003 and abs(vx)<.01 and drift<.00015 and abs(pose_v)<.0004,'light push did not stop at the detent'
        ctx.input_state['phase']=2;start=ctx.session.time;seated_force=None
        limit=lambda t:min(.3+.6*(t-start),5)
        grip.limit(force=limit)
        while ctx.session.time-start<8:
            ctx.step()
            if fit(body,cap)['opening']<.0002:
                # The cap snaps in once past the detent; the push it needed is
                # the largest before seating, not the push after the snap.
                peak=max(row['push_N'] for row in ctx.rows if row['time']>start)
                seated_force={'limit_N':limit(ctx.session.time),'peak_push_N':peak,'push_at_seating_N':ctx.rows[-1]['push_N']};break
        assert seated_force is not None,'increasing push did not seat the cap'
        ctx.input_state['phase']=3;grip.release();ctx.run(1.2)
        final=fit(body,cap)
        # As above, judge rest from the 0.4 s pose history, not the reported velocity.
        recent=[row for row in ctx.rows if row['time']>=ctx.session.time-.4]
        final_pose_v=float(np.linalg.norm([recent[-1][f'asset.cap.position.{a}']-recent[0][f'asset.cap.position.{a}'] for a in 'xyz'])/.4)
        assert abs(final['opening'])<.001 and final_pose_v<.01,'cap did not remain seated after release'
    ctx.experiment_details.update(light_push_N=.3,light_push_final_fit=weak,light_push_final_vx_m_s=vx,
        light_push_last_0_4s_x_drift_m=drift,light_push_pose_velocity_m_s=pose_v,seating_push=seated_force,final_fit=final,final_pose_velocity_m_s=final_pose_v,
        final_reported_velocity_m_s=float(np.linalg.norm(cap.state['velocity'][:3])),
        closing_grip='grip on the cap along world X only (sideways free, rotation held) toward the seated pose, force limited to 0.3 N, then raised 0.6 N/s up to 5 N; released after seating')


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
    # GPU TGS injects energy into the tiny maximal-coordinate ball constraints
    # on impact. PGS retains the original materials, joints and 4 ms step.
    ctx.simulation(gpu_solver='PGS')
    if ctx.physics == 'gpu':
        ctx.experiment_details['gpu_solver'] = 'PGS'
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
    grips=[ctx.grip(body,name='tight_body_grip'),ctx.grip(loose.body('body'),name='loose_body_grip')]
    ctx.input_state['angle_deg']=0
    ctx.observe('rotation_deg',lambda:ctx.input_state['angle_deg'],unit='deg')
    ctx.plots('asset.opening','loose.opening','rotation_deg')
    ctx.video_annotation=lambda:f'Left: tight | Right: loose 3 mm outside seat | grips hold bodies only | turn {ctx.input_state["angle_deg"]:.1f} deg from tip/cap up'
    ctx.run(.6)
    start=ctx.session.time
    def turn(t):
        ctx.input_state['angle_deg']=30*min(max(ctx.session.time-start,0),6)
        return quaternion(pi/2-pi*t/6)
    ctx.wait(*[g.follow(lambda t,p=g.position:(p,turn(t)),duration=6,wait=False) for g in grips])
    ctx.run(1.4)
    result={'tight':fit(body,cap),'loose':fit(loose.body('body'),loose.body('cap'))}
    assert abs(result['tight']['opening'])<.0015 and result['loose']['opening']>.04,'gravity rotation did not retain tight cap and separate loose cap'
    ctx.experiment_details.update(final_fit=result,rotation_duration_s=6,angular_speed_deg_s=30,
        loose_initial_opening_m=LOOSE_OPENING,fixtures='grips on the bodies only; no cap fixture or scripted coupling')


TESTS={'pull_comparison':pull_comparison,'close':close,'drop_heights':drop_heights,'gravity_rotation':gravity_rotation}
