"""Press to unlock native latch. All forces are test inputs, SI units.

No joint targets, collision filters, or asset poses change during tests.
"""
import math
import numpy as np
from table_1000.physics.testing import TestContext
from table_1000.physics.joints import frames

BODY='utility_knife.body'
CARRIAGE='utility_knife.carriage'
BUTTON='utility_knife.button'
SLIDE='utility_knife.slide'
PRESS='utility_knife.press'
MASS_KG=.020


def _observe(ctx, instance):
    for joint in (SLIDE,PRESS):ctx.observe_joint(instance,joint)
    for name in (BODY,CARRIAGE,BUTTON):ctx.observe_body(instance.body(name))
    ctx.observe(instance.name+'.speed',lambda:np.linalg.norm(instance.body(BODY).state['velocity'][:3]),unit='m/s')
    return instance.body(BUTTON)


def _q(ctx,instance=None):return frames(instance or ctx.asset,SLIDE)['position']
def _p(ctx,instance=None):return frames(instance or ctx.asset,PRESS)['position']
def _col(instance,joint):return instance+'.'+joint+'.position'


def _load(ctx,name,button,schedule):
    """Schedule time / axial extension force / downward pressure in body axes."""
    ctx.force(name,button,frame='body',keyframes=[{'time':t,'force':(-axial,0,-pressure)} for t,axial,pressure in schedule])


def _camera(ctx,wide=False):
    ctx.camera(position=(.25,-.35,.24) if wide else (.15,-.22,.16),target=(0,0,.006),focal_length=28)


def _stable(ctx,names):
    tail=[r for r in ctx.rows if r['time']>=ctx.session.time-.5-1e-9]
    for n in names:
        pos=np.array([[r[f'{n}.{BODY}.position.{a}'] for a in 'xyz'] for r in tail])
        assert max(r[n+'.speed'] for r in tail)<.01,n+' unsettled speed'
        assert np.linalg.norm(np.ptp(pos,axis=0))<.001,n+' unsettled position'


def lock_pressure(ctx:TestContext):
    """Equal axial loading: unpressed, shallow force, full press; zero gravity."""
    from contextlib import ExitStack
    ctx.simulation(dt=ctx.dt,gravity=(0,0,0))
    ctx.initial(position=(0,-.045,.015),joints={SLIDE:.010})
    shallow=ctx.spawn(ctx.asset_path,name='shallow',position=(0,0,.015))
    full=ctx.spawn(ctx.asset_path,name='full',position=(0,.045,.015))
    for a in (shallow,full):ctx.initial(a,position=(0,0 if a is shallow else .045,.015),joints={SLIDE:.010})
    for a,pressure in [(ctx.asset,0),(shallow,.28),(full,1.0)]:
        button=_observe(ctx,a)
        _load(ctx,a.name+'_load',button,[(0,0,0),(.20,0,pressure),(.32,0,pressure),(.36,.5,pressure),(.48,.5,pressure),(.52,0,pressure),(.64,0,pressure),(.68,0,0),(1.2,0,0)])
    _camera(ctx,True)
    ctx.video_annotation=lambda:'Bottom: unpressed | middle: shallow force | top: full press | same axial load'
    ctx.plots('full_load.force.z','asset_load.force.x',*[_col(n,SLIDE) for n in ('asset','shallow','full')],_col('full',PRESS))
    with ExitStack() as stack:
        for a in (ctx.asset,shallow,full):stack.enter_context(ctx.fixed(a.body(BODY)))
        ctx.run(1.2)
    metrics={}
    for a in (ctx.asset,shallow):
        q=[r[_col(a.name,SLIDE)] for r in ctx.rows]
        assert max(abs(x-.010) for x in q)<.0005
        assert max(r[_col(a.name,PRESS)] for r in ctx.rows)<.0004
        metrics[a.name]={'max_axial_error_m':max(abs(x-.010) for x in q)}
    assert max(r[_col('full',PRESS)] for r in ctx.rows)>.0012
    assert max(r[_col('full',SLIDE)] for r in ctx.rows)-.010>.005
    assert _p(ctx,full)<.0001
    ctx.metric('locked_error',max(v['max_axial_error_m'] for v in metrics.values())*1000,unit='mm',label='未按/浅按最大锁止偏差')
    ctx.metric('full_travel',(max(r[_col('full',SLIDE)] for r in ctx.rows)-.010)*1000,unit='mm',label='按足后轴向移动')
    ctx.experiment_details={'contract':'equal axial force; three pressure states, no pose writes','locked_error':metrics,'full_max_q_m':max(r[_col('full',SLIDE)] for r in ctx.rows)}


def cycle(ctx:TestContext):
    """Five complete press/push/release/pull/relock cycles, native contacts."""
    ctx.initial(position=(0,0,.015));button=_observe(ctx,ctx.asset)
    schedule=[(0,0,0)];checks=[]
    for i in range(10):
        t=.2+i*.8;direction=1 if i%2==0 else -1
        schedule.extend([(t,0,0),(t+.12,0,1.0),(t+.16,0,1.0),(t+.20,.12*direction,1.0),
                         (t+.44,.12*direction,1.0),(t+.48,0,1.0),(t+.56,0,1.0),(t+.60,0,0),(t+.76,0,0)])
        checks.append((t+.76,.020 if direction==1 else 0))
    schedule.append((8.8,0,0));_load(ctx,'operate',button,schedule);_camera(ctx)
    ctx.video_annotation=lambda:'Hold down to move | release at end stop | 5 extension/retraction cycles'
    ctx.plots('operate.force.z','operate.force.x',_col('asset',PRESS),_col('asset',SLIDE))
    with ctx.fixed(ctx.asset.body(BODY)):ctx.run(8.8)
    errors=[]
    for t,target in checks:
        r=min(ctx.rows,key=lambda r:abs(r['time']-t));errors.append(abs(r[_col('asset',SLIDE)]-target))
        assert errors[-1]<.0005,(t,r[_col('asset',SLIDE)],target)
        assert r[_col('asset',PRESS)]<.00015,(t,'button failed return')
    assert min(r[_col('asset',SLIDE)] for r in ctx.rows)>-.0005
    assert max(r[_col('asset',SLIDE)] for r in ctx.rows)<.0205
    # Vertical joint decouples blade height from button depression.
    height=[r[f'asset.{CARRIAGE}.position.z']-r[f'asset.{BODY}.position.z'] for r in ctx.rows]
    assert np.ptp(height)<.0001
    ctx.metric('cycle_end_error',max(errors)*1000,unit='mm',label='五次循环最大端点误差')
    ctx.metric('blade_height_range',float(np.ptp(height))*1000,unit='mm',label='刀片相对高度变化')
    ctx.experiment_details={'complete_cycles':5,'end_stop_errors_m':errors,'blade_relative_height_range_m':float(np.ptp(height))}


def loaded_unlock(ctx:TestContext):
    """Unlock while +/-0.5 N axial loads remain applied; then release/relock."""
    from contextlib import ExitStack
    ctx.initial(position=(0,-.03,.015),joints={SLIDE:.010})
    reverse=ctx.spawn(ctx.asset_path,name='reverse',position=(0,.03,.015));ctx.initial(reverse,position=(0,.03,.015),joints={SLIDE:.010})
    for a,direction in [(ctx.asset,1),(reverse,-1)]:
        _load(ctx,a.name+'_load',_observe(ctx,a),[(0,0,0),(.2,.5*direction,0),(.4,.5*direction,0),(.6,.5*direction,1.2),(.76,.5*direction,1.2),(.80,0,1.2),(.92,0,1.2),(.96,0,0),(1.6,0,0)])
    _camera(ctx,True);ctx.video_annotation=lambda:'Opposite axial loads applied before pressing; press to release both'
    ctx.plots('asset_load.force.x','asset_load.force.z',_col('asset',SLIDE),_col('reverse',SLIDE),_col('asset',PRESS))
    with ExitStack() as stack:
        for a in (ctx.asset,reverse):stack.enter_context(ctx.fixed(a.body(BODY)))
        ctx.run(1.6)
    for a,target in [(ctx.asset,.020),(reverse,0)]:
        pre=[r[_col(a.name,SLIDE)] for r in ctx.rows if r['time']<=.4]
        assert max(abs(x-.010) for x in pre)<.0005
        assert abs(_q(ctx,a)-target)<.0005
        assert _p(ctx,a)<.0001
    ctx.metric('loaded_unlock_end_error',max(abs(_q(ctx,a)-target) for a,target in [(ctx.asset,.020),(reverse,0)])*1000,unit='mm',label='带载解锁后最大端点误差')
    ctx.metric('press_force',1.2,unit='N',label='受测带载解锁按压力')
    ctx.experiment_details={'axial_load_N':.5,'press_load_N':1.2,'pressure_limit_N':1.4,'two_directions':True}


def relock_between(ctx:TestContext):
    """Release between slots: stay unlatched until physically moved into a slot."""
    ctx.simulation(dt=ctx.dt,gravity=(0,0,0))
    ctx.initial(position=(0,0,.015),joints={SLIDE:.0075,PRESS:.0015})
    button=_observe(ctx,ctx.asset)
    _load(ctx,'operate',button,[(0,0,1.0),(.16,0,1.0),(.20,0,0),(.48,0,0),(2.8,0,0)])
    _camera(ctx);ctx.video_annotation=lambda:'Release midway: not locked | gentle axial force finds next slot; no snapping'
    ctx.plots('operate.force.x','operate.force.z',_col('asset',SLIDE),_col('asset',PRESS))
    # Bounded external hand force, with each environment's own velocity feedback.
    # It moves slowly until the next slot, then releases; no asset state changes.
    original_actions=ctx.session.actions
    released=np.zeros(ctx.num_envs,dtype=bool)
    applied=[]
    def align(t,dt):
        original_actions(t,dt)
        force=np.zeros((ctx.num_envs,3))
        if .48<=t<2.20:
            view=ctx.asset.articulation.view
            index=view.shared_metatype.dof_names.index(ctx.asset.joint(SLIDE).GetName())
            q=view.get_dof_positions().detach().cpu().numpy()[:,index]
            v=view.get_dof_velocities().detach().cpu().numpy()[:,index]
            released[:] |= q>=.0099
            speed=np.minimum(.004,np.maximum(0,(.010-q)*4))
            effort=np.where(released,0,np.clip(100*(speed-v),-.20,.20))
            rotations=ctx.asset.environment_states()[1][:,ctx.asset.indices[BUTTON]]
            force=-rotations[:,:,0]*effort[:,None]
            ctx.session.force(button,force)
        applied.append(force.tolist())
        ctx.action_values['alignment']={'force':force[0]}
    ctx.session.actions=align
    for i,axis in enumerate('xyz'):
        ctx.observe('alignment.force.'+axis,lambda i=i:ctx.action_values.get('alignment',{}).get('force',np.zeros(3))[i],unit='N')
    ctx.plots('alignment.force.x','operate.force.z',_col('asset',SLIDE),_col('asset',PRESS))
    with ctx.fixed(ctx.asset.body(BODY)):ctx.run(2.8)
    ctx.session.actions=original_actions
    waiting=[r for r in ctx.rows if .3<=r['time']<=.48]
    assert max(abs(r[_col('asset',SLIDE)]-.0075) for r in waiting)<.0002
    assert min(r[_col('asset',PRESS)] for r in waiting)>.0004
    assert abs(_q(ctx)-.010)<.0005
    assert _p(ctx)<.0001
    ctx.metric('relock_position',_q(ctx)*1000,unit='mm',label='档间对槽后位置')
    ctx.metric('relock_press',_p(ctx)*1000,unit='mm',label='对槽后残余按压量')
    ctx.experiment_details={'release_between_slots_q_m':.0075,'final_q_m':_q(ctx),'final_p_m':_p(ctx),'no_snap_or_position_write':True,'alignment_controller':{'max_external_force_N':.20,'speed_limit_m_s':.004,'independent_feedback_per_environment':True,'released_by_q_m':.0099,'force_history_N':applied}}


def hold_gravity(ctx:TestContext):
    """Every slot in both vertical blade directions; +/-0.5 N lock proof."""
    from contextlib import ExitStack
    instances=[]
    for side,sign in enumerate((1,-1)):
        rotation=(math.sqrt(.5),0,sign*math.sqrt(.5),0)
        for i,q in enumerate((0,.005,.010,.015,.020)):
            name=f'hold_{side}_{i}'
            position=((i-2)*.027,(side-.5)*.06,.065)
            a=ctx.asset if side==0 and i==0 else ctx.spawn(ctx.asset_path,name=name,position=position,rotation=rotation)
            ctx.initial(a,position=position,rotation=rotation,joints={SLIDE:q})
            button=_observe(ctx,a);_load(ctx,a.name+'_proof',button,[(0,0,0),(.4,0,0),(.7,.5,0),(1.0,.5,0),(1.3,0,0),(1.6,-.5,0),(1.9,-.5,0),(2.2,0,0),(3.6,0,0)])
            instances.append((a,q))
    ctx.camera(position=(0,-.35,1.1),target=(0,0,.065),focal_length=65);ctx.video_annotation=lambda:'Five detents, two vertical directions | +/-0.5 N without pressing'
    ctx.plots(*[_col(a.name,SLIDE) for a,q in instances])
    with ExitStack() as stack:
        for a,q in instances:stack.enter_context(ctx.fixed(a.body(BODY)))
        ctx.run(3.6)
    errors={a.name:max(abs(r[_col(a.name,SLIDE)]-q) for r in ctx.rows) for a,q in instances}
    assert max(errors.values())<.0005,errors
    assert all(max(r[_col(a.name,PRESS)] for r in ctx.rows)<.0004 for a,q in instances)
    ctx.metric('vertical_lock_error',max(errors.values())*1000,unit='mm',label='五档两朝向最大锁止偏差')
    ctx.experiment_details={'detents_m':[0,.005,.010,.015,.020],'orientations':'blade vertically up/down','lock_load_N':.5,'max_q_errors_m':errors}


def free_reaction(ctx:TestContext):
    """Free assembly: external button impulse and untouched instance control."""
    ctx.simulation(dt=ctx.dt,gravity=(0,0,0));ctx.initial(position=(0,-.04,.06))
    idle=ctx.spawn(ctx.asset_path,name='idle',position=(0,.12,.06))
    _observe(ctx,ctx.asset);_observe(ctx,idle)
    # Subthreshold lateral load keeps the lock engaged; total momentum is checked.
    ctx.force('external',ctx.asset.body(BUTTON),keyframes=[{'time':0,'force':(0,0,0)},{'time':.2,'force':(0,0,0)},{'time':.24,'force':(0,.005,0)},{'time':.44,'force':(0,.005,0)},{'time':.48,'force':(0,0,0)}])
    ctx.camera(position=(.25,-.35,.30),target=(0,.04,.06));ctx.video_annotation=lambda:'Free assembly | side force on button | idle instance | zero gravity'
    ctx.plots('external.force.y',f'asset.{BODY}.linear_velocity.y',f'asset.{BUTTON}.linear_velocity.y',_col('asset',SLIDE),_col('asset',PRESS))
    ctx.run(1.2)
    impulse=sum(r['external.force.y'] for r in ctx.rows)*ctx.dt
    momentum=sum(float(ctx.asset.mass[ctx.asset.indices[n]])*ctx.asset.body(n).state['velocity'][1] for n in ctx.asset.body_names)
    error=abs(momentum-impulse)/impulse
    assert error<.02,error
    assert max(np.linalg.norm(idle.body(n).state['velocity'][:3]) for n in idle.body_names)<.002
    ctx.metric('momentum_error',error*100,unit='%',label='自由整体冲量动量相对误差')
    ctx.experiment_details={'external_impulse_N_s':impulse,'total_linear_momentum_kg_m_s':momentum,'relative_error':error,'world_anchor':False}


def drop_states(ctx:TestContext):
    """Low drops, retracted/extended and side orientations; no guaranteed shock lock."""
    ctx.ground(static_friction=.4,dynamic_friction=.3,contact_offset=.0015)
    ctx.initial(position=(-.10,0,.030))
    extended=ctx.spawn(ctx.asset_path,name='extended',position=(.04,0,.030));ctx.initial(extended,position=(.04,0,.030),joints={SLIDE:.020})
    side=ctx.spawn(ctx.asset_path,name='side',position=(.16,0,.040),rotation=(math.sqrt(.5),math.sqrt(.5),0,0))
    for a in (ctx.asset,extended,side):_observe(ctx,a)
    ctx.camera(position=(.32,-.43,.26),target=(.025,0,.02));ctx.video_annotation=lambda:'30 mm lowest-point drop | retracted, extended, side | observe any impact release'
    ctx.plots(*[f'{n}.{BODY}.position.z' for n in ('asset','extended','side')],*[_col(n,SLIDE) for n in ('asset','extended','side')])
    ctx.run(3.2);_stable(ctx,('asset','extended','side'))
    import json
    from table_1000.physics.test import ground_clearance
    contact=ground_clearance(json.loads((ctx.asset_path.parent/'model.json').read_text()),ctx.rows,ctx.session.instances,0)
    assert contact['maximum_geometry_penetration_m']<.002
    for n in ('asset','extended','side'):
        q=[r[_col(n,SLIDE)] for r in ctx.rows];assert min(q)>-.0005 and max(q)<.0205
    ctx.metric('ground_penetration',contact['maximum_geometry_penetration_m']*1000,unit='mm',label='低落放最大几何穿地')
    ctx.experiment_details={'drop_clearance_m':.030,'ground_contact':contact,'final_extension_m':{a.name:_q(ctx,a) for a in (ctx.asset,extended,side)},'impact_lock_not_guaranteed':True}


def push_release(ctx:TestContext):
    """Axial shell push/release, retracted and extended on an ideal surface."""
    ctx.ground(static_friction=.4,dynamic_friction=.3,contact_offset=.0015)
    ctx.initial(position=(0,-.025,.0002))
    extended=ctx.spawn(ctx.asset_path,name='extended',position=(0,.025,.0002))
    ctx.initial(extended,position=(0,.025,.0002),joints={SLIDE:.020})
    for a in (ctx.asset,extended):
        _observe(ctx,a)
        ctx.force(a.name+'_push',a.body(BODY),keyframes=[{'time':0,'force':(0,0,0)},{'time':.4,'force':(.04,0,0)},{'time':.8,'force':(.04,0,0)},{'time':1.2,'force':(.12,0,0)},{'time':1.24,'force':(0,0,0)}])
    ctx.camera(position=(.4,-.45,.3),target=(.10,0,.006))
    ctx.video_annotation=lambda:'Retracted / extended shell pushes; release 1.24s; watch latch and stop'
    ctx.plots('asset_push.force.x',f'asset.{BODY}.position.x',f'extended.{BODY}.position.x','asset.speed','extended.speed',_col('asset',SLIDE),_col('extended',SLIDE))
    ctx.run(3.2);_stable(ctx,('asset','extended'))
    for a in (ctx.asset,extended):
        assert a.body(BODY).state['position'][0]>.005
    assert abs(_q(ctx))<.0005 and abs(_q(ctx,extended)-.020)<.0005
    ctx.metric('shell_travel',ctx.asset.body(BODY).state['position'][0]*1000,unit='mm',label='收刀壳体推移距离')
    ctx.metric('final_speed',max(a.body(BODY).state['velocity'][:3].dot(a.body(BODY).state['velocity'][:3])**.5 for a in (ctx.asset,extended))*1000,unit='mm/s',label='两状态末态最大速度')
    ctx.experiment_details={'release_time_s':1.24,'two_extension_states':True}


def side_tip(ctx:TestContext):
    """High side load tips the new articulated shell; unloaded side control."""
    ctx.ground(static_friction=.4,dynamic_friction=.3,contact_offset=.0015)
    rotation=(math.sqrt(.5),math.sqrt(.5),0,0)
    ctx.initial(position=(.07,0,.0102),rotation=rotation)
    idle=ctx.spawn(ctx.asset_path,name='idle',position=(-.07,0,.0102),rotation=rotation)
    for a in (ctx.asset,idle):
        _observe(ctx,a)
        ctx.observe(a.name+'.tilt',lambda a=a:math.acos(float(np.clip(-a.body(BODY).state['rotation'][1,2],-1,1))),unit='rad')
    ctx.force('push',ctx.asset.body(BODY),point_frame='body',keyframes=[{'time':0,'force':(0,0,0),'point':(0,.009,.00465)}, {'time':.4,'force':(0,0,0)}, {'time':.64,'force':(0,.16,0)}, {'time':.84,'force':(0,.16,0)}, {'time':.88,'force':(0,0,0)}])
    ctx.camera(position=(.65,.42,.9),target=(0,.42,.005),focal_length=20)
    ctx.video_annotation=lambda:'New 3-body mechanism | high side push vs idle; no guaranteed impact lock'
    ctx.plots('push.force.y','asset.tilt','idle.tilt','asset.speed',_col('asset',SLIDE))
    ctx.run(3.2);_stable(ctx,('asset','idle'))
    assert max(r['asset.tilt'] for r in ctx.rows)>math.pi/4
    assert max(r['idle.tilt'] for r in ctx.rows)<math.radians(5)
    ctx.metric('peak_tilt',math.degrees(max(r['asset.tilt'] for r in ctx.rows)),unit='deg',label='受推实例峰值倾角')
    ctx.experiment_details={'high_push_N':.16,'final_q_m':_q(ctx),'peak_tilt_rad':max(r['asset.tilt'] for r in ctx.rows)}


def relock_slots(ctx:TestContext):
    """Spring return from fully pressed at each of the five detents, then proof loads."""
    from contextlib import ExitStack
    ctx.simulation(dt=ctx.dt,gravity=(0,0,0));instances=[]
    for i,q in enumerate((0,.005,.010,.015,.020)):
        a=ctx.asset if i==0 else ctx.spawn(ctx.asset_path,name=f'slot{i}',position=(0,(i-2)*.03,.015))
        ctx.initial(a,position=(0,(i-2)*.03,.015),joints={SLIDE:q,PRESS:.0015})
        button=_observe(ctx,a)
        _load(ctx,a.name+'_load',button,[(0,0,1.),(.16,0,1.),(.20,0,0),(.48,0,0),(.52,.5,0),(.72,.5,0),(.76,0,0),(.84,-.5,0),(1.04,-.5,0),(1.08,0,0),(3.2,0,0)])
        instances.append((a,q))
    ctx.camera(position=(.27,-.35,.27),target=(0,0,.015));ctx.video_annotation=lambda:'Fully pressed at five detents | release, spring return, +/-0.5 N proof'
    ctx.plots(*[_col(a.name,PRESS) for a,q in instances],*[_col(a.name,SLIDE) for a,q in instances])
    with ExitStack() as stack:
        for a,q in instances:stack.enter_context(ctx.fixed(a.body(BODY)))
        ctx.run(3.2)
    errors={a.name:max(abs(r[_col(a.name,SLIDE)]-q) for r in ctx.rows) for a,q in instances}
    assert max(errors.values())<.0005,errors
    for a,q in instances:
        assert max(r[_col(a.name,PRESS)] for r in ctx.rows)>.0012
        assert _p(ctx,a)<.0001
    ctx.metric('slot_return_error',max(errors.values())*1000,unit='mm',label='五档回弹受载最大偏差')
    ctx.experiment_details={'returned_and_proof_loaded_slots_m':[0,.005,.010,.015,.020],'q_error_m':errors}


TESTS={'lock_pressure':lock_pressure,'cycle':cycle,'loaded_unlock':loaded_unlock,
       'relock_between':relock_between,'relock_slots':relock_slots,'hold_gravity':hold_gravity,
       'free_reaction':free_reaction,'drop_states':drop_states,'push_release':push_release,'side_tip':side_tip}
