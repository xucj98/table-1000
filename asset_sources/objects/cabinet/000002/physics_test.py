"""Pegboard cabinet native-contact experiments, SI units; engineering estimates.

Attachments use only the delivered straight pegs, never a permanent constraint.
Fixtures in drawer-only tests hold the housing; free tests expose its reaction.
"""
from functools import partial
import math
import numpy as np
from table_1000.physics.testing import TestContext

PREFIX = 'pegboard_cabinet.'
TRAVEL = .170

def view(ctx, target=(0,-.04,.18)):
    ctx.camera(position=(1.1,-1.5,.9),target=target)

def observe(ctx, instance=None):
    a=instance or ctx.asset
    for name in a.body_names:
        ctx.observe_body(a.body(name))
    for name in a.joint_names:
        ctx.observe_joint(a,name)

def pull_release(ctx: TestContext, side):
    view(ctx);joint=PREFIX+'drawer_'+side+'.slide'
    ctx.initial(joints={joint:.045});observe(ctx)
    ctx.video_annotation=lambda:f'{side} drawer: ramp 0-0.75 N, release at 2.1 s'
    ctx.plots('pull.force.y','asset.'+joint+'.position','asset.'+joint+'.velocity')
    ctx.experiment_details={'purpose':'gradual pull and coast to rest','side':side,'initial_joint_m':.045,'fixture':'test-only fixed housing','acceptance':'breakaway during ramp; after release decelerates and stops; 0..170 mm limits','parameters':'0.45/0.32 N static/dynamic axis friction; 2.5 N s/m damping; estimates'}
    with ctx.fixed(ctx.asset.body(PREFIX+'housing')):
        ctx.force('pull',ctx.asset.body(PREFIX+'drawer_'+side),keyframes=[{'time':0,'force':(0,0,0)},{'time':.5,'force':(0,0,0)},{'time':2,'force':(0,-.75,0)},{'time':2.1,'force':(0,0,0)},{'time':5,'force':(0,0,0)}])
        ctx.run(5)

def stops(ctx: TestContext):
    view(ctx);ctx.initial(joints={PREFIX+'drawer_left.slide':.085,PREFIX+'drawer_right.slide':.085});observe(ctx)
    ctx.video_annotation=lambda:'Both drawers: sustain +1 N at upper stop, then -1 N at lower stop'
    ctx.plots('left.force.y',*[f'asset.{PREFIX}drawer_{s}.slide.position' for s in ('left','right')])
    ctx.experiment_details={'purpose':'both limits under sustained load, both drawers','limits_m':[0,TRAVEL],'initial_m':.085,'acceptance':'reach both stops; sustained load causes no runaway beyond 0.5 mm solver tolerance'}
    with ctx.fixed(ctx.asset.body(PREFIX+'housing')):
        for side in ('left','right'):
            ctx.force(side,ctx.asset.body(PREFIX+'drawer_'+side),keyframes=[{'time':0,'force':(0,-1,0)},{'time':2,'force':(0,-1,0)},{'time':2.2,'force':(0,1,0)},{'time':4.2,'force':(0,1,0)},{'time':4.3,'force':(0,0,0)},{'time':5,'force':(0,0,0)}])
        ctx.run(5)

def free_pull(ctx: TestContext):
    ctx.camera(position=(1.6,-2.4,1.4),target=(0,-.45,.18));ctx.ground(contact_offset=.0001);observe(ctx)
    ctx.video_annotation=lambda:'Housing free on ground: drawer pull 0-16 N, release'
    ctx.plots('pull.force.y',f'asset.{PREFIX}drawer_right.slide.position',f'asset.{PREFIX}housing.position.y',f'asset.{PREFIX}housing.position.z')
    ctx.experiment_details={'purpose':'reaction on unanchored housing; sliding/tipping observed','fixture':None,'acceptance':'finite coherent motion; release does not reset velocity; no persistent ground penetration'}
    ctx.force('pull',ctx.asset.body(PREFIX+'drawer_right'),keyframes=[{'time':0,'force':(0,0,0)},{'time':.5,'force':(0,0,0)},{'time':3,'force':(0,-16,0)},{'time':3.2,'force':(0,0,0)},{'time':6,'force':(0,0,0)}]);ctx.run(6)

def gravity_tilt(ctx: TestContext):
    from pxr import Gf,UsdGeom,UsdPhysics
    view(ctx,target=(0,-.04,.4));ctx.initial(position=(0,0,.25));observe(ctx)
    h=ctx.asset.body(PREFIX+'housing')
    ctx.observe('tilt.angle',lambda:math.atan2(h.state['rotation'][2,1],h.state['rotation'][1,1]),unit='rad')
    ctx.plots('tilt.angle',*[f'asset.{PREFIX}drawer_{s}.slide.position' for s in ('left','right')])
    ctx.video_annotation=lambda:f'Gravity only: horizontal to 45 deg; tilt {math.degrees(math.atan2(h.state["rotation"][2,1],h.state["rotation"][1,1])):.1f} deg'
    ctx.ensure_initialized()
    grip=UsdGeom.Xform.Define(ctx.session.stage,'/World/Fixtures/tilt_grip')
    t=grip.AddTranslateOp();t.Set(Gf.Vec3d(*h.state['position']))
    q=grip.AddOrientOp();q.Set(Gf.Quatf(1))
    api=UsdPhysics.RigidBodyAPI.Apply(grip.GetPrim());api.CreateKinematicEnabledAttr(True)
    with ctx.fixed(h) as j:
        j.CreateBody0Rel().SetTargets([grip.GetPath()]);j.CreateLocalPos0Attr(Gf.Vec3f(0));j.CreateLocalRot0Attr(Gf.Quatf(1))
        old=ctx.session.actions
        def action(time,dt):
            old(time,dt)
            angle=math.pi/4*min(1,max(0,(time-1)/10))
            q.Set(Gf.Quatf(math.cos(angle/2),Gf.Vec3f(math.sin(angle/2),0,0)))
        ctx.session.actions=action
        ctx.run(13)
    starts={}
    for side in ('left','right'):
        key=f'asset.{PREFIX}drawer_{side}.slide.position';base=ctx.rows[0][key]
        r=next((r for r in ctx.rows if r[key]-base>=.0005),None)
        starts[side]=None if r is None else {'angle_degrees':math.degrees(r['tilt.angle']),'time_s':r['time'],'displacement_m':r[key]-base}
    ctx.experiment_details={'purpose':'measured gravity onset, not a specified target','criterion_m':.0005,'onset':starts,'fixture':'native kinematic housing clamp; drawers free; no drawer force','schedule':'1 s horizontal, 10 s tilt to 45 deg, 2 s hold'}

def rest_drop_multi(ctx: TestContext):
    ctx.camera(position=(1.5,-2.0,1.2),target=(0,-.2,.17));ctx.ground(contact_offset=.0001)
    ctx.initial(position=(-.23,0,0))
    other=ctx.spawn(ctx.asset_path,name='drop',position=(.23,0,.06))
    ctx.initial(instance=other,position=(.23,0,.06),rotation=(math.cos(.05),0,math.sin(.05),0))
    observe(ctx);observe(ctx,other)
    ctx.plots(f'asset.{PREFIX}housing.position.z',f'drop.{PREFIX}housing.position.z',f'asset.{PREFIX}tray.position.z',f'asset.{PREFIX}hook.position.z')
    ctx.video_annotation=lambda:'Two free instances: rest vs 60 mm tilted drop; straight peg attachments'
    ctx.experiment_details={'purpose':'rest, typical low drop, independent instances and native attachment seating','instances':2,'fixture':None,'acceptance':'rest remains stable, dropped cabinet settles; no persistent ground penetration; attachment failures reported separately'}
    ctx.run(6)

def attachments(ctx: TestContext):
    view(ctx);ctx.ground(contact_offset=.0001)
    loose=ctx.spawn(ctx.asset_path,name='unseated',position=(.42,0,0))
    ctx.initial(instance=loose,position=(.42,0,0))
    # The shared mixed-articulation pose setter only restores articulation
    # links. Author these free-body initial offsets through official USD API
    # before initialization, so warmup/reset also preserves this contrast.
    from pxr import Gf, UsdGeom
    for part in ('tray','hook'):
        op=UsdGeom.Xformable(ctx.session.stage.GetPrimAtPath(loose.body(PREFIX+part).path)).GetOrderedXformOps()[0]
        matrix=Gf.Matrix4d(op.Get());p=matrix.ExtractTranslation();p[1]-=.030
        matrix.SetTranslateOnly(p);op.Set(matrix)
    observe(ctx);observe(ctx,loose)
    ctx.plots(f'asset.{PREFIX}tray.position.y',f'asset.{PREFIX}tray.position.z',f'asset.{PREFIX}hook.position.y',f'unseated.{PREFIX}tray.position.z')
    ctx.video_annotation=lambda:'Seated vs 30 mm withdrawn: gravity, eccentric 0.10 N tray load then 0.8 N tray / 0.005 N hook pull'
    ctx.experiment_details={'purpose':'native straight pegs: seated/unseated, hold, eccentric load and extraction','fixture':None,'load':'tray 0.10 N downward at left front for 1 s; then tray 0.8 N / hook 0.005 N forward; actual extraction and two reinstallations tested separately with grippers','acceptance':'report whether delivered geometry holds; no welded or world-fixed attachment; repeated installation requires successful baseline retention first'}
    ctx.force('eccentric',ctx.asset.body(PREFIX+'tray'),keyframes=[{'time':0,'force':(0,0,0),'point':(-.14,.025,.145)},{'time':1,'force':(0,0,0)},{'time':1.2,'force':(0,0,-.10)},{'time':2.2,'force':(0,0,-.10)},{'time':2.4,'force':(0,0,0)},{'time':6,'force':(0,0,0)}])
    for part in ('tray','hook'):
        effort=.8 if part=='tray' else .005
        ctx.force('extract_'+part,ctx.asset.body(PREFIX+part),keyframes=[{'time':0,'force':(0,0,0)},{'time':3,'force':(0,0,0)},{'time':3.4,'force':(0,-effort,0)},{'time':3.8,'force':(0,0,0)},{'time':6,'force':(0,0,0)}])
    ctx.run(6)


def attachment_reinstall(ctx):
    from pxr import Gf
    from table_1000.physics.testing import interpolate
    from contextlib import ExitStack
    ctx.camera(position=(1.1,-1.5,.9),target=(0,-.03,.20));ctx.ground(contact_offset=.0001)
    for n in ctx.asset.body_names:ctx.observe_body(ctx.asset.body(n))
    ctx.plots('asset.pegboard_cabinet.tray.position.y','asset.pegboard_cabinet.tray.position.z','asset.pegboard_cabinet.hook.position.y','asset.pegboard_cabinet.hook.position.z')
    ctx.video_annotation=lambda:'Native compliant grippers: withdraw, reinstall, release; repeat twice'
    with ctx.fixed(ctx.asset.body(PREFIX+"housing")):
        ctx.run(1)
        originals={'tray':np.array([-.060,.055,.144]),'hook':np.array([-.070,.094,.3365])}
        for cycle in range(2):
            start=ctx.session.time
            with ExitStack() as stack:
                joints=[]
                for n in ['tray','hook']:
                    b=ctx.asset.body('pegboard_cabinet.'+n)
                    j=stack.enter_context(ctx.spring(b,position=b.state['position'],rotation=b.state['quaternion'],linear_stiffness=(1000 if n=='hook' else 10000),linear_damping=(4 if n=='hook' else 80),angular_stiffness=(.2 if n=='hook' else 20),angular_damping=(.002 if n=='hook' else .5)))
                    j.GetLocalRot0Attr().Set(Gf.Quatf(1))
                    p0=b.state['position'].copy();p=originals[n];out=p.copy();out[1]-=.045
                    frames=[{'time':start,'position':p0},{'time':start+.6,'position':p},{'time':start+1.4,'position':out},{'time':start+1.8,'position':out},{'time':start+2.6,'position':p},{'time':start+3.2,'position':p}]
                    joints.append((j,frames))
                old=ctx.session.actions
                def action(t,dt):
                    old(t,dt)
                    for j,frames in joints:
                        v=interpolate(frames,t)
                        if v is not None:j.GetLocalPos0Attr().Set(Gf.Vec3f(*v['position']))
                ctx.session.actions=action
                ctx.run(3.2)
                ctx.session.actions=old
            ctx.run(1.2)
    ctx.experiment_details={'purpose':'two physical withdrawal / reinsertion / release cycles','fixture':'housing held by test-only native fixed joint; attachments grasped by native six-axis compliant grippers; no dynamic asset pose writes','target_schedule':'45 mm -Y withdrawal then return; release at original seat','gripper_parameters':{'tray':{'linear_stiffness_N_m':10000,'linear_damping_N_s_m':80,'angular_stiffness_Nm_rad':20,'angular_damping_Nm_s_rad':.5},'hook':{'linear_stiffness_N_m':1000,'linear_damping_N_s_m':4,'angular_stiffness_Nm_rad':.2,'angular_damping_Nm_s_rad':.002}},'release_preserves_state':True,'acceptance':'free attachment remains near installed seat after both releases; continuous trace/video review required'}

TESTS={'pull_release_left':partial(pull_release,side='left'),'pull_release_right':partial(pull_release,side='right'),'travel_stops':stops,'free_pull':free_pull,'gravity_tilt':gravity_tilt,'rest_drop_multi':rest_drop_multi,'attachments':attachments,'attachment_reinstall':attachment_reinstall}
