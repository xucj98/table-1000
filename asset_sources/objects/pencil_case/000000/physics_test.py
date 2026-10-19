"""Force-based PC-01 experiments; official loading through TESTS/TestContext.

External loads and housing clamps are test fixtures. Interlocking lives in the
asset. Units are SI; sampled applied forces are not constraint reactions.
"""
from functools import partial
import math
import numpy as np
from table_1000.physics.testing import TestContext

SLIDER = 'pencil_case.unzip'
FRONT = 'pencil_case.front_lip_open'
BACK = 'pencil_case.back_lip_open'
RING = 'pencil_case.ring_swing'


def observe(ctx, instance):
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    for name in instance.joint_names:
        ctx.observe_joint(instance, name)
    def gap():
        front = instance.body('pencil_case.front_lip').state
        back = instance.body('pencil_case.back_lip').state
        f = front['position'] + front['rotation'] @ np.array([0, .0334, .020])
        b = back['position'] + back['rotation'] @ np.array([0, -.0334, .020])
        return float(np.linalg.norm(b-f))
    ctx.observe(instance.name+'.opening_m', gap, unit='m')
    ctx.observe(instance.name+'.interlock_mode', lambda: {
        None: -1, 'lips_locked': 0, 'opening_allowed': 1, 'slider_locked': 2
    }[instance.behavior().mode], unit='1')
    ctx.observe(instance.name+'.transitions', lambda: instance.behavior().transition_count, unit='1')


def setup(ctx, s=0., *, height=.10, gravity=(0,0,-9.81), angles=(0.,0.)):
    ctx.simulation(dt=.004, gravity=gravity)
    ctx.initial(position=(0,0,height), joints={SLIDER:s, FRONT:angles[0], BACK:angles[1]})
    ctx.camera(position=(.42,-.55,.40), target=(0,0,height+.045), focal_length=28)
    observe(ctx,ctx.asset)
    ctx.experiment_details={'mechanism':'PC-01 approved rigid-lip native-limit interlock',
        'approximation':'half-unzipped lips remain closed; no cloth or local opening',
        'asset_pose_writes_during_experiment':False,'parameters':'engineering estimates, not measured',
        'applied_loads':'world vectors; points on moving lip inner edges are in local body coordinates',
        'acceptance':'contract in README; automatic completion is not media acceptance'}


def pulse(ctx, name, body, vector, start, end, *, point=None):
    frames=[{'time':start,'force':vector},{'time':end,'force':vector}]
    if point is not None:
        frames[0]['point']=point
    ctx.force(name,body,keyframes=frames,point_frame='body')


def lips(ctx, prefix, start, end, magnitude=1.):
    pulse(ctx,prefix+'_front',ctx.asset.body('pencil_case.front_lip'),(0,-magnitude,0),start,end,point=(0,.0334,.020))
    pulse(ctx,prefix+'_back',ctx.asset.body('pencil_case.back_lip'),(0,magnitude,0),start,end,point=(0,-.0334,.020))


def checks(ctx):
    values=np.array([[v for v in row.values()] for row in ctx.rows])
    assert np.isfinite(values).all(),'non-finite state'
    for instance in ctx.session.instances.values():
        for name,low,high,tol in [(SLIDER,0,.216,.0005),(FRONT,0,math.pi/4,math.radians(1)),
                                  (BACK,-math.pi/4,0,math.radians(1)),(RING,-math.pi/2,math.pi/2,math.radians(1))]:
            key=f'{instance.name}.{name}.position'
            q=[r[key] for r in ctx.rows]
            assert min(q)>=low-tol and max(q)<=high+tol,(key,min(q),max(q))
        assert instance.behavior().error is None


def lip_state(ctx:TestContext,s):
    setup(ctx,s)
    lips(ctx,'open',.2,1.2)
    ctx.plots('open_front.force.y','asset.'+SLIDER+'.position','asset.'+FRONT+'.position',
              'asset.'+BACK+'.position','asset.opening_m','asset.interlock_mode')
    with ctx.fixed(ctx.asset.body('pencil_case.housing')):
        ctx.run(2.)
    checks(ctx)
    peak=max(max(abs(r['asset.'+FRONT+'.position']),abs(r['asset.'+BACK+'.position'])) for r in ctx.rows)
    if s<.2158:
        assert peak<=math.radians(1),'closed/half state opened'
        assert max(r['asset.opening_m'] for r in ctx.rows)-ctx.rows[0]['asset.opening_m']<=.001
    else:
        for name in (FRONT,BACK):
            assert max(abs(r['asset.'+name+'.position']) for r in ctx.rows)>=math.radians(30),'full-retraction lip did not open'


def slider_pull(ctx):
    setup(ctx,.108)
    ring=ctx.asset.body('pencil_case.pull_ring')
    pulse(ctx,'weak',ring,(.2,0,0),.2,1.2)
    pulse(ctx,'pull',ring,(.8,0,0),1.6,3.2)
    pulse(ctx,'reverse',ring,(-.8,0,0),3.6,5.6)
    ctx.plots('weak.force.x','pull.force.x','reverse.force.x','asset.'+SLIDER+'.position',
              'asset.'+SLIDER+'.velocity','asset.'+RING+'.position')
    with ctx.fixed(ctx.asset.body('pencil_case.housing')):
        ctx.run(6.4)
    checks(ctx)
    q='asset.'+SLIDER+'.position'
    assert max(abs(r[q]-.108) for r in ctx.rows if r['time']<=1.2)<=.001,'weak pull exceeds breakaway contract'
    assert max(r[q] for r in ctx.rows if r['time']<=3.2)>=.2155,'positive ring pull did not reach end'
    assert ctx.rows[-1][q]<=.0005,'negative ring pull did not close'
    assert abs(ctx.rows[-1]['asset.'+SLIDER+'.velocity'])<.01


def close_cycle(ctx, repeat=1):
    setup(ctx,0)
    ring=ctx.asset.body('pencil_case.pull_ring')
    for i in range(repeat):
        t=8.*i
        pulse(ctx,f'unzip{i}',ring,(.8,0,0),t+.2,t+1.8)
        lips(ctx,f'open{i}',t+2.,t+3.8)
        pulse(ctx,f'blocked_return{i}',ring,(-.8,0,0),t+3.,t+3.8)
        lips(ctx,f'close{i}',t+4.,t+5.,magnitude=-1.)
        pulse(ctx,f'zip{i}',ring,(-.8,0,0),t+5.2,t+7.)
        lips(ctx,f'closed_probe{i}',t+7.2,t+7.6)
    ctx.plots('asset.'+SLIDER+'.position','asset.'+FRONT+'.position','asset.'+BACK+'.position',
              'asset.opening_m','asset.interlock_mode')
    with ctx.fixed(ctx.asset.body('pencil_case.housing')):
        ctx.run(8.*repeat)
    checks(ctx)
    for i in range(repeat):
        t=8*i
        blocked=[r for r in ctx.rows if t+3.<=r['time']<=t+3.8]
        assert min(r['asset.'+SLIDER+'.position'] for r in blocked)>=.2151,'open lips allowed return'
        closed=[r for r in ctx.rows if t+7.2<=r['time']<=t+7.8]
        assert max(r['asset.'+SLIDER+'.position'] for r in closed)<=.0005,'cycle failed to close'
        assert max(max(abs(r['asset.'+FRONT+'.position']),abs(r['asset.'+BACK+'.position'])) for r in closed)<=math.radians(1)


def drop(ctx,side=False):
    setup(ctx,0,height=.02)
    if side:
        # Rotate +90deg around X; the ring extends to about -39mm Y.
        ctx.initial(position=(0,0,.059),rotation=(math.sqrt(.5),math.sqrt(.5),0,0))
    # Anticipate up to 0.63m/s * 4ms = 2.52mm travel at first impact.
    # Ground-only broad contact envelope; rest offset and bag geometry stay zero/unchanged.
    ctx.ground(static_friction=.55,dynamic_friction=.40,contact_offset=.003)
    ctx.plots('asset.pencil_case.housing.position.z','asset.pencil_case.housing.linear_velocity.z',
              'asset.'+SLIDER+'.position','asset.opening_m')
    ctx.run(3.)
    checks(ctx)
    tail=[r for r in ctx.rows if r['time']>=2.5]
    for name in ctx.asset.body_names:
        assert max(np.linalg.norm([r[f'asset.{name}.linear_velocity.{a}'] for a in 'xyz']) for r in tail)<.02


def free_pair(ctx):
    setup(ctx,0,gravity=(0,0,0))
    other=ctx.spawn(ctx.asset_path,name='idle',position=(0,.3,.1))
    ctx.initial(other,position=(0,.3,.1))
    observe(ctx,other)
    ctx.camera(position=(.95,-1.0,.85),target=(.16,.15,.14),focal_length=24)
    pulse(ctx,'ring_pull',ctx.asset.body('pencil_case.pull_ring'),(.08,0,0),.2,.6)
    def momentum():
        return sum(float(ctx.asset.mass[i])*ctx.asset.body(n).state['velocity'][0]
                   for i,n in enumerate(ctx.asset.body_names))
    ctx.observe('momentum_x',momentum,unit='kg*m/s')
    ctx.plots('ring_pull.force.x','momentum_x','asset.pencil_case.housing.position.x',
              'idle.pencil_case.housing.position.x','asset.interlock_mode','idle.interlock_mode')
    ctx.run(1.2)
    checks(ctx)
    impulse=sum(r['ring_pull.force.x']*ctx.dt for r in ctx.rows[1:])
    assert abs(ctx.rows[-1]['momentum_x']-impulse)<=abs(impulse)*.05
    assert abs(ctx.rows[-1]['idle.pencil_case.housing.position.x'])<1e-5


TESTS={'closed_lip_load':partial(lip_state,s=0.),'half_lip_load':partial(lip_state,s=.108),
       'open_lip_load':partial(lip_state,s=.216),'slider_pull':slider_pull,
       'close_cycle':close_cycle,'repeat_cycle':partial(close_cycle,repeat=3),
       'drop':drop,'drop_side':partial(drop,side=True),'free_pair':free_pair}
