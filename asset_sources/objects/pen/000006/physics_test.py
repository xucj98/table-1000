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


def two_instances(ctx: TestContext):
    ctx.simulation(gravity=(0,0,0))
    ctx.initial(position=(0,-.1,.15))
    other = ctx.spawn(ctx.asset_path,name='pen_b',position=(0,.1,.15))
    ctx.camera(position=(.18,-.6,.4),target=(0,0,.15))
    observe(ctx,ctx.asset);observe(ctx,other)
    ctx.force('weak_pull',ctx.asset.body('cap'),keyframes=[
        {'time':0,'force':(-.5,0,0)},{'time':.28,'force':(-.5,0,0)},{'time':.3,'force':(0,0,0)}])
    ctx.plots('weak_pull.force.x','asset.cap_fit.opening','pen_b.cap_fit.opening')
    with ctx.fixed(ctx.asset.body('body')),ctx.fixed(other.body('body')):
        ctx.run(.6)
    assert abs(other.behavior().observations()['opening']) < 1e-6
    assert ctx.asset.behavior().male is ctx.asset.behavior()
    assert other.behavior().male is other.behavior()


def exchanged_caps(ctx: TestContext):
    ctx.simulation(gravity=(0,0,0))
    ctx.initial(position=(0,-.1,.15),bodies={'cap':{'position':(-.032,.1,.15)}})
    other = ctx.spawn(ctx.asset_path,name='pen_b',position=(0,.1,.15))
    ctx.initial(other,position=(0,.1,.15),bodies={'cap':{'position':(-.032,-.1,.15)}})
    ctx.camera(position=(.18,-.6,.4),target=(0,0,.15))
    observe(ctx,ctx.asset);observe(ctx,other)
    ctx.plots('asset.cap_fit.opening','pen_b.cap_fit.opening')
    ctx.run(.6)
    assert ctx.asset.behavior().male is other.behavior()
    assert other.behavior().male is ctx.asset.behavior()


TESTS = {'pull':pull,'close':close,'pressure_hold':pressure_hold,
         'drop_0.2m_4ms':partial(drop,height=.2,dt=.004),'drop_0.5m_4ms':partial(drop,height=.5,dt=.004),
         'drop_1m_4ms':partial(drop,height=1,dt=.004),'drop_1m_2ms':partial(drop,height=1,dt=.002),
         'drop_1m_1ms':partial(drop,height=1,dt=.001),
         'free_cap':free_cap,'two_instances':two_instances,'exchanged_caps':exchanged_caps}
