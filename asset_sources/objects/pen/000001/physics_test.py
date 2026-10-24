"""Four independent force/release checks of the approved click approximation."""
from table_1000.physics.pen_tests import click_tests
MASSES = {'pen.shell': 0.006, 'pen.button': 0.0004, 'pen.refill': 0.0016}
TESTS = click_tests(masses=MASSES, drop_dt=.001)


def environment_isolation(ctx):
    """Alternate full/shallow physical loads, with an idle peer in every world."""
    import numpy as np
    from table_1000.physics.pen_tests import _click_observe
    ctx.simulation(dt=.001,gravity=(0,0,0))
    ctx.initial(position=(0,-.025,.05))
    peer=ctx.spawn(ctx.asset_path,name='peer',position=(0,.025,.05))
    for a in [ctx.asset,peer]:_click_observe(ctx,a)
    ctx.camera(position=(.10,-.36,.22),target=(0,0,.05),focal_length=30)
    ctx.plots('env0.asset.button_press.position','env0.asset.refill_slide.position',
              'env0.asset.click.toggle_count','env0.peer.click.toggle_count')
    original=ctx.session.actions
    loads=np.array([1.2 if e%2==0 else .3 for e in range(ctx.num_envs)])
    def actions(t,dt):
        original(t,dt)
        gain=min((t-.1)/.2,1.,(1.-t)/.2) if .1<=t<1. else 0.
        ctx.session.force(ctx.asset.body('pen.button'),np.c_[-loads*gain,np.zeros((ctx.num_envs,2))])
    ctx.session.actions=actions
    with ctx.fixed(ctx.asset.body('pen.shell')),ctx.fixed(peer.body('pen.shell')):
        ctx.run(1.6)
    results=[]
    for e in range(ctx.num_envs):
        b=ctx.asset.behavior();idle=peer.behavior();expected=int(e%2==0)
        assert b.toggles_by_environment[e]==expected and idle.toggles_by_environment[e]==0
        assert bool(b.extended_by_environment[e])==bool(expected)
        results.append({'environment':e,'full_load_N':float(loads[e]),'toggles':int(b.toggles_by_environment[e]),'idle_toggles':int(idle.toggles_by_environment[e])})
    assert all(a.behavior().callback_count==ctx.session.steps for a in [ctx.asset,peer])
    ctx.experiment_details={'contract':'different physical inputs per environment, independent idle instance',
        'environment_checks':results,'runtime_USD_writes':False,'dynamic_pose_writes':False}
    ctx.metric('isolated_environments',ctx.num_envs,unit='1',label='异载荷独立状态环境数')


def behavior_lifecycle(ctx):
    """Official loader pause/reset/unload/reload; topology edits only stopped."""
    import numpy as np
    from pxr import Sdf
    from omni.kit.scripting import ScriptManager
    from table_1000.physics import simulation
    from table_1000.physics.pen_tests import _click_observe
    ctx.simulation(dt=.001,gravity=(0,0,0))
    ctx.initial(position=(0,-.025,.05))
    peer=ctx.spawn(ctx.asset_path,name='peer',position=(0,.025,.05))
    for a in [ctx.asset,peer]:_click_observe(ctx,a)
    # Equal/opposite external loads keep total momentum zero without a fixture.
    ctx.force('button',ctx.asset.body('pen.button'),keyframes=[{'time':0,'force':(0,0,0)},
        {'time':.1,'force':(-1.2,0,0)},{'time':.4,'force':(-1.2,0,0)},
        {'time':.5,'force':(0,0,0)},{'time':.8,'force':(0,0,0)}])
    ctx.force('shell',ctx.asset.body('pen.shell'),keyframes=[{'time':0,'force':(0,0,0)},
        {'time':.1,'force':(1.2,0,0)},{'time':.4,'force':(1.2,0,0)},
        {'time':.5,'force':(0,0,0)},{'time':.8,'force':(0,0,0)}])
    ctx.camera(position=(.10,-.36,.22),target=(0,0,.05),focal_length=30)
    ctx.plots('asset.button_press.position','asset.refill_slide.position','asset.click.toggle_count','peer.click.toggle_count')
    ctx.ensure_initialized()
    stage=ctx.session.stage;stage_id=stage.GetRootLayer().identifier
    saved={n:a.snapshot() for n,a in ctx.session.instances.items()}
    render=ctx.render;ctx.render=False
    def updates():
        if ctx.app:
            for _ in range(5):ctx.app.update()
    def pause_check():
        scripts=[ctx.asset.behavior(),peer.behavior()]
        state=[(b.callback_count,b.armed_by_environment.copy(),b.extended_by_environment.copy()) for b in scripts]
        ctx.world.pause();updates()
        for b,(count,armed,extended) in zip(scripts,state):
            assert b.callback_count==count and np.array_equal(b.armed_by_environment,armed) and np.array_equal(b.extended_by_environment,extended)
        ctx.world.play();updates();counts=[b.callback_count for b in scripts];ctx.step()
        assert [b.callback_count-c for b,c in zip(scripts,counts)]==[1,1]
    def stop():
        ctx.session.active=False;ctx.world._disable_app_control_on_stop_handle=True
        ctx.world.stop();updates()
    def start():
        ctx.world.reset();ctx.session.initialize()
        for n,a in ctx.session.instances.items():a.restore(saved[n])
        ctx.world.forward();ctx.session.refresh()
        error=max(float(np.max(np.abs(x-y))) for n,a in ctx.session.instances.items() for x,y in zip(a.snapshot(),saved[n]))
        assert error<1e-6,error
        ctx.session.time=0.;ctx.session.steps=ctx.session.callback_steps=0;ctx.session.error=None
        ctx.rows.clear();ctx.action_values.clear();ctx.experiment_started=None
        return error
    ctx.run(.4);assert ctx.asset.behavior().extended_by_environment.all()
    pause_check();ctx.run(.4)
    assert ctx.asset.behavior().extended_by_environment.all()
    pause_check()
    a,b=ctx.asset.behavior(),peer.behavior();stop()
    assert all(v.subscription is None and not v.armed_by_environment.any() and not v.toggles_by_environment.any() for v in [a,b])
    reset_error=start();ctx.run(.8)
    assert ctx.asset.behavior().toggles_by_environment.tolist()==[1]*ctx.num_envs
    stop();old=ctx.session
    for instance in [peer,ctx.asset]:
        for e in range(ctx.num_envs):stage.RemovePrim(old.replica_path(instance.path,e))
        updates()
    assert all(v.session is None and v.subscription is None for v in [a,b])
    assert not old.instances and old.key not in simulation._sessions
    assert old.prepare_subscription is None and old.flush_subscription is None
    ctx.session=simulation.session_for(stage);ctx.session.active=False;ctx.session.set_environments(ctx.num_envs)
    ctx.session.actions=ctx.apply_actions;ctx.session.fixtures=ctx.fixtures
    ctx.asset=ctx.spawn(ctx.asset_path,name='asset',position=(0,-.025,.05))
    peer=ctx.spawn(ctx.asset_path,name='peer',position=(0,.025,.05))
    for instance in [ctx.asset,peer]:
        for e in range(1,ctx.num_envs):assert Sdf.CopySpec(stage.GetRootLayer(),instance.path,stage.GetRootLayer(),ctx.session.replica_path(instance.path,e))
    updates();ctx.observers.clear();ctx.units.clear()
    for instance in [ctx.asset,peer]:_click_observe(ctx,instance)
    # Actions contain body handles; rebind them after the stopped reload.
    ctx.actions['button']['body']=ctx.asset.body('pen.button');ctx.actions['shell']['body']=ctx.asset.body('pen.shell')
    reload_error=start();assert ctx.asset.behavior() is not a and peer.behavior() is not b
    ctx.render=render;ctx.images.clear();ctx.integrating_seconds=0.
    ctx.run(.8)
    assert ctx.asset.behavior().toggles_by_environment.tolist()==[1]*ctx.num_envs
    assert peer.behavior().toggles_by_environment.tolist()==[0]*ctx.num_envs
    assert stage.GetRootLayer().identifier==stage_id
    assert all(v.behavior().callback_count==ctx.session.steps for v in [ctx.asset,peer])
    ctx.experiment_details={'contract':'official same-stage component lifecycle on current CPU/GPU framework',
        'pause_armed_and_extended_preserved':True,'resume_callbacks_per_step':1,
        'stop_cleared_callbacks_and_state':True,'reset_error':reset_error,'reload_error':reload_error,
        'unload_released_registration_and_session':True,'new_script_objects_after_reload':True,
        'validated_environments':ctx.num_envs,'media_scope':'final reloaded .8s; preceding lifecycle phases numerically checked',
        'performance_scope':'lifecycle diagnostic; excluded from steady-state benchmark','GUI':'headless official component only; old GUI evidence not reused'}
    ctx.metric('reset_error',reset_error,unit='1',label='复位最大状态标量误差')
    ctx.metric('reload_error',reload_error,unit='1',label='重载最大状态标量误差')


TESTS.update(environment_isolation=environment_isolation,behavior_lifecycle=behavior_lifecycle)
