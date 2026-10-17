"""Official component relocation, pairing, stop/reset and unload integration."""
import argparse
import json
from pathlib import Path
import shutil
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gpu',type=int,default=0);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    portable=args.output/'relocated_asset';portable.mkdir(exist_ok=True)
    for name in ['object.usda','object.usdz','behavior.py','physics_test.py']:
        shutil.copyfile(args.asset/name,portable/name)
    from table_1000.sim.isaaclab.app import start_app
    app=start_app(gpu=args.gpu, cameras=False)
    failed=False
    try:
        import numpy as np
        from table_1000.physics.simulation import enable_scripting
        from table_1000.physics.testing import TestContext
        enable_scripting();ctx=TestContext(portable,name='runtime',app=app);ctx.simulation(gravity=(0,0,0))
        ctx.initial(position=(0,-.1,.15))
        b=ctx.spawn(portable,name='pen_b',position=(0,.1,.15))
        a=ctx.asset
        with ctx.fixed(a.body('body')):
            ctx.run(.04)
            assert a.behavior().male is a.behavior() and b.behavior().male is b.behavior()
            baseline=b.behavior().observations()['opening']
            ctx.force('weak_pull',a.body('cap'),keyframes=[{'time':.04,'force':(-.5,0,0)},{'time':.12,'force':(0,0,0)}])
            ctx.run(.12)
        error=abs(b.behavior().observations()['opening']-baseline);assert error<1e-6
        original=a.behavior()
        ctx.world.stop()
        for _ in range(5):app.update()
        assert original.subscription is None and original.male is None
        ctx.world.reset();ctx.session.initialize()
        ctx.session.time=0;ctx.session.steps=0
        ctx.actions.clear();ctx.run(.02)
        assert a.behavior().male is a.behavior() and b.behavior().male is b.behavior()
        before=(*b.view.get_world_poses(),b.view.get_velocities())
        ctx.world.stop();ctx.session.stage.RemovePrim(a.path)
        for _ in range(10):app.update()
        assert original.subscription is None and a.name not in ctx.session.instances
        ctx.world.play();ctx.session.initialize()
        b.view.set_world_poses(before[0],before[1]);b.view.set_velocities(before[2])
        ctx.run(.02)
        assert b.behavior().male is b.behavior()
        result={'execution_status':'completed','relocated_entry':str(portable/'object.usda'),
                'official_script_resolved':b.session.stage.GetPrimAtPath(b.path).GetAttribute('omni:scripting:scripts').Get()[0].resolvedPath,
                'unaffected_instance_opening_error_m':error,'stop_released_subscription':True,'reset_reengaged':True,
                'unload_removed_instance_and_subscription':True,'remaining_instance_continued':True,
                'GUI':'not verified; headless official component only'}
        ctx.close();(args.output/'runtime.json').write_text(json.dumps(result,indent=2)+'\n');print('OFFICIAL_RUNTIME_CHECK_COMPLETED',flush=True)
    except BaseException:
        failed=True
        import traceback
        (args.output/'error.txt').write_text(traceback.format_exc());raise
    finally:app.close(exit_code=int(failed))


if __name__=='__main__':main()
