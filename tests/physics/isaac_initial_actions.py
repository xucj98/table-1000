"""Actual initial COM velocities and fixed→release constraint mobility check."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gpu',type=int,default=0);args=parser.parse_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'create_new_stage':False,'active_gpu':args.gpu,'physics_gpu':args.gpu,
                       'multi_gpu':False,'disable_viewport_updates':True,'limit_cpu_threads':4})
    try:
        import numpy as np
        from table_1000.physics.simulation import enable_scripting
        from table_1000.physics.testing import TestContext
        enable_scripting()
        ctx=TestContext(args.asset,name='initial',app=app)
        ctx.simulation(gravity=(0,0,0))
        ctx.initial(position=(0,0,.15),linear_velocity=(1,2,3),angular_velocity=(0,0,2),bodies={
            'body':{'position':(.2,.3,.4),'rotation':(2**-.5,0,0,2**-.5)},
            'cap':{'position':(-.1,.2,.4),'linear_velocity':(4,5,6),'angular_velocity':(0,1,0)}})
        ctx.force('first_step',ctx.asset.body('cap'),keyframes=[
            {'time':0,'force':(1,0,0)},{'time':.008,'force':(1,0,0)}])
        ctx.ensure_initialized()
        a=ctx.asset;cx,cy,cz=a.local_com[a.indices['body']]
        com=np.array([.2-cy,.3+cx,.4+cz]);velocity=np.array([1-2*com[1],2+2*com[0],3,0,0,2])
        body,cap=a.body('body'),a.body('cap')
        np.testing.assert_allclose(body.state['com'],com,atol=1e-6,rtol=0)
        np.testing.assert_allclose(body.state['velocity'],velocity,atol=1e-6,rtol=0)
        np.testing.assert_allclose(cap.state['velocity'],[4,5,6,0,1,0],atol=1e-6,rtol=0)
        result={'initial_COM_velocity_max_error':float(np.max(np.abs(body.state['velocity']-velocity)))}
        a.view.set_velocities(np.zeros((2,6),dtype=np.float32))
        A,B=ctx.fixed(body),ctx.fixed(cap);ja,jb=A.__enter__(),B.__enter__()
        ctx.session.refresh();assert body.state['inverse_mass']==cap.state['inverse_mass']==0
        fixed_position=cap.state['position'].copy()
        ctx.step()
        np.testing.assert_allclose(cap.state['position'],fixed_position,atol=1e-6,rtol=0)
        np.testing.assert_allclose(cap.state['velocity'],np.zeros(6),atol=1e-6,rtol=0)
        A.__exit__(None,None,None)
        C=ctx.fixed(body);jc=C.__enter__()
        assert ja.GetPath()!=jc.GetPath() and jb.GetPrim().IsValid()
        C.__exit__(None,None,None);B.__exit__(None,None,None)
        ctx.session.refresh()
        np.testing.assert_allclose(body.state['inverse_mass'],1/a.mass[a.indices['body']],rtol=1e-6)
        finite=body.state['inverse_mass']
        ctx.step()
        assert ctx.rows[0]['first_step.force.x']==0
        assert ctx.rows[1]['first_step.force.x']==1
        np.testing.assert_allclose(cap.state['velocity'][0],.004/a.mass[a.indices['cap']],atol=1e-6,rtol=0)
        result.update(execution_status='completed',separate_fixture_paths=[str(j.GetPath()) for j in [ja,jb,jc]],
                      native_mass_unchanged_kg=a.mass.tolist(),released_inverse_mass=finite,
                      released_inverse_inertia_positive=bool(np.linalg.eigvalsh(body.state['inverse_inertia']).min()>0),
                      initial_actions_zero=True,first_formal_step_force_N=ctx.rows[1]['first_step.force.x'],
                      fixed_body_held_under_force=True,released_cap_velocity_x=cap.state['velocity'][0])
        ctx.close();args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
        print('INITIAL_ACTIONS_CHECK_COMPLETED',flush=True)
    except Exception:
        import traceback
        args.output.with_suffix('.error').write_text(traceback.format_exc());raise
    finally:app.close()


if __name__=='__main__':main()
