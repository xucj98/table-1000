"""Regression on cabinet/000002: native drawer tree plus independent tray/hook.

Requires its built object.usdz and Isaac. Outputs are diagnostic, not asset
acceptance. Exercises both fixture declaration orders and free-body overrides.
"""
import argparse
import json
from pathlib import Path
import sys
import traceback
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--asset', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--physics', choices=['cpu', 'gpu'], required=True)
    p.add_argument('--gpu', type=int, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=a.gpu, device='cpu' if a.physics == 'cpu' else 'cuda', cameras=True)
    import numpy as np
    from pxr import UsdPhysics
    from table_1000.physics.testing import TestContext
    prefix = 'pegboard_cabinet.'
    result = {'physics': a.physics, 'cases': {}}
    ctx = None
    failed = False
    try:
        for label, held in [('free', ()), ('hook_only', ('hook',)),
                            ('housing_only', ('housing',)),
                            ('accessories_first', ('hook', 'tray', 'housing')),
                            ('housing_first', ('housing', 'tray', 'hook'))]:
            ctx = TestContext(a.asset.resolve(), name=label, app=app, render=True,
                              physics=a.physics, num_envs=4 if a.physics == 'gpu' else 1)
            ctx.simulation(gravity=(0, 0, 0))
            targets = {'hook': (-.3, -.45, .5), 'tray': (.3, -.45, .5)}
            ctx.initial(position=(0, 0, .2), bodies={prefix+part: {'position': pos,
                        'linear_velocity': (.1, 0, 0) if part not in held else (0, 0, 0)}
                        for part, pos in targets.items()})
            ctx.observe_body(ctx.asset.body(prefix+'hook'))
            ctx.camera(position=(1.2, -1.9, 1.1), target=(0, -.15, .35))
            grips = {part: ctx.grip(ctx.asset.body(prefix+part), name=part+'_grip') for part in held}
            root_prim = ctx.session.stage.GetPrimAtPath(ctx.asset.articulation_root)
            assert root_prim.HasAPI(UsdPhysics.ArticulationRootAPI) == ('housing' not in held)
            assert ctx.asset.articulation_bodies == {prefix+n for n in ('housing', 'drawer_left', 'drawer_right')}
            ctx.ensure_initialized()
            mass = {part: grip.gains['held_mass_kg'] for part, grip in grips.items()}
            for part, expected in [('housing', 3.26), ('hook', .003), ('tray', .16)]:
                if part in mass:
                    np.testing.assert_allclose(mass[part], expected, rtol=1e-6)
            positions = ctx.asset.environment_states()[0]
            for part, target in targets.items():
                i = ctx.asset.indices[prefix+part]
                np.testing.assert_allclose(positions[:, i], np.broadcast_to(target, (ctx.num_envs, 3)), atol=2e-6, rtol=0)
                if part not in held:
                    velocities = ctx.asset.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, -1, 6)
                    np.testing.assert_allclose(velocities[:, i, :3], np.broadcast_to([.1, 0, 0], (ctx.num_envs, 3)), atol=1e-6)
            ctx.run(.08)
            motions = [g.move_to(position=g.position+np.array([.01, 0, 0]), duration=.12, wait=False) for g in grips.values()]
            if motions:
                ctx.wait(*motions)
            ctx.run(.08)
            for g in grips.values():
                g.release()
            ctx.run(.08)
            pose = ctx.asset.view.view.get_transforms().detach().cpu().numpy()
            assert np.isfinite(pose).all()
            np.testing.assert_allclose(np.linalg.norm(pose[:, 3:], axis=1), 1, atol=1e-5)
            ctx.images[0].save(a.output/(label+'-first.png'))
            ctx.images[-1].save(a.output/(label+'-last.png'))
            result['cases'][label] = {'status': 'passed', 'held_mass_kg': mass,
                                      'frames': len(ctx.images), 'environments': ctx.num_envs,
                                      'render_pass_counts': ctx.camera_instance.capture_render_counts}
            ctx.close()
            ctx = None
            (a.output/'result.json').write_text(json.dumps(result, indent=2))
            print('MIXED_CHECK', label, result['cases'][label], flush=True)
    except BaseException:
        failed = True
        result['error'] = traceback.format_exc()
        traceback.print_exc()
    finally:
        if ctx is not None:
            try:
                ctx.close()
            except Exception:
                failed = True
                result['close_error'] = traceback.format_exc()
        result['status'] = 'failed' if failed else 'passed'
        (a.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        app.close(exit_code=int(failed))


if __name__ == '__main__':
    main()
