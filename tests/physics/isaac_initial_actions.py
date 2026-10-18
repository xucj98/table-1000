"""Initial COM velocities, tensor fixture release and first-step action checks."""
import argparse
import json
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--physics', choices=('cpu', 'gpu'), default='cpu')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=args.gpu, device='cpu' if args.physics == 'cpu' else 'cuda:0', cameras=False)
    ctx = None
    failed = False
    try:
        import numpy as np
        from table_1000.physics.simulation import enable_scripting
        from table_1000.physics.testing import TestContext
        enable_scripting()
        ctx = TestContext(args.asset, name='initial', app=app, physics=args.physics)
        ctx.simulation(gravity=(0, 0, 0))
        ctx.initial(position=(0, 0, .15), linear_velocity=(1, 2, 3), angular_velocity=(0, 0, 2), bodies={
            'body': {'position': (.2, .3, .4), 'rotation': (2**-.5, 0, 0, 2**-.5)},
            'cap': {'position': (-.1, .2, .4), 'linear_velocity': (4, 5, 6), 'angular_velocity': (0, 1, 0)}})
        ctx.ensure_initialized()
        asset = ctx.asset
        cx, cy, cz = asset.local_com[asset.indices['body']]
        com = np.array([.2-cy, .3+cx, .4+cz])
        velocity = np.array([1-2*com[1], 2+2*com[0], 3, 0, 0, 2])
        body, cap = asset.body('body'), asset.body('cap')
        np.testing.assert_allclose(body.state['com'], com, atol=1e-6, rtol=0)
        np.testing.assert_allclose(body.state['velocity'], velocity, atol=1e-6, rtol=0)
        np.testing.assert_allclose(cap.state['velocity'], [4, 5, 6, 0, 1, 0], atol=1e-6, rtol=0)
        result = {'physics': args.physics,
                  'initial_COM_velocity_max_error': float(np.max(np.abs(body.state['velocity']-velocity)))}
        ctx.close()
        ctx = TestContext(args.asset, name='release', app=app, physics=args.physics)
        ctx.simulation(gravity=(0, 0, 0))
        body, cap = ctx.asset.body('body'), ctx.asset.body('cap')
        # Both constraints are authored before physics initialization. Release
        # changes tensors only, and leaves the other fixture active.
        with ctx.fixed(body) as body_grip, ctx.fixed(cap) as cap_grip:
            ctx.force('first_step', cap, keyframes=[
                {'time': 0, 'force': (-10, 0, 0)}, {'time': .08, 'force': (-10, 0, 0)}])
            ctx.ensure_initialized()
            assert body_grip.path != cap_grip.path
            assert body.state['inverse_mass'] == cap.state['inverse_mass'] == 0
            fixed_position = cap.state['position'].copy()
            ctx.step()
            np.testing.assert_allclose(cap.state['position'], fixed_position, atol=1e-5, rtol=0)
            cap_grip.release()
            ctx.session.refresh()
            assert body.state['inverse_mass'] == 0
            np.testing.assert_allclose(cap.state['inverse_mass'], 1/ctx.asset.mass[ctx.asset.indices['cap']])
            for _ in range(10):
                ctx.step()
            assert ctx.rows[0]['first_step.force.x'] == 0
            assert ctx.rows[1]['first_step.force.x'] == -10
            assert cap.state['velocity'][0] < -.01
            result.update(execution_status='completed', separate_fixture_paths=[body_grip.path, cap_grip.path],
                          native_mass_unchanged_kg=ctx.asset.mass.tolist(), initial_actions_zero=True,
                          fixed_body_held_under_force=True, released_cap_velocity_x=float(cap.state['velocity'][0]))
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        print('INITIAL_ACTIONS_CHECK_COMPLETED', flush=True)
    except BaseException:
        failed = True
        args.output.with_suffix('.error').write_text(traceback.format_exc())
        raise
    finally:
        if ctx is not None:
            ctx.close()
        app.close(exit_code=int(failed))


if __name__ == '__main__':
    main()
