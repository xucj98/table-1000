"""Initial COM velocities, tensor fixture release, released rotations and first-step action checks."""
import argparse
import json
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def released_rotation(app, args):
    """A grip released at any orientation must leave all three rotations free.

    An isotropic cube is held at each orientation, released, and driven by a
    world torque with components on all three axes. Its angular velocity must
    match a never-held cube; a lost rotational direction shows as a mismatch.
    """
    import numpy as np
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
    from scipy.spatial.transform import Rotation
    from table_1000.physics.testing import TestContext
    path = (args.output.parent/'released_rotation_cube.usda').resolve()
    stage = Usd.Stage.CreateNew(str(path))
    cube = UsdGeom.Cube.Define(stage, '/cube')
    cube.CreateSizeAttr(.02)
    stage.SetDefaultPrim(cube.GetPrim())
    cube.GetPrim().SetCustomDataByKey('table1000:name', 'cube')
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass.CreateMassAttr(.1)
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.1*.02**2/6))
    PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim()).CreateDisableGravityAttr(True)
    stage.GetRootLayer().Save()
    torque = (1e-4, -.7e-4, 1.2e-4)
    # Euler chains were singular at a relative Y rotation of +-90 degrees.
    orientations = {'identity': (0, 0, 0), 'y_plus_90': (0, np.pi/2, 0),
                    'y_minus_90': (0, -np.pi/2, 0), 'generic': (.8, -1.9, 1.3)}
    velocities = {}
    for name, rotvec in [('free', (0, 0, 0)), *orientations.items()]:
        x, y, z, w = Rotation.from_rotvec(rotvec).as_quat()
        ctx = TestContext(path, name='rotation_'+name, app=app, physics=args.physics)
        try:
            ctx.simulation(gravity=(0, 0, 0))
            body = ctx.asset.body('cube')
            if name == 'free':
                ctx.ensure_initialized()
            else:
                with ctx.fixed(body, rotation=(w, x, y, z)):
                    ctx.run(.1)
                np.testing.assert_allclose(Rotation.from_quat(body.state['quaternion'][[1, 2, 3, 0]]).as_rotvec(),
                                           rotvec, atol=1e-5, rtol=0)
            start = ctx.session.time
            ctx.torque('spin', body, keyframes=[{'time': start, 'torque': torque},
                                                {'time': start+.1, 'torque': torque}])
            ctx.run(.1)
            velocities[name] = body.state['velocity'][3:].copy()
        finally:
            ctx.close()
    for name in orientations:
        np.testing.assert_allclose(velocities[name], velocities['free'],
                                   atol=2e-3*np.linalg.norm(velocities['free']), rtol=0, err_msg=name)
    return {name: value.tolist() for name, value in velocities.items()}


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
            result.update(separate_fixture_paths=[body_grip.path, cap_grip.path],
                          native_mass_unchanged_kg=ctx.asset.mass.tolist(), initial_actions_zero=True,
                          fixed_body_held_under_force=True, released_cap_velocity_x=float(cap.state['velocity'][0]))
        ctx.close()
        ctx = None
        result['released_rotation_angular_velocity'] = released_rotation(app, args)
        result['execution_status'] = 'completed'
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
