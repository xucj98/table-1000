"""Grip and fixture acceptance on an isotropic cube: holding, tracking, limits and release."""
import argparse
import json
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

MASS, SIZE = .1, .02
INERTIA = MASS*SIZE**2/6


def cube(path):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateNew(str(path))
    prim = UsdGeom.Cube.Define(stage, '/cube')
    prim.CreateSizeAttr(SIZE)
    stage.SetDefaultPrim(prim.GetPrim())
    prim.GetPrim().SetCustomDataByKey('table1000:name', 'cube')
    UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(prim.GetPrim())
    mass.CreateMassAttr(MASS)
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(INERTIA))
    stage.GetRootLayer().Save()
    return path


def quaternion(rotvec):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_rotvec(rotvec).as_quat()
    return (w, x, y, z)


def held_under_load(context):
    """A held cube carries its weight and applied loads; the recorded wrench balances them.

    Checked at rest and at a generic rotation of the held body.
    """
    import numpy as np
    load, moment = np.array([.3, -.2, .1]), np.array([2e-3, 1e-3, -3e-3])
    results = {}
    for name, rotvec in [('identity', (0, 0, 0)), ('generic', (.8, -1.9, 1.3))]:
        ctx = context('hold_'+name, rotation=quaternion(rotvec))
        body = ctx.asset.body('cube')
        ctx.grip(body, name='grip')
        ctx.force('load', body, keyframes=[{'time': .2, 'force': tuple(load)}, {'time': .6, 'force': tuple(load)}])
        ctx.torque('moment', body, keyframes=[{'time': .2, 'torque': tuple(moment)}, {'time': .6, 'torque': tuple(moment)}])
        ctx.run(.6)
        row = ctx.rows[-1]
        results[name] = {'force_N': [row[f'grip.force.{a}'] for a in 'xyz'],
                         'torque_N_m': [row[f'grip.torque.{a}'] for a in 'xyz'],
                         'position_error_m': row['grip.error.position'], 'angle_error_rad': row['grip.error.angle']}
    gains = ctx.grip_details()['grip']
    expected = -load + [0, 0, MASS*9.81]
    for name, value in results.items():
        np.testing.assert_allclose(value['force_N'], expected, atol=1e-2*np.linalg.norm(expected), err_msg=name)
        # GPU torque is a drive-law estimate; GPU PhysX reports no drive torque.
        tolerance = 1e-2 if ctx.physics == 'cpu' else .1
        np.testing.assert_allclose(value['torque_N_m'], -moment, atol=tolerance*np.linalg.norm(moment), err_msg=name)
        # Static deflection of the grip springs under the applied loads.
        assert value['position_error_m'] < 2*np.linalg.norm(load)/gains['linear_stiffness_N_per_m'], name
        assert value['angle_error_rad'] < 2*np.linalg.norm(moment)/gains['angular_stiffness_N_m_per_rad'], name
    return {**results, 'expected_force_N': expected.tolist(), 'expected_torque_N_m': (-moment).tolist(), 'gains': gains}


def tumbling_trajectory(context):
    """Track a translation circle and a tumbling rotation past 360 degrees and Y=90."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    ctx = context('tumble')
    body = ctx.asset.body('cube')
    grip = ctx.grip(body, name='grip')
    omega, duration, radius, rate = np.array([.6, 3., -1.2]), 2.6, .05, np.pi

    ctx.run(.1)
    origin = body.state['position'].copy()

    def path(t):
        # Ramp the angular rate up smoothly so the path starts at rest.
        s = t - (1-np.exp(-4*t))/4
        position = origin + [radius*(np.cos(rate*t)-1), radius*np.sin(rate*t), 0]
        return position, quaternion(omega*s)
    motion = grip.follow(path, duration=duration, wait=False)
    errors, angular, linear = [], [], []
    while ctx.session.time < motion.end - 1e-9:
        ctx.step()
        t = ctx.session.time - motion.start
        position, q = path(t)
        state = body.state
        errors.append((np.linalg.norm(state['position']-position),
                       (Rotation.from_quat(state['quaternion'][[1, 2, 3, 0]]) *
                        Rotation.from_quat(np.array(q)[[1, 2, 3, 0]]).inv()).magnitude()))
        if t > .5:
            angular.append(np.linalg.norm(state['velocity'][3:] - omega*(1-np.exp(-4*t))))
            linear.append(np.linalg.norm(state['velocity'][:3] - radius*rate*np.array([-np.sin(rate*t), np.cos(rate*t), 0])))
    total_angle = float(np.linalg.norm(omega)*(duration-(1-np.exp(-4*duration))/4))
    position_error, angle_error = map(float, np.max(errors, axis=0))
    result = {'total_rotation_rad': total_angle, 'max_position_error_m': position_error, 'max_angle_error_rad': angle_error,
              'max_angular_velocity_error_rad_s': float(max(angular)), 'max_linear_velocity_error_m_s': float(max(linear)),
              'commanded_angular_speed_rad_s': float(np.linalg.norm(omega)), 'commanded_linear_speed_m_s': radius*rate}
    assert total_angle > 2*np.pi
    assert position_error < 1e-3 and angle_error < 1e-2, result
    assert result['max_angular_velocity_error_rad_s'] < .05*np.linalg.norm(omega), result
    assert result['max_linear_velocity_error_m_s'] < .05*radius*rate, result
    return result


def force_limited_pull(context):
    """A force-limited grip pulling toward a far target equals a constant applied force."""
    import numpy as np
    pulled = {}
    for mode in ('grip', 'force'):
        ctx = context('limit_'+mode, gravity=(0, 0, 0))
        body = ctx.asset.body('cube')
        if mode == 'grip':
            grip = ctx.grip(body, name='grip')
            grip.limit(force=lambda t: .2 if t < .1 else .5)
            ctx.run(.1)
            grip.move_to(position=body.state['position']+(1, 0, 0), duration=.3)
            pulled['recorded_force_x_N'] = [row['grip.force.x'] for row in ctx.rows[-5:]]
        else:
            ctx.force('pull', body, keyframes=[{'time': .1, 'force': (.5, 0, 0)}, {'time': .4, 'force': (.5, 0, 0)}])
            ctx.run(.4)
        pulled[mode] = body.state['velocity'][:3].tolist()
    np.testing.assert_allclose(pulled['grip'], pulled['force'], atol=.01*pulled['force'][0], rtol=0)
    np.testing.assert_allclose(pulled['recorded_force_x_N'], .5, atol=1e-3)
    return pulled


def released_rotation(context):
    """A grip released at any orientation must leave all three rotations free.

    The cube is locked at each orientation, released, and driven by a world
    torque with components on all three axes. Its angular velocity must match
    a never-held cube; a lost rotational direction shows as a mismatch.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation
    torque = (1e-4, -.7e-4, 1.2e-4)
    orientations = {'identity': (0, 0, 0), 'y_plus_90': (0, np.pi/2, 0),
                    'y_minus_90': (0, -np.pi/2, 0), 'generic': (.8, -1.9, 1.3)}
    velocities = {}
    for name, rotvec in [('free', (0, 0, 0)), *orientations.items()]:
        ctx = context('rotation_'+name, gravity=(0, 0, 0))
        body = ctx.asset.body('cube')
        if name == 'free':
            ctx.ensure_initialized()
        else:
            with ctx.fixed(body, rotation=quaternion(rotvec)):
                ctx.run(.1)
            np.testing.assert_allclose(Rotation.from_quat(body.state['quaternion'][[1, 2, 3, 0]]).as_rotvec(),
                                       rotvec, atol=1e-5, rtol=0)
        start = ctx.session.time
        ctx.torque('spin', body, keyframes=[{'time': start, 'torque': torque}, {'time': start+.1, 'torque': torque}])
        ctx.run(.1)
        velocities[name] = body.state['velocity'][3:].copy()
    for name in orientations:
        np.testing.assert_allclose(velocities[name], velocities['free'],
                                   atol=2e-3*np.linalg.norm(velocities['free']), rtol=0, err_msg=name)
    return {name: value.tolist() for name, value in velocities.items()}


CHECKS = [held_under_load, tumbling_trajectory, force_limited_pull, released_rotation]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--physics', choices=('cpu', 'gpu'), default='cpu')
    parser.add_argument('--only')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=args.gpu, device='cpu' if args.physics == 'cpu' else 'cuda:0', cameras=False)
    results, contexts = {'physics': args.physics}, []
    try:
        from table_1000.physics.testing import TestContext
        asset = cube((args.output/'cube.usda').resolve())

        def context(name, gravity=(0, 0, -9.81), rotation=(1, 0, 0, 0)):
            while contexts:
                contexts.pop().close()
            ctx = TestContext(asset, name=name, app=app, physics=args.physics)
            contexts.append(ctx)
            ctx.simulation(gravity=gravity)
            # Keep the cube clear of everything; there is no ground.
            ctx.initial(position=(0, 0, .5), rotation=rotation)
            return ctx
        for check in CHECKS:
            if args.only and check.__name__ != args.only:
                continue
            try:
                results[check.__name__] = {'status': 'PASS', **check(context)}
            except BaseException:
                results[check.__name__] = {'status': 'FAIL', 'error': traceback.format_exc()}
            print('GRIP_CHECK', check.__name__, results[check.__name__]['status'], flush=True)
            (args.output/'grip.json').write_text(json.dumps(results, indent=2)+'\n')
    finally:
        while contexts:
            contexts.pop().close()
        app.close(exit_code=int(any(r.get('status') == 'FAIL' for r in results.values() if isinstance(r, dict))))


if __name__ == '__main__':
    main()
