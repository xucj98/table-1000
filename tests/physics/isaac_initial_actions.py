"""Check initial COM velocities and overlapping fixture lifetimes in Isaac.

This loads an existing asset and checks initialization/USD constraints without
running a dynamics experiment or rendering.
"""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'create_new_stage': False, 'active_gpu': args.gpu,
                         'physics_gpu': args.gpu, 'multi_gpu': False,
                         'disable_viewport_updates': True, 'limit_cpu_threads': 4})
    try:
        import numpy as np
        from isaacsim.core.api import World
        from pxr import UsdPhysics
        from table_1000.physics.runtime import Runtime
        from table_1000.physics.test import initial_state
        from table_1000.physics.actions import Actions

        world = World(physics_dt=.004, rendering_dt=.04, stage_units_in_meters=1,
                      backend='numpy', device='cpu')
        world.get_physics_context().set_gravity(0)
        runtime = Runtime(world)
        instance = runtime.load(args.asset, 'asset', position=(0, 0, .15))
        runtime.initialize()

        # A translated body rotated 90 degrees about Z has its COM offset along
        # Y. Its linear velocity must include that final world COM position.
        initial_state(runtime, instance, {
            'position': [0, 0, .15], 'linear_velocity': [1, 2, 3], 'angular_velocity': [0, 0, 2],
            'rigid_bodies': {
                'body': {'position': [.2, .3, .4], 'rotation': [2**-.5, 0, 0, 2**-.5]},
                'cap': {'position': [-.1, .2, .4], 'linear_velocity': [4, 5, 6],
                        'angular_velocity': [0, 1, 0]},
            },
        })
        cx, cy, cz = instance.local_com[instance.indices['body']]
        expected_com = np.array([.2 - cy, .3 + cx, .4 + cz])
        expected_velocity = np.array([1 - 2 * expected_com[1], 2 + 2 * expected_com[0], 3, 0, 0, 2])
        body = runtime.state(instance.body_key('body'))
        cap = runtime.state(instance.body_key('cap'))
        np.testing.assert_allclose(body['com'], expected_com, atol=1e-6, rtol=0)
        np.testing.assert_allclose(body['velocity'], expected_velocity, atol=1e-6, rtol=0)
        np.testing.assert_allclose(cap['velocity'], [4, 5, 6, 0, 1, 0], atol=1e-6, rtol=0)
        result = {'initial_state': {'expected_body_com': expected_com.tolist(),
                                   'actual_body_com': body['com'].tolist(),
                                   'expected_body_velocity': expected_velocity.tolist(),
                                   'actual_body_velocity': body['velocity'].tolist(),
                                   'cap_velocity_override': cap['velocity'].tolist()}}

        runtime.reset()
        actions = Actions(runtime, instance, {
            'A': {'type': 'fixture', 'body': 'body', 'keyframes': [{'time': 0}, {'time': 1}]},
            'B': {'type': 'fixture', 'body': 'cap', 'keyframes': [{'time': 0}, {'time': 3}]},
            'C': {'type': 'fixture', 'body': 'body', 'keyframes': [{'time': 1}, {'time': 3}]},
        })
        actions(0, .004)
        a_path, b_path = (actions.fixtures[name]['path'] for name in ('A', 'B'))
        b_target = UsdPhysics.FixedJoint.Get(runtime.stage, b_path).GetBody1Rel().GetTargets()
        actions(1, .004)
        c_path = actions.fixtures['C']['path']
        assert not runtime.stage.GetPrimAtPath(a_path).IsValid()
        assert b_path != c_path and actions.fixtures['B']['path'] == b_path
        assert UsdPhysics.FixedJoint.Get(runtime.stage, b_path).GetBody1Rel().GetTargets() == b_target
        assert str(b_target[0]) == instance.paths['cap']
        assert str(UsdPhysics.FixedJoint.Get(runtime.stage, c_path).GetBody1Rel().GetTargets()[0]) == instance.paths['body']
        result['fixtures'] = {'removed_A': a_path, 'surviving_B': b_path, 'new_C': c_path,
                              'B_target_unchanged': str(b_target[0]), 'separate_constraints': True}
        actions.close()
        runtime.close()
        result['execution_status'] = 'completed'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print('INITIAL_ACTIONS_CHECK_COMPLETED', args.output, flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
