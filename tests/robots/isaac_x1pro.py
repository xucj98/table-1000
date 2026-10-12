"""Isaac check of the built X1Pro: holds a pose at dt 0.004 and reports its cost.

``"$ISAAC_PYTHON" tests/robots/isaac_x1pro.py --robot assets/robots/x1pro/robot.usda --output outputs/x1pro-check``
writes ``result.json`` (bodies, collision shapes, joint tracking, step time) to the output.
"""

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

POSE = {'lift_joint': .40, 'right_arm_joint2': .5, 'right_arm_joint3': -.8,
        'left_arm_joint2': .5, 'left_arm_joint3': -.8,
        'right_arm_gripper_left_joint': .03, 'right_arm_gripper_right_joint': -.03}


def check(robot_usd, output, seconds):
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import Usd, UsdPhysics

    world = World(physics_dt=.004, rendering_dt=.04, stage_units_in_meters=1, backend='numpy', device='cpu')
    world.scene.add_default_ground_plane()
    add_reference_to_stage(str(robot_usd), '/World/Robot')
    stage = world.stage
    shapes = sum(1 for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/Robot'), Usd.TraverseInstanceProxies())
                 if prim.HasAPI(UsdPhysics.CollisionAPI))
    robot = world.scene.add(SingleArticulation('/World/Robot', name='robot'))
    started = time.perf_counter()
    world.reset()
    reset_seconds = time.perf_counter() - started
    names = robot.dof_names
    target = np.zeros(len(names))
    for name, value in POSE.items():
        target[names.index(name)] = value
    robot.set_joint_positions(target)
    robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=target))
    steps = round(seconds / .004)
    started = time.perf_counter()
    for _ in range(steps):
        world.step(render=False)
    step_seconds = time.perf_counter() - started
    q = robot.get_joint_positions()
    error = np.abs(q - target)
    worst = int(np.argmax(error))
    return {'status': 'PASS' if np.isfinite(q).all() and error.max() < .02 else 'FAIL',
            'dof': len(names), 'bodies': robot.num_bodies, 'collision_shapes': shapes,
            'reset_seconds': reset_seconds, 'steps': steps, 'step_ms': step_seconds / steps * 1e3,
            'realtime_factor': seconds / step_seconds,
            'max_tracking_error': float(error.max()), 'worst_joint': names[worst]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--seconds', type=float, default=2.)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.runtime.app import start_app
    app = start_app(output, gpu=args.gpu)
    try:
        result = check(args.robot.resolve(), output, args.seconds)
    except Exception:
        import traceback
        result = {'status': 'FAIL', 'error': traceback.format_exc()}
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    app.close()


if __name__ == '__main__':
    main()
