"""Keyboard teleoperation of a robot in its check scene (``robot_test.py``), in a browser.

``"$ISAAC_PYTHON" scripts/robots/teleop.py x1pro --output outputs/robots/x1pro-teleop --gpu 1 --port 8765``
then open ``http://<server>:8765/`` (for example through ``ssh -L 8765:localhost:8765 <server>``).
Keys are listed in ``table_1000/robots/teleop.py``; the session's actions go to
``<output>/actions.jsonl``.
"""

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

VIEWS = {'wide': {'position': (1.9, -1.7, 1.7), 'target': (.25, 0., .8), 'focal_length_mm': 20,
                  'resolution': (640, 360)}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--render-every', type=int, default=5,
                        help='control steps per streamed frame; rendering sets the pace')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app = start_app(gpu=args.gpu, workers=4, extra_args=RT2_ARGS)
    try:
        import numpy as np
        from table_1000.robots import load_robot_test, load_spec, teleop
        from table_1000.robots.checks import Checker, check_scene
        from table_1000.sim.isaaclab import render
        from table_1000.sim.isaaclab.env import RobotEnv
        render.configure()
        spec, settings = load_spec(args.robot), load_robot_test(args.robot)
        scene = check_scene(spec, settings)
        # A box, a can and a stick on the table around the work point.
        on_table = lambda dx, dy, z, rotation=None: Checker.work_point_in(settings, dx, dy, z, rotation)
        layout = scene.placement(box=on_table(0., 0., .04), can=on_table(.05, .20, .05),
                                 stick=on_table(-.05, -.15, .007, np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0.]])))
        # The page shows the outside view and the robot's own rgb cameras.
        cameras = {name: camera for name, camera in spec.cameras.items() if camera.sensor.kind == 'rgb'}
        # Keys move parts by increments: TCP deltas for the arms, joint deltas for the lift and head.
        env = RobotEnv(scene, spec, ROOT / 'assets', cameras=cameras, views=VIEWS,
                       modes={'lift': 'delta', 'head': 'delta', **{name: 'ee_delta' for name in spec.arms}})
        teleop.run(env, layout, output, args.port, [*VIEWS, *cameras], args.render_every)
    finally:
        app.close()


if __name__ == '__main__':
    main()
