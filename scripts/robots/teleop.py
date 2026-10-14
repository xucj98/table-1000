"""Keyboard teleoperation of a robot in the check scene, in a browser.

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
                  'resolution': (800, 450)}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.runtime.app import start_app
    app = start_app(output, gpu=args.gpu, workers=4, renderer='RaytracedLighting', disable_viewport_updates=True)
    try:
        import numpy as np
        from table_1000.robots import load_spec, teleop
        from table_1000.robots.checks import check_scene, translation, TOP
        from table_1000.robots.kinematics import RobotModel
        from table_1000.robots.x1pro import HEAD_CAMERA, camera_mount
        from table_1000.runtime.cameras import Cameras
        from table_1000.scenes.stage import ROBOT_PATH
        from table_1000.sim.env import Env
        from table_1000.sim.isaac import IsaacBackend
        spec = load_spec(args.robot)
        link, local = camera_mount(ROOT / spec.urdf, HEAD_CAMERA)
        views = {**VIEWS, 'head': {'parent': f'{ROBOT_PATH}/{link}', 'local': local, 'focal_length_mm': 15,
                                   'resolution': (800, 450)}}
        scene = check_scene(spec)
        layout = scene.placement(box=translation(.45, -.15, TOP + .04), can=translation(.5, .05, TOP + .05),
                                 stick=translation(.4, -.3, TOP + .007, np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0.]])))
        backend = IsaacBackend(scene, layout, ROOT / 'assets', cameras=lambda stage: Cameras(stage, views, output / 'frames', fps=25, subframes=1))
        model = RobotModel(spec, output / 'planning')
        env = Env(scene, backend, spec, [model], modes={name: 'ee_delta' for name in spec.arms})
        teleop.run(env, layout, output, args.port, views)
    finally:
        app.close()


if __name__ == '__main__':
    main()
