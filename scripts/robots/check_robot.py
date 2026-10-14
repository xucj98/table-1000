"""Robot acceptance in Isaac Sim: physical-sense cases and comparison metrics.

``"$ISAAC_PYTHON" scripts/robots/check_robot.py x1pro --output outputs/robots/x1pro-check --gpu 1 [--video]``
writes ``report.json`` (every case with its checks, then the metrics) and, with
``--video``, ``video.mp4`` of all cases. The robot's settings and cases come from
``asset_sources/robots/<name>/robot_test.py``; ``--cases a b`` runs only those
cases, ``--no-metrics`` skips the metrics. See ``table_1000/robots/checks.py``.
"""

import argparse
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

VIEW = {'wide': {'position': (1.9, -1.7, 1.7), 'target': (.25, 0., .8), 'focal_length_mm': 20,
                 'resolution': (960, 540)}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--cases', nargs='*')
    parser.add_argument('--no-metrics', action='store_true')
    parser.add_argument('--benchmark', action='store_true', help='only the collision-cost benchmark')
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the robot source')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaac.app import start_app
    from table_1000.sim.isaac.render import RT2_ARGS
    started = time.perf_counter()
    app = start_app(gpu=args.gpu, workers=4, renderer='RaytracedLighting', disable_viewport_updates=True,
                    extra_args=RT2_ARGS if args.video else ())
    report = {'robot': args.robot, 'startup_s': time.perf_counter() - started, 'cases': {}}
    checker = None
    try:
        from table_1000.robots import load_robot_test, load_spec
        from table_1000.robots.checks import Checker
        from table_1000.robots.controllers import CONTROL_HZ
        from table_1000.sim.isaac.cameras import Cameras
        cameras = (lambda stage: Cameras(stage, VIEW, output / 'frames', fps=CONTROL_HZ)) if args.video else None
        root = args.root.resolve()
        checker = Checker(load_spec(args.robot, root), load_robot_test(args.robot, root), output, args.assets.resolve(),
                          root, cameras)
        report['load_s'] = checker.load_s
        if args.benchmark:
            from table_1000.robots.checks import benchmark
            report['benchmark'] = benchmark(checker)
            args.cases, args.no_metrics = [], True
        for name in (checker.cases() if args.cases is None else args.cases):
            checker.case = name
            case_started = time.perf_counter()
            try:
                result = getattr(checker, name)()
            except Exception:
                result = {'status': 'ERROR', 'error': traceback.format_exc()}
            result['wall_s'] = time.perf_counter() - case_started
            report['cases'][name] = result
            print(name, result['status'], flush=True)
            (output / 'report.json').write_text(json.dumps(report, indent=1, default=float))
        if not args.no_metrics:
            checker.case = 'metrics'
            report['metrics'] = checker.metrics()
        if checker.backend.cameras:
            report['video'] = checker.backend.cameras.encode(output)
    except Exception:
        report['error'] = traceback.format_exc()
    report['passed'] = sum(c.get('status') == 'PASS' for c in report['cases'].values())
    report['total_s'] = time.perf_counter() - started
    (output / 'report.json').write_text(json.dumps(report, indent=1, default=float))
    if checker is not None:
        checker.backend.close()
    app.close()


if __name__ == '__main__':
    main()
