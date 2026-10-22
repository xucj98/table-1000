"""Robot acceptance in Isaac Sim: physical-sense cases and comparison metrics.

``uv run python scripts/robots/check_robot.py x1pro --output outputs/robots/x1pro-check --gpu 1 [--video]``
writes ``report.json`` (every case with its checks, then the metrics) and, with
``--video``, one video of all cases per view: ``wide.mp4`` from outside and
``<camera>.mp4`` for every robot camera (depth as grey, white near). The robot's settings and cases come from
``asset_sources/robots/<name>/robot_test.py``; ``--cases a b`` runs only those
cases, ``--no-metrics`` skips the metrics, ``--trials 1 2 3`` sets the grasp trials of ``grasp_and_push``
(run separate processes on different trials in parallel). The exit status is 1 when a case does not pass
or the run fails. See ``table_1000/robots/checks.py``.
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
    parser.add_argument('--physics', choices=('cpu', 'gpu'), default='cpu')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--cases', nargs='*')
    parser.add_argument('--trials', type=int, nargs='+', default=[0],
                        help='grasp trials of grasp_and_push (0: at the work point; k > 0: random placement, seed k)')
    parser.add_argument('--no-metrics', action='store_true')
    parser.add_argument('--benchmark', action='store_true', help='only the collision-cost benchmark')
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the robot source')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    started = time.perf_counter()
    # Robot camera images (the sensors case) and videos render with RTX Real-Time 2.0.
    device = 'cpu' if args.physics == 'cpu' else 'cuda:0'
    app = start_app(gpu=args.gpu, workers=4, device=device, extra_args=RT2_ARGS)
    report = {'robot': args.robot, 'physics': args.physics, 'device': device,
              'startup_s': time.perf_counter() - started, 'cases': {}}
    checker = None
    try:
        from table_1000.robots import load_robot_test, load_spec
        from table_1000.robots.checks import Checker
        from table_1000.sim.isaaclab import render
        render.configure()
        root = args.root.resolve()
        spec = load_spec(args.robot, root)
        # The outside view and every robot camera, one video each.
        checker = Checker(spec, load_robot_test(args.robot, root), output, args.assets.resolve(), root,
                          views=VIEW if args.video else None, video=args.video, device=device,
                          trials=args.trials)
        report['load_s'] = checker.load_s
        if args.benchmark:
            from table_1000.robots.checks import benchmark
            report['benchmark'] = benchmark(checker)
            args.cases, args.no_metrics = [], True
        for name in (checker.cases() if args.cases is None else args.cases):
            case_started = time.perf_counter()
            try:
                result = checker.run(name)
            except Exception:
                result = {'status': 'ERROR', 'error': traceback.format_exc()}
            result['wall_s'] = time.perf_counter() - case_started
            report['cases'][name] = result
            print(name, result['status'], flush=True)
            (output / 'report.json').write_text(json.dumps(report, indent=1, default=float))
        if not args.no_metrics:
            report['metrics'] = checker.run('metrics')
        if checker.video:
            report['video'] = checker.video.encode(output)
    except Exception:
        report['error'] = traceback.format_exc()
    report['passed'] = sum(c.get('status') == 'PASS' for c in report['cases'].values())
    report['total_s'] = time.perf_counter() - started
    (output / 'report.json').write_text(json.dumps(report, indent=1, default=float))
    if checker is not None:
        checker.env.close()
    # Closing ends the process: a failed case or run exits with status 1.
    failed = 'error' in report or any(case.get('status') != 'PASS' for case in report['cases'].values())
    app.close(exit_code=int(failed))


if __name__ == '__main__':
    main()
