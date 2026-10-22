"""Pick-skill coverage on X1Pro: random trials or given placements, with optional videos.

``uv run python scripts/experts/eval_pick.py --trials 200 --gpus 0 1 --jobs-per-gpu 4 --output outputs/grasp-exp/eval``
``uv run python scripts/experts/eval_pick.py --case cube1 0.62 0.37 0 standing --video --output ...``

Random trials cycle through the test objects (``table_1000/experts/pick_eval.py``),
each placed by its seed anywhere on the table. ``--case NAME X Y YAW POSE`` (world
frame: table centre at the origin, robot on the -Y side) runs given placements.
Each part writes ``reports.jsonl``; the batch writes ``summary.json`` (success by
object and failure reasons) and ``trials.jsonl`` into ``--output``. ``--video``
records every trial as ``videos/<index>_<object>.mp4`` (a wide and a close view
side by side).
"""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

WIDE = {'position': (1.5, -1.45, 1.6), 'target': (0., -.1, .8), 'focal_length_mm': 18, 'resolution': (960, 720)}
CLOSE = {'position': (.5, .6, 1.2), 'target': (0., 0., .8), 'focal_length_mm': 24, 'resolution': (960, 720)}


def tasks(args):
    """``[(index, name, placement)]`` of the batch."""
    import numpy as np
    from table_1000.experts.pick_eval import OBJECTS, sample
    if args.case:
        return [(index, name, {'x': float(x), 'y': float(y), 'yaw': float(yaw), 'pose': pose})
                for index, (name, x, y, yaw, pose) in enumerate(args.case)]
    names = args.objects or list(OBJECTS)
    return [(index, names[index % len(names)], sample(np.random.default_rng([args.seed, index]), names[index % len(names)]))
            for index in range(args.trials)]


def worker(args):
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=args.gpu, workers=4, cameras=args.video)
    failed = False
    evaluator = None
    work = args.work
    work.mkdir(parents=True, exist_ok=True)
    try:
        import numpy as np
        from table_1000.experts.pick_eval import Evaluator, scene
        from table_1000.sim.isaaclab.video import Video
        views = {'wide': WIDE, 'close': CLOSE} if args.video else {}
        evaluator = Evaluator(scene(args.gap, args.lift, robot=args.robot), args.assets, work, ROOT, views,
                              args.random_pushes)
        selected = set(args.indices)
        with open(work / 'reports.jsonl', 'a') as out:
            for index, name, placement in tasks(args):
                if index not in selected:
                    continue
                video = None
                if args.video:
                    camera = evaluator.env.env.scene['close']
                    import torch
                    side = 1. if placement['x'] >= 0 else -1.
                    target = [placement['x'], placement['y'], .80]
                    eye = [placement['x'] + side * .35, placement['y'] + .30, 1.05]
                    camera.set_world_poses_from_view(torch.tensor([eye], dtype=torch.float32),
                                                     torch.tensor([target], dtype=torch.float32))
                    shutil.rmtree(work / 'frames', ignore_errors=True)
                    video = Video(evaluator.env, list(views), work / 'frames', 25)
                report = evaluator.trial(name, placement, args.seed * 100000 + index, video)
                report['index'] = index
                out.write(json.dumps(report, default=float) + '\n')
                out.flush()
                print(index, name, placement['pose'], report['status'], report.get('failure', {}).get('reason', ''),
                      f"{report['wall_s']:.0f} s", flush=True)
                if video is not None:
                    destination = args.output / 'videos'
                    destination.mkdir(parents=True, exist_ok=True)
                    files = video.encode(work / 'frames')
                    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', files['wide'], '-i',
                                    files['close'], '-filter_complex', 'hstack=inputs=2', '-c:v', 'libx264',
                                    '-pix_fmt', 'yuv420p', '-crf', '23',
                                    str(destination / f'{index:03d}_{name}_{report["status"]}.mp4')], check=True)
    except Exception:
        traceback.print_exc()
        failed = True
    if evaluator is not None:
        evaluator.close()
    app.close(exit_code=int(failed))


def summarize(reports):
    import numpy as np
    by_object = {}
    for report in reports:
        entry = by_object.setdefault(f"{report['object']} {report['pose']}", {'trials': 0, 'success': 0, 'failures': {}})
        entry['trials'] += 1
        if report['status'] == 'success':
            entry['success'] += 1
        else:
            reason = report['status'] if report['status'] == 'slipped' else report['failure']['reason']
            entry['failures'][reason] = entry['failures'].get(reason, 0) + 1
    for entry in by_object.values():
        entry['rate'] = entry['success'] / entry['trials']
    success = [r for r in reports if r['status'] == 'success']
    return {'trials': len(reports), 'success': len(success), 'rate': len(success) / len(reports) if reports else None,
            'by_object': dict(sorted(by_object.items())),
            'max_slip_m': max([r['hold']['max_slip_m'] for r in success], default=None),
            'max_penetration_m': max([r['max_penetration_m'] for r in reports], default=None),
            # Trials whose fingers went more than 0.5 mm into the object.
            'finger_penetration_trials': sum(any('robot' in pair and r['object'] in pair for pair in r['penetration'])
                                             for r in reports),
            'mean_sim_s': float(np.mean([r['sim_s'] for r in success])) if success else None,
            'mean_wall_s': float(np.mean([r['wall_s'] for r in reports])) if reports else None}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--trials', type=int, default=0)
    parser.add_argument('--objects', nargs='+', help='test objects of the random trials (default: all)')
    parser.add_argument('--case', nargs=5, action='append', metavar=('NAME', 'X', 'Y', 'YAW', 'POSE'))
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--gap', type=float, default=.12, help='robot base to table edge (m)')
    parser.add_argument('--lift', type=float, default=.6, help='starting torso lift (m)')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--random-pushes', type=int, default=8,
                        help='random push directions after the six axes (0: a quicker run)')
    parser.add_argument('--robot', default='x1pro', help='robot name (asset_sources/robots/<name>)')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets', help='asset root holding robots/<name>')
    parser.add_argument('--gpus', type=int, nargs='+', default=[0])
    parser.add_argument('--jobs-per-gpu', type=int, default=1)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--indices', type=int, nargs='+', help=argparse.SUPPRESS)
    parser.add_argument('--work', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--gpu', type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    args.output = args.output.resolve()
    if args.worker:
        worker(args)
        return
    from table_1000.scenes.batch import run_parts, split
    indices = [index for index, _, _ in tasks(args)]
    if not indices:
        parser.error('give --trials or --case')
    shutil.rmtree(args.output, ignore_errors=True)
    args.output.mkdir(parents=True)
    passthrough = ['--seed', str(args.seed), '--gap', str(args.gap), '--lift', str(args.lift),
                   '--output', str(args.output), '--robot', args.robot, '--assets', str(args.assets.resolve())]
    if args.trials:
        passthrough += ['--trials', str(args.trials)]
    if args.objects:
        passthrough += ['--objects', *args.objects]
    for case in args.case or ():
        passthrough += ['--case', *case]
    if args.video:
        passthrough.append('--video')
    passthrough += ['--random-pushes', str(args.random_pushes)]
    jobs = []
    for part, chunk in enumerate(split(indices, len(args.gpus) * args.jobs_per_gpu)):
        work = args.output / 'parts' / f'part{part}'
        command = [sys.executable, str(Path(__file__).resolve()), '--worker', '--work', str(work),
                   '--indices', *map(str, chunk), *passthrough]
        jobs.append((command, args.output / 'parts' / f'part{part}.log', f'part{part}'))
    started = time.perf_counter()
    failed = run_parts(jobs, args.gpus, args.jobs_per_gpu)
    reports = [json.loads(line) for path in sorted((args.output / 'parts').glob('part*/reports.jsonl'))
               for line in path.read_text().splitlines()]
    reports.sort(key=lambda report: report['index'])
    with open(args.output / 'trials.jsonl', 'w') as out:
        for report in reports:
            out.write(json.dumps(report, default=float) + '\n')
    summary = {'gap': args.gap, 'lift': args.lift, 'seed': args.seed, 'robot': args.robot,
               **summarize(reports),
               'total_s': time.perf_counter() - started}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=1, default=float) + '\n')
    print(json.dumps({key: summary[key] for key in ('trials', 'success', 'rate', 'by_object')}, indent=1))
    if failed:
        raise SystemExit('failed: ' + ', '.join(failed))


if __name__ == '__main__':
    main()
