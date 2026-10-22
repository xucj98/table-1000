"""Pick-skill coverage on X1Pro: random trials or given placements, with optional videos.

``uv run python scripts/experts/eval_pick.py --trials 200 --gpus 0 1 --jobs 8 --output outputs/grasp-exp/eval``
``uv run python scripts/experts/eval_pick.py --case cube1 0.62 0.37 0 standing --video --output ...``

Random trials cycle through the test objects (``table_1000/experts/pick_eval.py``),
each placed by its seed anywhere on the table, or with ``--scenario`` among obstacles
(``box``: in an open box, ``side``/``back``/``corner``: walls beside and behind it). ``--case NAME X Y YAW POSE`` (world
frame: table centre at the origin, robot on the -Y side) runs given placements.
The trials run in ``--jobs`` processes at once: spread over ``--gpus`` (default one per
GPU), or each submitted through ``--launcher PREFIX``, a job scheduler that picks its GPU
and sets ``CUDA_VISIBLE_DEVICES`` (e.g. ``--launcher "srun --gres=gpu:1"``, default 16
jobs). Each trial writes ``trials/<index>.json``; trials whose process died before that run
again (``--retries``, default 1). The batch writes ``summary.json`` (success by object and
failure reasons) and ``trials.jsonl`` into ``--output``. ``--video`` records every trial as
``videos/<index>_<object>_<status>.mp4`` (a wide and a close view side by side).
"""

import argparse
import json
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

WIDE = {'position': (1.5, -1.45, 1.6), 'target': (0., -.1, .8), 'focal_length_mm': 18, 'resolution': (960, 720)}
LAUNCHED_JOBS = 16
CLOSE = {'position': (.5, .6, 1.2), 'target': (0., 0., .8), 'focal_length_mm': 24, 'resolution': (960, 720)}


def tasks(args):
    """``[(index, name, placement)]`` of the batch."""
    import numpy as np
    from table_1000.experts.pick_eval import OBJECTS, sample
    if args.case:
        if args.scenario != 'open':
            raise SystemExit('--case places objects in the open scenario only')
        return [(index, name, {'x': float(x), 'y': float(y), 'yaw': float(yaw), 'pose': pose})
                for index, (name, x, y, yaw, pose) in enumerate(args.case)]
    names = args.objects or list(OBJECTS)
    return [(index, names[index % len(names)], sample(np.random.default_rng([args.seed, index]), names[index % len(names)],
                                                         scenario=args.scenario))
            for index in range(args.trials)]


class Recorder:
    """Every step's frame of the env's views side by side, captioned, saved under ``frames``."""

    def __init__(self, env, views, frames):
        self.env, self.views, self.frames, self.count = env, list(views), Path(frames), 0
        shutil.rmtree(self.frames, ignore_errors=True)
        self.frames.mkdir(parents=True)

    def capture(self, text):
        from PIL import Image
        from table_1000.sim.isaaclab.video import caption, save_frame, tile
        rendered = self.env.render(self.views)
        images = [Image.fromarray(rendered[name]['rgb'][0]) for name in self.views]
        save_frame(caption(tile(images, images[0].height), text), self.frames / f'{self.count:05d}.jpg')
        self.count += 1


def trial_path(output, index):
    return output / 'trials' / f'{index:04d}.json'


def worker(args):
    """Run trials ``args.indices``, each writing ``trials/<index>.json`` (and with ``--video`` a video)."""
    from table_1000.scenes.batch import worker_gpu
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=worker_gpu(args.gpu), workers=4, cameras=args.video)
    failed = False
    evaluator = None
    work = args.work
    work.mkdir(parents=True, exist_ok=True)
    try:
        from table_1000.experts.pick_eval import Evaluator, scene
        from table_1000.sim.isaaclab.video import encode
        views = {'wide': WIDE, 'close': CLOSE} if args.video else {}
        evaluator = Evaluator(scene(args.gap, args.lift, robot=args.robot, scenario=args.scenario), args.assets, work, ROOT,
                              views, args.random_pushes)
        selected = set(args.indices)
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
                obstacles = placement.get('obstacles', {})
                if obstacles:   # look from the open side, past the walls
                    if 'wall_side' in obstacles:
                        side = -1. if obstacles['wall_side'][0][3] > placement['x'] else 1.
                    eye = [placement['x'] + side * .3, placement['y'] - .12, 1.15]
                camera.set_world_poses_from_view(torch.tensor([eye], dtype=torch.float32),
                                                 torch.tensor([target], dtype=torch.float32))
                video = Recorder(evaluator.env, views, work / 'frames')
            report = evaluator.trial(name, placement, args.seed * 100000 + index, video)
            report['index'] = index
            if video is not None:
                destination = args.output / 'videos'
                destination.mkdir(parents=True, exist_ok=True)
                encode(work / 'frames', 25, destination / f'{index:03d}_{name}_{report["status"]}.mp4')
            path = trial_path(args.output, index)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.with_suffix('.part').write_text(json.dumps(report, default=float))
            path.with_suffix('.part').replace(path)
            print(index, name, placement['pose'], report['status'], report.get('failure', {}).get('reason', ''),
                  f"{report['wall_s']:.0f} s", flush=True)
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
    parser.add_argument('--select', type=int, nargs='+', help='run only these trial indices (to re-run or record some)')
    parser.add_argument('--scenario', default='open', choices=('open', 'box', 'side', 'back', 'corner'),
                        help='obstacles around the object')
    parser.add_argument('--gap', type=float, default=.12, help='robot base to table edge (m)')
    parser.add_argument('--lift', type=float, default=.6, help='starting torso lift (m)')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--random-pushes', type=int, default=8,
                        help='random push directions after the six axes (0: a quicker run)')
    parser.add_argument('--robot', default='x1pro', help='robot name (asset_sources/robots/<name>)')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets', help='asset root holding robots/<name>')
    parser.add_argument('--jobs', type=int,
                        help=f'processes at once (default: one per --gpus; {LAUNCHED_JOBS} with --launcher)')
    parser.add_argument('--gpus', type=int, nargs='+', help='local GPUs (default 0); not with --launcher')
    parser.add_argument('--launcher', help='command prefix of a job scheduler that runs a process on a free GPU '
                        'it sets CUDA_VISIBLE_DEVICES to; instead of --gpus')
    parser.add_argument('--retries', type=int, default=1, help='reruns of trials whose process died before their report')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--indices', type=int, nargs='+', help=argparse.SUPPRESS)
    parser.add_argument('--work', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--gpu', type=int, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    args.output = args.output.resolve()
    if args.worker:
        worker(args)
        return
    if args.launcher and args.gpus:
        parser.error('--gpus runs locally; with --launcher the scheduler picks the GPUs')
    args.gpus = args.gpus or [0]
    from table_1000.scenes.batch import run_batch
    indices = [index for index, _, _ in tasks(args) if args.select is None or index in args.select]
    if not indices:
        parser.error('give --trials or --case')
    shutil.rmtree(args.output, ignore_errors=True)
    args.output.mkdir(parents=True)
    passthrough = ['--seed', str(args.seed), '--scenario', args.scenario, '--gap', str(args.gap), '--lift', str(args.lift),
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

    def command(group, index, attempt):
        name = f'part{index}' + (f'-retry{attempt}' if attempt else '')
        work = args.output / 'parts' / name
        argv = [sys.executable, str(Path(__file__).resolve()), '--worker', '--work', str(work),
                '--indices', *map(str, sorted(group)), *passthrough]
        return argv, args.output / 'parts' / f'{name}.log', name

    started = time.perf_counter()
    missing = run_batch(indices, key=lambda index: 0, cost=lambda index: 1., count=args.jobs or (
                            LAUNCHED_JOBS if args.launcher else len(args.gpus)),
                        command=command, missing=lambda index: not trial_path(args.output, index).exists(),
                        gpus=args.gpus, launcher=args.launcher, retries=args.retries)
    reports = [json.loads(trial_path(args.output, index).read_text()) for index in indices if index not in missing]
    reports.sort(key=lambda report: report['index'])
    with open(args.output / 'trials.jsonl', 'w') as out:
        for report in reports:
            out.write(json.dumps(report, default=float) + '\n')
    summary = {'gap': args.gap, 'lift': args.lift, 'seed': args.seed, 'robot': args.robot, 'scenario': args.scenario,
               **summarize(reports),
               'total_s': time.perf_counter() - started}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=1, default=float) + '\n')
    print(json.dumps({key: summary[key] for key in ('trials', 'success', 'rate', 'by_object')}, indent=1))
    if missing:
        raise SystemExit(f'no report of trials {missing}; see {args.output / "parts"}')


if __name__ == '__main__':
    main()
