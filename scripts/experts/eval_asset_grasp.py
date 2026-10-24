"""Evaluate isolated scene assets: random 10 cm placements, pick, then wrist shake.

Example: uv run python scripts/experts/eval_asset_grasp.py --config configs/asset_grasp_000103.py
         --trials 20 --gpus 0 1 --jobs 2
Each invocation creates a new runs/grasp/<scene>/<run>/ directory. Reports are also published
under assets/<asset>/grasp_test/<robot>/<backend>/<run>/; existing runs are never replaced.
"""

import argparse
from datetime import datetime
from importlib.metadata import version
import json
import runpy
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.part')
    temporary.write_text(json.dumps(value, indent=2, default=float, ensure_ascii=False) + '\n')
    temporary.replace(path)


def trial_path(output, asset, seed):
    return output / asset / 'trials' / f'{seed:04d}.json'


def worker(args):
    from table_1000.common.batch import Load, worker_gpu
    from table_1000.sim.isaaclab.app import start_app
    device = 'cpu' if args.physics == 'isaac-cpu' else 'cuda:0'
    app = start_app(gpu=worker_gpu(args.gpu), workers=4, device=device, cameras=args.video)
    evaluator = None
    try:
        from table_1000.experts.asset_grasp import Evaluator
        from scripts.experts.eval_pick import Recorder
        from table_1000.sim.isaaclab.video import encode
        # Camera coordinates match robot acceptance (robot faces +X).
        views = {'wide': {'position': (1.35, -1.2, 1.55), 'target': (.45, -.1, .85),
                          'focal_length_mm': 22, 'resolution': (960, 720)}} if args.video else {}
        for asset in args.asset:
            try:
                evaluator = Evaluator(asset, args.robot, args.assets, args.work, ROOT, device, views)
                for seed in args.seeds:
                    if trial_path(args.output, asset, seed).exists():
                        continue
                    recorder = Recorder(evaluator.env, views, args.work / 'frames') if args.video else None
                    with Load() as load:
                        report = evaluator.trial(seed, args.center, args.region_size, recorder)
                    report['load'] = load.record()
                    report.update(robot=args.robot, physics=args.physics)
                    if recorder:
                        video = args.output / asset / 'videos' / f'{seed:04d}.mp4'
                        video.parent.mkdir(parents=True, exist_ok=True)
                        encode(args.work / 'frames', 25, video)
                        report['video'] = str(video.relative_to(args.output / asset))
                    write_json(trial_path(args.output, asset, seed), report)
                    print(asset, seed, report['status'], report.get('failure', {}).get('reason', ''), flush=True)
            except Exception:
                # Environment/setup failures remain distinguishable from grasp failures.
                error = traceback.format_exc()
                print(error, flush=True)
                for seed in args.seeds:
                    path = trial_path(args.output, asset, seed)
                    if not path.exists():
                        write_json(path, {'asset': asset, 'seed': seed, 'status': 'error',
                                          'failure': {'reason': 'setup error', 'traceback': error}})
            finally:
                if evaluator is not None:
                    evaluator.close()
                    evaluator = None
    finally:
        app.close()


def main(argv=None):
    from table_1000.common.batch import add_arguments, commit, job_count, run_batch
    from table_1000.experts.asset_grasp import inventory, summarize
    from table_1000.scenes import load
    from table_1000.robots import load_robot_test
    from table_1000.sim import PHYSICS_DT
    from table_1000.experts.asset_grasp import SHAKE_AMPLITUDE, SHAKE_SECONDS, SLIP_LIMIT, WORK_OFFSET
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/asset_grasp_000103.py')
    parser.add_argument('--asset', nargs='+', help='only these asset paths, e.g. objects/soldering_iron/000000')
    parser.add_argument('--robot', help='override config robot')
    parser.add_argument('--physics', choices=['isaac-cpu', 'isaac-gpu'], default='isaac-cpu')
    parser.add_argument('--trials', type=int, default=20, help='random placements per asset')
    parser.add_argument('--seeds', nargs='+', type=int, help='explicit seeds instead of --trials')
    parser.add_argument('--center', nargs=2, type=float, help='world XY; default WORK + (0.05, 0) m')
    parser.add_argument('--region-size', type=float, default=.10, help='square side length in metres')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--output', type=Path, help='new directory for this run (must not exist)')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--work', type=Path, help=argparse.SUPPRESS)
    add_arguments(parser)
    args = parser.parse_args(argv)
    if args.worker:
        worker(args)
        return
    if args.trials < 1 or args.region_size <= 0:
        parser.error('--trials and --region-size must be positive')
    jobs = job_count(parser, args)
    config = runpy.run_path(str(args.config))
    args.robot = args.robot or config['ROBOT']
    items = inventory(load(config['SCENE']), config.get('EXCLUDED_INSTANCES', {}))
    selected = args.asset or [asset for asset, item in items.items() if not item['excluded']]
    if not selected or set(selected) - items.keys():
        parser.error('select at least one asset from the configured scene')
    seeds = sorted(set(args.seeds if args.seeds is not None else range(args.trials)))
    if not seeds or min(seeds) < 0:
        parser.error('seeds must be non-negative')
    run = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    output = (args.output or ROOT / 'runs/grasp' / config['SCENE'] / run).resolve()
    output.mkdir(parents=True, exist_ok=False)
    assets = args.assets.resolve()
    conditions = {'scene': config['SCENE'], 'robot': args.robot, 'physics': args.physics,
                  'versions': {package: version(package) for package in ('isaacsim', 'isaaclab', 'mplib')},
                  'commit': commit(ROOT), 'seeds': seeds, 'region_size_m': args.region_size,
                  'center_xy_m': args.center or [a + b for a, b in zip(load_robot_test(args.robot).WORK, WORK_OFFSET)],
                  'physics_dt_s': PHYSICS_DT, 'external_force': False,
                  'shake_amplitude_rad': SHAKE_AMPLITUDE, 'shake_segment_s': SHAKE_SECONDS,
                  'slip_limit_m': SLIP_LIMIT,
                  'excluded_instances': config.get('EXCLUDED_INSTANCES', {}),
                  'inventory': items}
    # Preserve the exact evaluator and launcher used by this run, including uncommitted edits.
    for relative in ('table_1000/experts/asset_grasp.py', 'scripts/experts/eval_asset_grasp.py',
                     'table_1000/experts/grasps.py', 'table_1000/experts/skills/grasping.py'):
        target = output / 'source' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, target)
    shutil.copy2(args.config, output / 'source' / 'config.py')
    write_json(output / 'conditions.json', conditions)
    print('run:', output, flush=True)

    def missing(unit):
        return not trial_path(output, *unit).exists()

    def command(group, index, attempt):
        work = output / 'workers' / f'{index}-attempt{attempt}'
        asset = group[0][0]
        cmd = [sys.executable, str(Path(__file__).resolve()), '--worker', '--asset', asset,
               '--config', str(args.config.resolve()),
               '--robot', args.robot, '--physics', args.physics, '--assets', str(assets),
               '--output', str(output), '--work', str(work), '--seeds', *[str(seed) for _, seed in group],
               '--region-size', str(args.region_size)]
        if args.center:
            cmd += ['--center', *map(str, args.center)]
        if args.video:
            cmd.append('--video')
        return cmd, work / 'run.log', f'asset grasp part {index}'

    started = time.perf_counter()
    units = [(asset, seed) for asset in selected for seed in seeds]
    run_batch(units, key=lambda unit: unit[0], count=jobs, command=command, missing=missing,
              gpus=args.gpus, launcher=args.launcher, retries=args.retries)
    overall = {**conditions, 'assets': {}, 'wall_s': time.perf_counter() - started}
    for asset in selected:
        paths = [trial_path(output, asset, seed) for seed in seeds]
        reports = [json.loads(path.read_text()) for path in paths if path.exists()]
        summary = {**conditions, 'asset': asset, **summarize(reports, len(seeds))}
        write_json(output / asset / 'summary.json', summary)
        destination = assets / asset / 'grasp_test' / args.robot / args.physics / run
        shutil.copytree(output / asset, destination)
        overall['assets'][asset] = {**summarize(reports, len(seeds)), 'results': str(destination)}
    write_json(output / 'summary.json', overall)
    lines = ['# Asset grasp results', '', '| Asset | Picked | Stable | Rate | Failures |',
             '| --- | ---: | ---: | ---: | --- |']
    for asset, entry in overall['assets'].items():
        reasons = ', '.join(f'{k}: {v}' for k, v in entry['failures'].items())
        lines.append(f"| {asset} | {entry['picked']}/{len(seeds)} | {entry['success']}/{len(seeds)} | {entry['rate']:.0%} | {reasons} |")
    (output / 'summary.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines), flush=True)
    if any(r['status'] != 'PASS' for r in overall['assets'].values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
