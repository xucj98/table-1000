"""Expert trajectories of scenes in Isaac Lab, in parallel over GPUs.

``uv run python scripts/scenes/generate.py 000000 --seeds 0:100 --physics isaac-cpu --gpus 0 1 2 3``

For each scene and backend the seeds are split into one contiguous part per GPU;
each part runs in its own process (``<output>/<scene>/<backend>/part<k>/``, log
``run.log``), so a failing part does not stop the others. Each part writes
``trajectories.hdf5`` (succeeded episodes, Isaac Lab's format, ``demo_<seed>``),
``trajectories_failed.hdf5`` and ``episodes/<seed>.json``. At the end
``<output>/<scene>/<backend>/summary.json`` and ``summary.csv`` summarize every
part, and the ``generation`` section of ``assets/scenes/<scene>/summary.json``
is updated. ``--seeds`` takes ``a:b`` ranges and single seeds. Failed episodes are
part of the result; the exit status is non-zero only when a part fails to run.
See ``table_1000/experts/generate.py``.
"""

import argparse
import csv
import json
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# Simulation backends, as for asset physics tests (only CPU PhysX is validated for scenes yet).
BACKENDS = {'isaac-cpu': 'cpu'}


def seeds(values):
    result = []
    for value in values:
        if ':' in value:
            start, stop = map(int, value.split(':'))
            result += list(range(start, stop))
        else:
            result.append(int(value))
    return result


def split(values, count):
    """``values`` in ``count`` contiguous parts of nearly equal size (no empty parts)."""
    count = min(count, len(values))
    size, extra = divmod(len(values), count)
    parts, start = [], 0
    for index in range(count):
        stop = start + size + (index < extra)
        parts.append(values[start:stop])
        start = stop
    return parts


def worker(args):
    """One part: the given seeds of one scene in this process."""
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=args.gpu, workers=4, cameras=False)
    failed = False
    generator = None
    try:
        from table_1000.experts.generate import Generator
        generator = Generator(args.scenes[0], output, args.assets.resolve(), args.root.resolve(),
                              physics=args.physics[0])
        for seed in seeds(args.seeds):
            report = generator.run(seed)
            print(seed, report['status'], report.get('failure', {}).get('reason', ''), f'{report["wall_s"]:.0f} s',
                  flush=True)
    except Exception:
        traceback.print_exc()
        failed = True
    if generator is not None:
        generator.close()
    app.close(exit_code=int(failed))


def summarize_run(directory, scene, backend, seed_list, failed_parts, root, assets, wall_s):
    from table_1000.experts.generate import SUMMARY_FIELDS, summarize
    from table_1000.scenes import summary as scene_summary
    reports = sorted((json.loads(path.read_text()) for path in directory.glob('part*/episodes/*.json')),
                     key=lambda report: report['seed'])
    summary = summarize(reports)
    missing = sorted(set(seed_list) - {report['seed'] for report in reports})
    summary.update(scene=scene, physics=backend, missing_seeds=missing, failed_parts=failed_parts,
                   total_s=wall_s)
    (directory / 'summary.json').write_text(json.dumps(summary, indent=1, default=float) + '\n')
    with open(directory / 'summary.csv', 'w', newline='') as file:
        writer = csv.DictWriter(file, SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(summary['rows'])
    section = {key: value for key, value in summary.items() if key not in ('rows', 'scene', 'physics')}
    section.update(output=str(directory), commit=scene_summary.commit(root),
                   seed_range=f'{min(seed_list)}..{max(seed_list)}' if seed_list else None)
    return summary, scene_summary.update(assets, scene, 'generation', section, key=backend)


def main(argv=None):
    from table_1000.physics.test import run_jobs
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenes', nargs='+')
    parser.add_argument('--seeds', nargs='+', default=['0:10'])
    parser.add_argument('--physics', nargs='+', choices=tuple(BACKENDS), required=True, help='simulation backends')
    parser.add_argument('--gpus', type=int, nargs='+', default=[0], help='physical GPUs; one part at a time on each')
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs' / 'scenes',
                        help='runs go to <output>/<scene>/<backend>/ (replaced)')
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the scene and robot sources')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--gpu', type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(args)
        return
    seed_list = seeds(args.seeds)
    jobs, runs = [], []
    for scene in args.scenes:
        for backend in args.physics:
            directory = args.output.resolve() / scene / backend
            shutil.rmtree(directory, ignore_errors=True)
            runs.append((scene, backend, directory))
            for index, part in enumerate(split(seed_list, len(args.gpus))):
                command = [sys.executable, str(Path(__file__).resolve()), scene, '--worker', '--physics', backend,
                           '--seeds', *map(str, part), '--output', str(directory / f'part{index}'),
                           '--root', str(args.root.resolve()), '--assets', str(args.assets.resolve())]
                jobs.append((command, directory / f'part{index}' / 'run.log',
                             f'{scene} {backend} part{index} seeds {part[0]}..{part[-1]}'))
    started = time.perf_counter()
    failed = run_jobs(jobs, args.gpus)
    wall_s = time.perf_counter() - started
    for scene, backend, directory in runs:
        parts = [label for label in failed if label.startswith(f'{scene} {backend} ')]
        summary, file = summarize_run(directory, scene, backend, seed_list, parts, args.root.resolve(),
                                      args.assets.resolve(), wall_s)
        print('SUMMARY', file, json.dumps({key: summary[key] for key in ('succeeded', 'episodes', 'failures')},
                                          default=float), flush=True)
    if failed:
        raise SystemExit('failed: ' + ', '.join(failed))


if __name__ == '__main__':
    main()
