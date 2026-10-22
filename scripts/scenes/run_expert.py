"""Expert trajectories of a scene from its initial states, in parallel processes.

``uv run python scripts/scenes/run_expert.py 000000 --physics isaac-cpu --gpus 0 1 2 3 [--jobs 8] [--input DIR] [--output DIR]``

Runs ``expert.py`` from every initial state of ``<input>/<scene>/initial_states.hdf5``
(``<input>`` = ``assets/scenes``, written by ``build_scene.py``). The seeds are
split into ``--jobs`` contiguous parts, one process each, spread over ``--gpus`` or
submitted through ``--launcher`` (see ``table_1000/common/batch.py``); seeds whose
process stopped before their episode run again (``--retries``). Logs and the parts'
files are in ``outputs/scenes/<scene>/<backend>/``. The batch replaces
``<output>/<scene>/<backend>/`` (``<output>`` = ``data/scenes``), renders included:

- ``trajectories.hdf5``, ``trajectories_failed.hdf5``: succeeded and failed episodes,
  ``data/demo_<seed>`` in Isaac Lab's recorder format;
- ``episodes/<seed>.json``: each episode's report;
- ``summary.json``: commit, seeds, success rate, failed seeds by ``"subtask: reason"``
  (``-: process exited`` for seeds a stopped process left without a result), mean durations.

``--stage <name>`` checks one step of the expert alone from every initial state
(``expert.STAGES``, see ``table_1000/experts/generate.py``); give it its own ``--output``.

Failed episodes are part of the result; the exit status is non-zero only when a
seed has no episode. See ``table_1000/experts/generate.py``.
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

# Simulation backends, as for asset physics tests (only CPU PhysX is validated for scenes yet).
BACKENDS = ('isaac-cpu',)


def scene_robot(scene, root):
    """Name of the robot in ``scene``."""
    from table_1000.robots import load_spec
    from table_1000.scenes import load
    return load_spec(Path(load(scene, 'scene', root).ROBOT['asset']).name, root).name


def worker(args):
    """One part: the given seeds' episodes in this process, into ``args.work``."""
    work = args.work.resolve()
    work.mkdir(parents=True, exist_ok=True)
    from table_1000.common.batch import worker_gpu
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=worker_gpu(args.gpu), workers=4, cameras=False)
    failed = False
    generator = None
    try:
        from table_1000.experts.generate import Generator
        from table_1000.common.batch import parse_seeds
        from table_1000.scenes.initial import read_states
        _, states = read_states(args.states)
        generator = Generator(args.scene, work, args.assets.resolve(), args.root.resolve(), physics=args.physics,
                              stage=args.stage)
        for seed in parse_seeds(args.seeds):
            report = generator.run(seed, *states[seed])
            print(seed, report['status'], report.get('failure', {}).get('reason', ''), f'{report["wall_s"]:.0f} s',
                  flush=True)
    except Exception:
        traceback.print_exc()
        failed = True
    if generator is not None:
        generator.close()
    app.close(exit_code=int(failed))


def main(argv=None):
    from table_1000.common.batch import add_arguments, commit, job_count, process_name, run_batch, seed_range
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scene')
    parser.add_argument('--physics', choices=BACKENDS, required=True, help='simulation backend')
    add_arguments(parser)
    parser.add_argument('--input', type=Path, default=ROOT / 'assets' / 'scenes',
                        help='initial states from <input>/<scene>/initial_states.hdf5')
    parser.add_argument('--output', type=Path, default=ROOT / 'data' / 'scenes',
                        help='the batch goes to <output>/<scene>/<backend>/ (replaced)')
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the scene and robot sources')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--stage', help="check one step of the expert alone: expert.STAGES[<stage>] (see generate.py)")
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--seeds', nargs='+', help=argparse.SUPPRESS)
    parser.add_argument('--states', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--work', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(args)
        return
    import h5py
    from table_1000.experts.generate import summarize
    from table_1000.scenes.recordings import merge
    jobs = job_count(parser, args)
    root = args.root.resolve()
    states = (args.input / args.scene / 'initial_states.hdf5').resolve()
    if not states.is_file():
        parser.error(f'no initial states {states}; run build_scene.py first')
    with h5py.File(states, 'r') as file:
        attrs = dict(file.attrs)
        seeds = sorted(int(name) for name in file)
    robot = scene_robot(args.scene, root)
    if attrs.get('scene') != args.scene or attrs.get('robot') != robot:
        parser.error(f'{states} holds scene {attrs.get("scene")} with robot {attrs.get("robot")}, the scene has '
                     f'{robot}; run build_scene.py again')
    if not seeds:
        parser.error(f'{states} holds no initial states')
    directory = (args.output / args.scene / args.physics).resolve()
    staging = root / 'outputs' / 'scenes' / args.scene / args.physics
    shutil.rmtree(directory, ignore_errors=True)
    shutil.rmtree(staging, ignore_errors=True)
    checkout = commit(root)
    works = {}

    def command(part, index, attempt):
        name = process_name(f'part{index}', attempt)
        works.update(dict.fromkeys(part, staging / name))
        argv = [sys.executable, str(Path(__file__).resolve()), args.scene, '--worker', '--physics', args.physics,
                '--seeds', *seed_range(part).split(), '--states', str(states), '--work', str(staging / name),
                '--root', str(root), '--assets', str(args.assets.resolve()), *(['--stage', args.stage] if args.stage else [])]
        return argv, staging / f'{name}.log', f'{args.scene} {args.physics} {name} seeds {seed_range(part)}'

    started = time.perf_counter()
    missing = run_batch(seeds, command=command, count=jobs, gpus=args.gpus, launcher=args.launcher, retries=args.retries,
                        missing=lambda seed: not (works[seed] / 'episodes' / f'{seed}.json').is_file())
    parts = sorted(staging.glob('part*/'))
    (directory / 'episodes').mkdir(parents=True)
    for name in ('trajectories', 'trajectories_failed'):
        # A stopped process may have recorded an episode without its report, which its rerun records again.
        merge([part / f'{name}.hdf5' for part in parts], directory / f'{name}.hdf5',
              keep=lambda source, seed: (source.parent / 'episodes' / f'{seed}.json').is_file())
        for part in parts:
            (part / f'{name}.hdf5').unlink(missing_ok=True)
    reports = []
    for path in sorted(staging.glob('part*/episodes/*.json')):
        reports.append(json.loads(path.read_text()))
        shutil.move(path, directory / 'episodes' / path.name)
    summary = {'scene': args.scene, 'stage': args.stage, 'robot': robot, 'physics': args.physics, 'commit': checkout,
               **summarize(reports, seeds), 'total_s': time.perf_counter() - started, 'videos': {}, 'fails': {}}
    (directory / 'summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False, default=float) + '\n')
    print('SUMMARY', directory / 'summary.json',
          json.dumps({key: summary[key] for key in ('succeeded', 'success_rate', 'failures')}, default=float), flush=True)
    if missing:
        raise SystemExit(f'no episode: seeds {seed_range(missing)}; see {staging}')


if __name__ == '__main__':
    main()
