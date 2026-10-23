"""Initial states of a scene: sampled and checked per seed, stored for the expert to start from.

``uv run python scripts/scenes/build_scene.py 000000 --seeds 0:100 --gpus 0 1 2 3 [--jobs N] [--grid 5x5] [--images] [--camera NAME] [--output DIR]``
``uv run python scripts/scenes/build_scene.py 000100 --end-states [NAME ...] [--grid 3x2] ...``

For each seed, makes the initial state as in ``table_1000/scenes/initial.py``
(``scene.initialize``, penetration and settling checks, retried with sub-seeds)
on CPU PhysX. The seeds are split into ``--jobs`` contiguous parts, one process each, spread
over ``--gpus`` or submitted through ``--launcher`` (see ``table_1000/common/batch.py``); each
seed's result is written when it is done, and seeds whose process stopped before theirs run
again (``--retries``), as do seeds that raised an error (the error may be the process's: GPU memory,
a lost device) and seeds without a valid state after an error in their process. Logs are in ``outputs/scenes/<scene>/build/``; the processes' results go to
``<output>/<scene>/staging/``, removed once merged. Into ``<output>/<scene>/``
(``<output>`` = ``assets/scenes`` by default), replacing what an earlier run wrote:

- ``initial_states.hdf5``: the settled state of every seed with one, its ``seed`` and ``attempt``;
- ``initial.json``: the summary (seeds, valid and invalid, mean attempts, settling motion,
  robot contacts) and every attempt of every seed with its problems;
- ``initial_states.jpg`` with ``--grid NxM``: the first N × M valid states in N columns;
- ``initial/<seed>.jpg`` with ``--images``: every valid state at the camera's resolution.

With ``--end-states``, the scene's ``END_STATES`` (all, or the named ones) are built the same
way instead, each ``arrange`` function as the initialization with seed 0, into
``end_states.hdf5`` (one group per state, by name), ``end_states.json``, ``end_states.jpg``
and ``end_states/<name>.jpg``; the initial states and the thumbnail are left alone.

Images come from ``--camera`` (default: the scene's ``IMAGE_VIEW``, else the robot's first
RGB camera; or another fixed view of the scene). When images are rendered into the default output, the
smallest valid seed's image also becomes ``asset_sources/scenes/<scene>/thumbnail.jpg``
(480 px wide). The exit status is non-zero when a seed has no result, an error or no valid state.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

TILE_WIDTH, CAPTION, THUMBNAIL_WIDTH = 384, 18, 480
PRODUCTS = ('initial_states.hdf5', 'initial.json', 'initial_states.jpg', 'initial')
END_PRODUCTS = ('end_states.hdf5', 'end_states.json', 'end_states.jpg', 'end_states')


def grid_size(text):
    columns, rows = map(int, text.lower().split('x'))
    if columns < 1 or rows < 1:
        raise argparse.ArgumentTypeError('--grid takes NxM with positive N and M')
    return columns, rows


def main_camera(spec):
    """The robot's first RGB camera."""
    return next(name for name, camera in spec.cameras.items() if camera.sensor.kind == 'rgb')


def worker(args):
    """Initial states of ``args.seeds`` (or end states ``args.end_states``) in this process, into
    ``args.work``, per seed (or state name) as it is done: ``states/<seed>.hdf5`` (a valid state),
    ``images/<seed>.jpg`` (all valid ones with ``--images``, else the first ``--first`` valid ones),
    then ``records/<seed>.json``, its result."""
    work = args.work.resolve()
    for name in ('states', 'images', 'records'):
        (work / name).mkdir(parents=True, exist_ok=True)
    render = args.images or args.first > 0
    from table_1000.common.batch import worker_gpu
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app = start_app(gpu=worker_gpu(args.gpu), workers=4, cameras=render, extra_args=RT2_ARGS if render else ())
    failed = False
    try:
        from PIL import Image
        from table_1000.robots import load_spec
        from table_1000.scenes import load
        from table_1000.common.batch import parse_seeds
        from table_1000.scenes.initial import InitializationFailed, initial_state, write_states
        from table_1000.sim.isaaclab.env import RobotEnv
        root, assets = args.root.resolve(), args.assets.resolve()
        scene = load(args.scene, 'scene', root)
        spec = load_spec(Path(scene.ROBOT['asset']).name, root)
        camera = None
        kwargs = {}
        if render:
            from table_1000.sim.isaaclab import render as rendering
            rendering.configure()
            camera = args.camera or getattr(scene, 'IMAGE_VIEW', None) or main_camera(spec)
            if camera in scene.VIEWS:
                kwargs['views'] = {camera: scene.VIEWS[camera]}
            else:
                kwargs['cameras'] = {camera: spec.cameras[camera]}
        env = RobotEnv(scene, spec, assets, root=root, **kwargs)
        pictured = 0

        def save(record):
            # Written whole or not at all: the file marks the seed as done.
            path = work / 'records' / f'{record["seed"]}.json'
            path.with_suffix('.tmp').write_text(
                json.dumps({'robot': spec.name, 'camera': camera, 'record': record}, indent=1, default=float))
            os.replace(path.with_suffix('.tmp'), path)

        erred = None      # the last seed that raised an error in this process
        for seed in args.end_states or parse_seeds(args.seeds):
            arrange = scene.END_STATES[seed]['arrange'] if args.end_states else None
            record = None
            try:
                _, record = initial_state(env, scene, assets, 0 if arrange else seed, initialize=arrange)
                record['seed'] = seed
                write_states(work / 'states' / f'{seed}.hdf5',
                             {seed: (record['attempt'], to_numpy(env.env.scene.get_state(is_relative=True)))}, {})
                if args.images or pictured < args.first:
                    # RT2 denoises over time: the first render after a state change blends in the last one.
                    env.render([camera])
                    Image.fromarray(env.render([camera])[camera]['rgb'][0]).save(work / 'images' / f'{seed}.jpg',
                                                                                quality=90)
                    pictured += 1
                print(seed, 'attempt', record['attempt'], flush=True)
            except InitializationFailed as error:
                record = {'seed': seed, 'attempt': None, 'attempts': error.attempts}
                if erred is not None:
                    # The error may have broken the process: the seed runs again in a new one.
                    record['after_error'] = erred
                print(seed, 'no valid state', flush=True)
            except Exception:
                # Kept as this seed's result if it errs again when retried in a new process.
                traceback.print_exc()
                erred = seed
                (work / 'states' / f'{seed}.hdf5').unlink(missing_ok=True)
                record = {'seed': seed, 'attempt': None, 'attempts': record['attempts'] if record else [],
                          'error': traceback.format_exc().strip().splitlines()[-1]}
            save(record)
        env.close()
    except Exception:
        traceback.print_exc()
        failed = True
    app.close(exit_code=int(failed))


def to_numpy(tree):
    return {key: to_numpy(value) if isinstance(value, dict) else value.cpu().numpy() for key, value in tree.items()}


def grid(images, captions, columns, path):
    """``images`` (paths) captioned in a grid of ``columns``, each ``TILE_WIDTH`` wide."""
    from PIL import Image, ImageDraw
    first = Image.open(images[0])
    height = round(first.height * TILE_WIDTH / first.width)
    rows = -(-len(images) // columns)
    canvas = Image.new('RGB', (columns * TILE_WIDTH, rows * (height + CAPTION)), 'white')
    draw = ImageDraw.Draw(canvas)
    for index, (image, caption) in enumerate(zip(images, captions)):
        x, y = (index % columns) * TILE_WIDTH, (index // columns) * (height + CAPTION)
        canvas.paste(Image.open(image).convert('RGB').resize((TILE_WIDTH, height), Image.LANCZOS), (x, y + CAPTION))
        draw.text((x + 4, y + 3), caption, fill='black')
    canvas.save(path, quality=90)


def thumbnail(image, path):
    from PIL import Image
    picture = Image.open(image)
    picture.resize((THUMBNAIL_WIDTH, round(picture.height * THUMBNAIL_WIDTH / picture.width)),
                   Image.LANCZOS).save(path, quality=85)


def main(argv=None):
    from table_1000.common.batch import add_arguments, commit, job_count, parse_seeds, process_name, run_batch, seed_range, seeds_record
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scene')
    parser.add_argument('--seeds', nargs='+', help='a:b (a to b - 1) and single seeds')
    parser.add_argument('--end-states', nargs='*', help="build the scene's END_STATES (all, or these) instead of initial states")
    add_arguments(parser)
    parser.add_argument('--grid', type=grid_size, help='NxM: the first N x M valid states in N columns')
    parser.add_argument('--images', action='store_true', help='one image per valid state')
    parser.add_argument('--camera', help="a robot camera or a view of the scene (default: the scene's IMAGE_VIEW, else the robot's first RGB camera)")
    parser.add_argument('--output', type=Path, help='results go to <output>/<scene>/ (default: assets/scenes)')
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the scene and robot sources')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--work', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--first', type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(args)
        return
    from table_1000.scenes.initial import read_states, summarize, write_states
    from table_1000.scenes import load
    jobs = job_count(parser, args)
    root = args.root.resolve()
    ends = args.end_states is not None
    if ends:
        end_states = load(args.scene, 'scene', root).END_STATES
        seeds = args.end_states or list(end_states)
        unknown = sorted(set(seeds) - set(end_states))
        if unknown:
            parser.error(f'unknown end states: {", ".join(unknown)}')
    else:
        seeds = parse_seeds(args.seeds or [])
    if not seeds:
        parser.error('no seeds')
    # A part of the keys as worker arguments and as text: seed ranges, or end state names.
    selection = (lambda part: ['--end-states', *part]) if ends else (lambda part: ['--seeds', *seed_range(part).split()])
    label = (lambda part: ' '.join(part)) if ends else (lambda part: f'seeds {seed_range(part)}')
    output = ((args.output or args.assets / 'scenes') / args.scene).resolve()
    staging, logs = output / 'staging', root / 'outputs' / 'scenes' / args.scene / 'build'

    def result(seed):
        """The seed's latest result and the process directory holding it, or ``(None, None)``."""
        for work in reversed(parts[seed]):
            if (work / 'records' / f'{seed}.json').is_file():
                return json.loads((work / 'records' / f'{seed}.json').read_text()), work
        return None, None

    def rerun(seed):
        done, _ = result(seed)
        return not done or 'error' in done['record'] or 'after_error' in done['record']

    shutil.rmtree(staging, ignore_errors=True)
    shutil.rmtree(logs, ignore_errors=True)
    checkout = commit(root)
    first = args.grid[0] * args.grid[1] if args.grid else 0
    parts = {}

    def command(part, index, attempt):
        name = process_name(f'part{index}', attempt)
        for seed in part:
            parts.setdefault(seed, []).append(staging / name)
        argv = [sys.executable, str(Path(__file__).resolve()), args.scene, '--worker', *selection(part),
                '--work', str(staging / name), '--first', str(first), '--root', str(root), '--assets', str(args.assets.resolve())]
        argv += ['--images'] if args.images else []
        argv += ['--camera', args.camera] if args.camera else []
        return argv, logs / f'{name}.log', f'{args.scene} {name} {label(part)}'

    started = time.perf_counter()
    missing = set(run_batch(seeds, command=command, count=jobs, gpus=args.gpus, launcher=args.launcher,
                        retries=args.retries, missing=rerun))

    records, states, images, robot, camera = {}, {}, {}, None, None
    for seed in seeds:
        done, work = result(seed)
        if not done:
            continue
        robot, camera, records[seed] = done['robot'], done['camera'], done['record']
        if (work / 'states' / f'{seed}.hdf5').is_file():
            states.update(read_states(work / 'states' / f'{seed}.hdf5')[1])
        if (work / 'images' / f'{seed}.jpg').is_file():
            images[seed] = work / 'images' / f'{seed}.jpg'
    for seed in seeds:
        records.setdefault(seed, {'seed': seed, 'attempt': None, 'attempts': [], 'problems': ['process exited']})
    output.mkdir(parents=True, exist_ok=True)
    states_file, summary_file, grid_file, images_dir = END_PRODUCTS if ends else PRODUCTS
    for name in (states_file, summary_file, grid_file, images_dir):
        path = output / name
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink(missing_ok=True)
    write_states(output / states_file, states,
                 {'scene': args.scene, 'robot': robot or '', 'physics': 'isaac-cpu', 'commit': checkout or ''})
    chosen = ({'states': {name: {'quality': end_states[name]['quality'], 'description': end_states[name]['description']}
                          for name in seeds}} if ends else {'seeds': seeds_record(seeds)})
    summary = {'scene': args.scene, 'robot': robot, 'physics': 'isaac-cpu', 'commit': checkout,
               **chosen, **summarize(records), 'camera': camera, 'total_s': time.perf_counter() - started}
    (output / summary_file).write_text(json.dumps({**summary, 'records': {seed: records[seed] for seed in seeds}},
                                                  indent=1, default=float) + '\n')
    valid = [seed for seed in seeds if seed in images and records[seed]['attempt'] is not None]
    if args.grid and valid:
        shown = valid[:first]
        captions = [f'{seed} ({end_states[seed]["quality"]})  attempt {records[seed]["attempt"]}' if ends else
                    f'seed {seed}  attempt {records[seed]["attempt"]}' for seed in shown]
        grid([images[seed] for seed in shown], captions, args.grid[0], output / grid_file)
    if args.images and valid:
        (output / images_dir).mkdir()
        for seed in valid:
            shutil.copy(images[seed], output / images_dir / f'{seed}.jpg')
    if valid and args.output is None and not ends:
        thumbnail(images[valid[0]], root / 'asset_sources' / 'scenes' / args.scene / 'thumbnail.jpg')
    shutil.rmtree(staging, ignore_errors=True)
    print(json.dumps(summary, indent=1, default=float))
    errors = {seed for seed, record in records.items() if 'error' in record}
    without = set(summary['without_state']) - missing
    named = (lambda group: ' '.join(sorted(group))) if ends else (lambda group: f'seeds {seed_range(group)}')
    problems = [f'{kind}: {named(group)} (see {logs})'
                for kind, group in (('no result', missing - errors), ('errors', errors)) if group]
    problems += [f'{"" if ends else "seeds "}without a valid state: {" ".join(sorted(without)) if ends else seed_range(without)}'] if without else []
    if problems:
        raise SystemExit('; '.join(problems))


if __name__ == '__main__':
    main()
