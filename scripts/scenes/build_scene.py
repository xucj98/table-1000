"""Initial states of a scene: sampled and checked per seed, stored for the expert to start from.

``uv run python scripts/scenes/build_scene.py 000000 --seeds 0:100 --gpus 0 1 2 3 [--jobs-per-gpu 1] [--grid 5x5] [--images] [--camera NAME] [--output DIR]``
``uv run python scripts/scenes/build_scene.py 000100 --end-states [NAME ...] [--grid 3x2] ...``

For each seed, makes the initial state as in ``table_1000/scenes/initial.py``
(``scene.initialize``, penetration and settling checks, retried with sub-seeds)
on CPU PhysX. The seeds are split into ``len(gpus) × jobs-per-gpu`` parts, one
process each (logs in ``outputs/scenes/<scene>/build/``). Into ``<output>/<scene>/``
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
(480 px wide). The exit status is non-zero when a part fails or a seed has no valid state.
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
    ``args.work``: ``states.hdf5``, ``report.json`` and ``images/<seed or name>.jpg`` (all valid
    ones with ``--images``, else the first ``--first`` valid ones)."""
    work = args.work.resolve()
    (work / 'images').mkdir(parents=True, exist_ok=True)
    render = args.images or args.first > 0
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app = start_app(gpu=args.gpu, workers=4, cameras=render, extra_args=RT2_ARGS if render else ())
    failed = False
    try:
        from PIL import Image
        from table_1000.robots import load_spec
        from table_1000.scenes import load
        from table_1000.scenes.batch import commit, parse_seeds
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
        records, states, pictured = {}, {}, 0
        for key in args.end_states or parse_seeds(args.seeds):
            arrange = scene.END_STATES[key]['arrange'] if args.end_states else None
            try:
                _, record = initial_state(env, scene, assets, 0 if arrange else key, initialize=arrange)
            except InitializationFailed as error:
                records[key] = {'seed': key, 'attempt': None, 'attempts': error.attempts}
                print(key, 'no valid state', flush=True)
                continue
            records[key] = record
            state = env.env.scene.get_state(is_relative=True)
            states[key] = (record['attempt'], to_numpy(state))
            if args.images or pictured < args.first:
                # RT2 denoises over time: the first render after a state change blends in the last one.
                env.render([camera])
                Image.fromarray(env.render([camera])[camera]['rgb'][0]).save(work / 'images' / f'{key}.jpg',
                                                                            quality=90)
                pictured += 1
            print(key, 'attempt', record['attempt'], flush=True)
        write_states(work / 'states.hdf5', states, {'scene': args.scene, 'robot': spec.name, 'physics': 'isaac-cpu',
                                                    'commit': commit(root) or ''})
        (work / 'report.json').write_text(json.dumps({'robot': spec.name, 'camera': camera, 'records': records},
                                                     indent=1, default=float))
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
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scene')
    parser.add_argument('--seeds', nargs='+', help='a:b (a to b - 1) and single seeds')
    parser.add_argument('--end-states', nargs='*', help="build the scene's END_STATES (all, or these) instead of initial states")
    parser.add_argument('--gpus', type=int, nargs='+', default=[0], help='physical GPUs')
    parser.add_argument('--jobs-per-gpu', type=int, default=1, help='processes at a time on each GPU')
    parser.add_argument('--grid', type=grid_size, help='NxM: the first N x M valid states in N columns')
    parser.add_argument('--images', action='store_true', help='one image per valid state')
    parser.add_argument('--camera', help="a robot camera or a view of the scene (default: the scene's IMAGE_VIEW, else the robot's first RGB camera)")
    parser.add_argument('--output', type=Path, help='results go to <output>/<scene>/ (default: assets/scenes)')
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the scene and robot sources')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--work', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--first', type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument('--gpu', type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(args)
        return
    from table_1000.scenes import load
    from table_1000.scenes.batch import commit, parse_seeds, run_parts, seed_range, seeds_record, split
    from table_1000.scenes.initial import read_states, summarize, write_states
    if args.jobs_per_gpu < 1:
        parser.error('--jobs-per-gpu must be positive')
    root = args.root.resolve()
    ends = args.end_states is not None
    if ends:
        end_states = load(args.scene, 'scene', root).END_STATES
        keys = args.end_states or list(end_states)
        unknown = sorted(set(keys) - set(end_states))
        if unknown:
            parser.error(f'unknown end states: {", ".join(unknown)}')
    else:
        keys = parse_seeds(args.seeds or [])
    if not keys:
        parser.error('no seeds')
    output = ((args.output or args.assets / 'scenes') / args.scene).resolve()
    staging = root / 'outputs' / 'scenes' / args.scene / 'build'
    shutil.rmtree(staging, ignore_errors=True)
    checkout = commit(root)
    first = args.grid[0] * args.grid[1] if args.grid else 0
    jobs = []
    for index, part in enumerate(split(keys, len(args.gpus) * args.jobs_per_gpu)):
        work = staging / f'part{index}'
        selection = ['--end-states', *part] if ends else ['--seeds', *seed_range(part).split()]
        command = [sys.executable, str(Path(__file__).resolve()), args.scene, '--worker', *selection,
                   '--work', str(work), '--first', str(first), '--root', str(root), '--assets', str(args.assets.resolve())]
        command += ['--images'] if args.images else []
        command += ['--camera', args.camera] if args.camera else []
        label = ' '.join(part) if ends else f'seeds {seed_range(part)}'
        jobs.append((command, staging / f'part{index}.log', f'{args.scene} part{index} {label}'))
    started = time.perf_counter()
    failed = run_parts(jobs, args.gpus, args.jobs_per_gpu)

    records, states, attrs, robot, camera = {}, {}, {}, None, None
    for work in sorted(staging.glob('part*/'), key=lambda path: int(path.name[4:])):
        if not (work / 'report.json').is_file():
            continue
        report = json.loads((work / 'report.json').read_text())
        robot, camera = report['robot'], report['camera']
        records.update({key if ends else int(key): record for key, record in report['records'].items()})
        part_attrs, part_states = read_states(work / 'states.hdf5')
        attrs = attrs or part_attrs
        states.update(part_states)
    for key in keys:
        records.setdefault(key, {'seed': key, 'attempt': None, 'attempts': [], 'problems': ['process exited']})
    output.mkdir(parents=True, exist_ok=True)
    states_file, summary_file, grid_file, images_dir = END_PRODUCTS if ends else PRODUCTS
    for name in (states_file, summary_file, grid_file, images_dir):
        path = output / name
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink(missing_ok=True)
    attrs.update(commit=checkout or '')
    write_states(output / states_file, states, attrs)
    selection = ({'states': {key: {'quality': end_states[key]['quality'], 'description': end_states[key]['description']}
                             for key in keys}} if ends else {'seeds': seeds_record(keys)})
    summary = {'scene': args.scene, 'robot': robot, 'physics': 'isaac-cpu', 'commit': checkout,
               **selection, **summarize(records), 'camera': camera, 'total_s': time.perf_counter() - started}
    ordered = [(key, records[key]) for key in keys]
    (output / summary_file).write_text(json.dumps({**summary, 'records': dict(ordered)}, indent=1, default=float) + '\n')
    images = {(path.stem if ends else int(path.stem)): path for path in staging.glob('part*/images/*.jpg')}
    valid = [key for key in keys if key in images and records[key]['attempt'] is not None]
    if args.grid and valid:
        shown = valid[:first]
        captions = [f'{key} ({end_states[key]["quality"]})  attempt {records[key]["attempt"]}' if ends else
                    f'seed {key}  attempt {records[key]["attempt"]}' for key in shown]
        grid([images[key] for key in shown], captions, args.grid[0], output / grid_file)
    if args.images and valid:
        (output / images_dir).mkdir()
        for key in valid:
            shutil.copy(images[key], output / images_dir / f'{key}.jpg')
    if valid and args.output is None and not ends:
        thumbnail(images[valid[0]], root / 'asset_sources' / 'scenes' / args.scene / 'thumbnail.jpg')
    print(json.dumps(summary, indent=1, default=float))
    problems = ([f'failed: {", ".join(failed)}'] if failed else []) + (
        [f'without a valid state: {" ".join(map(str, summary["without_state"])) if ends else seed_range(summary["without_state"])}']
        if summary['invalid'] else [])
    if problems:
        raise SystemExit('; '.join(problems))


if __name__ == '__main__':
    main()
