"""Initial states of a scene: sampled and checked per seed, stored for the expert to start from.

``uv run python scripts/scenes/build_scene.py 000000 --seeds 0:100 --gpus 0 1 2 3 [--jobs N] [--grid 5x5] [--images] [--camera NAME] [--output DIR]``

For each seed, makes the initial state as in ``table_1000/scenes/initial.py``
(``scene.initialize``, penetration and settling checks, retried with sub-seeds)
on CPU PhysX. The seeds are split into ``--jobs`` contiguous parts, one process each, spread
over ``--gpus`` or submitted through ``--launcher`` (see ``table_1000/common/batch.py``); seeds
whose process stopped before its result run again (``--retries``). Logs are in
``outputs/scenes/<scene>/build/``. Into ``<output>/<scene>/``
(``<output>`` = ``assets/scenes`` by default), replacing what an earlier run wrote:

- ``initial_states.hdf5``: the settled state of every seed with one, its ``seed`` and ``attempt``;
- ``initial.json``: the summary (seeds, valid and invalid, mean attempts, settling motion,
  robot contacts) and every attempt of every seed with its problems;
- ``initial_states.jpg`` with ``--grid NxM``: the first N × M valid states in N columns;
- ``initial/<seed>.jpg`` with ``--images``: every valid state at the camera's resolution.

Images come from ``--camera`` (default: the robot's first RGB camera; or a fixed
view of the scene). When images are rendered into the default output, the
smallest valid seed's image also becomes ``asset_sources/scenes/<scene>/thumbnail.jpg``
(480 px wide). The exit status is non-zero when a seed has no result or no valid state.
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


def grid_size(text):
    columns, rows = map(int, text.lower().split('x'))
    if columns < 1 or rows < 1:
        raise argparse.ArgumentTypeError('--grid takes NxM with positive N and M')
    return columns, rows


def main_camera(spec):
    """The robot's first RGB camera."""
    return next(name for name, camera in spec.cameras.items() if camera.sensor.kind == 'rgb')


def worker(args):
    """Initial states of ``args.seeds`` in this process, into ``args.work``: ``states.hdf5``,
    ``report.json`` and ``images/<seed>.jpg`` (all valid seeds with ``--images``, else the first
    ``--first`` valid ones)."""
    work = args.work.resolve()
    (work / 'images').mkdir(parents=True, exist_ok=True)
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
        from table_1000.common.batch import commit, parse_seeds
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
            camera = args.camera or main_camera(spec)
            if camera in scene.VIEWS:
                kwargs['views'] = {camera: scene.VIEWS[camera]}
            else:
                kwargs['cameras'] = {camera: spec.cameras[camera]}
        env = RobotEnv(scene, spec, assets, root=root, **kwargs)
        records, states, pictured = {}, {}, 0
        for seed in parse_seeds(args.seeds):
            try:
                _, record = initial_state(env, scene, assets, seed)
            except InitializationFailed as error:
                records[seed] = {'seed': seed, 'attempt': None, 'attempts': error.attempts}
                print(seed, 'no valid state', flush=True)
                continue
            records[seed] = record
            state = env.env.scene.get_state(is_relative=True)
            states[seed] = (record['attempt'], to_numpy(state))
            if args.images or pictured < args.first:
                # RT2 denoises over time: the first render after a state change blends in the last one.
                env.render([camera])
                Image.fromarray(env.render([camera])[camera]['rgb'][0]).save(work / 'images' / f'{seed}.jpg',
                                                                            quality=90)
                pictured += 1
            print(seed, 'attempt', record['attempt'], flush=True)
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
    from table_1000.common.batch import add_arguments, commit, job_count, parse_seeds, process_name, run_batch, seed_range, seeds_record
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scene')
    parser.add_argument('--seeds', nargs='+', required=True, help='a:b (a to b - 1) and single seeds')
    add_arguments(parser)
    parser.add_argument('--grid', type=grid_size, help='NxM: the first N x M valid states in N columns')
    parser.add_argument('--images', action='store_true', help='one image per valid state')
    parser.add_argument('--camera', help="a robot camera or a view of the scene (default: the robot's first RGB camera)")
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
    jobs = job_count(parser, args)
    seeds = parse_seeds(args.seeds)
    if not seeds:
        parser.error('no seeds')
    root = args.root.resolve()
    output = ((args.output or args.assets / 'scenes') / args.scene).resolve()
    staging = root / 'outputs' / 'scenes' / args.scene / 'build'
    shutil.rmtree(staging, ignore_errors=True)
    checkout = commit(root)
    first = args.grid[0] * args.grid[1] if args.grid else 0
    parts = {}

    def command(part, index, attempt):
        name = process_name(f'part{index}', attempt)
        parts.update(dict.fromkeys(part, staging / name))
        argv = [sys.executable, str(Path(__file__).resolve()), args.scene, '--worker', '--seeds', *seed_range(part).split(),
                '--work', str(staging / name), '--first', str(first), '--root', str(root), '--assets', str(args.assets.resolve())]
        argv += ['--images'] if args.images else []
        argv += ['--camera', args.camera] if args.camera else []
        return argv, staging / f'{name}.log', f'{args.scene} {name} seeds {seed_range(part)}'

    started = time.perf_counter()
    # A part writes its seeds' report at the end.
    missing = run_batch(seeds, command=command, missing=lambda seed: not (parts[seed] / 'report.json').is_file(),
                        count=jobs, gpus=args.gpus, launcher=args.launcher, retries=args.retries)

    records, states, images, attrs, robot, camera = {}, {}, {}, {}, None, None
    for work in sorted(staging.glob('part*/')):
        if not (work / 'report.json').is_file():
            continue
        report = json.loads((work / 'report.json').read_text())
        robot, camera = report['robot'], report['camera']
        records.update({int(seed): record for seed, record in report['records'].items()})
        part_attrs, part_states = read_states(work / 'states.hdf5')
        attrs = attrs or part_attrs
        states.update(part_states)
        images.update({int(path.stem): path for path in (work / 'images').glob('*.jpg')})
    for seed in seeds:
        records.setdefault(seed, {'seed': seed, 'attempt': None, 'attempts': [], 'problems': ['process exited']})
    output.mkdir(parents=True, exist_ok=True)
    for name in PRODUCTS:
        path = output / name
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink(missing_ok=True)
    attrs.update(commit=checkout or '')
    write_states(output / 'initial_states.hdf5', states, attrs)
    summary = {'scene': args.scene, 'robot': robot, 'physics': 'isaac-cpu', 'commit': checkout,
               'seeds': seeds_record(seeds), **summarize(records), 'camera': camera,
               'total_s': time.perf_counter() - started}
    (output / 'initial.json').write_text(json.dumps({**summary, 'records': dict(sorted(records.items()))},
                                                    indent=1, default=float) + '\n')
    valid = sorted(seed for seed in images if records[seed]['attempt'] is not None)
    if args.grid and valid:
        shown = valid[:first]
        grid([images[seed] for seed in shown], [f'seed {seed}  attempt {records[seed]["attempt"]}' for seed in shown],
             args.grid[0], output / 'initial_states.jpg')
    if args.images and valid:
        (output / 'initial').mkdir()
        for seed in valid:
            shutil.copy(images[seed], output / 'initial' / f'{seed}.jpg')
    if valid and args.output is None:
        thumbnail(images[valid[0]], root / 'asset_sources' / 'scenes' / args.scene / 'thumbnail.jpg')
    print(json.dumps(summary, indent=1, default=float))
    problems = ([f'no result: seeds {seed_range(missing)}; see {staging}'] if missing else []) + (
        [f'seeds without a valid state: {seed_range(summary["without_state"])}'] if summary['invalid'] else [])
    if problems:
        raise SystemExit('; '.join(problems))


if __name__ == '__main__':
    main()
