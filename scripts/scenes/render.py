"""Render recorded trajectories from their states: videos, robot camera frames and review sheets.

``uv run python scripts/scenes/render.py outputs/scenes/000000/isaac-cpu/part*/trajectories.hdf5 --gpus 0 1 [--demos demo_3 demo_7 | --count 5] [--views wide] [--cameras face_view left_wrist_view right_wrist_view] [--sheets] [--output DIR]``

The chosen episodes of all the given files (one scene) are split over the GPUs,
one process at a time on each (logs in ``outputs/scenes/<scene>/render/``); at
the end the ``preview`` section of ``assets/scenes/<scene>/summary.json`` lists
what is rendered. Per episode ``<output>/<demo>/``, by default in the scene's preview directory
``assets/scenes/<scene>/preview/``:

- ``<view>.mp4`` for each fixed view of the scene (``VIEWS``, or ``--views``), captioned with the
  time and the subtask, one frame per control step (25 fps, 1x);
- ``frames/<camera>/%05d.jpg``: the robot cameras' images before each step
  (the observation the step's action answers), uncaptioned, for
  ``scripts/scenes/to_lerobot.py``;
- ``sheet.jpg``: twelve frames of the first view spread over the episode with
  their subtasks, for reviewing the trajectory at a glance.

Every frame restores the recorded state of that step; nothing is simulated.
``--count`` picks that many episodes at random (``--seed``); the default is all.
``--sheets`` renders only the twelve frames of each review sheet (no videos, no
robot cameras), enough to review every episode of a batch; it replaces only
``sheet.jpg`` and keeps an episode's videos and frames.
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

SHEET = (4, 3)          # columns, rows
SHEET_WIDTH = 480       # px per frame


def sheet(frames, captions, path):
    from PIL import Image, ImageDraw
    columns, rows = SHEET
    images = [Image.open(frame) for frame in frames]
    height = round(images[0].height * SHEET_WIDTH / images[0].width)
    canvas = Image.new('RGB', (columns * SHEET_WIDTH, rows * (height + 18)), 'white')
    draw = ImageDraw.Draw(canvas)
    for i, (image, caption) in enumerate(zip(images, captions)):
        x, y = (i % columns) * SHEET_WIDTH, (i // columns) * (height + 18)
        canvas.paste(image.resize((SHEET_WIDTH, height)), (x, y + 18))
        draw.text((x + 4, y + 3), caption, fill='black')
    canvas.save(path, quality=90)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectories', type=Path, nargs='+', help='trajectory files of one scene')
    parser.add_argument('--output', type=Path, help='default: assets/scenes/<scene>/preview')
    parser.add_argument('--demos', nargs='*')
    parser.add_argument('--count', type=int)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--cameras', nargs='*', default=None, help='robot cameras (default: every rgb camera)')
    parser.add_argument('--views', nargs='*', default=None, help='fixed views of the scene (default: all)')
    parser.add_argument('--sheets', action='store_true', help='only the review sheets')
    parser.add_argument('--gpus', type=int, nargs='+', default=[0], help='physical GPUs; one process at a time on each')
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--gpu', type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(args)
        return
    import h5py
    import numpy as np
    from table_1000.physics.test import run_jobs
    from table_1000.scenes import summary as scene_summary
    available, scenes = [], set()
    for path in args.trajectories:
        with h5py.File(path, 'r') as file:
            scenes.add(json.loads(file['data'].attrs['env_args'])['scene'])
            available += [(path.resolve(), name) for name in file['data'] if args.demos is None or name in args.demos]
    if len(scenes) != 1:
        parser.error(f'the files hold scenes {sorted(scenes)}; give the files of one scene')
    scene = scenes.pop()
    available.sort(key=lambda item: int(item[1].split('_')[1]))
    if args.count:
        picked = np.random.default_rng(args.seed).choice(len(available), min(args.count, len(available)), replace=False)
        available = [available[i] for i in sorted(picked)]
    if not available:
        parser.error('no episodes to render')
    output = (args.output or args.assets / 'scenes' / scene / 'preview').resolve()
    logs = args.root.resolve() / 'outputs' / 'scenes' / scene / 'render'
    jobs = []
    for index in range(min(len(args.gpus), len(available))):
        chosen = available[index::len(args.gpus)]
        for path in dict.fromkeys(path for path, _ in chosen):
            names = [name for p, name in chosen if p == path]
            command = [sys.executable, str(Path(__file__).resolve()), str(path), '--worker', '--demos', *names,
                       '--output', str(output), '--root', str(args.root.resolve()), '--assets', str(args.assets.resolve())]
            command += ['--sheets'] if args.sheets else []
            command += ['--views', *args.views] if args.views is not None else []
            command += ['--cameras', *args.cameras] if args.cameras is not None else []
            jobs.append((command, logs / f'run{len(jobs)}.log', f'{path.parent.name} {len(names)} episodes'))
    failed = run_jobs(jobs, args.gpus)
    print('SUMMARY', scene_summary.update(args.assets.resolve(), scene, 'preview',
                                          scene_summary.preview_section(args.assets.resolve() / 'scenes' / scene / 'preview')))
    if failed:
        raise SystemExit('failed: ' + ', '.join(failed))


def worker(args):
    """Render ``args.demos`` of one trajectory file in this process."""
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app = start_app(gpu=args.gpu, workers=4, extra_args=RT2_ARGS)
    failed = False
    try:
        import numpy as np
        from table_1000.robots import load_spec
        from table_1000.scenes import load
        from table_1000.sim import CONTROL_HZ
        from table_1000.sim.isaaclab import render, replay
        from table_1000.sim.isaaclab.env import RobotEnv
        from table_1000.sim.isaaclab.video import Video
        render.configure()
        file, episodes = replay.episodes(args.trajectories[0])
        meta = replay.env_args(file)
        scene = load(meta['scene'], 'scene', args.root)
        spec = load_spec(meta['robot'], args.root)
        output = (args.output or args.assets / 'scenes' / meta['scene'] / 'preview').resolve()
        names = args.demos or list(episodes)
        cameras = {name: camera for name, camera in spec.cameras.items() if not args.sheets and (
            (args.cameras is None and camera.sensor.kind == 'rgb') or name in (args.cameras or ()))}
        views = {name: scene.VIEWS[name] for name in (args.views if args.views is not None else scene.VIEWS)}
        env = RobotEnv(scene, spec, args.assets.resolve(), cameras=cameras, views=views, root=args.root)
        for name in names:
            started = time.perf_counter()
            episode = episodes[name]
            directory = output / name
            if not args.sheets:
                shutil.rmtree(directory, ignore_errors=True)
            videos = Video(env, list(views), directory / 'views', CONTROL_HZ)
            frames = Video(env, list(cameras), directory / 'frames', CONTROL_HZ)
            picks = np.linspace(0, episode.length, SHEET[0] * SHEET[1]).round().astype(int)
            for t in picks if args.sheets else range(episode.length + 1):
                replay.set_state(env, episode.state(t))
                # Each render draws every camera. RT2 denoises over time, so the first render after a
                # state change still blends in the previous one; the second is kept.
                env.render([])
                rendered = env.render([*views, *cameras])
                videos.capture(f'{name}  t={t / CONTROL_HZ:6.2f}s  {episode.subtask(t) or ""}', rendered)
                if t < episode.length:
                    frames.capture(None, rendered)
            first = directory / 'views' / next(iter(views))
            if args.sheets:
                shots = [first / f'{i:05d}.jpg' for i in range(len(picks))]
            else:
                videos.encode(directory)
                shots = [first / f'{t:05d}.jpg' for t in picks]
            sheet(shots, [f't={t / CONTROL_HZ:.1f}s {episode.subtask(t) or ""}' for t in picks], directory / 'sheet.jpg')
            shutil.rmtree(directory / 'views')
            print(name, episode.length, f'{time.perf_counter() - started:.0f} s', flush=True)
        env.close()
    except Exception:
        traceback.print_exc()
        failed = True
    app.close(exit_code=int(failed))


if __name__ == '__main__':
    main()
