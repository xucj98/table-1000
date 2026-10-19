"""Videos of a batch of trajectories, rendered from their recorded states.

``uv run python scripts/scenes/render.py 000000 --physics isaac-cpu --gpus 0 1 2 3 [--count N] [--seeds ...] [--failed] [--input DIR]``

Reads the batch ``<input>/<scene>/<backend>/`` (``<input>`` = ``data/scenes``,
written by ``run_expert.py``) and writes ``demo_<seed>.mp4`` into it: the
scene's fixed views side by side, one frame per control step (25 fps, 1x),
captioned with the time and subtask.

The episodes are the batch's succeeded ones, by default all; ``--seeds`` (as in
``build_scene.py``) and ``--count N`` (the first N by seed) narrow them;
``--failed`` adds every failed episode, captioned with its failure. A run
replaces the batch's earlier videos. Every frame restores the recorded state of
that step; nothing is simulated. The episodes are split into
``len(gpus) × jobs-per-gpu`` parts, one process each (logs in
``outputs/scenes/<scene>/<backend>/render/``); ``summary.json`` lists the
seeds with a video.
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

def choose(succeeded, seeds=None, count=None):
    """The succeeded seeds to render: those of ``seeds`` (all by default), the first ``count``.
    Raises ``ValueError`` naming the requested seeds that did not succeed."""
    chosen = sorted(succeeded)
    if seeds is not None:
        missing = sorted(set(seeds) - set(succeeded))
        if missing:
            raise ValueError(missing)
        chosen = sorted(seeds)
    return chosen[:count] if count is not None else chosen


def worker(args):
    """Render ``args.seeds`` of the batch in this process."""
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app = start_app(gpu=args.gpu, workers=4, extra_args=RT2_ARGS)
    failed = False
    try:
        from table_1000.robots import load_spec
        from table_1000.scenes import load
        from table_1000.scenes.batch import parse_seeds
        from table_1000.scenes.recordings import demo_name, video_path
        from table_1000.sim import CONTROL_HZ
        from table_1000.sim.isaaclab import render, replay
        from table_1000.sim.isaaclab.env import RobotEnv
        from table_1000.sim.isaaclab.video import Video
        render.configure()
        directory, work = args.batch.resolve(), args.work.resolve()
        file, episodes = replay.episodes(directory / 'trajectories.hdf5')
        meta = replay.env_args(file)
        if (directory / 'trajectories_failed.hdf5').is_file():
            episodes.update(replay.episodes(directory / 'trajectories_failed.hdf5')[1])
        scene = load(meta['scene'], 'scene', args.root)
        spec = load_spec(meta['robot'], args.root)
        views = dict(scene.VIEWS)
        env = RobotEnv(scene, spec, args.assets.resolve(), views=views, root=args.root)
        for seed in parse_seeds(args.seeds):
            started = time.perf_counter()
            name = demo_name(seed)
            episode = episodes[name]
            frames = work / name
            shutil.rmtree(frames, ignore_errors=True)
            video = Video(env, list(views), frames, CONTROL_HZ)
            failure = episode.attrs.get('failure')
            note = f'  FAILED {failure["subtask"]}: {failure["reason"]}' if failure else ''
            for t in range(episode.length + 1):
                replay.set_state(env, episode.state(t))
                # RT2 denoises over time, so the first render after a state change still blends in
                # the previous one; the second is kept.
                env.render([])
                video.capture(f'{name}  t={t / CONTROL_HZ:6.2f}s  {episode.subtask(t) or ""}{note}',
                              env.render(list(views)))
            shutil.move(video.encode_side_by_side(work / f'{name}.mp4'), video_path(directory, seed))
            shutil.rmtree(frames)
            print(name, episode.length, f'{time.perf_counter() - started:.0f} s', flush=True)
        env.close()
    except Exception:
        traceback.print_exc()
        failed = True
    app.close(exit_code=int(failed))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scene')
    parser.add_argument('--physics', required=True, help='the backend the batch was generated on')
    parser.add_argument('--count', type=int, help='the first N succeeded episodes by seed')
    parser.add_argument('--seeds', nargs='+', help='a:b (a to b - 1) and single seeds of succeeded episodes')
    parser.add_argument('--failed', action='store_true', help='also every failed episode')
    parser.add_argument('--gpus', type=int, nargs='+', default=[0], help='physical GPUs')
    parser.add_argument('--jobs-per-gpu', type=int, default=1, help='processes at a time on each GPU')
    parser.add_argument('--input', type=Path, default=ROOT / 'data' / 'scenes',
                        help='the batch in <input>/<scene>/<backend>/, renders written next to it')
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--batch', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--work', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--gpu', type=int, default=0, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        worker(args)
        return
    import h5py
    from table_1000.scenes.batch import parse_seeds, run_parts, seed_range, split
    from table_1000.scenes.recordings import videos
    if args.count is not None and args.count < 1:
        parser.error('--count must be positive')
    if args.jobs_per_gpu < 1:
        parser.error('--jobs-per-gpu must be positive')
    directory = (args.input / args.scene / args.physics).resolve()
    trajectories = directory / 'trajectories.hdf5'
    if not trajectories.is_file():
        parser.error(f'no trajectories {trajectories}; run run_expert.py first')
    summary = json.loads((directory / 'summary.json').read_text())
    with h5py.File(trajectories, 'r') as file:
        succeeded = [int(name.split('_')[1]) for name in file['data']]
    try:
        seeds = choose(succeeded, parse_seeds(args.seeds) if args.seeds else None, args.count)
    except ValueError as error:
        failed = sorted(seed for seeds in summary['failures'].values() for seed in seeds)
        parser.error(f'seeds {seed_range(error.args[0])} have no succeeded episode; the batch covers '
                     f'{summary["seeds"]["range"]}, failed: {seed_range(failed) or "none"}')
    if args.failed and (directory / 'trajectories_failed.hdf5').is_file():
        with h5py.File(directory / 'trajectories_failed.hdf5', 'r') as file:
            seeds = sorted(seeds + [int(name.split('_')[1]) for name in file['data']])
    if not seeds:
        parser.error('no episodes to render')
    root = args.root.resolve()
    staging = root / 'outputs' / 'scenes' / args.scene / args.physics / 'render'
    shutil.rmtree(staging, ignore_errors=True)
    for video in directory.glob('demo_*.mp4'):
        video.unlink()
    jobs = []
    for index, part in enumerate(split(seeds, len(args.gpus) * args.jobs_per_gpu)):
        command = [sys.executable, str(Path(__file__).resolve()), args.scene, '--worker', '--physics', args.physics,
                   '--seeds', *seed_range(part).split(), '--batch', str(directory), '--work', str(staging / f'part{index}'),
                   '--root', str(root), '--assets', str(args.assets.resolve())]
        jobs.append((command, staging / f'part{index}.log', f'{args.scene} render part{index} seeds {seed_range(part)}'))
    failed = run_parts(jobs, args.gpus, args.jobs_per_gpu)
    summary['videos'] = seed_range(videos(directory))
    (directory / 'summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False, default=float) + '\n')
    print('VIDEOS', summary['videos'], flush=True)
    if failed:
        raise SystemExit('failed: ' + ', '.join(failed))


if __name__ == '__main__':
    main()
