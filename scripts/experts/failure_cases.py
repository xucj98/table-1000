"""Failed expert episodes as cases to look at: views of the scene where the failing subtask began
and where the episode stopped, with the numbers of its report.

``uv run python scripts/experts/failure_cases.py data/scenes/000002/isaac-cpu outputs/cases/000002 [--seeds 4 44]``

Runs on a GPU (e.g. under ``gq submit``). For every failed seed of the batch (``--seeds``: only
those), into ``<out>/<seed>/``:

- ``start_<view>.jpg``, ``end_<view>.jpg``: the first state of the failing subtask and the last
  state, from the robot's head camera (``face_view``), the scene's own views (``wide``, ``close``)
  and four around the scene's ``wide`` target (``top``, ``right``, ``left``, ``front``);
- ``start.jpg``, ``end.jpg``: those views tiled;
- ``case.json``: the commit and seed, the failure (subtask, reason, details), what the motion
  planners rejected in that subtask, the events it noted, and the steps the frames are at.

To see whether a case fails again with the same code, rerun its seed from the checkpoint of the
step before (``run_expert.py --steps k --from <batch> --seeds <seed>``).
"""

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# Views around the target of the scene's wide view: (offset of the camera from it, focal length mm).
AROUND = {
    'top': ((0., -.25, 1.55), 16),
    'right': ((1.25, -.10, .15), 22),
    'left': ((-1.25, -.10, .15), 22),
    'front': ((0., 1.25, .25), 20),
}
CELL_HEIGHT = 360       # px of each view in the tiled images
SETTLE_RENDERS = 4      # renders before the captured one, for the denoiser to settle


def views(scene):
    """The scene's views and those around its wide view's target."""
    target = scene.VIEWS['wide']['target']
    return {**scene.VIEWS, **{name: {'position': tuple(t + o for t, o in zip(target, offset)), 'target': target,
                                     'focal_length_mm': focal, 'resolution': (960, 720 if name == 'top' else 540)}
                              for name, (offset, focal) in AROUND.items()}}


def case(report):
    """The numbers of a failed episode's report for its case."""
    failure = report['failure']
    events = [event for event in report['events'] if event['subtask'] == failure['subtask']]
    tried = [event for event in events if event['note'].startswith('next ')]     # an option given up for the next
    collisions = Counter(f'{link} - {other}' for event in tried for link, other, _ in event.get('collisions', []))
    return {'seed': report['seed'], 'failure': failure, 'events': Counter(event['note'] for event in events),
            'rejections': dict(Counter(event['reason'] for event in tried).most_common()),
            'collisions': dict(collisions.most_common()), 'duration_s': report['duration_s'],
            'wall_s': report['wall_s']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('batch', type=Path, help='a batch directory of run_expert.py')
    parser.add_argument('out', type=Path)
    parser.add_argument('--seeds', nargs='+', help='only these failed seeds: 12 27, 0:20')
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the scene and robot sources')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--gpu', type=int)
    args = parser.parse_args(argv)
    from table_1000.common.batch import parse_seeds, worker_gpu
    summary = json.loads((args.batch / 'summary.json').read_text())
    failures = sorted(seed for seeds in summary['failures'].values() for seed in seeds)
    seeds = parse_seeds(args.seeds) if args.seeds else failures
    if absent := sorted(set(seeds) - set(failures)):
        parser.error(f'seeds {absent} did not fail in {args.batch}')
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    app = start_app(gpu=worker_gpu(args.gpu), workers=4, extra_args=RT2_ARGS)
    failed = False
    try:
        from PIL import Image
        from table_1000.robots import load_spec
        from table_1000.scenes import load
        from table_1000.sim.isaaclab import render, replay
        from table_1000.sim.isaaclab.env import RobotEnv
        from table_1000.sim.isaaclab.video import tile
        render.configure()
        file, episodes = replay.episodes(args.batch / 'trajectories_failed.hdf5')
        meta = replay.env_args(file)
        scene = load(meta['scene'], 'scene', args.root)
        spec = load_spec(meta['robot'], args.root)
        env = RobotEnv(scene, spec, args.assets, cameras={'face_view': spec.cameras['face_view']}, views=views(scene),
                       root=args.root)
        names = ['face_view', *views(scene)]
        for seed in seeds:
            episode = episodes[f'demo_{seed}']
            report = json.loads((args.batch / 'episodes' / f'{seed}.json').read_text())
            subtask, last = report['failure']['subtask'], episode.length - 1
            start = next((t for t in range(episode.length) if episode.subtask(t) == subtask), last)
            out = args.out / str(seed)
            out.mkdir(parents=True, exist_ok=True)
            for moment, t in (('start', start), ('end', last)):
                replay.set_state(env, episode.state(t))
                for _ in range(SETTLE_RENDERS):
                    env.render([])
                shots = env.render(names)
                images = {name: Image.fromarray(shots[name]['rgb'][0]) for name in names}
                for name, image in images.items():
                    image.save(out / f'{moment}_{name}.jpg', quality=88)
                tile([images[name] for name in names], CELL_HEIGHT).save(out / f'{moment}.jpg', quality=85)
            (out / 'case.json').write_text(json.dumps({'commit': summary['commit'], 'batch': str(args.batch.resolve()),
                                                       **case(report), 'frames': {'start': start, 'end': last}},
                                                      indent=1, default=str))
            print(seed, subtask, report['failure']['reason'], flush=True)
        env.close()
    except Exception:
        traceback.print_exc()
        failed = True
    app.close(exit_code=int(failed))


if __name__ == '__main__':
    main()
