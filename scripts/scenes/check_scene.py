"""Initial states of a scene for review: the common checks over seeds and a 5 x 5 picture grid.

``uv run python scripts/scenes/check_scene.py 000000 --gpu 1 [--seeds 0:25] [--view top] [--output DIR]``

For each seed, makes the initial state as an episode would (``scene.initialize``,
penetration and settling checks, retried with sub-seeds; see
``table_1000/scenes/initial.py``) and renders it from one of the scene's views.
Writes ``initial.json`` (every attempt with its problems and settling report,
attempts per seed, seeds without a valid state) and ``initial_states.jpg`` (one
picture per seed, captioned with the seed and the accepted attempt) into the
scene's preview directory ``assets/scenes/<scene>/preview/`` (or ``--output``). The exit status is 1
when a seed has no valid initial state or the run fails.
"""

import argparse
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.scenes.generate import seeds  # noqa: E402

COLUMNS, TILE_WIDTH = 5, 384


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene')
    parser.add_argument('--seeds', nargs='+', default=['0:25'])
    parser.add_argument('--output', type=Path, help='default: assets/scenes/<scene>/preview')
    parser.add_argument('--view', default='top', help='a view of the scene')
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    args = parser.parse_args(argv)
    output = (args.output or args.assets / 'scenes' / args.scene / 'preview').resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    from table_1000.sim.isaaclab.render import RT2_ARGS
    started = time.perf_counter()
    app = start_app(gpu=args.gpu, workers=4, extra_args=RT2_ARGS)
    failed = False
    try:
        import numpy as np
        from PIL import Image, ImageDraw
        from table_1000.robots import load_spec
        from table_1000.scenes import load
        from table_1000.scenes.initial import InitializationFailed, initial_state
        from table_1000.sim.isaaclab import render
        from table_1000.sim.isaaclab.env import RobotEnv
        render.configure()
        root, assets = args.root.resolve(), args.assets.resolve()
        scene = load(args.scene, 'scene', root)
        view = args.view
        env = RobotEnv(scene, load_spec(Path(scene.ROBOT['asset']).name, root), assets,
                       views={view: scene.VIEWS[view]}, root=root)
        report, tiles = {'scene': args.scene, 'view': view, 'seeds': {}}, []
        for seed in seeds(args.seeds):
            try:
                _, record = initial_state(env, scene, assets, seed)
            except InitializationFailed as error:
                report['seeds'][seed] = {'seed': seed, 'attempt': None, 'attempts': error.attempts}
                tiles.append((f'seed {seed}: no valid state', None))
                failed = True
                continue
            report['seeds'][seed] = record
            env.render([view])
            image = env.render([view])[view]['rgb'][0]
            tiles.append((f'seed {seed}  attempt {record["attempt"]}', image))
            print(seed, 'attempt', record['attempt'], flush=True)
        accepted = [r['attempt'] + 1 for r in report['seeds'].values() if r['attempt'] is not None]
        report.update(valid=len(accepted), invalid=len(report['seeds']) - len(accepted),
                      mean_attempts=float(np.mean(accepted)) if accepted else None,
                      acceptance_rate=len(accepted) / sum(len(r['attempts']) for r in report['seeds'].values()),
                      total_s=time.perf_counter() - started)
        (output / 'initial.json').write_text(json.dumps(report, indent=1, default=float))
        width, height = scene.VIEWS[view]['resolution']
        tile_height = round(height * TILE_WIDTH / width)
        rows = -(-len(tiles) // COLUMNS)
        grid = Image.new('RGB', (COLUMNS * TILE_WIDTH, rows * (tile_height + 18)), 'white')
        draw = ImageDraw.Draw(grid)
        for i, (caption, image) in enumerate(tiles):
            x, y = (i % COLUMNS) * TILE_WIDTH, (i // COLUMNS) * (tile_height + 18)
            if image is not None:
                grid.paste(Image.fromarray(image).resize((TILE_WIDTH, tile_height)), (x, y + 18))
            draw.text((x + 4, y + 3), caption, fill='black')
        grid.save(output / 'initial_states.jpg', quality=90)
        print(json.dumps({key: value for key, value in report.items() if key != 'seeds'}, indent=1))
        env.close()
    except Exception:
        traceback.print_exc()
        failed = True
    app.close(exit_code=int(failed))


if __name__ == '__main__':
    main()
