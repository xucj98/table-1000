"""Convert rendered trajectories to a LeRobot v3.0 dataset (needs the ``data`` extra).

``uv run python scripts/scenes/to_lerobot.py outputs/scenes/000000/gen/part*/trajectories.hdf5 --output outputs/scenes/000000/lerobot [--frames DIR] [--task subtask|instruction]``

Every episode rendered by ``scripts/scenes/render.py`` (``<frames>/<demo>/frames/<camera>/``;
``--frames`` defaults to the scene's preview directory ``assets/scenes/<scene>/preview/``)
becomes one LeRobot episode at the control rate (25 fps):

- ``observation.state``: joint positions (``obs/qpos``, the robot's joints then
  the virtual base joints; names in the dataset info)
- ``action``: the flat action (``actions``; names ``<group>.<index>``, groups as in
  the trajectory file's ``action_layout``)
- ``observation.images.<camera>``: each robot camera, stored as video
- ``task``: the English instruction of the step's subtask (``--task subtask``,
  default), or the scene's one instruction (``--task instruction``)
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import h5py
import numpy as np
from PIL import Image

from table_1000.scenes.recordings import fingerprint, read_render, subtask


def rendered_episodes(files, frames):
    """Select completed renders, refusing stale or incomplete camera observations."""
    found, names, cameras = [], set(), None
    for file in files:
        for name, group in file['data'].items():
            if name in names:
                raise ValueError(f'duplicate episode {name}; give one recording per seed')
            names.add(name)
            directory = frames / name
            if not (directory / 'frames').is_dir():
                continue
            record = read_render(directory)
            if record.get('trajectory') != fingerprint(group) or not record.get('cameras'):
                raise ValueError(f'{directory}: stale or incomplete camera render; rerun render.py without --sheets')
            current = sorted(record['cameras'])
            if cameras is not None and current != cameras:
                raise ValueError(f'{directory}: camera set differs; rerender with the same --cameras')
            cameras = current
            expected = {f'{t:05d}.jpg' for t in range(int(group.attrs['num_samples']))}
            for camera in cameras:
                if {p.name for p in (directory / 'frames' / camera).glob('*.jpg')} != expected:
                    raise ValueError(f'{directory}: incomplete frames for {camera}; rerun render.py without --sheets')
            found.append((group, name))
    return sorted(found, key=lambda item: int(item[1].split('_')[1]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectories', type=Path, nargs='+', help='trajectory files (e.g. the parts of a run)')
    parser.add_argument('--frames', type=Path, help='output directory of render.py (default: assets/scenes/<scene>/preview)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo-id', default=None, help='default: table-1000/<scene>')
    parser.add_argument('--task', choices=('subtask', 'instruction'), default='subtask')
    args = parser.parse_args(argv)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    files = [h5py.File(path, 'r') for path in args.trajectories]
    meta = json.loads(files[0]['data'].attrs['env_args'])
    args.frames = args.frames or Path(__file__).resolve().parents[2] / 'assets' / 'scenes' / meta['scene'] / 'preview'
    episodes = rendered_episodes(files, args.frames)
    if not episodes:
        raise SystemExit(f'no rendered episodes of {[str(p) for p in args.trajectories]} in {args.frames}')
    first = args.frames / episodes[0][1] / 'frames'
    cameras = sorted(read_render(first.parent)['cameras'])
    sizes = {camera: Image.open(next((first / camera).iterdir())).size for camera in cameras}
    actions = [f'{group}.{i}' for group, (start, stop) in meta['action_layout'].items() for i in range(stop - start)]
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (len(meta['joints']),), 'names': meta['joints']},
        'action': {'dtype': 'float32', 'shape': (len(actions),), 'names': actions},
        **{f'observation.images.{camera}': {'dtype': 'video', 'shape': (height, width, 3),
                                            'names': ['height', 'width', 'channels']}
           for camera, (width, height) in sizes.items()}}
    shutil.rmtree(args.output, ignore_errors=True)
    dataset = LeRobotDataset.create(args.repo_id or f'table-1000/{meta["scene"]}', fps=meta['control_hz'],
                                    features=features, root=args.output, robot_type=meta['robot'])
    for group, name in episodes:
        length = int(group.attrs['num_samples'])
        qpos, action = np.asarray(group['obs/qpos'], np.float32), np.asarray(group['actions'], np.float32)
        for t in range(length):
            task = subtask(group.attrs, t, length) if args.task == 'subtask' else None
            frame = {'observation.state': qpos[t], 'action': action[t], 'task': task or meta['instruction']}
            for camera in cameras:
                frame[f'observation.images.{camera}'] = np.asarray(
                    Image.open(args.frames / name / 'frames' / camera / f'{t:05d}.jpg').convert('RGB'))
            dataset.add_frame(frame)
        dataset.save_episode()
        print(name, length, flush=True)
    dataset.finalize()
    print(f'{len(episodes)} episodes -> {args.output}')


if __name__ == '__main__':
    main()
