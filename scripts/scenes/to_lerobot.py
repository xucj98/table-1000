"""Convert rendered trajectories to a LeRobot v3.0 dataset (runs in the ``data`` environment).

``.venv-data/bin/python scripts/scenes/to_lerobot.py outputs/scenes/scene-000000/gen/trajectories.hdf5 --frames outputs/scenes/scene-000000/render --output outputs/scenes/scene-000000/lerobot [--task subtask|instruction]``

Every episode rendered by ``scripts/scenes/render.py`` (``<frames>/<demo>/frames/<camera>/``)
becomes one LeRobot episode at the control rate (25 fps):

- ``observation.state``: joint positions (``obs/qpos``, the robot's joints then
  the virtual base joints; names in the dataset info)
- ``action``: the flat action (``actions``; names ``<group>.<index>``, groups as in
  the trajectory file's ``action_layout``)
- ``observation.images.<camera>``: each robot camera, stored as video
- ``task``: the English instruction of the step's subtask (``--task subtask``,
  default), or the scene's one instruction (``--task instruction``)

LeRobot requires newer packages than Isaac Lab allows; set up its environment with
``uv sync --extra data`` into ``.venv-data`` (see the tutorial).
"""

import argparse
import json
from pathlib import Path
import shutil

import h5py
import numpy as np
from PIL import Image


def subtask(attrs, t, length):
    for entry in json.loads(attrs['subtasks']):
        if entry['start'] <= t < (entry['end'] if entry['end'] is not None else length):
            return entry['instruction']
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectories', type=Path)
    parser.add_argument('--frames', type=Path, required=True, help='output directory of render.py')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo-id', default=None, help='default: table-1000/<scene>')
    parser.add_argument('--task', choices=('subtask', 'instruction'), default='subtask')
    args = parser.parse_args(argv)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    with h5py.File(args.trajectories, 'r') as file:
        meta = json.loads(file['data'].attrs['env_args'])
        demos = [name for name in sorted(file['data'], key=lambda name: int(name.split('_')[1]))
                 if (args.frames / name / 'frames').is_dir()]
        if not demos:
            raise SystemExit(f'no rendered episodes of {args.trajectories} in {args.frames}')
        cameras = sorted(path.name for path in (args.frames / demos[0] / 'frames').iterdir())
        sizes = {camera: Image.open(next((args.frames / demos[0] / 'frames' / camera).iterdir())).size
                 for camera in cameras}
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
        for name in demos:
            group = file['data'][name]
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
    print(f'{len(demos)} episodes -> {args.output}')


if __name__ == '__main__':
    main()
