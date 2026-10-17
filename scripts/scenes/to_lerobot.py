"""Convert rendered trajectories to a LeRobot v3.0 dataset (needs the ``data`` extra).

``python scripts/scenes/to_lerobot.py outputs/scenes/scene-000000/gen/part*/trajectories.hdf5 --frames outputs/scenes/scene-000000/render --output outputs/scenes/scene-000000/lerobot [--task subtask|instruction]``

Every episode rendered by ``scripts/scenes/render.py`` (``<frames>/<demo>/frames/<camera>/``)
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

import h5py
import numpy as np
from PIL import Image


def subtask(attrs, t, length):
    for entry in json.loads(attrs['subtasks']):
        if entry['start'] <= t < (entry['end'] if entry['end'] is not None else length):
            return entry['instruction']
    return None


def rendered_episodes(files, frames):
    """``[(group, name)]`` of the episodes in ``files`` rendered under ``frames``, in seed order."""
    found = [(file['data'][name], name) for file in files for name in file['data'] if (frames / name / 'frames').is_dir()]
    return sorted(found, key=lambda item: int(item[1].split('_')[1]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectories', type=Path, nargs='+', help='trajectory files (e.g. the parts of a run)')
    parser.add_argument('--frames', type=Path, required=True, help='output directory of render.py')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo-id', default=None, help='default: table-1000/<scene>')
    parser.add_argument('--task', choices=('subtask', 'instruction'), default='subtask')
    args = parser.parse_args(argv)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    files = [h5py.File(path, 'r') for path in args.trajectories]
    meta = json.loads(files[0]['data'].attrs['env_args'])
    episodes = rendered_episodes(files, args.frames)
    if not episodes:
        raise SystemExit(f'no rendered episodes of {[str(p) for p in args.trajectories]} in {args.frames}')
    first = args.frames / episodes[0][1] / 'frames'
    cameras = sorted(path.name for path in first.iterdir())
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
