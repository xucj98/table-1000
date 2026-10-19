"""A batch of trajectories in ``data/scenes/<scene>/<backend>/``, without changing Isaac Lab's HDF5 format.

``trajectories.hdf5`` and ``trajectories_failed.hdf5`` hold one ``data/demo_<seed>``
group per episode; the processes of a batch write their own files, merged at the
end (:func:`merge`). Videos sit next to them, those of succeeded episodes in
``videos/``, of failed ones in ``fails/``: ``<camera>/demo_<seed>.mp4`` per
camera, ``demo_<seed>.mp4`` for the cameras merged into one. Each is encoded
elsewhere and moved in when complete, so an interrupted render leaves nothing
half-written.
"""

import json
from pathlib import Path


def merge(sources, target):
    """Copy every episode of the Isaac Lab trajectory files ``sources`` into ``target``, in seed order;
    returns the number of episodes. ``data`` keeps the first file's attributes, with the summed ``total``."""
    import h5py
    episodes, attrs = [], {}
    files = [h5py.File(path, 'r') for path in sources if Path(path).is_file()]
    try:
        for file in files:
            attrs = attrs or dict(file['data'].attrs)
            episodes += [(int(name.split('_')[1]), file, name) for name in file['data']]
        with h5py.File(target, 'w') as out:
            data = out.create_group('data')
            data.attrs.update(attrs)
            total = 0
            for _, file, name in sorted(episodes, key=lambda episode: episode[0]):
                file.copy(file['data'][name], data, name)
                total += int(file['data'][name].attrs.get('num_samples', 0))
            data.attrs['total'] = total
    finally:
        for file in files:
            file.close()
    return len(episodes)


def demo_name(seed):
    return f'demo_{seed}'


# The key of the merged videos in :func:`videos`.
MERGED = 'merged'


def video_dir(directory, failed=False):
    return Path(directory) / ('fails' if failed else 'videos')


def video_path(directory, seed, camera=None, failed=False):
    """A camera's video of episode ``seed``, or the merged one (``camera`` None)."""
    folder = video_dir(directory, failed)
    return (folder if camera is None else folder / camera) / f'{demo_name(seed)}.mp4'


def videos(directory):
    """Seeds with a video in a batch directory, ``{'videos': {camera: seeds}, 'fails': {...}}``,
    the merged videos under :data:`MERGED`."""
    def seeds(folder):
        return sorted(int(path.stem.split('_')[1]) for path in folder.glob('demo_*.mp4'))

    found = {}
    for failed, key in ((False, 'videos'), (True, 'fails')):
        folder = video_dir(directory, failed)
        kinds = {MERGED: seeds(folder)} if folder.is_dir() else {}
        kinds.update({camera.name: seeds(camera) for camera in sorted(folder.glob('*/'))})
        found[key] = {kind: values for kind, values in kinds.items() if values}
    return found


def subtask(attrs, t, length):
    """Instruction of the subtask at step t."""
    entries = attrs.get('subtasks', [])
    if isinstance(entries, str):
        entries = json.loads(entries)
    for entry in entries:
        if entry['start'] <= t < (entry['end'] if entry['end'] is not None else length):
            return entry['instruction']
    return None
