"""Link completed renders to Isaac Lab episodes without changing its HDF5 format.

Fingerprints cover metadata, labels, states and actions and survive file copies.
Only completed products are published in render.json.

A render output (by default the scene's directory ``assets/scenes/<scene>/``) holds
each episode's video ``<demo>.mp4`` at the top, where a person looks first, and
its other products in ``demos/<demo>/``: ``sheet.jpg``, ``frames/<camera>/``,
``render.json``.
"""

import hashlib
import json
from pathlib import Path
import shutil


def fingerprint(group):
    """SHA-256 of an HDF5 episode, streamed in rows to bound memory use."""
    digest = hashlib.sha256()

    def metadata(value):
        digest.update(json.dumps(value, sort_keys=True, default=lambda x: x.tolist()).encode())
        digest.update(b'\0')

    metadata(json.loads(group.parent.attrs['env_args']))
    metadata(dict(group.attrs))

    def visit(name, dataset):
        if not hasattr(dataset, 'shape'):
            return
        metadata([name, dataset.dtype.str, dataset.shape])
        if dataset.ndim == 0:
            digest.update(dataset[()].tobytes())
        else:
            for start in range(0, len(dataset), 256):
                digest.update(dataset[start:start + 256].tobytes())

    group.visititems(visit)
    return digest.hexdigest()


def demo_dir(output, name):
    return Path(output) / 'demos' / name


def video_path(output, name):
    return Path(output) / f'{name}.mp4'


def clear_renders(output):
    """Remove every episode's products from ``output``, e.g. when a new batch replaces the episodes."""
    shutil.rmtree(Path(output) / 'demos', ignore_errors=True)
    for video in Path(output).glob('demo_*.mp4'):
        video.unlink()


def read_render(directory):
    file = Path(directory) / 'render.json'
    return json.loads(file.read_text()) if file.exists() else {}


def write_render(directory, record):
    file = Path(directory) / 'render.json'
    temporary = file.with_suffix('.tmp')
    temporary.write_text(json.dumps(record, indent=1) + '\n')
    temporary.replace(file)


def begin_render(output, name, identity, *, sheets):
    """Invalidate the episode's products in ``output`` before rendering; keep the video and camera
    frames of a full render only for the same episode. Returns the record."""
    directory = demo_dir(output, name)
    record = read_render(directory)
    if not sheets or record.get('trajectory') != identity:
        shutil.rmtree(directory, ignore_errors=True)
        video_path(output, name).unlink(missing_ok=True)
        record = {'trajectory': identity, 'cameras': [], 'views': []}
    record['sheet'] = False
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'sheet.jpg').unlink(missing_ok=True)
    shutil.rmtree(directory / 'views', ignore_errors=True)
    write_render(directory, record)
    return record


def subtask(attrs, t, length):
    """Instruction of the subtask at step t, shared by rendering and export."""
    entries = attrs.get('subtasks', [])
    if isinstance(entries, str):
        entries = json.loads(entries)
    for entry in entries:
        if entry['start'] <= t < (entry['end'] if entry['end'] is not None else length):
            return entry['instruction']
    return None
