"""Where a scene's renders go and which of them are complete, without changing Isaac Lab's HDF5 format.

A render output (by default the scene's directory ``assets/scenes/<scene>/``) holds
each episode's video ``<demo>.mp4`` at the top, where a person looks first, and
its other products in ``demos/<demo>/``: ``sheet.jpg``, ``frames/<camera>/``,
``render.json``. ``render.json`` lists only completed products, so an interrupted
render is not exported. Renders belong to the batch they were made from;
generating a new batch removes them (:func:`clear_renders`).
"""

import json
from pathlib import Path
import shutil


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


def begin_render(output, name, *, sheets):
    """Invalidate the episode's products about to be rendered: all of them, or only the review sheet
    (``sheets``). Returns the record."""
    directory = demo_dir(output, name)
    record = read_render(directory) if sheets else {}
    if not sheets:
        shutil.rmtree(directory, ignore_errors=True)
        video_path(output, name).unlink(missing_ok=True)
    record = {'sheet': False, 'cameras': record.get('cameras', []), 'views': record.get('views', [])}
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
