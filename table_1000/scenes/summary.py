"""A scene's acceptance summary, ``assets/scenes/<scene>/summary.json``: what a person reads first.

Each script rewrites its own section; generating a new batch also invalidates
the preview counts until the recordings are rendered again:

- ``initial`` (``check_scene.py``): seeds with and without a valid initial state,
  mean attempts, the largest settling motion of each object body, robot contacts;
- ``generation.<backend>`` (``generate.py``): where the trajectories are, the
  commit, seeds, success rate, failures by subtask and reason, unexpected
  contacts, durations and wall times;
- ``preview`` (``render.py``): the rendered episodes, with or without videos and
  robot camera frames.
"""

import json
import math
from pathlib import Path
import subprocess


def path(assets, scene):
    return Path(assets) / 'scenes' / scene / 'summary.json'


def update(assets, scene, section, value, key=None):
    """Set ``section`` (or ``section[key]``) of the scene's summary to ``value``; returns the file."""
    file = path(assets, scene)
    data = json.loads(file.read_text()) if file.exists() else {}
    data['scene'] = scene
    if key is None:
        data[section] = value
    else:
        data.setdefault(section, {})[key] = value
    file.parent.mkdir(parents=True, exist_ok=True)
    file.write_text(json.dumps(data, indent=1, ensure_ascii=False, default=float) + '\n')
    return file


def commit(root):
    """The checked-out commit of ``root``, ``+dirty`` with uncommitted changes (or None)."""
    try:
        head = subprocess.run(['git', '-C', str(root), 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', str(root), 'status', '--porcelain', '--untracked-files=no'],
                               capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    return head + ('+dirty' if dirty else '')


def initial_section(report):
    """``check_scene.py``'s report reduced to the numbers to accept it by."""
    settled = [attempt['settle'] for record in report['seeds'].values() for attempt in record['attempts']
               if attempt.get('settle')]
    objects = {}
    for settle in settled:
        for name, motion in settle['objects'].items():
            entry = objects.setdefault(name, {'max_move_mm': 0., 'max_turn_deg': 0.})
            entry['max_move_mm'] = max(entry['max_move_mm'], 1000 * motion['move_m'])
            entry['max_turn_deg'] = max(entry['max_turn_deg'], math.degrees(motion['turn_rad']))
    return {'seeds': len(report['seeds']), 'valid': report['valid'], 'invalid': report['invalid'],
            'without_state': [seed for seed, record in report['seeds'].items() if record['attempt'] is None],
            'mean_attempts': report['mean_attempts'], 'settle': objects,
            'robot_contacts': sum(len(settle['robot_contacts']) for settle in settled),
            'view': report['view'], 'files': ['preview/initial.json', 'preview/initial_states.jpg']}


def preview_section(preview, identities):
    """Only completed products matching the supplied episodes count toward acceptance."""
    from table_1000.scenes.recordings import read_render
    preview = Path(preview).resolve()
    sheets, videos, cameras, stale = [], [], [], []
    for name in sorted(identities, key=lambda name: int(name.split('_')[1])):
        directory = preview / name
        record = read_render(directory)
        if record.get('trajectory') != identities[name]:
            if directory.exists():
                stale.append(name)
            continue
        if record.get('sheet') and (directory / 'sheet.jpg').is_file():
            sheets.append(name)
        if record.get('videos') and all((directory / video).is_file() for video in record['videos']):
            videos.append(name)
        if record.get('cameras') and all((directory / 'frames' / camera).is_dir() for camera in record['cameras']):
            cameras.append(name)
    return {'output': str(preview), 'sheets': len(sheets), 'videos': videos,
            'camera_frames': cameras, 'stale': stale,
            'files': str(preview / 'demo_<seed>') + '/: sheet.jpg, <view>.mp4, frames/<camera>/, render.json'}
