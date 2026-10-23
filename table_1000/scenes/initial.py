"""Initial states of a scene: the scene's ``initialize`` plus the common checks, retried with sub-seeds.

One attempt of seed ``s`` runs ``scene.initialize(layout)`` with
``layout.rng = numpy.random.default_rng([s, attempt])``, then checks:

1. **no penetration** of the placed objects and the table (collision geometry,
   at rest; touching is allowed, :data:`~table_1000.scenes.layout.PENETRATION_LIMIT`);
2. after the reset, **settling** for :data:`SETTLE_S` with the robot holding still:
   every object body stays within :data:`MOVE_LIMIT` / :data:`TURN_LIMIT` of where it was
   put, relative to its support (the table or another object), its joints within
   :data:`JOINT_LIMIT`, and the robot touches nothing.

A failed attempt is retried with the next attempt number, at most
:data:`ATTEMPTS` times; the episode starts from the settled state, whatever the
layout predicted. The positions are compared, not velocities: a pen whose cap
balls jitter by micrometres never sleeps but stays put.

``scripts/scenes/build_scene.py`` keeps the settled states in
``initial_states.hdf5`` (:func:`write_states`), one group per seed named by it,
with attributes ``seed`` and ``attempt`` and the scene state as
``InteractiveScene.get_state(is_relative=True)`` gives it; the file's attributes
name the scene, robot and physics backend. Trajectories start from these states.
"""

import math
import time

import numpy as np

from table_1000.scenes.layout import Layout, SamplingFailed
from table_1000.sim import CONTROL_HZ

SETTLE_S = 1.
MOVE_LIMIT = .003            # m
TURN_LIMIT = np.radians(2.)
JOINT_LIMIT = .003           # m or rad
ATTEMPTS = 20


class InitializationFailed(RuntimeError):
    def __init__(self, seed, attempts):
        super().__init__(f'seed {seed}: no valid initial state in {len(attempts)} attempts')
        self.attempts = attempts


def relative_poses(env, layout):
    """Every object body's pose relative to its support (the world for the table)."""
    poses = env.privileged()['poses']
    supports = {on: np.linalg.inv(env.object_pose(on)[0])
                for on in set(layout.supports.values()) if on != 'table'}
    relative = {}
    for key, pose in poses.items():
        name, separator, _ = key.partition(':')
        if separator and name in layout.supports:
            on = layout.supports[name]
            relative[key] = supports[on] @ pose[0] if on != 'table' else pose[0].copy()
    return relative


def settle_report(env, layout, seconds=SETTLE_S):
    """Settle the reset environment; returns how far objects and joints moved and what the robot touched."""
    before, joints = relative_poses(env, layout), env.privileged()['joints']
    env.contacts.clear()
    for _ in range(round(seconds * CONTROL_HZ)):
        env.step()
    after, joints_after = relative_poses(env, layout), env.privileged()['joints']
    moved = {}
    for name in before:
        delta = np.linalg.inv(before[name]) @ after[name]
        angle = np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1., 1.))
        moved[name] = {'move_m': float(np.linalg.norm(after[name][:3, 3] - before[name][:3, 3])),
                       'turn_rad': float(angle)}
    joint_moves = {f'{name}.{joint}': float(abs(joints_after[name][joint][0] - value[0]))
                   for name, values in joints.items() for joint, value in values.items()}
    touched = sorted({' / '.join(entry['pair']) for entry in env.contacts.intervals()})
    return {'objects': moved, 'joints': joint_moves, 'robot_contacts': touched}


def problems_after_settling(report):
    problems = [f'{name} moved {value["move_m"] * 1000:.1f} mm / {np.degrees(value["turn_rad"]):.1f} deg'
                for name, value in report['objects'].items()
                if value['move_m'] > MOVE_LIMIT or value['turn_rad'] > TURN_LIMIT]
    problems += [f'{joint} moved {value:.4f}' for joint, value in report['joints'].items() if value > JOINT_LIMIT]
    problems += [f'robot touches {pair}' for pair in report['robot_contacts']]
    return problems


def initial_state(env, scene, assets, seed, attempts=ATTEMPTS):
    """Reset ``env`` to a checked, settled initial state of ``scene`` for ``seed``.

    Returns ``(layout, record)``; ``record`` lists every attempt with its problems,
    the accepted one with its settling report. Raises :class:`InitializationFailed`.
    """
    tried = []
    for attempt in range(attempts):
        started = time.perf_counter()
        layout = Layout(scene, assets, np.random.default_rng([seed, attempt]))
        entry = {'attempt': attempt}
        tried.append(entry)
        try:
            scene.initialize(layout)
        except SamplingFailed as error:
            entry['problems'] = [str(error)]
            continue
        overlaps = layout.penetrations()
        if overlaps:
            entry['problems'] = [f'{a} penetrates {b} by {depth * 1000:.1f} mm' for (a, b), depth in overlaps.items()]
            continue
        env.reset(layout.poses, object_q=layout.joints)
        entry['settle'] = settle_report(env, layout)
        entry['problems'] = problems_after_settling(entry['settle'])
        entry['wall_s'] = time.perf_counter() - started
        if not entry['problems']:
            return layout, {'seed': seed, 'attempt': attempt, 'attempts': tried}
    raise InitializationFailed(seed, tried)


def write_tree(group, tree):
    """Write a nested dict of arrays into the HDF5 ``group``."""
    for key, value in tree.items():
        if isinstance(value, dict):
            write_tree(group.create_group(key), value)
        else:
            group.create_dataset(key, data=np.asarray(value))


def read_tree(group):
    import h5py
    return {key: read_tree(value) if isinstance(value, h5py.Group) else value[()] for key, value in group.items()}


def write_states(path, states, attrs):
    """Write ``{seed: (attempt, state)}`` (a state: nested dict of arrays) with file attributes ``attrs``."""
    import h5py
    with h5py.File(path, 'w') as file:
        file.attrs.update(attrs)
        for seed, (attempt, state) in sorted(states.items()):
            group = file.create_group(str(seed))
            group.attrs.update(seed=seed, attempt=attempt)
            write_tree(group, state)


def read_states(path):
    """``(attrs, {seed: (attempt, state)})`` of an ``initial_states.hdf5`` file, states as nested NumPy dicts."""
    import h5py
    with h5py.File(path, 'r') as file:
        return dict(file.attrs), {int(name): (int(group.attrs['attempt']), read_tree(group))
                                  for name, group in sorted(file.items(), key=lambda item: int(item[0]))}


def summarize(records):
    """Numbers to accept initial states by, from :func:`initial_state`'s records ``{seed: record}``
    (``attempt`` None for a seed without a valid state): seeds with and without a state, mean
    attempts, the largest settling motion of each object body, robot contacts while settling."""
    accepted = [record['attempt'] + 1 for record in records.values() if record['attempt'] is not None]
    settled = [attempt['settle'] for record in records.values() for attempt in record['attempts']
               if attempt.get('settle')]
    objects = {}
    for settle in settled:
        for name, motion in settle['objects'].items():
            entry = objects.setdefault(name, {'max_move_mm': 0., 'max_turn_deg': 0.})
            entry['max_move_mm'] = max(entry['max_move_mm'], 1000 * motion['move_m'])
            entry['max_turn_deg'] = max(entry['max_turn_deg'], math.degrees(motion['turn_rad']))
    return {'valid': len(accepted), 'invalid': len(records) - len(accepted),
            'without_state': sorted(seed for seed, record in records.items() if record['attempt'] is None),
            'mean_attempts': float(np.mean(accepted)) if accepted else None, 'settle': objects,
            'robot_contacts': sum(len(settle['robot_contacts']) for settle in settled)}

