"""Initial states of a scene: the scene's ``initialize`` plus the common checks, retried with sub-seeds.

One attempt of seed ``s`` runs ``scene.initialize(layout)`` with
``layout.rng = numpy.random.default_rng([s, attempt])``, then checks:

1. **no penetration** of the placed objects and the table (collision geometry,
   at rest; touching is allowed, :data:`~table_1000.scenes.layout.PENETRATION_LIMIT`);
2. when the layout has loose objects (dropped ones, :meth:`~table_1000.scenes.layout.Layout.drop`),
   **landing** for :data:`LANDING_S` after the reset: loose objects may move as long as their
   origin stays over their support (a pen tipping against the side of its cup, a clip sliding
   in a drawer: within the support's collision box, or over the table), or anywhere with the
   layout's ``free_landing`` (a messy desk: a pen falling out of its cup); the others are held
   to the settling limits below, and nothing may fall off the table (what the layout
   stands on the ``'floor'`` stays there);
3. **settling** for :data:`SETTLE_S` with the robot holding still: every object body
   stays within :data:`MOVE_LIMIT` / :data:`TURN_LIMIT` of where it was, relative to its
   support (the table or another object), its joints within :data:`JOINT_LIMIT`, and the
   robot touches nothing.

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
LANDING_S = 3.
FALLEN_DEPTH = .05           # m below the table top: an object body there fell off the table
ATTEMPTS = 20


class InitializationFailed(RuntimeError):
    def __init__(self, seed, attempts):
        super().__init__(f'seed {seed}: no valid initial state in {len(attempts)} attempts')
        self.attempts = attempts


def relative_poses(env, layout):
    """Every object body's pose relative to its support (the world for the table)."""
    poses = env.privileged()['poses']
    supports = {on: np.linalg.inv(env.object_pose(on)[0])
                for on in set(layout.supports.values()) if on not in ('table', 'floor')}
    relative = {}
    for key, pose in poses.items():
        name, separator, _ = key.partition(':')
        if separator and name in layout.supports:
            on = layout.supports[name]
            relative[key] = supports[on] @ pose[0] if on in supports else pose[0].copy()
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
    top = layout.scene.TABLE['height'] - FALLEN_DEPTH
    fallen = sorted(key for key, pose in env.privileged()['poses'].items()
                    if pose[0, 2, 3] < top and layout.supports.get(key.partition(':')[0]) != 'floor')
    return {'objects': moved, 'joints': joint_moves, 'robot_contacts': touched, 'fallen': fallen}


def problems_after_settling(report, loose=()):
    """What exceeds the settling limits; bodies of ``loose`` objects have none (see :func:`strayed`)."""
    def limits(key):
        return (np.inf, np.inf) if key.partition(':')[0] in loose else (MOVE_LIMIT, TURN_LIMIT)

    problems = [f'{name} moved {value["move_m"] * 1000:.1f} mm / {np.degrees(value["turn_rad"]):.1f} deg'
                for name, value in report['objects'].items()
                if value['move_m'] > limits(name)[0] or value['turn_rad'] > limits(name)[1]]
    problems += [f'{joint} moved {value:.4f}' for joint, value in report['joints'].items() if value > JOINT_LIMIT]
    problems += [f'robot touches {pair}' for pair in report['robot_contacts']]
    problems += [f'{key} fell off the table' for key in report.get('fallen', ())]
    return problems


def strayed(env, layout):
    """Loose objects whose foot (the middle of their collision points within 2 mm of the lowest;
    a notebook leaning in a basket has its middle behind the wall) is no longer over what holds
    them: the first object that is not loose down the chain of supports (the container a clip was
    dropped into, onto another clip), within its collision box (asset frame, joints as laid out: an
    open drawer's contents are held), or over the table top."""
    found = []
    for name in sorted(layout.loose):
        on = layout.supports[name]
        points = layout.world_points(name, env.object_pose(name)[0])
        foot = np.append(points[points[:, 2] < points[:, 2].min() + .002].mean(0), 1.)
        while on in layout.loose:
            on = layout.supports[on]
        if on == 'table':
            (cx, cy), size = layout.scene.TABLE['center'], np.asarray(layout.scene.TABLE['size'])
            xy, low, high = foot[:2], np.array([cx, cy]) - size / 2, np.array([cx, cy]) + size / 2
        else:
            frame = env.object_pose(on)[0]
            xy = (np.linalg.inv(frame) @ foot)[:2]
            local = layout.world_points(on, frame) @ frame[:3, :3] - frame[:3, :3].T @ frame[:3, 3]
            low, high = local[:, :2].min(0), local[:, :2].max(0)
        if (xy < low).any() or (xy > high).any():
            found.append(f'{name} left {on}')
    return found


def initial_state(env, scene, assets, seed, attempts=ATTEMPTS, initialize=None):
    """Reset ``env`` to a checked, settled initial state of ``scene`` for ``seed``; ``initialize``
    replaces ``scene.initialize`` (e.g. an end state's arrangement).

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
            (initialize or scene.initialize)(layout)
        except SamplingFailed as error:
            entry['problems'] = [str(error)]
            continue
        overlaps = layout.penetrations()
        if overlaps:
            entry['problems'] = [f'{a} penetrates {b} by {depth * 1000:.1f} mm' for (a, b), depth in overlaps.items()]
            continue
        env.reset(layout.poses, object_q=layout.joints)
        if layout.loose:
            entry['landing'] = settle_report(env, layout, LANDING_S)
            entry['problems'] = problems_after_settling(entry['landing'], layout.loose)
            entry['problems'] += [] if layout.free_landing else strayed(env, layout)
            if entry['problems']:
                entry['wall_s'] = time.perf_counter() - started
                continue
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
    """Write ``{seed: (attempt, state)}`` (a state: nested dict of arrays) with file attributes ``attrs``;
    a key may also be the name of an end state."""
    import h5py
    with h5py.File(path, 'w') as file:
        file.attrs.update(attrs)
        for key, (attempt, state) in sorted(states.items(), key=lambda item: str(item[0]).zfill(12)):
            group = file.create_group(str(key))
            group.attrs.update(attempt=attempt, **{'seed' if isinstance(key, int) else 'state': key})
            write_tree(group, state)


def read_states(path):
    """``(attrs, {seed: (attempt, state)})`` of an ``initial_states.hdf5`` file (or ``{name: ...}`` of an
    ``end_states.hdf5`` one), states as nested NumPy dicts."""
    import h5py
    with h5py.File(path, 'r') as file:
        return dict(file.attrs), {int(name) if name.isdigit() else name: (int(group.attrs['attempt']), read_tree(group))
                                  for name, group in sorted(file.items(), key=lambda item: item[0].zfill(12))}


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

