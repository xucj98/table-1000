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
"""

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
