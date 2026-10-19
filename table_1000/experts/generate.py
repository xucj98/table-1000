"""Expert trajectories of a scene: initial state, ``expert.solve(ctx)``, recording and a report per seed.

One :class:`Generator` builds the environment once and runs seeds one after
another (each from a reset). A seed whose initial state cannot be made is
reported and skipped; an expert failure is recorded as a failed episode
(``trajectories_failed.hdf5``) with the subtask and reason, so the success rate
is a metric, not a requirement. Each seed also gets ``episodes/<seed>.json``:
initial-state attempts, subtasks, failure, planning log, robot contacts (the
fingers' on objects, and unexpected ones), the objects' final positions and
joints, and timings.
"""

import json
from pathlib import Path
import time
import traceback

import numpy as np

from table_1000.experts.context import Context, ExpertFailed
from table_1000.scenes import INSTRUCTION, load
from table_1000.scenes.initial import InitializationFailed, initial_state
from table_1000.sim import CONTROL_HZ, PHYSICS_DT


def finger_links(spec):
    return {f'robot.{link}' for arm in spec.arms.values() for link in arm.finger_links}


def contact_summary(intervals, spec):
    """Robot contacts by pair, ``{pair: {'max_penetration_m', 'seconds', 'subtasks'}}``, split into
    ``fingers`` (a finger on something other than the robot: what the expert touches, to be
    checked against what it means to touch) and ``unexpected`` (everything else but the two
    fingers of one gripper)."""
    fingers = finger_links(spec)
    grippers = [{f'robot.{link}' for link in arm.finger_links} for arm in spec.arms.values()]
    summary = {'fingers': {}, 'unexpected': {}}
    for entry in intervals:
        a, b = entry['pair']
        if {a, b} in grippers:
            continue
        on_object = (a in fingers and not b.startswith('robot')) or (b in fingers and not a.startswith('robot'))
        item = summary['fingers' if on_object else 'unexpected'].setdefault(
            ' / '.join(entry['pair']), {'max_penetration_m': 0., 'seconds': 0., 'subtasks': []})
        item['max_penetration_m'] = max(item['max_penetration_m'], entry['max_penetration_m'])
        if entry.get('end') is not None:
            item['seconds'] += entry['end'] - entry['start']
        if entry.get('phase') not in item['subtasks']:
            item['subtasks'].append(entry.get('phase'))
    return summary


class Generator:
    """Runs scene ``scene_name`` (import after ``start_app``) on ``physics`` (``'isaac-cpu'``)."""

    DEVICES = {'isaac-cpu': 'cpu'}

    def __init__(self, scene_name, output, assets, root, physics='isaac-cpu'):
        from table_1000.robots import load_spec
        from table_1000.robots.kinematics import RobotModel
        from table_1000.sim.isaaclab import recording
        from table_1000.sim.isaaclab.env import RobotEnv
        self.recording = recording
        started = time.perf_counter()
        self.name, self.output, self.assets = scene_name, Path(output), Path(assets)
        self.scene, self.expert = load(scene_name, 'scene', root), load(scene_name, 'expert', root)
        self.spec = load_spec(Path(self.scene.ROBOT['asset']).name, root)
        self.env = RobotEnv(self.scene, self.spec, self.assets, root=root, device=self.DEVICES[physics],
                            recorders=recording.recorder_cfg(self.output))
        self.model = RobotModel(self.spec, self.output / 'planning', root)
        (self.output / 'episodes').mkdir(parents=True, exist_ok=True)
        self.env_args = {'scene': scene_name, 'robot': self.spec.name, 'physics': physics, 'instruction': INSTRUCTION,
                         'control_hz': CONTROL_HZ, 'physics_dt': PHYSICS_DT, 'joints': self.env.joints,
                         'action_layout': self.env.action_layout}
        self.load_s = time.perf_counter() - started

    def run(self, seed):
        started = time.perf_counter()
        report = {'scene': self.name, 'seed': seed}
        try:
            layout, report['initial'] = initial_state(self.env, self.scene, self.assets, seed)
        except InitializationFailed as error:
            report.update(status='init_failed', initial={'attempts': error.attempts})
            return self.save(report, started)
        init_s = time.perf_counter() - started
        self.recording.begin(self.env)
        # MPlib's IK restarts and RRTConnect draw from its own generator.
        import mplib.pymp
        mplib.pymp.set_global_seed(seed + 1)   # OMPL rejects 0
        ctx = Context(self.env, self.model, self.scene, self.assets,
                      np.random.default_rng([seed, report['initial']['attempt'], 1]))
        try:
            self.expert.solve(ctx)
            report['status'] = 'success'
        except ExpertFailed as error:
            report.update(status='failed', failure={'subtask': error.subtask, 'reason': error.reason,
                                                    'details': error.details})
        except Exception:   # an error in the expert's code is reported with the episode
            report.update(status='error', failure={'subtask': ctx.current, 'reason': 'error',
                                                   'traceback': traceback.format_exc()})
        finally:
            for solver in ctx.solvers.values():
                solver.world.clear()
        report.update(
            steps=ctx.steps, duration_s=ctx.steps / CONTROL_HZ, subtasks=ctx.subtasks, events=ctx.events,
            plans=[entry for solver in ctx.solvers.values() for entry in solver.log],
            robot_contacts=contact_summary(self.env.contacts.intervals(), self.spec),
            final={'objects': {name: self.env.object_pose(name)[0][:3, 3].tolist() for name in self.scene.OBJECTS},
                   'joints': {name: {joint: float(value[0]) for joint, value in joints.items()}
                              for name, joints in self.env.privileged()['joints'].items()}},
            timings={'init_s': init_s, **{key: sum(s.timings[key] for s in ctx.solvers.values())
                                          for key in ('planning_s', 'executing_s')}})
        attrs = {'scene': self.name, 'seed': seed, 'attempt': report['initial']['attempt'],
                 'instruction': INSTRUCTION, 'subtasks': ctx.subtasks, 'status': report['status']}
        if 'failure' in report:
            attrs['failure'] = {key: report['failure'][key] for key in ('subtask', 'reason')}
        self.recording.finish(self.env, seed, report['status'] == 'success', attrs, self.env_args)
        return self.save(report, started)

    def save(self, report, started):
        report['wall_s'] = time.perf_counter() - started
        (self.output / 'episodes' / f'{report["seed"]}.json').write_text(json.dumps(report, indent=1, default=float))
        return report

    def close(self):
        self.env.env.recorder_manager.close()
        self.env.close()


SUMMARY_FIELDS = ('seed', 'status', 'attempt', 'failed_subtask', 'reason', 'steps', 'duration_s', 'plans',
                  'unexpected_contacts', 'max_unexpected_penetration_mm', 'wall_s')


def summary_row(report):
    contacts = report.get('robot_contacts', {}).get('unexpected', {})
    failure = report.get('failure', {})
    return {'seed': report['seed'], 'status': report['status'], 'attempt': report.get('initial', {}).get('attempt'),
            'failed_subtask': failure.get('subtask'), 'reason': failure.get('reason'),
            'steps': report.get('steps'), 'duration_s': report.get('duration_s'), 'plans': len(report.get('plans', [])),
            'unexpected_contacts': len(contacts),
            'max_unexpected_penetration_mm': 1000 * max((c['max_penetration_m'] for c in contacts.values()), default=0.),
            'wall_s': report['wall_s']}


def summarize(reports):
    """Counts, success rate (of seeds with an initial state), failures by subtask and reason, means."""
    rows = [summary_row(report) for report in reports]
    started = [row for row in rows if row['status'] != 'init_failed']
    succeeded = [row for row in started if row['status'] == 'success']
    failures = {}
    for row in started:
        if row['status'] != 'success':
            key = f'{row["failed_subtask"]}: {row["reason"]}'
            failures[key] = failures.get(key, 0) + 1
    attempts = [report['initial']['attempt'] + 1 for report in reports if report['status'] != 'init_failed']
    return {'seeds': len(rows), 'init_failed': len(rows) - len(started), 'episodes': len(started),
            'succeeded': len(succeeded), 'success_rate': len(succeeded) / len(started) if started else None,
            'mean_init_attempts': float(np.mean(attempts)) if attempts else None,
            'failures': failures,
            'mean_duration_s': float(np.mean([row['duration_s'] for row in succeeded])) if succeeded else None,
            'with_unexpected_contacts': sum(1 for row in started if row['unexpected_contacts']),
            'mean_wall_s': float(np.mean([row['wall_s'] for row in rows])) if rows else None,
            'rows': rows}
