"""Expert trajectories of a scene: ``expert.solve(ctx)`` from a stored initial state, recording and a report per seed.

With ``stage``, one step of the expert is checked alone instead: ``expert.STAGES[stage](ctx)``
arranges the scene where that step starts (:meth:`~table_1000.experts.context.Context.arrange`)
and runs it.

One :class:`Generator` builds the environment once and runs seeds one after
another, each from its initial state of ``initial_states.hdf5``
(:mod:`table_1000.scenes.initial`). An expert failure is recorded as a failed
episode (``trajectories_failed.hdf5``) with the subtask and reason, so the
success rate is a metric, not a requirement. Each seed also gets
``episodes/<seed>.json``: subtasks, failure, planning log, the parts the fingers
touched, the objects' final positions and joints, and timings.
"""

import json
from pathlib import Path
import time
import traceback

import numpy as np

from table_1000.experts.context import Context, ExpertFailed
from table_1000.scenes import INSTRUCTION, load
from table_1000.sim import CONTROL_HZ, PHYSICS_DT

# A seed without a report: its process stopped before finishing it.
PROCESS_EXITED = '-: process exited'


def finger_contacts(intervals, spec):
    """What the fingers touched outside the robot, ``{pair: {'max_penetration_m', 'seconds', 'subtasks'}}``."""
    fingers = {f'robot.{link}' for arm in spec.arms.values() for link in arm.finger_links}
    summary = {}
    for entry in intervals:
        a, b = entry['pair']
        if not ((a in fingers and not b.startswith('robot')) or (b in fingers and not a.startswith('robot'))):
            continue
        item = summary.setdefault(' / '.join(entry['pair']), {'max_penetration_m': 0., 'seconds': 0., 'subtasks': []})
        item['max_penetration_m'] = max(item['max_penetration_m'], entry['max_penetration_m'])
        if entry.get('end') is not None:
            item['seconds'] += entry['end'] - entry['start']
        if entry.get('phase') not in item['subtasks']:
            item['subtasks'].append(entry.get('phase'))
    return summary


class Generator:
    """Runs scene ``scene_name`` (import after ``start_app``) on ``physics`` (``'isaac-cpu'``)."""

    DEVICES = {'isaac-cpu': 'cpu'}

    def __init__(self, scene_name, output, assets, root, physics='isaac-cpu', stage=None):
        from table_1000.robots import load_spec
        from table_1000.robots.kinematics import RobotModel
        from table_1000.sim.isaaclab import recording
        from table_1000.sim.isaaclab.env import RobotEnv
        self.recording = recording
        started = time.perf_counter()
        self.name, self.output, self.assets, self.stage = scene_name, Path(output), Path(assets), stage
        self.scene, self.expert = load(scene_name, 'scene', root), load(scene_name, 'expert', root)
        if stage is not None and stage not in getattr(self.expert, 'STAGES', {}):
            raise KeyError(f'scene {scene_name} has no stage {stage!r}')
        self.spec = load_spec(Path(self.scene.ROBOT['asset']).name, root)
        self.env = RobotEnv(self.scene, self.spec, self.assets, root=root, device=self.DEVICES[physics],
                            recorders=recording.recorder_cfg(self.output))
        self.model = RobotModel(self.spec, self.output / 'planning', root)
        (self.output / 'episodes').mkdir(parents=True, exist_ok=True)
        self.env_args = {'scene': scene_name, 'robot': self.spec.name, 'physics': physics, 'instruction': INSTRUCTION,
                         'control_hz': CONTROL_HZ, 'physics_dt': PHYSICS_DT, 'joints': self.env.joints,
                         'action_layout': self.env.action_layout}
        self.load_s = time.perf_counter() - started

    def run(self, seed, attempt, state):
        """Episode ``seed`` from its initial ``state`` (made by sub-seed ``attempt``)."""
        started = time.perf_counter()
        report = {'scene': self.name, 'seed': seed, 'attempt': attempt, 'stage': self.stage}
        self.env.reset_to_state(state)
        init_s = time.perf_counter() - started
        self.recording.begin(self.env)
        # MPlib's IK restarts and RRTConnect draw from its own generator.
        import mplib.pymp
        mplib.pymp.set_global_seed(seed + 1)   # OMPL rejects 0
        ctx = Context(self.env, self.model, self.scene, self.assets,
                      np.random.default_rng([seed, attempt, 1]))
        try:
            (self.expert.STAGES[self.stage] if self.stage else self.expert.solve)(ctx)
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
            finger_contacts=finger_contacts(self.env.contacts.intervals(), self.spec),
            final={'objects': {name: self.env.object_pose(name)[0][:3, 3].tolist() for name in self.scene.OBJECTS},
                   'joints': {name: {joint: float(value[0]) for joint, value in joints.items()}
                              for name, joints in self.env.privileged()['joints'].items()}},
            timings={'init_s': init_s, **{key: sum(s.timings[key] for s in ctx.solvers.values())
                                          for key in ('planning_s', 'executing_s')}})
        attrs = {'scene': self.name, 'seed': seed, 'attempt': attempt,
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


def failure_key(report):
    failure = report.get('failure', {})
    return f'{failure.get("subtask")}: {failure.get("reason")}'


def summarize(reports, seeds):
    """A batch over ``seeds`` from its episode reports: success rate of all the seeds, the failed
    seeds by ``"subtask: reason"`` (seeds without a report under :data:`PROCESS_EXITED`), means."""
    from table_1000.common.batch import seeds_record
    reports = {report['seed']: report for report in reports}
    succeeded = [report for report in reports.values() if report['status'] == 'success']
    failures = {}
    for seed in seeds:
        report = reports.get(seed)
        if report is None or report['status'] != 'success':
            failures.setdefault(PROCESS_EXITED if report is None else failure_key(report), []).append(seed)
    return {'seeds': seeds_record(seeds), 'succeeded': len(succeeded),
            'success_rate': len(succeeded) / len(seeds) if seeds else None,
            'failures': dict(sorted(failures.items(), key=lambda item: (-len(item[1]), item[0]))),
            'mean_duration_s': float(np.mean([report['duration_s'] for report in succeeded])) if succeeded else None,
            'mean_wall_s': float(np.mean([report['wall_s'] for report in reports.values()])) if reports else None}
