"""Expert trajectories of a scene: ``expert.solve(ctx)`` from a stored initial state, recording and a report per seed.

An expert made of steps (:func:`~table_1000.experts.context.run_steps`) leaves a
checkpoint after each step that passed its check: the scene state, in
``checkpoints/<seed>.hdf5`` (merged by step into ``checkpoints/step<k>.hdf5``,
:func:`merge_checkpoints`). With ``steps`` ``(first, last)`` only those steps run,
from the initial states (``first`` 1) or from the checkpoints of step ``first - 1``;
the robot's home stays where the seed's episode began.

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

from table_1000.experts.context import Context, ExpertFailed, Unreachable
from table_1000.scenes import INSTRUCTION, load
from table_1000.scenes.initial import read_tree, write_tree
from table_1000.sim import CONTROL_HZ, PHYSICS_DT

# A seed without a report: its process stopped before finishing it.
PROCESS_EXITED = '-: process exited'
RESUME_S = .2       # s of holding still after resuming from a checkpoint


def checkpoint_path(directory, step):
    """The checkpoints of step ``step`` in a batch directory."""
    return Path(directory) / 'checkpoints' / f'step{step}.hdf5'


def write_checkpoints(path, seed, attempt, start, checkpoints):
    """One seed's checkpoints (:attr:`Context.checkpoints`) and the robot where its episode began."""
    import h5py
    with h5py.File(path, 'w') as file:
        file.attrs.update(seed=seed, attempt=attempt)
        write_tree(file.create_group('start'), start)
        for entry in checkpoints:
            group = file.create_group(str(entry['step']))
            group.attrs.update(name=entry['name'], at=entry['at'])
            write_tree(group.create_group('state'), entry['state'])


def merge_checkpoints(sources, directory, attrs):
    """Gather the seeds' checkpoint files ``sources`` by step into ``checkpoint_path(directory, k)``:
    one group per seed (attributes ``seed``, ``attempt``, ``at``; ``state`` and ``start``), file
    attributes ``attrs`` and the step's ``step`` and ``name``. Returns the steps written."""
    import h5py
    files = [h5py.File(path, 'r') for path in sorted(sources)]
    try:
        steps = sorted({int(key) for file in files for key in file if key != 'start'})
        for step in steps:
            path = checkpoint_path(directory, step)
            path.parent.mkdir(parents=True, exist_ok=True)
            with h5py.File(path, 'w') as out:
                out.attrs.update(attrs, step=step)
                for file in sorted(files, key=lambda file: int(file.attrs['seed'])):
                    if str(step) not in file:
                        continue
                    out.attrs['name'] = file[str(step)].attrs['name']
                    group = out.create_group(str(int(file.attrs['seed'])))
                    group.attrs.update(seed=file.attrs['seed'], attempt=file.attrs['attempt'],
                                       at=file[str(step)].attrs['at'])
                    file.copy(file[str(step)]['state'], group, 'state')
                    file.copy(file['start'], group, 'start')
        return steps
    finally:
        for file in files:
            file.close()


def read_checkpoints(path):
    """``(attrs, {seed: (attempt, state, start)})`` of a step's checkpoint file."""
    import h5py
    with h5py.File(path, 'r') as file:
        return dict(file.attrs), {int(name): (int(group.attrs['attempt']), read_tree(group['state']),
                                              read_tree(group['start']))
                                  for name, group in sorted(file.items(), key=lambda item: int(item[0]))}


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

    def __init__(self, scene_name, output, assets, root, physics='isaac-cpu', steps=None, source=None):
        """``steps``: ``(first, last)`` of the expert's steps to run (all when None); ``source``: where
        the states come from (a checkpoint file), for the reports."""
        from table_1000.robots import load_spec
        from table_1000.robots.kinematics import RobotModel
        from table_1000.sim.isaaclab import recording
        from table_1000.sim.isaaclab.env import RobotEnv
        self.recording = recording
        started = time.perf_counter()
        self.name, self.output, self.assets = scene_name, Path(output), Path(assets)
        self.steps, self.source = steps, source
        self.scene, self.expert = load(scene_name, 'scene', root), load(scene_name, 'expert', root)
        if steps and not 1 <= steps[0] <= steps[1] <= len(getattr(self.expert, 'STEPS', ())):
            raise ValueError(f'steps {steps[0]}-{steps[1]}: the expert of {scene_name} has '
                             f'{len(getattr(self.expert, "STEPS", ()))} STEPS')
        self.spec = load_spec(Path(self.scene.ROBOT['asset']).name, root)
        self.env = RobotEnv(self.scene, self.spec, self.assets, root=root, device=self.DEVICES[physics],
                            recorders=recording.recorder_cfg(self.output))
        self.model = RobotModel(self.spec, self.output / 'planning', root)
        for name in ('episodes', 'checkpoints'):
            (self.output / name).mkdir(parents=True, exist_ok=True)
        self.env_args = {'scene': scene_name, 'robot': self.spec.name, 'physics': physics, 'instruction': INSTRUCTION,
                         'control_hz': CONTROL_HZ, 'physics_dt': PHYSICS_DT, 'joints': self.env.joints,
                         'action_layout': self.env.action_layout}
        self.load_s = time.perf_counter() - started

    def run(self, seed, attempt, state, start=None):
        """Episode ``seed`` from its initial ``state`` (made by sub-seed ``attempt``), or from a
        checkpoint's with ``start`` the robot where the seed's episode began; its report has the
        machine's load meanwhile (``load``, see :class:`~table_1000.common.batch.Load`) and the CPU
        seconds the process used (``cpu_s``)."""
        from table_1000.common.batch import Load
        with Load() as load:
            report, started = self.episode(seed, attempt, state, start)
        report.update(load=load.record(), cpu_s=sum(sample['cpu_s'] for sample in load.samples))
        return self.save(report, started)

    def episode(self, seed, attempt, state, start):
        started = time.perf_counter()
        first = self.steps[0] if self.steps else 1
        report = {'scene': self.name, 'seed': seed, 'attempt': attempt, 'steps_run': self.steps, 'from': self.source}
        self.env.reset_to_state(state)
        init_s = time.perf_counter() - started
        self.recording.begin(self.env)
        # MPlib's IK restarts and RRTConnect draw from its own generator; a later step from its own.
        import mplib.pymp
        mplib.pymp.set_global_seed(seed + 1 + 1_000_003 * (first - 1))   # OMPL rejects 0
        ctx = Context(self.env, self.model, self.scene, self.assets,
                      np.random.default_rng([seed, attempt, 1] + ([first] if first > 1 else [])), start=start,
                      only=self.steps)
        try:
            if start is not None:
                ctx.hold(RESUME_S)
            self.expert.solve(ctx)
            report['status'] = 'success'
        except ExpertFailed as error:
            report.update(status='failed', failure={'subtask': error.subtask, 'reason': error.reason,
                                                    'unreachable': isinstance(error, Unreachable),
                                                    'details': error.details})
        except Exception:   # an error in the expert's code is reported with the episode
            report.update(status='error', failure={'subtask': ctx.current, 'reason': 'error',
                                                   'traceback': traceback.format_exc()})
        finally:
            for solver in ctx.solvers.values():
                solver.world.clear()
        report.update(
            step_names=ctx.step_names, steps=[{key: entry[key] for key in ('step', 'name', 'at', 'wall_s', 'cpu_s')}
                                              for entry in ctx.checkpoints],
            control_steps=ctx.steps, duration_s=ctx.steps / CONTROL_HZ, subtasks=ctx.subtasks, events=ctx.events,
            expert_wall_s=time.perf_counter() - ctx.started, subtask_wall_s=ctx.subtask_wall_s,
            plans=[entry for solver in ctx.solvers.values() for entry in solver.log],
            finger_contacts=finger_contacts(self.env.contacts.intervals(), self.spec),
            final={'objects': {name: self.env.object_pose(name)[0][:3, 3].tolist() for name in self.scene.OBJECTS},
                   'joints': {name: {joint: float(value[0]) for joint, value in joints.items()}
                              for name, joints in self.env.privileged()['joints'].items()}},
            timings={'init_s': init_s, **{key: sum(s.timings[key] for s in ctx.solvers.values())
                                          for key in ('planning_s', 'executing_s')}})
        if ctx.checkpoints:
            write_checkpoints(self.output / 'checkpoints' / f'{seed}.hdf5', seed, attempt, ctx.start, ctx.checkpoints)
        attrs = {'scene': self.name, 'seed': seed, 'attempt': attempt,
                 'instruction': INSTRUCTION, 'subtasks': ctx.subtasks, 'status': report['status'],
                 'steps': [{key: entry[key] for key in ('step', 'name', 'at')} for entry in report['steps']]}
        if self.source:
            attrs['from'] = self.source
        if 'failure' in report:
            attrs['failure'] = {key: report['failure'][key] for key in ('subtask', 'reason')}
        self.recording.finish(self.env, seed, report['status'] == 'success', attrs, self.env_args)
        return report, started

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


def step_rates(reports, seeds, parent=None):
    """Per step of an expert made of steps: ``{'step', 'name', 'started', 'completed', 'success_rate',
    'progressive_rate'}``. A step starts in the seeds that completed the one before (all the seeds
    for the first run here); the progressive rate counts the seeds that completed it and every
    step before, out of the seeds the first step started in: those of the ``parent`` summary (the
    batch whose checkpoints this one resumed from), whose rows come first. ``None`` for an expert
    without steps."""
    reports = [report for report in reports if report.get('step_names')]
    if not reports:
        return None
    names = reports[0]['step_names']
    first, last = reports[0].get('steps_run') or (1, len(names))
    base = parent['steps']['seeds'] if parent and parent.get('steps') else len(seeds)
    rows = [row for row in (parent or {}).get('steps', {}).get('rows', []) if row['step'] < first]
    done = {report['seed']: {entry['step'] for entry in report['steps']} for report in reports}
    started = len(seeds)
    for step in range(first, last + 1):
        completed = sum(step in steps for steps in done.values())
        rows.append({'step': step, 'name': names[step - 1], 'started': started, 'completed': completed,
                     'success_rate': completed / started if started else None,
                     'progressive_rate': completed / base if base else None})
        started = completed
    return {'seeds': base, 'rows': rows}


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
    unreachable = sorted(seed for seed, report in reports.items() if report.get('failure', {}).get('unreachable'))
    return {'seeds': seeds_record(seeds), 'succeeded': len(succeeded),
            'success_rate': len(succeeded) / len(seeds) if seeds else None,
            # Nothing in reach: a high share points at the planning (parameters, collisions) or at the
            # state the step before left.
            'unreachable': unreachable, 'unreachable_rate': len(unreachable) / len(seeds) if seeds else None,
            'failures': dict(sorted(failures.items(), key=lambda item: (-len(item[1]), item[0]))),
            'mean_duration_s': float(np.mean([report['duration_s'] for report in succeeded])) if succeeded else None,
            'mean_wall_s': float(np.mean([report['wall_s'] for report in reports.values()])) if reports else None}
