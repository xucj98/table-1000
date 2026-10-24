"""Teleoperating one step of an expert from a case: a seed's state before a step its expert failed.

A **case** is ``(batch, step, seed)`` of a development batch (``runs/expert/<scene>/<run>/``):
the step the seed's episode failed in, started from the checkpoint of the step before
(:func:`cases`). Batches run with ``--repeats K`` give each seed's success rate over its K
runs; the cases come sorted by it, lowest first.

The operator does the step by hand (:class:`HumanExpert`): the expert's other steps stay as
they are, the case's step runs :func:`operate` instead, a loop at the control rate that turns
the operator's input into the same joint targets an expert sends (TCP targets solved by IK,
base speeds, lift and gripper targets). The step's check runs when the operator asks and only
shows its verdict; the operator ends the step as done or not done. A done step is kept as the
step's checkpoint, so later steps can run from it (``run_expert.py --from``).

Input (:class:`Operator`, fed by the page of ``scripts/experts/teleop.py``):

=================  =================================================================
drag on a view     the TCP target in the view's image plane (through the target)
wheel on a view    the TCP target toward / away from the view's camera
Z / X, C / V,      TCP target roll, pitch, yaw (+ / -), about the base's axes
B / N
I / K, J / L,      TCP target along the base's x, y, z (+ / -)
U / O
W / S, A / D       base forward / back, turn left / right
R / F              torso lift up / down
arrows             head yaw / pitch
Tab                switch the controlled arm
Space              open / close its gripper
P                  pause / resume the simulation
Shift              faster
=================  =================================================================
"""

import json
from pathlib import Path
import queue
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.experts.context import ExpertFailed, Step, run_steps
from table_1000.robots.teleop import FAST, Keys

LINEAR, ROTATION = .10, .6       # TCP target m/s, rad/s from keys
BASE_SPEED, BASE_TURN = .3, .6   # m/s, rad/s
LIFT_SPEED, HEAD_SPEED = .1, .6  # m/s, rad/s
WHEEL = .01                      # m per wheel notch
JOINT_SPEED = 1.5                # rad/s an arm joint target moves at most
GRIPPER_TIME = .6                # s for the gripper to open or close fully
APERTURE = 20.955                # mm, a view camera's horizontal aperture (see sim.isaaclab.cameras)


# -- cases ---------------------------------------------------------------------------------
def failed_step(report):
    """The step ``report``'s episode failed in (the first of its steps it did not complete), None
    for a success or an episode of an expert without steps."""
    if report.get('status') == 'success' or not report.get('step_names'):
        return None
    first, last = report.get('steps_run') or (1, len(report['step_names']))
    done = {entry['step'] for entry in report.get('steps') or []}
    return next((step for step in range(first, last + 1) if step not in done), None)


def start_checkpoint(batch, summary, step):
    """The checkpoint file of step ``step - 1`` a case of ``batch`` starts from: the batch's own, or,
    for the batch's first step, that of the batch it ran from (``from``)."""
    own = Path(batch) / 'checkpoints' / f'step{step - 1}.hdf5'
    first = (summary.get('steps_run') or [1])[0]
    if step == first and summary.get('from'):
        return Path(summary['from']) / 'checkpoints' / f'step{step - 1}.hdf5'
    return own


def batch_cases(batch):
    """The cases of one batch: ``{'batch', 'run', 'scene', 'step', 'step_name', 'seed', 'successes',
    'runs', 'rate', 'failure', 'start'}`` per seed that failed in any of its runs (``rate`` None
    without ``--repeats``)."""
    batch = Path(batch)
    summary = json.loads((batch / 'summary.json').read_text())
    repeats = summary.get('repeats') or {}
    reports = {}
    for path in sorted((batch / 'episodes').glob('*.json')):
        seed, _, repeat = path.stem.partition('-')
        reports.setdefault(int(seed), []).append((int(repeat or 0), json.loads(path.read_text())))
    cases = []
    for seed, runs in sorted(reports.items()):
        failed = [report for _, report in sorted(runs, key=lambda run: run[0]) if failed_step(report)]
        if not failed:
            continue
        report = failed[0]
        step = failed_step(report)
        successes = repeats.get('succeeded', {}).get(str(seed)) if repeats else None
        count = repeats.get('count') if repeats else None
        cases.append({'batch': str(batch.resolve()), 'run': summary.get('run') or batch.name,
                      'scene': summary.get('scene'), 'step': step, 'step_name': report['step_names'][step - 1],
                      'seed': seed, 'successes': successes, 'runs': count,
                      'rate': successes / count if count else None,
                      'failure': (report.get('failure') or {}).get('reason'),
                      'start': str(start_checkpoint(batch, summary, step))})
    return cases


def cases(runs, scene=None):
    """The cases of every batch under ``runs`` (``runs/expert``), of ``scene`` only when given:
    lowest success rate first, cases without a rate last."""
    found = []
    for summary in sorted(Path(runs).glob(f'{scene or "*"}/*/summary.json')):
        found += batch_cases(summary.parent)
    return sorted(found, key=lambda case: (case['rate'] is None, case['rate'] or 0., case['run'], case['seed']))


# -- views -----------------------------------------------------------------------------------
def view_camera(view):
    """``(cam2world rotation, position, fx in pixels)`` of a fixed view, camera axes x right, y down,
    z forward."""
    position = np.asarray(view['position'], dtype=float)
    forward = np.asarray(view['target'], dtype=float) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray(view.get('up', (0., 0., 1.)), dtype=float))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.c_[right, down, forward], position, view['focal_length_mm'] / APERTURE * view['resolution'][0]


def drag(camera, point, dx, dy, wheel=0.):
    """World displacement of ``point`` dragged ``(dx, dy)`` pixels in a camera's image (``camera``
    as :func:`view_camera`), and moved ``wheel`` notches toward the camera."""
    rotation, position, fx = camera
    depth = max(float((np.asarray(point) - position) @ rotation[:, 2]), .05)
    return rotation @ np.array([dx * depth / fx, dy * depth / fx, -wheel * WHEEL])


# -- operating ---------------------------------------------------------------------------------
class Operator:
    """The operator's input, shared between the page's thread and the simulation loop: held keys,
    drags and commands (``check``, ``done``, ``not done``, ``load`` of a case)."""

    def __init__(self):
        self.keys = Keys()
        self.drags, self.commands = queue.Queue(), queue.Queue()
        self.lock = threading.Lock()
        self.status = {}

    def drag(self, view, dx, dy, wheel=0.):
        self.drags.put((view, float(dx), float(dy), float(wheel)))

    def command(self, name, **values):
        self.commands.put({'command': name, **values})

    @staticmethod
    def drain(source):
        while not source.empty():
            yield source.get()


def check(ctx, step):
    """``{'passed', 'reason'}`` of ``step``'s check (a :class:`Step`) on the current state."""
    if step.check is None:
        return {'passed': True, 'reason': None}
    current = ctx.current
    ctx.current = step.name
    try:
        step.check(ctx)
        return {'passed': True, 'reason': None}
    except ExpertFailed as error:
        return {'passed': False, 'reason': error.reason}
    finally:
        ctx.current = current


def operate(ctx, step, operator, show, cameras, log):
    """Do ``step`` by the operator's input until they end it, at the control rate in real time.
    ``show(ctx, status, targets)`` renders and publishes now and then; ``cameras`` are the views a
    drag may come from (:func:`view_camera`, or a callable giving one for a robot camera); ``log``
    (a dict) receives the checks asked for, the operator's verdict (``done``, ``not done``, or
    ``left`` for another command, kept as ``next``) and the check when it ended. A step not ended
    as done fails."""
    from table_1000.experts.solver import OPEN_GAP
    from table_1000.sim import CONTROL_HZ
    dt, spec, model = 1 / CONTROL_HZ, ctx.spec, ctx.model
    arms = list(spec.arms)
    arm = spec.primary_arm
    base = ctx.base_transform()
    targets = {name: np.linalg.inv(base) @ ctx.tcp(name) for name in arms}       # in the base frame
    commands = {name: ctx.env.qpos(spec.arms[name].joints)[0].copy() for name in arms}
    gaps = {name: spec.arms[name].gap(ctx.env.qpos(spec.arms[name].fingers)[0]) for name in arms}
    wanted = dict(gaps)
    lift = ctx.env.qpos(spec.lift)[0].copy() if spec.lift else None
    head = ctx.env.qpos(spec.head)[0].copy() if spec.head else None
    lift_limits = model.limits[model.lift_indices()] if spec.lift else None
    paused, reached, last_check = False, {name: True for name in arms}, None
    log.setdefault('checks', [])
    keys = operator.keys
    list(keys.presses()), list(Operator.drain(operator.drags))      # input from before the step
    with ctx.subtask(step.name):
        while True:
            started = time.perf_counter()
            base = ctx.base_transform()
            for command in Operator.drain(operator.commands):
                if command['command'] == 'check':
                    last_check = {**check(ctx, step), 'at': ctx.steps}
                    log['checks'].append(last_check)
                elif command['command'] in ('done', 'not done'):
                    log['verdict'] = command['command']
                    log['final_check'] = {**check(ctx, step), 'at': ctx.steps}
                    if command['command'] == 'not done':
                        ctx.fail('the operator ended the step as not done')
                    return
                else:       # another case: this one ends as not done, the command waits in the log
                    log.update(verdict='left', final_check={**check(ctx, step), 'at': ctx.steps}, next=command)
                    ctx.fail('the operator left the step')
            for key in keys.presses():
                if key == 'Tab':
                    arm = arms[(arms.index(arm) + 1) % len(arms)]
                elif key == 'Space':
                    wanted[arm] = OPEN_GAP if wanted[arm] < OPEN_GAP / 2 else 0.
                elif key == 'KeyP':
                    paused = not paused
            k = keys.fast()
            target = targets[arm]
            for view, dx, dy, wheel in Operator.drain(operator.drags):
                camera = cameras.get(view)
                camera = camera(ctx) if callable(camera) else camera
                if camera is not None:
                    point = base @ target
                    target[:3, 3] += (np.linalg.inv(base)[:3, :3] @ drag(camera, point[:3, 3], dx, dy, wheel))
            move = np.array([keys.axis('KeyI', 'KeyK'), keys.axis('KeyJ', 'KeyL'), keys.axis('KeyU', 'KeyO')])
            turn = np.array([keys.axis('KeyZ', 'KeyX'), keys.axis('KeyC', 'KeyV'), keys.axis('KeyB', 'KeyN')])
            target[:3, 3] += move * LINEAR * k * dt
            if turn.any():
                target[:3, :3] = Rotation.from_rotvec(turn * ROTATION * k * dt).as_matrix() @ target[:3, :3]
            if paused:
                show(ctx, status(ctx, arm, paused, reached, last_check), targets, arm)
                time.sleep(dt)
                continue
            action = {}
            for name in arms:
                status_, q = model.ik(name, base @ targets[name], world=True, attempts=1, check_collision=False)
                reached[name] = q is not None
                if q is not None:
                    goal = np.asarray(q)[model.arm_indices(name)]
                    commands[name] += np.clip(goal - commands[name], -JOINT_SPEED * dt, JOINT_SPEED * dt)
                action[name] = commands[name][None]
                gaps[name] += np.clip(wanted[name] - gaps[name], -OPEN_GAP / GRIPPER_TIME * dt, OPEN_GAP / GRIPPER_TIME * dt)
                action[f'{name}_gripper'] = [[gaps[name]]]
            if spec.base:
                action['base'] = [[BASE_SPEED * k * keys.axis('KeyW', 'KeyS'), BASE_TURN * k * keys.axis('KeyA', 'KeyD')]]
            if lift is not None:
                lift = np.clip(lift + LIFT_SPEED * k * keys.axis('KeyR', 'KeyF') * dt, lift_limits[:, 0], lift_limits[:, 1])
                action['lift'] = lift[None]
            if head is not None:
                head = head + HEAD_SPEED * k * dt * np.array([keys.axis('ArrowLeft', 'ArrowRight'),
                                                              keys.axis('ArrowDown', 'ArrowUp')])[:len(head)]
                action['head'] = head[None]
            ctx.step(action)
            show(ctx, status(ctx, arm, paused, reached, last_check), targets, arm)
            spare = dt - (time.perf_counter() - started)
            if spare > 0:
                time.sleep(spare)


def status(ctx, arm, paused, reached, last_check):
    """The status the page shows."""
    from table_1000.sim import CONTROL_HZ
    tcp = ctx.tcp(arm)[:3, 3]
    return {'arm': arm, 'paused': paused, 'time_s': ctx.steps / CONTROL_HZ,
            'steps': ctx.steps, 'tcp': [round(float(v), 3) for v in tcp],
            'unreachable': [name for name, ok in reached.items() if not ok], 'check': last_check}


class HumanExpert:
    """The expert ``expert`` (a module with ``STEPS``) with its step ``step`` done by
    :func:`operate` (``operate(ctx, step)``), unchecked: the operator decides when it is done."""

    def __init__(self, expert, step, operate_step):
        self.STEPS = [Step(entry.name, (lambda ctx, entry=entry: operate_step(ctx, entry)), None) if index == step
                      else entry for index, entry in enumerate(expert.STEPS, 1)]

    def solve(self, ctx):
        run_steps(ctx, self.STEPS)
