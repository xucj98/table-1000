"""Supervised single run of the drawer-and-pen task (see scripts/tasks/drawer_pen.py)."""

import csv
import json
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

from table_1000.runtime.process import mark_shutdown, supervise, write_json

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'assets'
SCENE = 'scene-000000'
ROBOT_SOURCE = ROOT / 'asset_sources' / 'robots' / 'x1pro' / 'robot.urdf'
VIEWS = {
    'wide': {'position': (1.45, -1.25, 1.6), 'target': (.05, -.25, .85), 'focal_length_mm': 22,
             'resolution': (960, 540)},
    'close': {'position': (.78, -.62, 1.25), 'target': (.17, -.24, .82), 'focal_length_mm': 26,
              'resolution': (960, 540)},
}
HELD_AFTER_LIFT = .2
FINGER_LINKS = ('robot.right_arm_gripper_left_link', 'robot.right_arm_gripper_right_link')


def views():
    """Fixed wide and close views plus the robot's head camera."""
    from table_1000.robots.x1pro import camera_mount
    from table_1000.scenes.stage import ROBOT_PATH
    link, local = camera_mount(ROBOT_SOURCE)
    return {**VIEWS, 'head': {'parent': f'{ROBOT_PATH}/{link}', 'local': local, 'focal_length_mm': 15,
                              'resolution': (960, 540)}}


def classify(intervals, handle, pen_prefix, held):
    """Split contacts into expected and unexpected ones.

    Robot links may only touch each other's fingers, the handle and the pen
    (with the fingers) and the ground (with the wheels). While the pen is held,
    ``held = (start, end)`` in seconds, it may touch only the fingers and itself.
    """
    expected, unexpected = [], []
    for entry in intervals:
        a, b = entry['pair']
        if not (a.startswith('robot') or b.startswith('robot')):
            end = float('inf') if entry.get('end') is None else entry['end']
            during = held is not None and entry['start'] < held[1] and end > held[0]
            internal = a.startswith(pen_prefix) and b.startswith(pen_prefix)
            (unexpected if during and not internal else expected).append(entry)
            continue
        robot, other = (a, b) if a.startswith('robot') else (b, a)
        if other.startswith('robot'):
            fingers = {a, b} == set(FINGER_LINKS)
            (expected if fingers else unexpected).append(entry)
            continue
        if robot in FINGER_LINKS and (other == handle or other.startswith(pen_prefix)):
            expected.append(entry)
        elif other == 'ground' and robot.endswith('_wheel_link'):
            expected.append(entry)
        else:
            unexpected.append(entry)
    return expected, unexpected


def case_list(args):
    """``[(directory, seed, offsets)]``: one case into ``--output``, or a ``--cases`` file
    (``[{"name", "seed", "offsets", "directory"}]``), each into its ``directory``
    (default ``<--output>/../<name>``)."""
    output = args.output.resolve()
    if not args.cases:
        return [(output, args.seed, args.offsets)]
    return [(Path(case.get('directory') or output.parent / case['name']), case.get('seed'), case.get('offsets'))
            for case in json.loads(Path(args.cases).read_text())]


class Runner:
    """Isaac app, stage, robot model and Env built once; :meth:`run` is one episode."""

    def __init__(self, args, output):
        started = time.perf_counter()
        from table_1000.scenes import load
        from table_1000.runtime.cameras import Cameras
        from table_1000.robots.controllers import CONTROL_HZ
        from table_1000.robots.kinematics import RobotModel
        from table_1000.robots.x1pro import SPEC
        from table_1000.sim.env import Env, EnvView
        from table_1000.sim.isaac import IsaacBackend
        self.output = output
        self.timings = {'imports_s': time.perf_counter() - started}
        self.scene = load(SCENE)
        layout = self.scene.placement()
        cameras = (lambda stage: Cameras(stage, views(), output / 'frames', fps=CONTROL_HZ,
                                         subframes=args.subframes)) if args.video else None
        # Reports come from the robot and the cabinet; reporting on the pen
        # itself would also deliver its cap detent's contacts on every step.
        self.backend = IsaacBackend(self.scene, layout, ASSETS, cameras=cameras, contact_objects=('cabinet',),
                                    contact_watch=('robot', 'pen:'))
        self.timings['stage_s'] = self.backend.timings['load_s']
        started = time.perf_counter()
        self.model = RobotModel(SPEC, output / 'planning')
        self.timings['planning_model_s'] = time.perf_counter() - started
        self.env = Env(self.scene, self.backend, SPEC, [self.model])
        self.view = EnvView(self.env)
        self.heartbeat = time.monotonic()

    def run(self, directory, seed, offsets):
        from table_1000.robots.x1pro import ARM
        from table_1000.tasks.motion import PlanningFailed, run_expert
        from table_1000.tasks.drawer_pen import Solver, TaskFailed, evaluate, DRAWER_JOINT
        from table_1000.tasks import drawer_pen_geometry as g
        env, view, backend = self.env, self.view, self.backend
        started = time.perf_counter()
        before = dict(backend.timings)
        result = {'status': 'FAIL', 'seed': seed, 'offsets_argument': offsets}
        if offsets:
            offsets = {'cabinet': tuple(offsets[:2]), 'pen': tuple(offsets[2:])}
        layout = self.scene.placement(seed=seed, offsets=offsets)
        result['layout'] = {'offsets': layout['offsets'], 'problems': self.scene.check(layout)}
        env.reset(layout)
        initial = view.body_poses()
        result['initial_poses'] = {key: initial[key].tolist() for key in ('cabinet:cabinet.housing', 'pen:body')}
        solver = Solver(view, ASSETS)
        motion = solver.motion
        env.caption = lambda env: f't={env.time:5.2f}s  {backend.contacts.phase}'
        rows = []

        def record(env):
            pen = view.body_poses()['pen:body']
            q = view.qpos(ARM)
            rows.append({'time': round(env.time, 4), 'stage': solver.stage,
                         **{f'q{i + 1}': float(v) for i, v in enumerate(q)},
                         **{f'q{i + 1}_target': float(v) for i, v in enumerate(view.targets(ARM))},
                         'finger_gap': motion.finger_gap(),
                         'drawer_opening': view.object_joint('cabinet', DRAWER_JOINT),
                         'pen_x': pen[0, 3], 'pen_y': pen[1, 3], 'pen_z': pen[2, 3], 'pen_axis_z': pen[2, 0]})
            if time.monotonic() - self.heartbeat > 2:
                self.heartbeat = time.monotonic()
                write_json(self.output / 'heartbeat.json', {'case': directory.name, 'time': env.time,
                                                            'stage': solver.stage, 'epoch': time.time()})

        env.observers[:] = [record]
        try:
            [outcome] = run_expert(env, [solver.solve()])
            if isinstance(outcome, (TaskFailed, PlanningFailed)):
                result['failure'] = {'stage': outcome.stage, 'reason': outcome.reason,
                                     'details': json.loads(json.dumps(outcome.details, default=float))}
            elif outcome is not None:
                raise outcome
            else:
                result['failure'] = None
            run_expert(env, [motion.hold(.5)])
            evaluation = evaluate(solver)
            starts = {event['stage']: event['time'] for event in solver.events}
            held = None
            if 'pick: lift pen' in starts:
                # The pen leaves the table within the first fraction of a second of the lift.
                held = (starts['pick: lift pen'] + HELD_AFTER_LIFT, starts.get('place: release pen', env.time))
            expected, unexpected = classify(backend.contacts.intervals(), 'cabinet:' + g.HANDLE, 'pen:', held)
            evaluation['unexpected_contacts'] = len(unexpected)
            success = (result['failure'] is None and evaluation['drawer_closed'] and evaluation['pen_in_drawer']
                       and evaluation['hand_withdrawn'] and not unexpected)
            result.update(status='PASS' if success else 'FAIL', evaluation=evaluation, regrasps=solver.regrasps,
                          stages=solver.events, plans=motion.log, max_tracking_error_rad=motion.max_tracking_error,
                          simulated_s=env.time)
            write_json(directory / 'contacts.json', {'expected': expected, 'unexpected': unexpected})
            if rows:
                with (directory / 'trace.csv').open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
            timings = {key: backend.timings[key] - before.get(key, 0.) for key in ('step_s', 'render_s')}
            timings.update(motion.timings)
            if backend.cameras:
                encode = time.perf_counter()
                result['videos'] = backend.cameras.encode(directory)
                timings['encode_s'] = time.perf_counter() - encode
        except Exception:
            result['error'] = traceback.format_exc()
            timings = {}
        finally:
            motion.close()
        timings['case_s'] = time.perf_counter() - started
        result['timings'] = timings
        write_json(directory / 'result.json', json.loads(json.dumps(result, default=float)))
        return result


def child(args):
    output = args.output.resolve()
    run_started = time.perf_counter()
    from table_1000.runtime.app import start_app
    from table_1000.runtime.render import RT2_ARGS
    # Only video runs need RTX Real-Time 2.0 (see table_1000.runtime.render).
    app = start_app(output, gpu=args.gpu, workers=4, renderer='RaytracedLighting', disable_viewport_updates=True,
                    extra_args=RT2_ARGS if args.video else ())
    process = {'startup_s': time.perf_counter() - run_started, 'cases': []}
    runner, status = None, 'FAIL'
    try:
        started = time.perf_counter()
        runner = Runner(args, output)
        process['load_s'] = time.perf_counter() - started
        process['load'] = runner.timings
        statuses = []
        for directory, seed, offsets in case_list(args):
            directory.mkdir(parents=True, exist_ok=True)
            result = runner.run(directory, seed, offsets)
            if directory != output:
                result['timings'].update(startup_s=process['startup_s'], load_s=process['load_s'])
                write_json(directory / 'result.json', json.loads(json.dumps(result, default=float)))
            statuses.append(result['status'])
            process['cases'].append({'case': directory.name, 'status': result['status']})
        status = 'PASS' if statuses and all(s == 'PASS' for s in statuses) else 'FAIL'
    except Exception:
        process['error'] = traceback.format_exc()
    process['total_before_close_s'] = time.perf_counter() - run_started
    if not args.cases:
        # A single case keeps its result in --output; add the process timings to it.
        path = output / 'result.json'
        result = json.loads(path.read_text()) if path.exists() else {'status': 'FAIL'}
        if 'error' in process:
            result['error'] = process['error']
        result.setdefault('timings', {}).update(startup_s=process['startup_s'], load_s=process.get('load_s'),
                                                total_before_close_s=process['total_before_close_s'])
        write_json(path, json.loads(json.dumps(result, default=float)))
    else:
        write_json(output / 'process.json', process)
    mark_shutdown(output, status)
    if runner is not None:
        runner.backend.close()
    app.close()


def parent(args, script):
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f'output {output} is not empty')
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.runtime.app import isolated_environment
    gpus = subprocess.run(['nvidia-smi', '--query-gpu=index,memory.used,utilization.gpu', '--format=csv'],
                          capture_output=True, text=True).stdout
    (output / 'gpu-before.txt').write_text(gpus)
    command = [sys.executable, str(script), '--child', '--output', str(output), '--gpu', str(args.gpu)]
    if args.cases:
        command += ['--cases', str(Path(args.cases).resolve())]
    elif args.seed is not None:
        command += ['--seed', str(args.seed)]
    elif args.offsets:
        command += ['--offsets', *map(str, args.offsets)]
    if args.video:
        command += ['--video', '--subframes', str(args.subframes)]
    code = supervise(command, cwd=ROOT, output=output, env=isolated_environment(output))
    if args.cases:
        process = json.loads((output / 'process.json').read_text()) if (output / 'process.json').exists() else {}
        summary = {'returncode': code, 'cases': process.get('cases'), 'error': (process.get('error') or '')[-600:] or None}
        print(json.dumps(summary, indent=2))
        return 0 if code == 0 and not summary['error'] else 1
    result = json.loads((output / 'result.json').read_text()) if (output / 'result.json').exists() else {}
    summary = {'status': result.get('status', 'NO_RESULT'), 'returncode': code, 'failure': result.get('failure'),
               'evaluation': result.get('evaluation'), 'error': (result.get('error') or '')[-600:] or None}
    print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'PASS' else 1
