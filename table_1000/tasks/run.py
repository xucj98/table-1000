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


def child(args):
    output = args.output.resolve()
    run_started = time.perf_counter()
    from table_1000.runtime.app import start_app
    from table_1000.runtime.render import RT2_ARGS
    # Only video runs need RTX Real-Time 2.0 (see table_1000.runtime.render).
    app = start_app(output, gpu=args.gpu, workers=4, renderer='RaytracedLighting', disable_viewport_updates=True,
                    extra_args=RT2_ARGS if args.video else ())
    timings = {'startup_s': time.perf_counter() - run_started}
    result = {'status': 'FAIL', 'seed': args.seed, 'offsets_argument': args.offsets}
    backend = None
    try:
        from table_1000.scenes import load
        from table_1000.runtime.cameras import Cameras
        from table_1000.robots.controllers import CONTROL_HZ
        from table_1000.robots.kinematics import RobotModel
        from table_1000.robots.x1pro import ARM, SPEC
        from table_1000.sim.env import Env, EnvView
        from table_1000.sim.isaac import IsaacBackend
        from table_1000.tasks.motion import PlanningFailed, run_expert
        from table_1000.tasks.drawer_pen import Solver, TaskFailed, evaluate, DRAWER_JOINT
        from table_1000.tasks import drawer_pen_geometry as g
        scene = load(SCENE)
        offsets = None
        if args.offsets:
            offsets = {'cabinet': tuple(args.offsets[:2]), 'pen': tuple(args.offsets[2:])}
        layout = scene.placement(seed=args.seed, offsets=offsets)
        result['layout'] = {'offsets': layout['offsets'], 'problems': scene.check(layout)}
        cameras = (lambda stage: Cameras(stage, views(), output / 'frames', fps=CONTROL_HZ,
                                         subframes=args.subframes)) if args.video else None
        # Reports come from the robot and the cabinet; reporting on the pen
        # itself would also deliver its cap detent's contacts on every step.
        backend = IsaacBackend(scene, layout, ASSETS, cameras=cameras, contact_objects=('cabinet',),
                               contact_watch=('robot', 'pen:'))
        model = RobotModel(SPEC, output / 'planning')
        env = Env(scene, backend, SPEC, [model])
        env.reset(layout)
        view = EnvView(env)
        initial = view.body_poses()
        result['initial_poses'] = {key: initial[key].tolist() for key in ('cabinet:cabinet.housing', 'pen:body')}
        solver = Solver(view, ASSETS)
        motion = solver.motion
        env.caption = lambda env: f't={env.time:5.2f}s  {backend.contacts.phase}'
        rows = []
        heartbeat = [time.monotonic()]

        def record(env):
            pen = view.body_poses()['pen:body']
            q = view.qpos(ARM)
            row = {'time': round(env.time, 4), 'stage': solver.stage,
                   **{f'q{i + 1}': float(v) for i, v in enumerate(q)},
                   **{f'q{i + 1}_target': float(v) for i, v in enumerate(view.targets(ARM))},
                   'finger_gap': motion.finger_gap(), 'drawer_opening': view.object_joint('cabinet', DRAWER_JOINT),
                   'pen_x': pen[0, 3], 'pen_y': pen[1, 3], 'pen_z': pen[2, 3],
                   'pen_axis_z': pen[2, 0]}
            rows.append(row)
            if time.monotonic() - heartbeat[0] > 2:
                heartbeat[0] = time.monotonic()
                write_json(output / 'heartbeat.json', {'time': env.time, 'stage': solver.stage, 'epoch': time.time()})

        env.observers.append(record)
        timings['load_s'] = backend.timings['load_s']
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
        write_json(output / 'contacts.json', {'expected': expected, 'unexpected': unexpected})
        if rows:
            with (output / 'trace.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        timings.update(backend.timings, **motion.timings)
        if backend.cameras:
            started = time.perf_counter()
            result['videos'] = backend.cameras.encode(output)
            timings['encode_s'] = time.perf_counter() - started
    except Exception:
        result['error'] = traceback.format_exc()
    timings['total_before_close_s'] = time.perf_counter() - run_started
    result['timings'] = timings
    write_json(output / 'result.json', json.loads(json.dumps(result, default=float)))
    mark_shutdown(output, result['status'])
    if backend is not None:
        backend.close()
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
    if args.seed is not None:
        command += ['--seed', str(args.seed)]
    if args.offsets:
        command += ['--offsets', *map(str, args.offsets)]
    if args.video:
        command += ['--video', '--subframes', str(args.subframes)]
    code = supervise(command, cwd=ROOT, output=output, env=isolated_environment(output))
    result = json.loads((output / 'result.json').read_text()) if (output / 'result.json').exists() else {}
    summary = {'status': result.get('status', 'NO_RESULT'), 'returncode': code, 'failure': result.get('failure'),
               'evaluation': result.get('evaluation'), 'error': (result.get('error') or '')[-600:] or None}
    print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'PASS' else 1
