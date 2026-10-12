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
FINGER_LINKS = ('robot.right_arm_gripper_left_link', 'robot.right_arm_gripper_right_link')


def views():
    """Fixed wide and close views plus the robot's head camera."""
    from table_1000.robots.x1pro import camera_mount
    from table_1000.scenes.stage import ROBOT_PATH
    link, local = camera_mount(ROBOT_SOURCE)
    return {**VIEWS, 'head': {'parent': f'{ROBOT_PATH}/{link}', 'local': local, 'focal_length_mm': 15,
                              'resolution': (960, 540)}}


def classify(intervals, handle, pen_prefix):
    """Split robot contacts into expected finger contacts and everything else."""
    expected, unexpected = [], []
    for entry in intervals:
        a, b = entry['pair']
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
    app = start_app(output, gpu=args.gpu, workers=4, renderer='RaytracedLighting',
                    disable_viewport_updates=True)
    timings = {'startup_s': time.perf_counter() - run_started}
    result = {'status': 'FAIL', 'seed': args.seed, 'offsets_argument': args.offsets}
    env = None
    try:
        from table_1000.scenes import load
        from table_1000.runtime.cameras import Cameras
        from table_1000.tasks.environment import Environment, RENDER_EVERY, DT
        from table_1000.tasks.motion import MotionPlanner, PlanningFailed
        from table_1000.tasks.drawer_pen import Solver, TaskFailed, evaluate, DRAWER_JOINT
        from table_1000.tasks import drawer_pen_geometry as g
        scene = load(SCENE)
        offsets = None
        if args.offsets:
            offsets = {'cabinet': tuple(args.offsets[:2]), 'pen': tuple(args.offsets[2:])}
        layout = scene.placement(seed=args.seed, offsets=offsets)
        result['layout'] = {'offsets': layout['offsets'], 'problems': scene.check(layout)}
        cameras = (lambda stage: Cameras(stage, views(), output / 'frames', fps=round(1 / (DT * RENDER_EVERY)))) \
            if args.video else None
        env = Environment(scene, layout, ASSETS, cameras=cameras)
        initial = env.body_poses()
        result['initial_poses'] = {key: initial[key].tolist() for key in ('cabinet:cabinet.housing', 'pen:body')}
        motion = MotionPlanner(env, ROBOT_SOURCE, output / 'planning', ASSETS)
        solver = Solver(env, motion, ASSETS / scene.CABINET['asset'] / 'model.json')
        from table_1000.robots.x1pro import ARM
        rows = []
        heartbeat = [time.monotonic()]

        def record(env):
            poses = env.body_poses()
            pen = poses['pen:body']
            q = env.joint_positions(ARM)
            row = {'time': round(env.time, 4), 'stage': solver.stage,
                   **{f'q{i + 1}': float(v) for i, v in enumerate(q)},
                   **{f'q{i + 1}_target': float(env.targets[env.dof[name]]) for i, name in enumerate(ARM)},
                   'finger_gap': motion.finger_gap(), 'drawer_opening': env.drawer_opening(DRAWER_JOINT),
                   'pen_x': pen[0, 3], 'pen_y': pen[1, 3], 'pen_z': pen[2, 3],
                   'pen_axis_z': pen[2, 0]}
            rows.append(row)
            if time.monotonic() - heartbeat[0] > 2:
                heartbeat[0] = time.monotonic()
                write_json(output / 'heartbeat.json', {'time': env.time, 'stage': solver.stage, 'epoch': time.time()})

        env.observers.append(record)
        timings['load_s'] = env.timings['load_s']
        try:
            solver.solve()
            result['failure'] = None
        except (TaskFailed, PlanningFailed) as error:
            result['failure'] = {'stage': error.stage, 'reason': error.reason,
                                 'details': json.loads(json.dumps(error.details, default=float))}
        env.step(round(.5 / DT))
        evaluation = evaluate(env, solver)
        expected, unexpected = classify(env.contacts.intervals(), 'cabinet:' + g.HANDLE, 'pen:')
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
        timings.update(env.timings, **motion.timings)
        if env.cameras:
            started = time.perf_counter()
            result['videos'] = env.cameras.encode(output)
            timings['encode_s'] = time.perf_counter() - started
    except Exception:
        result['error'] = traceback.format_exc()
    timings['total_before_close_s'] = time.perf_counter() - run_started
    result['timings'] = timings
    write_json(output / 'result.json', json.loads(json.dumps(result, default=float)))
    mark_shutdown(output, result['status'])
    if env is not None:
        env.close()
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
        command.append('--video')
    code = supervise(command, cwd=ROOT, output=output, env=isolated_environment(output))
    result = json.loads((output / 'result.json').read_text()) if (output / 'result.json').exists() else {}
    summary = {'status': result.get('status', 'NO_RESULT'), 'returncode': code, 'failure': result.get('failure'),
               'evaluation': result.get('evaluation'), 'error': (result.get('error') or '')[-600:] or None}
    print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'PASS' else 1
