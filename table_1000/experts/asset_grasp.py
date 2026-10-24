"""Isolated asset pick and wrist-shake evaluation using the existing grasp skill."""

from collections import Counter
import math
from pathlib import Path
import time
import types

import numpy as np

from table_1000.scenes.layout import Layout
from table_1000.sim import CONTROL_HZ

REGION_SIZE = .10
WORK_OFFSET = (.05, 0.)  # keep the near edge of the square clear of the folded-arm workspace
SLIP_LIMIT = .005
SHAKE_AMPLITUDE = .4
SHAKE_SECONDS = .5
SHAKE_OFFSETS = (1, -1, 1, -1, 0)

# One candidate policy for every asset. Actual lift/shake measurements, not these
# geometric screening thresholds, determine whether a grasp holds.
HAND_PARAMETERS = dict(pad_spacing=.0005, min_pad_area=1e-6, hold_margin=0.,
                       min_hold_area=1e-6, opening_clearance=.002, cost_width=10.)


def oriented_shape(points):
    """Fit in the geometry's oriented box, so a diagonal wire keeps its true width."""
    import trimesh
    from table_1000.experts.grasps import fitted
    to_box, _ = trimesh.bounds.oriented_bounds(points)
    local = np.asarray(points) @ to_box[:3, :3].T + to_box[:3, 3]
    shape, offset = fitted(local)
    return shape, np.linalg.inv(to_box) @ offset


def targets(skill):
    """Convex whole objects first; hollow assemblies use solid collision parts."""
    geometry = skill.ctx.geometry['object']
    def target(part=None):
        result = skill.target('object', part)
        result.shape, result.offset = oriented_shape(geometry.points(part, result.body.split(':', 1)[1]))
        return result
    whole = target()
    solid = geometry.solid(None, narrow=0.)
    if solid:
        yield whole
    parts = [target(name) for body, name in geometry.parts()
             if f'object:{body}' == whole.body and geometry.solid(name, narrow=0.)]
    # A folded sheet or a ring has no solid part-level box. Its authored convex
    # collision pieces provide actual contact surfaces, without asset-specific points.
    for body, name, transform, kind, data in geometry.shapes:
        if f'object:{body}' != whole.body or geometry.solid(name, narrow=0.) or kind != 'convex':
            continue
        shape, offset = oriented_shape(data)
        parts.append(skill.target('object', name, shape=shape,
                                  offset=transform @ offset))
    parts.sort(key=lambda target: -float(np.prod(np.ptp(target.shape.points(), axis=0))))
    for target in parts:
        if not solid or target.names != whole.names:
            yield target
    if not solid:
        yield whole


def pick(skill, report):
    """Try geometric targets until the whole rigid body actually lifts."""
    from table_1000.experts.context import ExpertFailed
    ctx = skill.ctx
    report['target_attempts'] = []
    for target in targets(skill):
        attempt = {'shape': type(target.shape).__name__, 'parameters': vars(target.shape),
                   'parts': list(target.names)}
        report['target_attempts'].append(attempt)
        thickness = float(np.ptp(target.shape.points(), axis=0).min())
        margin = min(.002, thickness / 4)
        for hand in skill.hands.values():
            hand.hold_margin = margin
        attempt['hold_margin_m'] = margin
        try:
            result = skill.grasp(target, approach=(0., 0., -1.), then=(0., 0., .10), base=False)
            report['target'] = attempt
            solver = skill.solvers[result['arm']]
            attempt['closure'] = {'tcp_pose': ctx.tcp(result['arm']).tolist(),
                                 'object_pose': ctx.poses()[target.body].tolist(),
                                 'finger_gap_m': solver.finger_gap()}
            report['closure'] = attempt['closure']
            rest = [name for name, obj in solver.world.objects.items()
                    if obj.body == target.body and name not in target.names]
            solver.carry(rest, target.names + result['supports'])
            before = ctx.poses()[target.body].copy()
            skill.move_by_displacement(result['arm'], (0., 0., .10))
            ctx.hold(.2)
            rise = float(ctx.poses()[target.body][2, 3] - before[2, 3])
            attempt['lift'] = {'rise_m': rise, 'tcp_pose': ctx.tcp(result['arm']).tolist(),
                              'object_pose': ctx.poses()[target.body].tolist(),
                              'finger_gap_m': solver.finger_gap()}
            report['lift'] = attempt['lift']
            if rise < .05:
                solver.release(rest, target.names + result['supports'])
                skill.release(result['arm'])
                ctx.note('missed lift', rise=rise)
                ctx.fail('object not lifted', rise=rise, grasp=result['grasp'])
            result.update(rise=rise)
            return target, result
        except ExpertFailed as error:
            attempt['failure'] = {'reason': error.reason, 'details': error.details}
            if error.reason not in ('no reachable grasp', 'object not held', 'object not lifted'):
                raise
            last = error
    if any('lift' in attempt for attempt in report['target_attempts']):
        ctx.fail('object not lifted', attempts=len(report['target_attempts']))
    raise last


def inventory(source, excluded=()):
    """Deduplicate asset paths, retaining all instance names and explicit exclusions."""
    grouped = {}
    for name, spec in source.OBJECTS.items():
        grouped.setdefault(spec['asset'], []).append(name)
    return {asset: {'instances': names, 'excluded': all(name in excluded for name in names)}
            for asset, names in sorted(grouped.items())}


def scene(asset, spec, settings):
    return types.SimpleNamespace(
        TABLE=settings.TABLE, OBJECTS={'object': {'asset': asset}},
        ROBOT={'asset': f'robots/{spec.name}', 'position': settings.ROBOT['position'],
               'yaw': settings.ROBOT['yaw'],
               'joints': spec.keyframe(settings.ROBOT.get('keyframe', 'zero'))})


def placement(source, assets, seed, center, size=REGION_SIZE):
    rng = np.random.default_rng(seed)
    xy = np.asarray(center) + rng.uniform(-size / 2, size / 2, 2)
    yaw = float(rng.uniform(-math.pi, math.pi))
    layout = Layout(source, assets, rng)
    pose = layout.resting_pose('object', xy, yaw)
    return pose, {'xy_m': xy.tolist(), 'yaw_rad': yaw, 'pose': pose.tolist()}


def displacement(reference, current, points):
    """Largest collision-vertex displacement in the TCP frame (includes rotation)."""
    before = points @ reference[:3, :3].T + reference[:3, 3]
    after = points @ current[:3, :3].T + current[:3, 3]
    return float(np.linalg.norm(after - before, axis=1).max())


def grasp_family(report):
    """Coarse orientation family in the settled body's frame; exact poses stay in trials."""
    pick = report['pick']
    local = np.linalg.inv(report['settled_pose']) @ np.asarray(pick['grasp']['pose'])
    axes = ['xyz'[int(np.argmax(np.abs(local[:3, column])))] for column in (0, 1)]
    return f"{pick['arm']} / approach {axes[0]} / close {axes[1]}"


def failure_reason(report):
    """Keep a missed closure distinct from an initially unreachable placement."""
    reason = report.get('failure', {}).get('reason', report['status'])
    if reason == 'no reachable grasp' and any(event['note'] == 'missed grasp'
                                               for event in report.get('events', [])):
        return 'object not held'
    return reason


def summarize(reports, expected):
    success = sum(r['status'] == 'success' for r in reports)
    failures = Counter(failure_reason(r) for r in reports if r['status'] != 'success')
    if expected > len(reports):
        failures['missing report'] += expected - len(reports)
    by_grasp = {}
    for report in reports:
        if 'pick' not in report:
            continue
        key = grasp_family(report)
        entry = by_grasp.setdefault(key, {'trials': 0, 'success': 0})
        entry['trials'] += 1
        entry['success'] += report['status'] == 'success'
    for entry in by_grasp.values():
        entry['rate'] = entry['success'] / entry['trials']
    return {'expected': expected, 'reported': len(reports), 'success': success,
            'picked': sum('pick' in r for r in reports), 'rate': success / expected if expected else None,
            'initial_unreachable': sum(failure_reason(r) == 'no reachable grasp' for r in reports),
            'failures': dict(failures), 'by_grasp': by_grasp,
            'status': 'PASS' if success == expected and expected else 'FAIL'}


class Evaluator:
    def __init__(self, asset, robot, assets, work, root, device='cpu', views=None):
        from table_1000.robots import load_robot_test, load_spec
        from table_1000.robots.kinematics import RobotModel
        from table_1000.sim.isaaclab.env import RobotEnv
        self.asset, self.assets = asset, Path(assets)
        self.spec = load_spec(robot, root)
        self.settings = load_robot_test(robot, root)
        self.center = np.asarray(self.settings.WORK) + WORK_OFFSET
        self.scene = scene(asset, self.spec, self.settings)
        self.env = RobotEnv(self.scene, self.spec, assets, root=root, device=device,
                            views=views or {}, contact_watch=('robot', 'object'))
        self.model = RobotModel(self.spec, Path(work) / 'planning', root)

    def shake(self, ctx, target, arm, phase):
        """Same wrist excursion as robot acceptance; monitor the entire motion."""
        indices = self.model.arm_indices(arm)
        baseline = self.model.q[indices].copy()
        wrist = indices[-1]
        lo, hi = self.model.limits[wrist]
        # Shift the oscillation centre inside the limits when the picked wrist is near an end.
        # This transition is monitored too; never skip the shake for a valid grasp.
        baseline[-1] = np.clip(baseline[-1], lo + SHAKE_AMPLITUDE, hi - SHAKE_AMPLITUDE)
        points = ctx.geometry['object'].points(frame=target.body.split(':', 1)[1])
        reference = np.linalg.inv(ctx.tcp(arm)) @ ctx.poses()[target.body]
        trace = []

        def observe(c):
            relative = np.linalg.inv(c.tcp(arm)) @ c.poses()[target.body]
            trace.append({'step': c.steps, 'slip_m': displacement(reference, relative, points),
                          'origin_drift_m': float(np.linalg.norm(relative[:3, 3] - reference[:3, 3])),
                          'wrist_rad': float(self.model.q[wrist])})

        ctx.observers.append(observe)
        phase['text'] = 'shake'
        ctx.env.contacts.phase = 'shake'
        try:
            for offset in SHAKE_OFFSETS:
                start = self.model.q[indices].copy()
                end = baseline.copy()
                end[-1] += offset * SHAKE_AMPLITUDE
                count = round(SHAKE_SECONDS * CONTROL_HZ)
                for step in range(1, count + 1):
                    ctx.step({arm: (start + (end - start) * step / count)[None]})
                if trace[-1]['slip_m'] > .03:
                    break
            ctx.hold(.5)
        finally:
            ctx.observers.remove(observe)
        worst = max(row['slip_m'] for row in trace)
        excursion = max(row['wrist_rad'] for row in trace) - min(row['wrist_rad'] for row in trace)
        completed = offset == SHAKE_OFFSETS[-1]
        return {'held': completed and worst <= SLIP_LIMIT and excursion >= 1.8 * SHAKE_AMPLITUDE,
                'completed': completed, 'max_slip_m': worst, 'wrist_excursion_rad': excursion,
                'segment_s': round(SHAKE_SECONDS * CONTROL_HZ) / CONTROL_HZ,
                'trace': trace}

    def trial(self, seed, center=None, size=REGION_SIZE, video=None):
        import mplib.pymp
        from table_1000.experts.context import Context, ExpertFailed
        started = time.perf_counter()
        cpu_started = time.process_time()
        pose, sampled = placement(self.scene, self.assets, seed,
                                  self.center if center is None else center, size)
        self.env.reset({'object': pose})
        mplib.pymp.set_global_seed(seed + 1)
        ctx = Context(self.env, self.model, self.scene, self.assets, np.random.default_rng(seed))
        phase = {'text': 'settle'}
        if video:
            ctx.observers.append(lambda c: video.capture(f'{self.asset} seed={seed} {phase["text"]}'))
        report = {'asset': self.asset, 'seed': seed, 'placement': sampled,
                  'physics_dt_s': float(self.env.env.cfg.sim.dt), 'control_hz': CONTROL_HZ}
        try:
            self.env.contacts.phase = 'settle'
            ctx.hold(.5)
            skill = ctx.skills()
            from table_1000.experts.grasps import Hand
            skill.hands = {arm: Hand(self.model, arm, **HAND_PARAMETERS) for arm in skill.arms}
            skill.support_clearance = -.002
            skill.held_tolerance = .025
            report['candidate_policy'] = {**HAND_PARAMETERS, 'support_clearance_m': -.002,
                                          'held_tolerance_m': .025}
            report['settled_pose'] = ctx.poses()[skill.target('object').body].tolist()
            phase['text'] = 'pick'
            self.env.contacts.phase = 'pick'
            target, result = pick(skill, report)
            settled = np.asarray(report['settled_pose'])
            grasp_local = np.linalg.inv(settled) @ np.asarray(result['grasp']['pose'])
            result['body_grasp_pose'] = grasp_local.tolist()
            report['pick'] = result
            result['family'] = grasp_family(report)
            report['hold'] = self.shake(ctx, target, result['arm'], phase)
            report['status'] = 'success' if report['hold']['held'] else 'failed'
            if not report['hold']['held']:
                reason = 'slipped during shake' if report['hold']['max_slip_m'] > SLIP_LIMIT else 'shake not executed'
                report['failure'] = {'reason': reason}
        except ExpertFailed as error:
            reason = error.reason
            details = error.details
            if reason == 'no reachable grasp' and any(event['note'] == 'missed grasp' for event in ctx.events):
                # A failed closure can push a light object out of reach. That is
                # a physical grasp failure, not a placement without candidates.
                reason = 'object not held'
                details = {'subsequent_planning_failure': error.details}
            report.update(status='failed', failure={'reason': reason, 'details': details})
        except Exception:
            import traceback
            report.update(status='error', failure={'reason': 'execution error', 'traceback': traceback.format_exc()})
        finally:
            report['cpu_s'] = time.process_time() - cpu_started
            report['final_object_pose'] = self.env.object_pose('object')[0].tolist()
            report['events'] = ctx.events
            report['plans'] = [entry for solver in ctx.solvers.values() for entry in solver.log]
            report['contacts'] = self.env.contacts.intervals()
            report['sim_s'], report['wall_s'] = ctx.steps / CONTROL_HZ, time.perf_counter() - started
            for solver in ctx.solvers.values():
                solver.world.clear()
        return report

    def close(self):
        self.env.close()
