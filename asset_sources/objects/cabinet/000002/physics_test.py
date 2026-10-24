"""Native pegboard-cabinet experiments, SI units and uncalibrated design values.

Declare fixtures before initialization; drive/release through PhysX tensors.
Straight pegs remain removable contacts, never permanently attached to housing.
"""
from functools import partial
import math
import numpy as np
from table_1000.physics.testing import TestContext

PREFIX = 'pegboard_cabinet.'
TRAVEL = .170
STOP_TOLERANCE = .0005


def view(ctx, target=(0, -.04, .20)):
    ctx.camera(position=(1.1, -1.5, .9), target=target)


def observe(ctx, instance=None):
    """Record ordinary traces plus physical outcomes in every environment."""
    from pxr import UsdPhysics
    a = instance or ctx.asset
    for name in a.body_names:
        ctx.observe_body(a.body(name))
    for name in a.joint_names:
        ctx.observe_joint(a, name)
    anchors = {}
    for side in ('left', 'right'):
        joint = UsdPhysics.Joint(a.joint(PREFIX+'drawer_'+side+'.slide'))
        anchors[side] = [np.array(getattr(joint, f'GetLocalPos{i}Attr')().Get()) for i in (0, 1)]
    cache = {}

    def values():
        key = (ctx.session.steps, ctx.session.time)
        if cache.get('key') != key:
            positions, rotations, _ = a.environment_states()
            h = a.indices[PREFIX+'housing']
            origin, rotation = positions[:, h], rotations[:, h]
            result = {'housing_y': origin[:, 1], 'housing_z': origin[:, 2],
                      'housing_up': rotation[:, 2, 2]}
            for side in ('left', 'right'):
                i = a.indices[PREFIX+'drawer_'+side]
                p0, p1 = anchors[side]
                delta = positions[:, i] + np.einsum('eij,j->ei', rotations[:, i], p1)
                delta -= origin + np.einsum('eij,j->ei', rotation, p0)
                result[side+'_opening'] = -np.einsum('ei,ei->e', delta, rotation[:, :, 1])
            for part in ('tray', 'hook'):
                relative = np.einsum('eji,ej->ei', rotation, positions[:, a.indices[PREFIX+part]]-origin)
                result[part+'_y'], result[part+'_z'] = relative[:, 1], relative[:, 2]
            cache.update(key=key, values=result)
        return cache['values']

    fields = ['housing_y', 'housing_z', 'housing_up', 'left_opening', 'right_opening',
              'tray_y', 'tray_z', 'hook_y', 'hook_z']
    for environment in range(ctx.num_envs):
        for field in fields:
            ctx.observe(f'{a.name}.env{environment}.{field}',
                        lambda e=environment, f=field: values()[f][e],
                        unit='1' if field == 'housing_up' else 'm')


def series(ctx, field, environment=0, instance='asset', after=0, before=math.inf):
    return np.array([row[f'{instance}.env{environment}.{field}'] for row in ctx.rows
                     if after-1e-9 <= row['time'] <= before+1e-9])


def limits(ctx):
    for side in ('left', 'right'):
        all_positions = np.concatenate([series(ctx, side+'_opening', e) for e in range(ctx.num_envs)])
        assert all_positions.min() >= -STOP_TOLERANCE and all_positions.max() <= TRAVEL+STOP_TOLERANCE, \
            f'{side} drawer exceeded native 0..170 mm limit tolerance'


def pull_release(ctx: TestContext, side):
    view(ctx)
    joint = PREFIX+'drawer_'+side+'.slide'
    ctx.initial(joints={joint: .045})
    observe(ctx)
    ctx.video_annotation = lambda: f'{side}: 0-0.75 N ramp, release 2.1 s; housing held'
    ctx.plots('pull.force.y', 'asset.'+joint+'.position', 'asset.'+joint+'.velocity')
    ctx.force('pull', ctx.asset.body(PREFIX+'drawer_'+side), keyframes=[
        {'time': 0, 'force': (0, 0, 0)}, {'time': .5, 'force': (0, 0, 0)},
        {'time': 2, 'force': (0, -.75, 0)}, {'time': 2.1, 'force': (0, 0, 0)},
        {'time': 5, 'force': (0, 0, 0)}])
    with ctx.fixed(ctx.asset.body(PREFIX+'housing')):
        ctx.run(5)
    onsets, coasts = [], []
    for e in range(ctx.num_envs):
        q = series(ctx, side+'_opening', e)
        assert abs(q[0]-.045) < 1e-6, 'initial drawer opening not applied'
        onset = next((r for r in ctx.rows if r[f'asset.env{e}.{side}_opening']-q[0] >= .0005), None)
        assert onset is not None and onset['time'] <= 2.1, 'drawer did not break away during ramp'
        onsets.append(abs(onset['pull.force.y']))
        rest = series(ctx, side+'_opening', e, after=4)
        assert np.ptp(rest) < .0001, 'drawer failed to stop after withdrawal of force'
        coasts.append((q[-1]-series(ctx, side+'_opening', e, after=2.1)[0])*1000)
    limits(ctx)
    ctx.metric('breakaway_force', onsets[0], unit='N', label='0.5 mm位移起滑时施加力')
    ctx.metric('rest_opening', series(ctx, side+'_opening')[-1]*1000, unit='mm', label='撤力后停止开度')
    ctx.metric('coast', coasts[0], unit='mm', label='撤力后续行距离')
    ctx.metric('all_env_max_final_motion', max(np.ptp(series(ctx, side+'_opening', e, after=4))*1000 for e in range(ctx.num_envs)), unit='mm', label='所有环境末秒最大位移变化')
    ctx.experiment_details = {'purpose': 'ramp breakaway and coast to rest', 'side': side,
        'initial_joint_m': .045, 'release_time_s': 2.1, 'onset_displacement_m': .0005,
        'all_environment_breakaway_force_N': onsets, 'all_environment_coast_mm': coasts,
        'fixture': 'static housing fixture declared before initialization'}


def stops(ctx: TestContext):
    view(ctx)
    ctx.initial(joints={PREFIX+'drawer_left.slide': .085, PREFIX+'drawer_right.slide': .085})
    observe(ctx)
    ctx.video_annotation = lambda: 'Both drawers: +1 N opening then -1 N closing; 0..170 mm stops'
    ctx.plots('left.force.y', *[f'asset.{PREFIX}drawer_{s}.slide.position' for s in ('left', 'right')])
    for side in ('left', 'right'):
        ctx.force(side, ctx.asset.body(PREFIX+'drawer_'+side), keyframes=[
            {'time': 0, 'force': (0, -1, 0)}, {'time': 2, 'force': (0, -1, 0)},
            {'time': 2.2, 'force': (0, 1, 0)}, {'time': 4.2, 'force': (0, 1, 0)},
            {'time': 4.3, 'force': (0, 0, 0)}, {'time': 5, 'force': (0, 0, 0)}])
    with ctx.fixed(ctx.asset.body(PREFIX+'housing')):
        ctx.run(5)
    limits(ctx)
    for side in ('left', 'right'):
        for e in range(ctx.num_envs):
            assert series(ctx, side+'_opening', e, before=2).max() >= TRAVEL-STOP_TOLERANCE, 'upper stop not reached'
            assert series(ctx, side+'_opening', e, after=2.2, before=4.2).min() <= STOP_TOLERANCE, 'lower stop not reached'
        values = np.concatenate([series(ctx, side+'_opening', e) for e in range(ctx.num_envs)])
        ctx.metric(side+'_max_opening', values.max()*1000, unit='mm', label=side+' 所有环境最大开度')
        ctx.metric(side+'_min_opening', values.min()*1000, unit='mm', label=side+' 所有环境最小开度')
    ctx.experiment_details = {'purpose': 'both native stops under sustained load', 'limits_m': [0, TRAVEL],
        'solver_tolerance_m': STOP_TOLERANCE, 'initial_m': .085, 'all_environments_checked': ctx.num_envs}


def free_pull(ctx: TestContext):
    ctx.camera(position=(1.6, -2.4, 1.4), target=(0, -.45, .20))
    ctx.ground(contact_offset=.0001)
    observe(ctx)
    ctx.video_annotation = lambda: 'Free housing on ground: right drawer pull 0-16 N, release'
    ctx.plots('pull.force.y', f'asset.{PREFIX}drawer_right.slide.position',
              f'asset.{PREFIX}housing.position.y', f'asset.{PREFIX}housing.position.z')
    ctx.force('pull', ctx.asset.body(PREFIX+'drawer_right'), keyframes=[
        {'time': 0, 'force': (0, 0, 0)}, {'time': .5, 'force': (0, 0, 0)},
        {'time': 3, 'force': (0, -16, 0)}, {'time': 3.2, 'force': (0, 0, 0)},
        {'time': 6, 'force': (0, 0, 0)}])
    ctx.run(6)
    limits(ctx)
    moved = []
    for e in range(ctx.num_envs):
        q, y = series(ctx, 'right_opening', e), series(ctx, 'housing_y', e)
        assert np.isfinite(y).all() and q.max() >= TRAVEL-STOP_TOLERANCE, 'free pull failed to reach drawer stop'
        moved.append(abs(y[-1]-y[0]))
        assert moved[-1] > .001, 'housing showed no reaction to pull'
        assert series(ctx, 'housing_z', e, after=5).min() > -.0005, 'persistent housing-ground overlap'
    ctx.metric('housing_travel', moved[0]*1000, unit='mm', label='自由主体前后位移')
    ctx.metric('drawer_max_opening', series(ctx, 'right_opening').max()*1000, unit='mm', label='右抽屉最大开度')
    ctx.metric('minimum_upright_cosine', min(series(ctx, 'housing_up', e).min() for e in range(ctx.num_envs)), unit='1', label='所有环境主体Z轴最小竖直分量')
    ctx.experiment_details = {'purpose': 'unanchored housing reaction, no fixture',
        'all_environment_housing_travel_m': moved, 'release_time_s': 3.2, 'force_peak_N': 16}


def gravity_tilt(ctx: TestContext):
    view(ctx, target=(0, -.04, .40))
    ctx.initial(position=(0, 0, .25))
    observe(ctx)
    h = ctx.asset.body(PREFIX+'housing')
    ctx.observe('tilt.angle', lambda: math.atan2(h.state['rotation'][2, 1], h.state['rotation'][1, 1]), unit='rad')
    ctx.plots('tilt.angle', *[f'asset.{PREFIX}drawer_{s}.slide.position' for s in ('left', 'right')])
    ctx.video_annotation = lambda: 'Gravity only: 1 s horizontal, 10 s smooth tilt to 45 deg, hold 2 s'
    for e in range(ctx.num_envs):
        ctx.observe(f'tilt.env{e}.angle', lambda e=e: math.atan2(
            ctx.asset.environment_states()[1][e, ctx.asset.indices[PREFIX+'housing'], 2, 1],
            ctx.asset.environment_states()[1][e, ctx.asset.indices[PREFIX+'housing'], 1, 1]), unit='rad')
    grip = ctx.grip(h, name='housing_grip')
    ctx.ensure_initialized()
    held_mass = sum(float(ctx.asset.mass[ctx.asset.indices[PREFIX+name]])
                    for name in ('housing', 'drawer_left', 'drawer_right'))
    assert math.isclose(grip.gains['held_mass_kg'], held_mass, rel_tol=1e-6), \
        'public housing grip carries unrelated free attachments; fix native-tree membership before tilt acceptance'
    ctx.run(1)
    angle = math.pi/4
    grip.move_to(rotation=(math.cos(angle/2), math.sin(angle/2), 0, 0), duration=10)
    ctx.run(2)
    starts = {}
    for side in ('left', 'right'):
        starts[side] = []
        for e in range(ctx.num_envs):
            column = f'asset.env{e}.{side}_opening'
            initial = ctx.rows[0][column]
            onset = next((r for r in ctx.rows if r[column]-initial >= .0005), None)
            assert onset is not None, f'{side} drawer never opened during 45 degree gravity tilt'
            starts[side].append({'angle_deg': math.degrees(onset[f'tilt.env{e}.angle']), 'time_s': onset['time']})
        ctx.metric(side+'_onset', starts[side][0]['angle_deg'], unit='deg', label=side+' 抽屉0.5 mm起滑角')
    limits(ctx)
    ctx.experiment_details = {'purpose': 'gravity breakaway, not a copied angle target',
        'criterion_m': .0005, 'onset_by_environment': starts, 'external_drawer_forces': None,
        'fixture': 'predeclared tensor-controlled housing grip; drawers free',
        'schedule': '1 s horizontal, quintic 45 degree rotation over 10 s, hold 2 s',
        'difference_from_historical_test': 'smooth quintic grip replaces linear USD kinematic-target writes'}


def rest_drop_multi(ctx: TestContext):
    ctx.camera(position=(1.5, -2., 1.2), target=(0, -.20, .20))
    ctx.ground(contact_offset=.0001)
    ctx.initial(position=(-.23, 0, 0))
    other = ctx.spawn(ctx.asset_path, name='drop', position=(.23, 0, .06))
    ctx.initial(instance=other, position=(.23, 0, .06), rotation=(math.cos(.05), 0, math.sin(.05), 0))
    observe(ctx)
    observe(ctx, other)
    ctx.plots(f'asset.{PREFIX}housing.position.z', f'drop.{PREFIX}housing.position.z',
              f'asset.{PREFIX}tray.position.z', f'asset.{PREFIX}hook.position.z')
    ctx.video_annotation = lambda: 'Two free cabinet instances per environment: rest vs 60 mm tilted drop'
    ctx.run(6)
    for instance in ('asset', 'drop'):
        for e in range(ctx.num_envs):
            z = series(ctx, 'housing_z', e, instance, after=5)
            assert np.isfinite(z).all() and z.min() > -.0005, 'persistent final housing-ground overlap'
            assert np.ptp(z) < .0005, 'housing failed to settle in final second'
        ctx.metric(instance+'_settled_z', series(ctx, 'housing_z', instance=instance)[-1]*1000, unit='mm', label=instance+' 主体最终底面高度')
    ctx.experiment_details = {'purpose': 'independent resting and 60 mm low-drop instances',
        'instances_per_environment': 2, 'environment_count': ctx.num_envs, 'fixture': None,
        'attachment_retention': 'straight pegs may detach on drop; no anti-drop mechanism claimed'}


def attachments(ctx: TestContext):
    ctx.camera(position=(1.45, -1.8, 1.), target=(.20, -.04, .20))
    ctx.ground(contact_offset=.0001)
    loose = ctx.spawn(ctx.asset_path, name='unseated', position=(.42, 0, 0))
    # Legal initial USD/body overrides are copied into every environment.
    # No World.scene registration, dynamic USD writes or post-init fixtures.
    ctx.initial(instance=loose, position=(.42, 0, 0), bodies={
        PREFIX+'tray': {'position': (.36, .025, .144)},
        PREFIX+'hook': {'position': (.35, .064, .3365)}})
    observe(ctx)
    observe(ctx, loose)
    ctx.plots(f'asset.{PREFIX}tray.position.y', f'asset.{PREFIX}tray.position.z',
              f'asset.{PREFIX}hook.position.y', f'unseated.{PREFIX}tray.position.z')
    ctx.video_annotation = lambda: 'Seated vs 30 mm withdrawn; eccentric 0.10 N load; nominal forward pulls'
    ctx.force('eccentric', ctx.asset.body(PREFIX+'tray'), keyframes=[
        {'time': 0, 'force': (0, 0, 0), 'point': (-.14, .025, .145)},
        {'time': 1, 'force': (0, 0, 0)}, {'time': 1.2, 'force': (0, 0, -.10)},
        {'time': 2.2, 'force': (0, 0, -.10)}, {'time': 2.4, 'force': (0, 0, 0)},
        {'time': 6, 'force': (0, 0, 0)}])
    for part, effort in [('tray', .8), ('hook', .005)]:
        ctx.force('extract_'+part, ctx.asset.body(PREFIX+part), keyframes=[
            {'time': 0, 'force': (0, 0, 0)}, {'time': 3, 'force': (0, 0, 0)},
            {'time': 3.4, 'force': (0, -effort, 0)}, {'time': 3.8, 'force': (0, 0, 0)},
            {'time': 6, 'force': (0, 0, 0)}])
    ctx.run(6)
    for part in ('tray', 'hook'):
        falls = []
        settling = []
        loaded_motion = []
        for e in range(ctx.num_envs):
            # Resolve assembly clearance under gravity before the 1.2 s load.
            # Measure load-induced motion against the settled 1 s baseline;
            # authored pose is not the equilibrium pose of a loose straight peg.
            seated = series(ctx, part+'_z', e, after=1, before=3)
            settling.append(series(ctx, part+'_z', e)[0]-seated[0])
            loaded_motion.append(float(np.max(np.abs(seated-seated[0]))))
            assert loaded_motion[-1] < .001, \
                f'{part} env{e}: seated attachment moved over 1 mm under eccentric load'
            loose_z = series(ctx, part+'_z', e, 'unseated')
            falls.append(loose_z[0]-loose_z[-1])
            assert falls[-1] > .030, 'truly unseated attachment did not fall'
        y = series(ctx, part+'_y')
        pull_start = series(ctx, part+'_y', after=3)[0]
        ctx.metric(part+'_nominal_pull_displacement', (pull_start-y[-1])*1000, unit='mm', label=part+' 标称前拉阶段相对位移（非起拔阈值）')
        ctx.metric(part+'_unseated_drop', falls[0]*1000, unit='mm', label=part+' 未就位对照下落距离')
        ctx.metric(part+'_gravity_settling', settling[0]*1000, unit='mm', label=part+' 前1秒自重落座位移')
        ctx.metric(part+'_all_env_load_motion', max(loaded_motion)*1000, unit='mm', label=part+' 所有环境偏载阶段最大竖向位移')
    ctx.experiment_details = {'purpose': 'native peg retention and genuinely unseated control',
        'eccentric_load_N': .10, 'tray_forward_effort_N': .8, 'hook_forward_effort_N': .005,
        'note': 'nominal pull is not a measured extraction threshold; full withdrawal is a separate gripper experiment',
        'environment_count_checked': ctx.num_envs}


def attachment_reinstall(ctx: TestContext):
    view(ctx)
    ctx.ground(contact_offset=.0001)
    observe(ctx)
    ctx.plots(f'asset.{PREFIX}tray.position.y', f'asset.{PREFIX}tray.position.z',
              'tray_grip.force.y', 'tray_grip.force.z', 'hook_grip.force.y')
    ctx.video_annotation = lambda: 'Predeclared grips: 45 mm withdrawal, reinsertion, release; two cycles'
    # All three fixtures must independently address their actual native tree.
    # Public Fixture must support the mixed housing articulation/free bodies.
    grips = {part: ctx.grip(ctx.asset.body(PREFIX+part), name=part+'_grip')
             for part in ('tray', 'hook')}
    originals = {part: grip.position.copy() for part, grip in grips.items()}
    cycles = []
    with ctx.fixed(ctx.asset.body(PREFIX+'housing')):
        for grip in grips.values():
            grip.release()
        ctx.ensure_initialized()
        for part, grip in grips.items():
            mass = float(ctx.asset.mass[ctx.asset.indices[PREFIX+part]])
            assert math.isclose(grip.gains['held_mass_kg'], mass, rel_tol=1e-6), \
                f'public {part} grip carries unrelated cabinet bodies; expected held mass {mass:g} kg'
        ctx.run(1)
        settled = {part: [(series(ctx, part+'_y', e)[-1], series(ctx, part+'_z', e)[-1])
                          for e in range(ctx.num_envs)] for part in grips}
        for cycle in range(2):
            for grip in grips.values():
                grip.grasp()
            ctx.wait(*[grip.move_to(position=originals[part], rotation=(1, 0, 0, 0), duration=.6, wait=False)
                       for part, grip in grips.items()])
            ctx.wait(*[grip.move_to(position=originals[part]+np.array([0, -.045, 0]), duration=.8, wait=False)
                       for part, grip in grips.items()])
            for part in grips:
                for e in range(ctx.num_envs):
                    y = series(ctx, part+'_y', e)
                    assert y[0]-y[-1] >= .040, 'grip failed to withdraw peg clear of board'
            ctx.run(.4)
            ctx.wait(*[grip.move_to(position=originals[part], duration=.8, wait=False)
                       for part, grip in grips.items()])
            ctx.run(.6)
            release_time = ctx.session.time
            for grip in grips.values():
                grip.release()
            ctx.run(1.2)
            errors = {}
            for part in grips:
                errors[part] = []
                for e in range(ctx.num_envs):
                    y = series(ctx, part+'_y', e)
                    z = series(ctx, part+'_z', e)
                    # Check the freely settled contact pose, not the authored
                    # assembly height before gravity takes up the peg clearance.
                    base_y, base_z = settled[part][e]
                    error = float(np.hypot(y[-1]-base_y, z[-1]-base_z))
                    errors[part].append(error)
                    assert error < .001, 'released attachment did not remain in original seat'
            cycles.append({'cycle': cycle+1, 'release_time_s': release_time, 'all_environment_seat_error_m': errors})
    for part in grips:
        peak = max(abs(r[part+'_grip.force.y']) for r in ctx.rows)
        ctx.metric(part+'_peak_grip_forward_force', peak, unit='N', label=part+' 夹具前后力峰值（含惯性）')
        ctx.metric(part+'_max_release_error', max(max(c['all_environment_seat_error_m'][part]) for c in cycles)*1000, unit='mm', label=part+' 所有环境释放后最大落座偏差')
    ctx.metric('completed_cycles', len(cycles), unit='1', label='取下并重新安装次数')
    ctx.experiment_details = {'purpose': 'two physical withdrawal/reinsertion/release cycles',
        'fixture': 'all grips and housing lock declared before initialization; tensor motion and release',
        'withdrawal_m': .045, 'cycles': cycles, 'runtime_usd_writes': False,
        'released_seat_baseline_yz_m': settled,
        'grip_gain_change': 'new public grip derives gains from held native tree; replaces legacy custom spring gains',
        'force_interpretation': 'grip wrench includes carried weight/contact/inertia; not a calibrated pull-off threshold'}


TESTS = {
    'pull_release_left': partial(pull_release, side='left'),
    'pull_release_right': partial(pull_release, side='right'),
    'travel_stops': stops, 'free_pull': free_pull, 'gravity_tilt': gravity_tilt,
    'rest_drop_multi': rest_drop_multi, 'attachments': attachments,
    'attachment_reinstall': attachment_reinstall,
}
