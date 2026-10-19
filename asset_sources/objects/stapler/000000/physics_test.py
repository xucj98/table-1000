"""Force-driven checks of the native two-body stapler approximation.

The existing hinge carries an approved angular return drive. Applied loads are
not measured reactions. All poses are set only before integration; fixtures use
native fixed joints.
"""
import json
from contextlib import nullcontext
import numpy as np

from table_1000.physics.joints import frames
from table_1000.physics.testing import TestContext


LOWER = -.2992541830008421
PRESS_POINT = (-.022, 0, .032)
CHANNEL_FRONT = np.array([-.0313, 0, .0145])
ANVIL_TOP = .00955


def observe(ctx, instance):
    for name in instance.body_names:
        ctx.observe_body(instance.body(name))
    ctx.observe_joint(instance, 'upper_press')
    cache = {'time': None}

    def geometry():
        if cache['time'] != ctx.session.time:
            info = frames(instance, 'upper_press')
            base, upper = instance.body('stapler.base').state, instance.body('stapler.upper').state
            front = upper['position'] + upper['rotation'] @ CHANNEL_FRONT
            local = base['rotation'].T @ (front - base['position'])
            cache.update(time=ctx.session.time,
                         anchor_error=float(np.linalg.norm(info['anchors'][1]['position'] - info['anchors'][0]['position'])),
                         end_gap=float(local[2] - ANVIL_TOP))
        return cache

    for field in ('anchor_error', 'end_gap'):
        ctx.observe(instance.name + '.' + field, lambda f=field: geometry()[f], unit='m')


def setup(ctx, *, gravity=(0, 0, -9.81), ground=False, dt=.004):
    ctx.simulation(dt=dt, gravity=gravity)
    from pxr import PhysxSchema, UsdPhysics
    scene = next(p for p in ctx.session.stage.Traverse() if p.IsA(UsdPhysics.Scene))
    # Native numerical integration setting, required in consumer TGS scenes too.
    # It adds no spring/holding force; the return drive lives in the asset USD.
    PhysxSchema.PhysxSceneAPI.Apply(scene).CreateEnableExternalForcesEveryIterationAttr(True)
    if ground:
        ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.00005)
    ctx.camera(position=(-.14, -.26, .16), target=(0, 0, .022), focal_length=32)
    ctx.experiment_details = {
        'asset_pose_writes_during_experiment': False,
        'mechanism': 'native hinge with approved angular return drive',
        'force_columns': 'applied external loads, not contact/fixture reactions',
        'press_point_upper_local_m': PRESS_POINT,
        'parameters': 'uncalibrated engineering estimates; mass base/upper=35/25 g',
        'solver_exception': 'TGS 64/4; native external forces every iteration enabled; required consumer scene setting',
        'external_forces_every_iteration': bool(PhysxSchema.PhysxSceneAPI(scene).GetEnableExternalForcesEveryIterationAttr().Get()),
    }


def hinge_checks(ctx, instances):
    assert all(np.isfinite(v) for row in ctx.rows for v in row.values())
    results = {}
    for instance in instances:
        q = [r[f'{instance.name}.upper_press.position'] for r in ctx.rows]
        anchor = max(r[f'{instance.name}.anchor_error'] for r in ctx.rows)
        gap = min(r[f'{instance.name}.end_gap'] for r in ctx.rows)
        results[instance.name] = {'q_min_rad': min(q), 'q_max_rad': max(q),
                                  'max_anchor_error_m': anchor, 'min_end_gap_m': gap}
    ctx.experiment_details['hinge_checks'] = results
    for result in results.values():
        assert result['q_min_rad'] >= LOWER - .01 and result['q_max_rad'] <= .01, results
        assert result['max_anchor_error_m'] < .0002, results
        assert result['min_end_gap_m'] > -.0003, results


def _press_cycles(ctx: TestContext, *, fixed):
    setup(ctx, ground=not fixed, dt=.004)
    from pxr import PhysxSchema, PhysicsSchemaTools
    from omni.physx import get_physx_simulation_interface
    PhysxSchema.PhysxContactReportAPI.Apply(
        ctx.session.stage.GetPrimAtPath(ctx.asset.paths['stapler.upper'])).CreateThresholdAttr(0)
    contacts = []
    contact_state = {'time_s': -1., 'impulse_N_s': 0.}

    def contact_report(headers, data):
        if ctx.session.time < contact_state['time_s']:
            contacts.clear()  # warmup/reset starts a new physical experiment
        if ctx.session.time != contact_state['time_s']:
            contact_state.update(time_s=ctx.session.time, impulse_N_s=0.)
        for header in headers:
            pair = [str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                    str(PhysicsSchemaTools.intToSdfPath(header.collider1))]
            if not any('/upper/channel/' in p for p in pair) or not any('/base/anvil/' in p for p in pair):
                continue
            for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                impulse = float(np.linalg.norm(list(data[i].impulse)))
                contacts.append({'time_s': ctx.session.time, 'impulse_N_s': impulse})
                contact_state['impulse_N_s'] += impulse

    subscription = get_physx_simulation_interface().subscribe_contact_report_events(contact_report)
    ctx.initial(joints={'upper_press': 0})
    observe(ctx, ctx.asset)
    ctx.observe('channel_anvil.contact_impulse',
                lambda: contact_state['impulse_N_s'] if abs(contact_state['time_s'] - (ctx.session.time - ctx.dt)) < 1e-8 else 0.,
                unit='N·s')
    ctx.plots('press.force.z', 'asset.upper_press.position', 'asset.upper_press.velocity',
              'asset.end_gap', 'asset.anchor_error', 'channel_anvil.contact_impulse')
    ctx.video_annotation = lambda: f'Base {"fixed" if fixed else "free on table"} | three 3 N press/release cycles | native return drive'
    ctx.experiment_details.update(purpose='three force-driven press/release cycles; check lower stop, contact and return',
                                  fixture='test-only fixed base' if fixed else None, return_expected=True,
                                  force_schedule_N='0.4 s settle; three 0.6 s ramps to 3 N, 0.2 s holds and 0.04 s withdrawals; dwell 0.56/0.56/0.76 s')
    if not fixed:
        ctx.plots('press.force.z', 'asset.upper_press.position', 'asset.upper_press.velocity',
                  'asset.stapler.base.position.x', 'asset.stapler.base.position.z', 'asset.anchor_error',
                  'channel_anvil.contact_impulse')
    with ctx.fixed(ctx.asset.body('stapler.base')) if fixed else nullcontext():
        ctx.force('press', ctx.asset.body('stapler.upper'), point_frame='body', keyframes=[
            {'time': 0, 'force': (0, 0, 0), 'point': PRESS_POINT},
            {'time': .4, 'force': (0, 0, 0)},
            {'time': 1., 'force': (0, 0, -3.)},
            {'time': 1.2, 'force': (0, 0, -3.)},
            {'time': 1.24, 'force': (0, 0, 0)},
            {'time': 1.8, 'force': (0, 0, 0)},
            {'time': 2.4, 'force': (0, 0, -3.)},
            {'time': 2.6, 'force': (0, 0, -3.)},
            {'time': 2.64, 'force': (0, 0, 0)},
            {'time': 3.2, 'force': (0, 0, 0)},
            {'time': 3.8, 'force': (0, 0, -3.)},
            {'time': 4., 'force': (0, 0, -3.)},
            {'time': 4.04, 'force': (0, 0, 0)},
            {'time': 4.8, 'force': (0, 0, 0)},
        ])
        ctx.run(4.8)
    hinge_checks(ctx, [ctx.asset])
    # Away from contact/return impacts, native angular speed must agree with
    # the measured coordinate change. Default TGS force integration violated
    # even the sign of this relationship; tail-speed checks alone missed it.
    rates = [(b['asset.upper_press.position'] - a['asset.upper_press.position']) / ctx.dt
             for a, b in zip(ctx.rows, ctx.rows[1:]) if .5 <= b['time'] <= .8]
    speeds = [r['asset.upper_press.velocity'] for r in ctx.rows if .5 <= r['time'] <= .8]
    rate_error = float(np.sqrt(np.mean((np.asarray(rates) - speeds) ** 2)))
    ctx.experiment_details['ramp_velocity_consistency'] = {
        'window_s': [.5, .8], 'pose_rate_mean_rad_s': float(np.mean(rates)),
        'native_velocity_mean_rad_s': float(np.mean(speeds)),
        'rmse_rad_s': rate_error, 'rmse_limit_rad_s': .05,
        'method': 'backward angle difference versus end-of-step native angular velocity during smooth first ramp',
    }
    assert rate_error < .05, ctx.experiment_details['ramp_velocity_consistency']
    cycles = []
    for start, withdraw, finish in ((.4, 1.24, 1.8), (1.8, 2.64, 3.2), (3.2, 4.04, 4.8)):
        loaded = [r for r in ctx.rows if start + .6 <= r['time'] <= withdraw - .04]
        returned = [r for r in ctx.rows if withdraw <= r['time'] <= finish]
        min_q = min(r['asset.upper_press.position'] for r in loaded)
        final_q = returned[-1]['asset.upper_press.position']
        cycles.append({'loaded_min_q_rad': min_q, 'return_q_rad': final_q,
                       'return_max_abs_velocity_rad_s': max(abs(r['asset.upper_press.velocity']) for r in returned),
                       'tail_max_abs_velocity_rad_s': max(abs(r['asset.upper_press.velocity'])
                                                         for r in returned if r['time'] >= finish - .2)})
        assert min_q <= LOWER + .01, cycles
        assert final_q >= -.01, cycles
        assert max(r['channel_anvil.contact_impulse'] for r in loaded) > 0, 'no channel/anvil contact during loaded cycle'
    ctx.experiment_details['press_cycles'] = cycles
    ctx.experiment_details['endpoint_contact'] = {
        'samples': len(contacts),
        'peak_point_impulse_N_s': max((c['impulse_N_s'] for c in contacts), default=0.),
        'measurement': 'native channel/anvil contact report; excludes joint and fixture reactions',
    }
    assert contacts and max(c['impulse_N_s'] for c in contacts) > 0, 'no channel/anvil contact impulse'
    subscription = None
    assert max(abs(r['asset.upper_press.position']) for r in ctx.rows if r['time'] >= 4.6) < .01
    assert all(c['tail_max_abs_velocity_rad_s'] < .05 for c in cycles), cycles
    if not fixed:
        ground_and_settling(ctx, [ctx.asset], 4.3)


def press_release(ctx: TestContext):
    _press_cycles(ctx, fixed=True)


def table_press(ctx: TestContext):
    _press_cycles(ctx, fixed=False)


def free_reaction(ctx: TestContext):
    setup(ctx, gravity=(0, 0, 0))
    ctx.initial(position=(0, -.05, .12))
    idle = ctx.spawn(ctx.asset_path, name='idle')
    ctx.initial(idle, position=(0, .05, .12))
    for instance in (ctx.asset, idle):
        observe(ctx, instance)
    ctx.camera(position=(-.27, -.48, .29), target=(0, 0, .11), focal_length=28)
    ctx.plots('press.force.z', 'asset.upper_press.position', 'asset.stapler.base.linear_velocity.z',
              'asset.momentum.z', 'idle.stapler.base.linear_velocity.z')
    for axis, label in enumerate('xyz'):
        ctx.observe('asset.momentum.' + label,
                    lambda a=axis: sum(ctx.asset.mass[i] * ctx.asset.body(n).state['velocity'][a]
                                       for i, n in enumerate(ctx.asset.body_names)), unit='kg·m/s')
    ctx.video_annotation = lambda: 'Zero gravity | free base | front instance loaded, rear idle | return drive active'
    ctx.force('press', ctx.asset.body('stapler.upper'), point_frame='body', keyframes=[
        {'time': 0, 'force': (0, 0, 0), 'point': PRESS_POINT},
        {'time': .1, 'force': (0, 0, 0)},
        {'time': .12, 'force': (0, 0, -.03)},
        {'time': .28, 'force': (0, 0, -.03)},
        {'time': .3, 'force': (0, 0, 0)},
        {'time': .8, 'force': (0, 0, 0)},
    ])
    ctx.run(.8)
    impulse = np.array([sum(r['press.force.' + a] for r in ctx.rows[1:]) * ctx.dt for a in 'xyz'])
    momentum = np.array([ctx.rows[-1]['asset.momentum.' + a] - ctx.rows[0]['asset.momentum.' + a] for a in 'xyz'])
    error = float(np.linalg.norm(momentum - impulse) / np.linalg.norm(impulse))
    idle_speed = max(np.linalg.norm([r[f'idle.{n}.linear_velocity.{a}'] for a in 'xyz'])
                     for r in ctx.rows for n in idle.body_names)
    base_speed = float(np.linalg.norm(ctx.asset.body('stapler.base').state['velocity'][:3]))
    ctx.experiment_details.update(purpose='free base reacts through hinge; total momentum matches impulse',
        fixture=None, impulse_N_s=impulse.tolist(), momentum_change_kg_m_s=momentum.tolist(),
        relative_momentum_error=error, idle_max_speed_m_s=float(idle_speed), final_base_speed_m_s=base_speed)
    hinge_checks(ctx, [ctx.asset, idle])
    assert error < .01 and idle_speed < 1e-6 and base_speed > .001, ctx.experiment_details


def drop_states(ctx: TestContext):
    # The 20 mm impact is a small-contact diagnostic.  Use the targeted 2 ms
    # step here; the 4 ms result is retained separately as a sensitivity audit.
    setup(ctx, ground=True, dt=.002)
    ctx.initial(position=(-.055, 0, .020), joints={'upper_press': 0})
    pressed = ctx.spawn(ctx.asset_path, name='pressed')
    ctx.initial(pressed, position=(.055, 0, .020), joints={'upper_press': LOWER})
    for instance in (ctx.asset, pressed):
        observe(ctx, instance)
    ctx.camera(position=(-.17, -.42, .23), target=(0, 0, .022), focal_length=32)
    ctx.plots('asset.upper_press.position', 'pressed.upper_press.position',
              'asset.stapler.base.position.z', 'pressed.stapler.base.position.z',
              'asset.anchor_error', 'pressed.anchor_error')
    ctx.video_annotation = lambda: '20 mm drop | left open, right pressed | return drive active | no fixture'
    ctx.run(3)
    ctx.experiment_details.update(purpose='open/pressed low drop and connected-body integrity',
                                  initial_lowest_collision_height_m=.020, fixture=None)
    hinge_checks(ctx, [ctx.asset, pressed])
    ground_and_settling(ctx, [ctx.asset, pressed], 2.5)


def ground_and_settling(ctx, instances, tail_start):
    from table_1000.physics.test import ground_clearance
    ground = ground_clearance(json.loads((ctx.asset_path.parent / 'model.json').read_text()),
                              ctx.rows, [instance.name for instance in instances], 0)
    tail = [r for r in ctx.rows if r['time'] >= tail_start]
    settled = {}
    for instance in instances:
        for name in instance.body_names:
            prefix = f'{instance.name}.{name}'
            speed = max(np.linalg.norm([r[f'{prefix}.linear_velocity.{a}'] for a in 'xyz']) for r in tail)
            position = np.array([[r[f'{prefix}.position.{a}'] for a in 'xyz'] for r in tail])
            settled[prefix] = {'max_tail_speed_m_s': float(speed),
                               'max_tail_position_span_m': float(np.ptp(position, axis=0).max())}
    ctx.experiment_details.update(ground_geometry=ground, settling=settled)
    assert ground['maximum_geometry_penetration_m'] < .002, ground
    assert all(v['max_tail_speed_m_s'] < .01 and v['max_tail_position_span_m'] < .001
               for v in settled.values()), settled


TESTS = {'press_release': press_release, 'table_press': table_press,
         'free_reaction': free_reaction, 'drop_states': drop_states}
