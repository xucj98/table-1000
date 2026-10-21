"""Force-driven checks of the native two-body stapler approximation.

The existing hinge carries an approved angular return drive. Applied loads are
not measured reactions. All poses are set only before integration; the fixed
base uses the framework's declared static fixture.
"""
import json
from contextlib import nullcontext
import numpy as np
from scipy.spatial.transform import Rotation

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
    observe_replicas(ctx, instance)


def observe_replicas(ctx, instance):
    """Observe other environments from their own tensors, never broadcast env 0."""
    from pxr import UsdPhysics
    from table_1000.physics.simulation import rotation
    joint = UsdPhysics.RevoluteJoint(instance.joint('upper_press'))
    offsets = [np.array(getattr(joint, f'GetLocalPos{i}Attr')().Get()) for i in (0, 1)]
    local_rotations = []
    for i in (0, 1):
        q = getattr(joint, f'GetLocalRot{i}Attr')().Get()
        local_rotations.append(rotation([q.GetReal(), *q.GetImaginary()]))
    axis_index = 'XYZ'.index(joint.GetAxisAttr().Get())
    indices = [instance.indices[n] for n in ('stapler.base', 'stapler.upper')]
    cache = {'time': None}

    def sample():
        if cache['time'] == ctx.session.time:
            return cache['data']
        positions, rotations, _ = instance.environment_states()
        velocities = instance.view.view.get_velocities().detach().cpu().numpy().reshape(ctx.num_envs, len(instance.body_names), 6)
        data = {}
        for env in range(ctx.num_envs):
            prefix = (f'env_{env}.' if env else '') + instance.name + '.'
            for axis, label in enumerate('xyz'):
                data[prefix + 'momentum.' + label] = float(sum(instance.mass[i] * velocities[env, i, axis] for i in range(len(instance.body_names))))
            if env == 0:
                continue
            for i, name in enumerate(instance.body_names):
                quat = Rotation.from_matrix(rotations[env, i]).as_quat()[[3, 0, 1, 2]]
                for field, value, axes in [('position', positions[env, i], 'xyz'), ('quaternion', quat, 'wxyz'),
                                          ('linear_velocity', velocities[env, i, :3], 'xyz'), ('angular_velocity', velocities[env, i, 3:], 'xyz')]:
                    data.update({prefix + name + '.' + field + '.' + axis: float(v) for axis, v in zip(axes, value)})
            base, upper = indices
            anchors = [positions[env, body] + rotations[env, body] @ offsets[i] for i, body in enumerate(indices)]
            frames = [rotations[env, body] @ local_rotations[i] for i, body in enumerate(indices)]
            front = positions[env, upper] + rotations[env, upper] @ CHANNEL_FRONT
            data[prefix + 'upper_press.position'] = float(Rotation.from_matrix(frames[0].T @ frames[1]).as_rotvec()[axis_index])
            data[prefix + 'upper_press.velocity'] = float((velocities[env, upper, 3:] - velocities[env, base, 3:]) @ frames[0][:, axis_index])
            data[prefix + 'anchor_error'] = float(np.linalg.norm(anchors[1] - anchors[0]))
            data[prefix + 'end_gap'] = float((rotations[env, base].T @ (front - positions[env, base]))[2] - ANVIL_TOP)
        cache.update(time=ctx.session.time, data=data)
        return data

    fields = [('position', 'xyz', 'm'), ('quaternion', 'wxyz', '1'),
              ('linear_velocity', 'xyz', 'm/s'), ('angular_velocity', 'xyz', 'rad/s')]
    for env in range(ctx.num_envs):
        prefix = (f'env_{env}.' if env else '') + instance.name + '.'
        columns = [(prefix + 'momentum.' + a, 'kg·m/s') for a in 'xyz']
        if env:
            columns += [(prefix + n + '.' + f + '.' + axis, unit) for n in instance.body_names for f, axes, unit in fields for axis in axes]
            columns += [(prefix + 'upper_press.position', 'rad'), (prefix + 'upper_press.velocity', 'rad/s'),
                        (prefix + 'anchor_error', 'm'), (prefix + 'end_gap', 'm')]
        for column, unit in columns:
            ctx.observe(column, lambda key=column: sample()[key], unit=unit)


def rows_for_environment(ctx, env):
    """Normalize a replica's recorded columns for the same acceptance checks."""
    if env == 0:
        return [{k: v for k, v in row.items() if not k.startswith('env_')} for row in ctx.rows]
    prefix = f'env_{env}.'
    return [{'time': row['time'], **{k[len(prefix):]: v for k, v in row.items() if k.startswith(prefix)},
             **{k: v for k, v in row.items() if k.startswith('press.')}} for row in ctx.rows]


def gpu_endpoint_contacts(ctx, contacts):
    """GPU supports body filters, so locate native points on the existing endpoint.

    The view is created lazily after initialization using the existing simulation
    view. No physical prim, fixture, or contact property is changed at runtime.
    """
    model = json.loads((ctx.asset_path.parent / 'model.json').read_text())
    worlds, owners, bounds = {}, {}, {}
    for node in model['nodes']:
        name, parent = node['name'], node['parent']
        worlds[name] = worlds.get(parent, np.eye(4)) @ np.asarray(node['matrix'])
        owners[name] = name if node['rigid'] else owners.get(parent)
        if node.get('role') == 'collision' and ('.anvil.' in name or '.channel.' in name):
            assert node['shape'] == 'BOX', 'Revisit endpoint localization if geometry changes'
            vertices = np.c_[node['vertices'], np.ones(len(node['vertices']))]
            local = vertices @ (np.linalg.inv(worlds[owners[name]]) @ worlds[name]).T
            bounds[name] = (local[:, :3].min(axis=0), local[:, :3].max(axis=0))
    anvil = bounds['stapler.base.anvil.collision']
    channels = [b for name, b in bounds.items() if '.channel.' in name]
    # Existing per-shape contact offset is 0.05 mm; both surfaces contribute.
    margin = 2 * .00005
    cache = {'view': None, 'time': None, 'impulses': np.zeros(ctx.num_envs)}

    def read(env):
        if cache['view'] is None:
            from isaacsim.core.simulation_manager import SimulationManager
            upper, base = (ctx.asset.paths[n] for n in ('stapler.upper', 'stapler.base'))
            cache['view'] = SimulationManager.get_physics_sim_view().create_rigid_contact_view(
                [ctx.session.replica_path(upper, i) for i in range(ctx.num_envs)],
                [[ctx.session.replica_path(base, i)] for i in range(ctx.num_envs)],
                max_contact_data_count=64 * ctx.num_envs)
            assert cache['view'].sensor_count == ctx.num_envs
        if cache['time'] != ctx.session.time:
            cache['time'] = ctx.session.time
            cache['impulses'] = np.zeros(ctx.num_envs)
            if ctx.session.steps:
                impulses, points, _, _, counts, starts = [v.detach().cpu().numpy().copy()
                    for v in cache['view'].get_contact_data(1.)]  # dt=1 returns N·s
                positions, rotations, _ = ctx.asset.environment_states()
                bi, ui = (ctx.asset.indices[n] for n in ('stapler.base', 'stapler.upper'))
                assert int(counts.sum()) < 64 * ctx.num_envs, 'contact buffer capacity exhausted'
                for e in range(ctx.num_envs):
                    start, count = int(starts[e, 0]), int(counts[e, 0])
                    for i in range(start, start + count):
                        point = points[i] - ctx.session.offsets[e]
                        bp = rotations[e, bi].T @ (point - positions[e, bi])
                        up = rotations[e, ui].T @ (point - positions[e, ui])
                        on_anvil = (np.all(bp >= anvil[0] - margin) and np.all(bp <= anvil[1] + margin)
                                    and abs(bp[2] - anvil[1][2]) <= margin)
                        on_channel = any(np.all(up >= lo - margin) and np.all(up <= hi + margin) for lo, hi in channels)
                        if on_anvil and on_channel:
                            impulse = abs(float(impulses[i, 0]))
                            cache['impulses'][e] += impulse
                            contacts.append({'time_s': ctx.session.time, 'impulse_N_s': impulse, 'environment': e})
        return cache['impulses'][env]

    return read


def setup(ctx, *, gravity=(0, 0, -9.81), ground=False, dt=.004):
    ctx.simulation(dt=dt, gravity=gravity)
    from pxr import PhysxSchema, UsdPhysics
    scene = next(p for p in ctx.session.stage.Traverse() if p.IsA(UsdPhysics.Scene))
    # Native numerical integration setting, required in consumer TGS scenes too.
    # It adds no spring/holding force; the return drive lives in the asset USD.
    PhysxSchema.PhysxSceneAPI.Apply(scene).CreateEnableExternalForcesEveryIterationAttr(True)
    # Keep the report's configuration mirror consistent with the native value.
    ctx.world.cfg.physx.enable_external_forces_every_iteration = True
    if ground:
        ctx.ground(static_friction=.40, dynamic_friction=.30, contact_offset=.00005)
    ctx.camera(position=(-.14, -.26, .16), target=(0, 0, .022), focal_length=32)
    ctx.experiment_details = {
        'asset_pose_writes_during_experiment': False,
        'mechanism': 'native hinge with approved angular return drive',
        'force_columns': 'applied external loads, not contact/fixture reactions',
        'press_point_upper_local_m': PRESS_POINT,
        'parameters': 'uncalibrated engineering estimates; mass base/upper=35/25 g',
        'solver_exception': 'Asset TGS 64/4; GPU framework scene minimum 64/16 with CCD disabled; native external forces every iteration enabled',
        'external_forces_every_iteration': bool(PhysxSchema.PhysxSceneAPI(scene).GetEnableExternalForcesEveryIterationAttr().Get()),
    }


def hinge_checks(ctx, instances):
    assert all(np.isfinite(v) for row in ctx.rows for v in row.values())
    results = {}
    for env in range(ctx.num_envs):
        rows = rows_for_environment(ctx, env)
        for instance in instances:
            q = [r[f'{instance.name}.upper_press.position'] for r in rows]
            anchor = max(r[f'{instance.name}.anchor_error'] for r in rows)
            gap = min(r[f'{instance.name}.end_gap'] for r in rows)
            results[f'env_{env}.{instance.name}'] = {'q_min_rad': min(q), 'q_max_rad': max(q),
                                      'max_anchor_error_m': anchor, 'min_end_gap_m': gap}
    ctx.experiment_details['hinge_checks'] = results
    ctx.metric('anchor_error', max(r['max_anchor_error_m'] for r in results.values()) * 1000, unit='mm', label='各环境最大锚点误差')
    for result in results.values():
        assert result['q_min_rad'] >= LOWER - .01 and result['q_max_rad'] <= .01, results
        assert result['max_anchor_error_m'] < .0002, results
        assert result['min_end_gap_m'] > -.0003, results


def _press_cycles(ctx: TestContext, *, fixed):
    # GPU table support needs the validated 2 ms numerical exception.
    # Keep CPU and the fixed-base control at their existing 4 ms settings.
    dt = .002 if ctx.physics == 'gpu' and not fixed else .004
    setup(ctx, ground=not fixed, dt=dt)
    ctx.experiment_details['timestep_condition'] = {
        'dt_s': dt, 'baseline_dt_s': .004,
        'exception': 'GPU table_press only; 4 ms failed the unchanged tail-speed gate' if dt == .002 else None,
        'mechanism_and_acceptance_unchanged': True,
    }
    import carb
    # Isaac Lab disables CPU contact-report processing by default. Opt in
    # before initialization for this measurement; this does not create forces.
    carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
    from pxr import PhysxSchema, PhysicsSchemaTools
    from omni.physx import get_physx_simulation_interface
    PhysxSchema.PhysxContactReportAPI.Apply(
        ctx.session.stage.GetPrimAtPath(ctx.asset.paths['stapler.upper'])).CreateThresholdAttr(0)
    contacts = []
    contact_state = {'time_s': -1., 'impulse_N_s': np.zeros(ctx.num_envs)}

    def contact_report(headers, data):
        if ctx.session.time < contact_state['time_s']:
            contacts.clear()  # warmup/reset starts a new physical experiment
        if ctx.session.time != contact_state['time_s']:
            contact_state.update(time_s=ctx.session.time, impulse_N_s=np.zeros(ctx.num_envs))
        for header in headers:
            pair = [str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                    str(PhysicsSchemaTools.intToSdfPath(header.collider1))]
            if not any('/upper/channel/' in p for p in pair) or not any('/base/anvil/' in p for p in pair):
                continue
            env_ids = [int(p.split('/env_')[1].split('/')[0]) for p in pair]
            assert env_ids[0] == env_ids[1], 'cross-environment channel/anvil contact'
            env = env_ids[0]
            for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                impulse = float(np.linalg.norm(list(data[i].impulse)))
                contacts.append({'time_s': ctx.session.time, 'impulse_N_s': impulse, 'environment': env})
                contact_state['impulse_N_s'][env] += impulse

    subscription = (get_physx_simulation_interface().subscribe_contact_report_events(contact_report)
                    if ctx.physics == 'cpu' else None)
    gpu_contact = gpu_endpoint_contacts(ctx, contacts) if ctx.physics == 'gpu' else None
    ctx.initial(joints={'upper_press': 0})
    observe(ctx, ctx.asset)
    for env in range(ctx.num_envs):
        prefix = f'env_{env}.' if env else ''
        getter = (lambda e=env: gpu_contact(e)) if gpu_contact else (
            lambda e=env: contact_state['impulse_N_s'][e] if abs(contact_state['time_s'] - (ctx.session.time - ctx.dt)) < 1e-8 else 0.)
        ctx.observe(prefix + 'channel_anvil.contact_impulse', getter, unit='N·s')
    ctx.plots('press.force.z', 'asset.upper_press.position', 'asset.upper_press.velocity',
              'asset.end_gap', 'asset.anchor_error', 'channel_anvil.contact_impulse')
    ctx.video_annotation = lambda: f'Base {"fixed" if fixed else "free on table"} | three 3 N press/release cycles | native return drive'
    ctx.experiment_details.update(purpose='three force-driven press/release cycles; check lower stop, contact and return',
                                  contact_report_processing_enabled=True,
                                  contact_measurement=('GPU native body-pair contact normal impulses localized on anvil top and channel boxes; 0.1 mm combined contact margin'
                                                       if gpu_contact else 'CPU native collider-pair contact impulse vector magnitude'),
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
    per_environment, failures = [], []
    for env in range(ctx.num_envs):
        rows = rows_for_environment(ctx, env)
        try:
            per_environment.append(press_checks(ctx, contacts, rows, env))
        except AssertionError as error:
            failures.append({'environment': env, 'error': str(error),
                             'diagnostics': press_failure_details(rows, contacts, env, ctx.dt)})
    if failures:
        ctx.experiment_details['environment_press_failures'] = failures
        # Keep a compact log as well as the runner's failed-run diagnostics.
        print('STAPLER_PRESS_FAILURE_DIAGNOSTICS ' + json.dumps(failures), flush=True)
        raise AssertionError(f'Press checks failed in environments {[f["environment"] for f in failures]}; see STAPLER_PRESS_FAILURE_DIAGNOSTICS in run.log')
    subscription = None
    ctx.experiment_details.update(per_environment[0])
    ctx.experiment_details['environment_press_checks'] = per_environment
    ctx.metric('tail_angular_speed', max(c['tail_max_abs_velocity_rad_s'] for d in per_environment for c in d['press_cycles']), unit='rad/s', label='各环境最大回位末速')
    ctx.metric('pressed_angle', min(c['loaded_min_q_rad'] for d in per_environment for c in d['press_cycles']), unit='rad', label='各环境最小按压角')
    ctx.metric('velocity_rmse', max(d['ramp_velocity_consistency']['rmse_rad_s'] for d in per_environment), unit='rad/s', label='最大角速一致性误差')
    ctx.metric('contact_impulse', max(d['endpoint_contact']['peak_point_impulse_N_s'] for d in per_environment), unit='N·s', label='最大原生接触点冲量')
    if not fixed:
        ground_and_settling(ctx, [ctx.asset], 4.3)


def press_failure_details(rows, contacts, env, dt):
    cycles = []
    for start, withdraw, finish in ((.4, 1.24, 1.8), (1.8, 2.64, 3.2), (3.2, 4.04, 4.8)):
        tail_indices = [i for i, row in enumerate(rows) if finish - .2 <= row['time'] <= finish]
        peak = max(tail_indices, key=lambda i: abs(rows[i]['asset.upper_press.velocity']))
        row, previous = rows[peak], rows[peak - 1]
        cycle_contacts = [c for c in contacts if c['environment'] == env
                          and start + .6 <= c['time_s'] <= withdraw - .04]
        cycles.append({
            'tail_window_s': [finish - .2, finish],
            'tail_q_span_rad': float(np.ptp([rows[i]['asset.upper_press.position'] for i in tail_indices])),
            'tail_velocity_range_rad_s': [float(f(rows[i]['asset.upper_press.velocity'] for i in tail_indices)) for f in (min, max)],
            'peak_time_s': row['time'], 'peak_q_rad': row['asset.upper_press.position'],
            'peak_native_velocity_rad_s': row['asset.upper_press.velocity'],
            'peak_backward_pose_rate_rad_s': (row['asset.upper_press.position'] - previous['asset.upper_press.position']) / dt,
            'peak_base_angular_velocity_rad_s': [row[f'asset.stapler.base.angular_velocity.{a}'] for a in 'xyz'],
            'peak_upper_angular_velocity_rad_s': [row[f'asset.stapler.upper.angular_velocity.{a}'] for a in 'xyz'],
            'loaded_endpoint_samples': len(cycle_contacts),
            'loaded_peak_point_impulse_N_s': max((c['impulse_N_s'] for c in cycle_contacts), default=0.),
        })
    return {'cycles': cycles, 'contact_samples': sum(c['environment'] == env for c in contacts)}


def press_checks(ctx, contacts, rows, env):
    # Away from contact/return impacts, native angular speed must agree with
    # the measured coordinate change. Default TGS force integration violated
    # even the sign of this relationship; tail-speed checks alone missed it.
    rates = [(b['asset.upper_press.position'] - a['asset.upper_press.position']) / ctx.dt
             for a, b in zip(rows, rows[1:]) if .5 <= b['time'] <= .8]
    speeds = [r['asset.upper_press.velocity'] for r in rows if .5 <= r['time'] <= .8]
    rate_error = float(np.sqrt(np.mean((np.asarray(rates) - speeds) ** 2)))
    consistency = {
        'window_s': [.5, .8], 'pose_rate_mean_rad_s': float(np.mean(rates)),
        'native_velocity_mean_rad_s': float(np.mean(speeds)),
        'rmse_rad_s': rate_error, 'rmse_limit_rad_s': .05,
        'method': 'backward angle difference versus end-of-step native angular velocity during smooth first ramp',
    }
    assert rate_error < .05, consistency
    cycles = []
    for start, withdraw, finish in ((.4, 1.24, 1.8), (1.8, 2.64, 3.2), (3.2, 4.04, 4.8)):
        loaded = [r for r in rows if start + .6 <= r['time'] <= withdraw - .04]
        returned = [r for r in rows if withdraw <= r['time'] <= finish]
        min_q = min(r['asset.upper_press.position'] for r in loaded)
        final_q = returned[-1]['asset.upper_press.position']
        cycles.append({'loaded_min_q_rad': min_q, 'return_q_rad': final_q,
                       'return_max_abs_velocity_rad_s': max(abs(r['asset.upper_press.velocity']) for r in returned),
                       'tail_max_abs_velocity_rad_s': max(abs(r['asset.upper_press.velocity'])
                                                         for r in returned if r['time'] >= finish - .2)})
        assert min_q <= LOWER + .01, cycles
        assert final_q >= -.01, cycles
        assert max(r['channel_anvil.contact_impulse'] for r in loaded) > 0, 'no channel/anvil contact during loaded cycle'
    env_contacts = [c for c in contacts if c['environment'] == env]
    contact = {
        'samples': len(env_contacts),
        'peak_point_impulse_N_s': max((c['impulse_N_s'] for c in env_contacts), default=0.),
        'measurement': ctx.experiment_details['contact_measurement'] + '; excludes joint and fixture reactions',
    }
    assert env_contacts and max(c['impulse_N_s'] for c in env_contacts) > 0, 'no channel/anvil contact impulse'
    assert max(abs(r['asset.upper_press.position']) for r in rows if r['time'] >= 4.6) < .01
    assert all(c['tail_max_abs_velocity_rad_s'] < .05 for c in cycles), cycles
    return {'ramp_velocity_consistency': consistency, 'press_cycles': cycles, 'endpoint_contact': contact}


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
    hinge_checks(ctx, [ctx.asset, idle])
    results = [free_checks(ctx, idle, rows_for_environment(ctx, env)) for env in range(ctx.num_envs)]
    ctx.experiment_details.update(results[0])
    ctx.experiment_details['environment_reaction_checks'] = results
    ctx.metric('momentum_error', max(r['relative_momentum_error'] for r in results), unit='1', label='各环境最大动量相对误差')
    ctx.metric('idle_speed', max(r['idle_max_speed_m_s'] for r in results) * 1000, unit='mm/s', label='各环境闲置实例最大速度')


def free_checks(ctx, idle, rows):
    impulse = np.array([sum(r['press.force.' + a] for r in rows[1:]) * ctx.dt for a in 'xyz'])
    momentum = np.array([rows[-1]['asset.momentum.' + a] - rows[0]['asset.momentum.' + a] for a in 'xyz'])
    error = float(np.linalg.norm(momentum - impulse) / np.linalg.norm(impulse))
    idle_speed = max(np.linalg.norm([r[f'idle.{n}.linear_velocity.{a}'] for a in 'xyz'])
                     for r in rows for n in idle.body_names)
    base_speed = float(np.linalg.norm([rows[-1]['asset.stapler.base.linear_velocity.' + a] for a in 'xyz']))
    details = dict(purpose='free base reacts through hinge; total momentum matches impulse',
        fixture=None, impulse_N_s=impulse.tolist(), momentum_change_kg_m_s=momentum.tolist(),
        relative_momentum_error=error, idle_max_speed_m_s=float(idle_speed), final_base_speed_m_s=base_speed)
    assert error < .01 and idle_speed < 1e-6 and base_speed > .001, details
    return details


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
    results = [ground_check(ctx, instances, tail_start, rows_for_environment(ctx, env)) for env in range(ctx.num_envs)]
    ctx.experiment_details.update(results[0])
    ctx.experiment_details['environment_ground_checks'] = results
    ctx.metric('ground_penetration', max(r['ground_geometry']['maximum_geometry_penetration_m'] for r in results) * 1000, unit='mm', label='各环境最大穿地')
    ctx.metric('tail_speed', max(v['max_tail_speed_m_s'] for r in results for v in r['settling'].values()) * 1000, unit='mm/s', label='各环境最大末段线速')
    ctx.metric('tail_position_span', max(v['max_tail_position_span_m'] for r in results for v in r['settling'].values()) * 1000, unit='mm', label='各环境最大末段位移跨度')


def ground_check(ctx, instances, tail_start, rows):
    from table_1000.physics.test import ground_clearance
    ground = ground_clearance(json.loads((ctx.asset_path.parent / 'model.json').read_text()),
                              rows, [instance.name for instance in instances], 0)
    tail = [r for r in rows if r['time'] >= tail_start]
    settled = {}
    for instance in instances:
        for name in instance.body_names:
            prefix = f'{instance.name}.{name}'
            speed = max(np.linalg.norm([r[f'{prefix}.linear_velocity.{a}'] for a in 'xyz']) for r in tail)
            position = np.array([[r[f'{prefix}.position.{a}'] for a in 'xyz'] for r in tail])
            settled[prefix] = {'max_tail_speed_m_s': float(speed),
                               'max_tail_position_span_m': float(np.ptp(position, axis=0).max())}

    assert ground['maximum_geometry_penetration_m'] < .002, ground
    assert all(v['max_tail_speed_m_s'] < .01 and v['max_tail_position_span_m'] < .001
               for v in settled.values()), settled
    return {'ground_geometry': ground, 'settling': settled}


def lifecycle_reset_reload(ctx: TestContext):
    """Same-stage reset/reload audit; tensor restoration occurs only while reset."""
    import torch
    from pxr import UsdGeom, UsdPhysics
    setup(ctx, gravity=(0, 0, 0), dt=.002 if ctx.physics == 'gpu' else .004)
    ctx.initial(position=(0, -.05, .12), joints={'upper_press': 0})
    idle = ctx.spawn(ctx.asset_path, name='idle')
    ctx.initial(idle, position=(0, .05, .12), joints={'upper_press': 0})
    ctx.camera(position=(-.27, -.48, .29), target=(0, 0, .11), focal_length=28)
    ctx.ensure_initialized()
    stage = ctx.session.stage
    stage_id = stage.GetRootLayer().identifier
    original_render = ctx.render
    ctx.render = False

    def snapshot():
        return {n: (a.view.view.get_transforms().clone(), a.view.view.get_velocities().clone())
                for n, a in ctx.session.instances.items()}

    def restore(saved):
        # Called after native stop/reset, never inside force-driven integration.
        for n, (poses, speeds) in saved.items():
            a = ctx.session.instances[n]
            ids = torch.arange(len(poses), dtype=torch.int32, device=poses.device)
            a.view.view.set_transforms(poses.clone(), ids)
            a.view.view.set_velocities(speeds.clone(), ids)
        ctx.world.forward()
        ctx.session.refresh()

    def configure():
        ctx.rows.clear()
        ctx.observers.clear()
        ctx.units.clear()
        ctx.actions.clear()
        ctx.action_values.clear()
        ctx.session.time = 0.
        ctx.session.steps = ctx.session.callback_steps = 0
        ctx.session.error = None
        for a in (ctx.asset, idle):
            observe(ctx, a)
        ctx.force('press', ctx.asset.body('stapler.upper'), point_frame='body', keyframes=[
            {'time': 0, 'force': (0, 0, 0), 'point': PRESS_POINT},
            {'time': .1, 'force': (0, 0, 0)}, {'time': .12, 'force': (0, 0, -.03)},
            {'time': .28, 'force': (0, 0, -.03)}, {'time': .3, 'force': (0, 0, 0)},
            {'time': .8, 'force': (0, 0, 0)}])
        ctx.plots('press.force.z', 'asset.upper_press.position', 'asset.anchor_error',
                  'asset.momentum.z', 'idle.stapler.base.linear_velocity.z')

    def maximum_difference(a, b):
        return max(float(torch.max(torch.abs(a[n][i] - b[n][i])).cpu())
                   for n in a for i in (0, 1))

    def paused_reset():
        ctx.session.active = False
        ctx.world._disable_app_control_on_stop_handle = True
        ctx.world.stop()
        ctx.world.reset()
        ctx.session.initialize()
        assert ctx.session.stage.GetRootLayer().identifier == stage_id

    saved = snapshot()
    configure()
    ctx.run(.8)
    hinge_checks(ctx, [ctx.asset, idle])
    first_checks = [free_checks(ctx, idle, rows_for_environment(ctx, e)) for e in range(ctx.num_envs)]
    first = snapshot()
    first_rows = list(ctx.rows)
    assert ctx.session.callback_steps == ctx.session.steps
    paused_reset()
    restore(saved)
    reset_error = maximum_difference(saved, snapshot())
    assert reset_error < 1e-6, reset_error
    configure()
    ctx.run(.8)
    hinge_checks(ctx, [ctx.asset, idle])
    second_checks = [free_checks(ctx, idle, rows_for_environment(ctx, e)) for e in range(ctx.num_envs)]
    repeat_error = maximum_difference(first, snapshot())
    trace_error = max(abs(a[k] - b[k]) for a, b in zip(first_rows, ctx.rows) for k in a)
    # Solver warm state need not reproduce bitwise after native reset.
    # Validate each segment with the original hinge/momentum/idle gates above;
    # report trajectory differences by unit instead of a mixed-unit tolerance.
    trace_differences = {k: max(abs(a[k] - b[k]) for a, b in zip(first_rows, ctx.rows))
                         for k in first_rows[0]}
    assert len(first_rows) == len(ctx.rows)
    assert ctx.session.callback_steps == ctx.session.steps

    # Remove and re-reference the asset only with the native timeline stopped.
    ctx.session.active = False
    ctx.world._disable_app_control_on_stop_handle = True
    ctx.world.stop()
    paths = {n: [ctx.session.replica_path(a.path, e) for e in range(ctx.num_envs)]
             for n, a in ctx.session.instances.items()}
    for p in paths['asset']:
        stage.RemovePrim(p)
    assert all(not stage.GetPrimAtPath(p) for p in paths['asset'])
    assert all(stage.GetPrimAtPath(p) for p in paths['idle'])
    ctx.session.unregister(ctx.asset.path)
    for p in paths['idle']:
        stage.RemovePrim(p)
    ctx.session.unregister(idle.path)
    assert not ctx.session.instances
    for n, group in paths.items():
        for p in group:
            UsdGeom.Xform.Define(stage, p).GetPrim().GetReferences().AddReference(str(ctx.asset_path))
        ctx.session.register(group[0], n)
    ctx.asset, idle = (ctx.session.instances[n] for n in ('asset', 'idle'))
    ctx.world.reset()
    ctx.session.initialize()
    restore(saved)
    reload_error = maximum_difference(saved, snapshot())
    assert reload_error < 1e-6, reload_error
    for a in (ctx.asset, idle):
        assert a.body_names == ['stapler.base', 'stapler.upper']
        assert a.joint_names == ['upper_press']
        assert len(a.view.view.get_transforms()) == 2 * ctx.num_envs
        assert not a.articulation_root
        assert UsdPhysics.Joint(a.joint('upper_press')).GetJointEnabledAttr().Get()
    configure()
    ctx.render = original_render
    ctx.experiment_started = None
    ctx.integrating_seconds = 0.
    ctx.video_annotation = lambda: 'Reloaded same stage | native reset checked in every environment | loaded and idle instances'
    ctx.run(.8)
    hinge_checks(ctx, [ctx.asset, idle])
    reload_checks = [free_checks(ctx, idle, rows_for_environment(ctx, e)) for e in range(ctx.num_envs)]
    assert ctx.session.callback_steps == ctx.session.steps
    ctx.experiment_details['lifecycle'] = {
        'same_stage': ctx.session.stage.GetRootLayer().identifier == stage_id,
        'environments_checked': ctx.num_envs, 'reset_state_max_scalar_error': reset_error,
        'repeat_final_max_scalar_error': repeat_error, 'repeat_trace_max_scalar_error': trace_error,
        'repeat_trace_differences': trace_differences,
        'repeat_comparison': 'original per-segment hinge, momentum and idle-speed acceptance; scalar difference is diagnostic only',
        'reload_state_max_scalar_error': reload_error, 'removal_preserved_sibling': True,
        'all_environment_prims_removed_and_referenced_again': True,
        'first_checks': first_checks, 'reset_checks': second_checks, 'reload_checks': reload_checks,
        'state_writes': 'only stopped/reset initialization; no pose writes during integration',
        'media_scope': 'final reloaded segment; first and reset segments numerically audited',
        'performance_scope': 'lifecycle diagnostic, exclude from steady-state benchmark',
    }
    ctx.metric('reset_error', reset_error, unit='1', label='复位最大状态标量误差')
    ctx.metric('repeat_error', trace_error, unit='1', label='同场景重复轨迹最大标量误差')
    ctx.metric('reload_error', reload_error, unit='1', label='重载最大状态标量误差')


TESTS = {'press_release': press_release, 'table_press': table_press,
         'free_reaction': free_reaction, 'drop_states': drop_states,
         'lifecycle_reset_reload': lifecycle_reset_reload}
