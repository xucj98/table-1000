"""Native three-drawer dynamics. All parameters are simulation design values.

A: gradual axial pull and release for each drawer; B: both travel stops under
continued force; C: free cabinet on its rubber feet; D: gravity with a native
housing fixture. No animation, drawer velocity preload, or behavior plugin.
"""
from functools import partial
import math

from table_1000.physics.testing import TestContext


TRAVEL = .173


def _view(ctx, *, wide=False):
    ctx.camera(position=(1.25, -1.7, .95) if wide else (1.1, -1.5, .85),
               target=(0, -.14 if wide else -.07, .14))


def _joint(ctx, index):
    name = f'cabinet.drawer{index}.slide'
    ctx.observe_joint(ctx.asset, name)
    return name


def pull_release(ctx: TestContext, index):
    _view(ctx)
    ctx.video_annotation = lambda: f'Housing fixed | drawer {index} | ramp pull then release at 2.1 s'
    joint = _joint(ctx, index)
    drawer = ctx.asset.body(f'cabinet.drawer{index}')
    ctx.initial(joints={joint: .045})
    ctx.plots('pull.force.y', f'asset.{joint}.position', f'asset.{joint}.velocity')
    ctx.experiment_details = {
        'purpose': 'A: ramp pull, withdraw force, observe deceleration and stop',
        'drawer': index, 'initial_joint_m': .045,
        'native_resistance': {'static_joint_friction_N': .50, 'dynamic_joint_friction_N': .35,
            'viscous_drive_damping_N_s_per_m': 3,
            'note': 'constant native joint-axis friction; damping is additional viscous resistance'},
        'fixture': 'housing held by a fixed articulation root',
        'force_schedule_N': '0 to 0.80 over 1.5 s; withdraw over 0.1 s; observe to 5 s',
    }
    ctx.clamp_root(ctx.asset.body('cabinet.housing'))
    ctx.force('pull', drawer, keyframes=[
        {'time': 0, 'force': (0, 0, 0)},
        {'time': .5, 'force': (0, 0, 0)},
        {'time': 2, 'force': (0, -.80, 0)},
        {'time': 2.1, 'force': (0, 0, 0)},
        {'time': 5, 'force': (0, 0, 0)},
    ])
    ctx.run(5)
    assert abs(ctx.rows[0][f'asset.{joint}.position'] - .045) < 1e-6, 'initial drawer opening was not applied'


def travel_stops(ctx: TestContext):
    _view(ctx)
    ctx.video_annotation = lambda: 'Housing fixed | sustained opening force to 2 s, closing force to 4 s'
    joint = _joint(ctx, 1)
    ctx.initial(joints={joint: .085})
    ctx.plots('pull.force.y', f'asset.{joint}.position', f'asset.{joint}.velocity')
    ctx.experiment_details = {
        'purpose': 'B: sustained opening and closing force at both native travel limits',
        'initial_joint_m': .085, 'limits_m': [0, TRAVEL],
        'fixture': 'housing held by a fixed articulation root',
        'force_schedule_N': '2 N opening until 2 s, reverse to 2 N closing by 2.2 s, hold until 4 s',
    }
    ctx.clamp_root(ctx.asset.body('cabinet.housing'))
    ctx.force('pull', ctx.asset.body('cabinet.drawer1'), keyframes=[
        {'time': 0, 'force': (0, -2, 0)},
        {'time': 2, 'force': (0, -2, 0)},
        {'time': 2.2, 'force': (0, 2, 0)},
        {'time': 4, 'force': (0, 2, 0)},
        {'time': 4.1, 'force': (0, 0, 0)},
        {'time': 5, 'force': (0, 0, 0)},
    ])
    ctx.run(5)
    assert abs(ctx.rows[0][f'asset.{joint}.position'] - .085) < 1e-6, 'initial drawer opening was not applied'


def pull_cabinet(ctx: TestContext):
    _view(ctx, wide=True)
    ctx.video_annotation = lambda: 'Housing free | real rubber feet on ground | drawer3 pull'
    ctx.ground(static_friction=.35, dynamic_friction=.35, contact_offset=.0001)
    # Full poses also allow the common result writer to measure ground overlap.
    for name in ctx.asset.body_names:
        ctx.observe_body(ctx.asset.body(name), fields=('position', 'quaternion'))
    joint = _joint(ctx, 3)
    ctx.plots('pull.force.y', f'asset.{joint}.position', 'asset.cabinet.housing.position.y')
    ctx.experiment_details = {
        'purpose': 'C: cabinet free on its real rubber feet, gradual drawer3 pull',
        'fixture': None, 'initial_joint_m': 0,
        'force_schedule_N': 'settle 0.5 s, ramp to 8 N by 3.5 s, withdraw by 3.7 s; observe to 7 s',
        'note': 'no forced ordering between drawer reaching its stop and housing sliding',
    }
    ctx.force('pull', ctx.asset.body('cabinet.drawer3'), keyframes=[
        {'time': 0, 'force': (0, 0, 0)},
        {'time': .5, 'force': (0, 0, 0)},
        {'time': 3.5, 'force': (0, -8, 0)},
        {'time': 3.7, 'force': (0, 0, 0)},
        {'time': 7, 'force': (0, 0, 0)},
    ])
    ctx.run(7)


def gravity_tilt(ctx: TestContext):
    """Slowly incline the housing; observe drawer motion, never lock drawers."""
    ctx.initial(position=(0, 0, .22))
    ctx.camera(position=(1.25, -1.65, 1.0), target=(0, -.15, .3))
    housing = ctx.asset.body('cabinet.housing')
    for index in (1, 2, 3):
        _joint(ctx, index)
    ctx.observe('housing.angle', lambda: math.atan2(housing.state['rotation'][2, 1],
                                                  housing.state['rotation'][1, 1]), unit='rad')
    ctx.plots('housing.angle', *(f'asset.cabinet.drawer{i}.slide.position' for i in (1, 2, 3)))
    ctx.video_annotation = lambda: f"Horizontal start | housing tilt {math.degrees(math.atan2(housing.state['rotation'][2, 1], housing.state['rotation'][1, 1])):.1f} deg | three drawers free | gravity only"
    original = ctx.session.actions
    grip = ctx.clamp_root(housing)
    def actions(t, dt):
        original(t, dt)
        u = min(1., max(0., (t-1)/15))
        angle = math.radians(45)*(u*u*(3-2*u))
        grip.move(quaternion=(math.cos(angle/2), math.sin(angle/2), 0, 0))
    ctx.session.actions = actions
    ctx.run(18)
    ctx.session.actions = original
    starts = {}
    for i in (1, 2, 3):
        column = f'asset.cabinet.drawer{i}.slide.position'
        initial = ctx.rows[0][column]
        row = next((r for r in ctx.rows if r[column]-initial >= .0005), None)
        starts[f'drawer{i}'] = None if row is None else {
            'time_s': row['time'], 'angle_degrees': math.degrees(row['housing.angle']),
            'relative_displacement_m': row[column]-initial}
    ctx.experiment_details = {
        'purpose': 'continuous gravity inclination from horizontal to 45 degrees',
        'fixture': 'fixed articulation root moved through tensors; drawers always free',
        'usd_pose_writes_during_experiment': False, 'external_drawer_forces': None,
        'initial_joints_m': [ctx.rows[0][f'asset.cabinet.drawer{i}.slide.position'] for i in (1, 2, 3)],
        'schedule': 'horizontal 1 s, smooth 45-degree tilt over 15 s, hold 2 s',
        'onset_displacement_m': .0005, 'onset': starts,
        'design_target_minimum_angle_degrees': 15,
        'note': 'onset defined by 0.5 mm relative displacement, not a calibrated static friction coefficient',
    }

    assert all(value is not None and value['angle_degrees'] >= 15 for value in starts.values()), \
        'a drawer slid before the 15-degree design target or never opened'


TESTS = {
    **{f'pull_release_drawer{i}': partial(pull_release, index=i) for i in (1, 2, 3)},
    'travel_stops': travel_stops,
    'pull_cabinet': pull_cabinet,
    'gravity_tilt': gravity_tilt,
}
