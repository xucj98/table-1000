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
    ctx.camera(position=(.85, -1.15, .68) if wide else (.66, -.86, .51),
               target=(0, -.25 if wide else -.09, .14))


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
        'native_resistance': {'joint_friction_coefficient': .25,
            'viscous_drive_damping_N_s_per_m': 3,
            'note': 'load-dependent native articulation friction; damping is not static friction'},
        'fixture': 'housing held by a test-only native fixed joint',
        'force_schedule_N': '0 to 0.30 over 1.5 s; withdraw over 0.1 s; observe to 5 s',
    }
    with ctx.fixed(ctx.asset.body('cabinet.housing')):
        ctx.force('pull', drawer, keyframes=[
            {'time': 0, 'force': (0, 0, 0)},
            {'time': .5, 'force': (0, 0, 0)},
            {'time': 2, 'force': (0, -.30, 0)},
            {'time': 2.1, 'force': (0, 0, 0)},
            {'time': 5, 'force': (0, 0, 0)},
        ])
        ctx.run(5)


def travel_stops(ctx: TestContext):
    _view(ctx)
    ctx.video_annotation = lambda: 'Housing fixed | sustained opening force to 2 s, closing force to 4 s'
    joint = _joint(ctx, 1)
    ctx.initial(joints={joint: .085})
    ctx.plots('pull.force.y', f'asset.{joint}.position', f'asset.{joint}.velocity')
    ctx.experiment_details = {
        'purpose': 'B: sustained opening and closing force at both native travel limits',
        'initial_joint_m': .085, 'limits_m': [0, TRAVEL],
        'fixture': 'housing held by a test-only native fixed joint',
        'force_schedule_N': '2 N opening until 2 s, reverse to 2 N closing by 2.2 s, hold until 4 s',
    }
    with ctx.fixed(ctx.asset.body('cabinet.housing')):
        ctx.force('pull', ctx.asset.body('cabinet.drawer1'), keyframes=[
            {'time': 0, 'force': (0, -2, 0)},
            {'time': 2, 'force': (0, -2, 0)},
            {'time': 2.2, 'force': (0, 2, 0)},
            {'time': 4, 'force': (0, 2, 0)},
            {'time': 4.1, 'force': (0, 0, 0)},
            {'time': 5, 'force': (0, 0, 0)},
        ])
        ctx.run(5)


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


def gravity_tilt(ctx: TestContext, degrees):
    angle = math.radians(degrees)
    ctx.initial(position=(0, 0, .15), rotation=(math.cos(angle/2), math.sin(angle/2), 0, 0))
    ctx.camera(position=(.95, -1.25, .8), target=(0, -.12, .24))
    ctx.video_annotation = lambda: f'Housing fixed at {degrees} deg | gravity only | no drawer force'
    for index in (1, 2, 3):
        joint = _joint(ctx, index)
    ctx.plots(*(f'asset.cabinet.drawer{i}.slide.position' for i in (1, 2, 3)))
    ctx.experiment_details = {
        'purpose': 'D: gravity alone along the drawer opening axis',
        'tilt_degrees_about_positive_X': degrees,
        'opening_axis_gravity_m_s2': 9.81*math.sin(angle),
        'initial_joints_m': [0, 0, 0], 'initial_drawer_velocities_m_s': [0, 0, 0],
        'fixture': 'native fixed housing joint at the tilted initial pose',
        'external_drawer_forces': None,
    }
    with ctx.fixed(ctx.asset.body('cabinet.housing')):
        ctx.run(5)


TESTS = {
    **{f'pull_release_drawer{i}': partial(pull_release, index=i) for i in (1, 2, 3)},
    'travel_stops': travel_stops,
    'pull_cabinet': pull_cabinet,
    **{f'gravity_tilt_{degrees}deg': partial(gravity_tilt, degrees=degrees) for degrees in (0, 15, 35)},
}
