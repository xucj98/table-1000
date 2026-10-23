"""Physics tests of a drawer unit (``table_1000.modeling.drawer_unit_assets``), as for objects/cabinet/000000.

drawer_slide: pull and release the top drawer, then drive it into both travel stops; pull_unit: the
unit free on its feet, the bottom drawer pulled; gravity_tilt: gravity while a grip slowly tilts the
housing. An asset's ``physics_test.py``::

    from table_1000.physics.drawer_tests import tests

    TESTS = tests('tool_chest', travel=.170)
"""
import math

from table_1000.physics.testing import TestContext


def tests(kind, travel):
    """The tests of drawer unit ``kind`` whose drawers travel ``travel`` (m)."""

    def view(ctx, wide=False):
        ctx.camera(position=(1.25, -1.7, .95) if wide else (1.1, -1.5, .85),
                   target=(0, -.14 if wide else -.07, .12))

    def joint(ctx, index):
        name = f'{kind}.drawer{index}.slide'
        ctx.observe_joint(ctx.asset, name)
        return name

    def drawer_slide(ctx: TestContext):
        view(ctx)
        ctx.video_annotation = lambda: 'Housing fixed | drawer 1 | ramp pull, release at 2.1 s | 2 N open to 5 s | 2 N close to 7.5 s'
        name = joint(ctx, 1)
        ctx.initial(joints={name: .045})
        ctx.plots('pull.force.y', f'asset.{name}.position', f'asset.{name}.velocity')
        ctx.experiment_details = {
            'purpose': 'ramp pull and release, then sustained opening and closing force at both travel limits',
            'drawer': 1, 'initial_joint_m': .045, 'limits_m': [0, travel], 'fixture': 'housing rigidly locked',
            'force_schedule_N': '0 to 0.80 over 1.5 s, withdraw by 2.1 s, rest to 3 s; 2 N opening from 3.2 to 5 s; '
                                'reverse to 2 N closing by 5.2 s, hold to 7.5 s; withdraw by 7.6 s; observe to 8 s',
        }
        ctx.force('pull', ctx.asset.body(f'{kind}.drawer1'), keyframes=[
            {'time': 0, 'force': (0, 0, 0)}, {'time': .5, 'force': (0, 0, 0)},
            {'time': 2, 'force': (0, -.80, 0)}, {'time': 2.1, 'force': (0, 0, 0)},
            {'time': 3, 'force': (0, 0, 0)}, {'time': 3.2, 'force': (0, -2, 0)},
            {'time': 5, 'force': (0, -2, 0)}, {'time': 5.2, 'force': (0, 2, 0)},
            {'time': 7.5, 'force': (0, 2, 0)}, {'time': 7.6, 'force': (0, 0, 0)},
            {'time': 8, 'force': (0, 0, 0)},
        ])
        with ctx.fixed(ctx.asset.body(f'{kind}.housing')):
            ctx.run(8)
        rest = next(row for row in ctx.rows if row['time'] >= 3)
        release = next(row for row in ctx.rows if row['time'] >= 2.1)
        assert abs(rest[f'asset.{name}.velocity']) < 1e-4, 'drawer did not stop after release'
        positions = [row[f'asset.{name}.position'] for row in ctx.rows]
        ctx.metric('release_speed', release[f'asset.{name}.velocity'] * 1000, unit='mm/s', label='松手时速度')
        ctx.metric('rest_opening', rest[f'asset.{name}.position'] * 1000, unit='mm', label='松手后停下的开度')
        ctx.metric('max_opening', max(positions) * 1000, unit='mm', label=f'最大开度（限位 {travel * 1000:g} mm）')
        ctx.metric('min_opening', min(positions) * 1000, unit='mm', label='最小开度（限位 0 mm）')

    def pull_unit(ctx: TestContext):
        view(ctx, wide=True)
        ctx.video_annotation = lambda: 'Housing free on its feet | bottom drawer pulled'
        ctx.ground(static_friction=.35, dynamic_friction=.35, contact_offset=.0001)
        for body in ctx.asset.body_names:
            ctx.observe_body(ctx.asset.body(body), fields=('position', 'quaternion'))
        name = joint(ctx, 3)
        ctx.plots('pull.force.y', f'asset.{name}.position', f'asset.{kind}.housing.position.y')
        ctx.experiment_details = {
            'purpose': 'the unit free on its feet, a gradual pull on the bottom drawer',
            'fixture': None, 'initial_joint_m': 0,
            'force_schedule_N': 'settle 0.5 s, ramp to 8 N by 3.5 s, withdraw by 3.7 s; observe to 7 s',
        }
        ctx.force('pull', ctx.asset.body(f'{kind}.drawer3'), keyframes=[
            {'time': 0, 'force': (0, 0, 0)}, {'time': .5, 'force': (0, 0, 0)},
            {'time': 3.5, 'force': (0, -8, 0)}, {'time': 3.7, 'force': (0, 0, 0)},
            {'time': 7, 'force': (0, 0, 0)},
        ])
        ctx.run(7)
        ctx.metric('drawer_opening', max(row[f'asset.{name}.position'] for row in ctx.rows) * 1000, unit='mm',
                   label=f'抽屉最大开度（限位 {travel * 1000:g} mm）')
        moved = ctx.rows[-1][f'asset.{kind}.housing.position.y'] - ctx.rows[0][f'asset.{kind}.housing.position.y']
        ctx.metric('housing_travel', abs(moved) * 100, unit='cm', label='柜体被拉动距离')

    def gravity_tilt(ctx: TestContext):
        """Slowly incline the housing; the drawers stay free."""
        ctx.initial(position=(0, 0, .22))
        ctx.camera(position=(1.25, -1.65, 1.0), target=(0, -.15, .3))
        housing = ctx.asset.body(f'{kind}.housing')
        for index in (1, 2, 3):
            joint(ctx, index)

        def angle():
            return math.atan2(housing.state['rotation'][2, 1], housing.state['rotation'][1, 1])

        ctx.observe('housing.angle', angle, unit='rad')
        ctx.plots('housing.angle', *(f'asset.{kind}.drawer{i}.slide.position' for i in (1, 2, 3)))
        ctx.video_annotation = lambda: f'Housing tilt {math.degrees(angle()):.1f} deg | three drawers free | gravity only'
        grip = ctx.grip(housing, name='housing_grip')
        ctx.run(1)
        tilt = math.radians(45)
        grip.move_to(rotation=(math.cos(tilt / 2), math.sin(tilt / 2), 0, 0), duration=15)
        ctx.run(2)
        starts = {}
        for i in (1, 2, 3):
            column = f'asset.{kind}.drawer{i}.slide.position'
            initial = ctx.rows[0][column]
            row = next((r for r in ctx.rows if r[column] - initial >= .0005), None)
            starts[f'drawer{i}'] = None if row is None else {
                'time_s': row['time'], 'angle_degrees': math.degrees(row['housing.angle'])}
        ctx.experiment_details = {
            'purpose': 'continuous gravity inclination from horizontal to 45 degrees',
            'fixture': 'grip on the housing tilting about its bottom-centre origin; drawers always free',
            'schedule': 'horizontal 1 s, quintic 45-degree tilt over 15 s, hold 2 s',
            'onset_displacement_m': .0005, 'onset': starts, 'design_target_minimum_angle_degrees': 15,
        }
        for i in (1, 2, 3):
            if starts[f'drawer{i}'] is not None:
                ctx.metric(f'drawer{i}_onset', starts[f'drawer{i}']['angle_degrees'], unit='deg', label=f'抽屉 {i} 起滑角')
        assert all(value is not None and value['angle_degrees'] >= 15 for value in starts.values()), \
            'a drawer slid before the 15-degree design target or never opened'

    return {'drawer_slide': drawer_slide, 'pull_unit': pull_unit, 'gravity_tilt': gravity_tilt}
