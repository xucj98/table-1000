"""Controllers turn 25 Hz actions into per-physics-step joint targets (no simulator)."""

import numpy as np
import pytest

from table_1000.robots import controllers as c
from table_1000.robots.agent import BaseSpec

from conftest import arm_spec

ARM = arm_spec()
# A differential base like X1Pro's.
BASE = BaseSpec(wheels=('left_wheel_joint', 'right_wheel_joint'), wheel_radius=.086, track=.48,
                max_speed=(.5, 1.), max_acceleration=(1., 2.), max_force=(116., 28.))


def run(group, steps):
    rows = [group.substep(k, c.SUBSTEPS, c.PHYSICS_DT) for k in range(1, c.SUBSTEPS + 1)]
    return rows if steps else rows[-1]


def test_rate():
    assert c.SUBSTEPS == 10


def test_joint_position_interpolates_with_feed_forward():
    group = c.JointPosition(['a', 'b'], [(-1., 1.), (-1., 1.)])
    group.reset([[0., 0.]])
    group.set_action([[.4, 2.]])            # b is clipped to its limit
    rows = run(group, True)
    np.testing.assert_allclose(rows[0][0], [[.04, .1]])
    np.testing.assert_allclose(rows[-1][0], [[.4, 1.]])
    np.testing.assert_allclose(rows[0][1], [[10., 25.]])
    group.hold()
    np.testing.assert_allclose(run(group, False)[1], [[0., 0.]])


def test_delta_mode_adds_to_the_target():
    group = c.JointPosition(['a'], [(-1., 1.)], delta=True)
    group.reset([[.1]])
    group.set_action([[.2]])
    group.set_action([[.2]])
    np.testing.assert_allclose(group.target, [[.5]])


def test_gripper_gap():
    arm = ARM.arms['arm']
    group = c.Gripper(arm)
    group.reset([[0., 0.]])
    group.set_action([[.06]])
    np.testing.assert_allclose(run(group, False)[0], [[.03, -.03]])
    group.set_action([[1.]])
    np.testing.assert_allclose(run(group, False)[0], [[arm.max_gap / 2, -arm.max_gap / 2]])


def drive(base, command, seconds):
    for _ in range(round(seconds * c.CONTROL_HZ)):
        base.set_action(command)
        position, velocity = run(base, False)
    return position, velocity


def test_base_drives_straight_and_ramps():
    base = c.BaseVelocity(BASE)
    base.reset([[1., 2., np.pi / 2]])
    base.set_action([[.5, 0.]])
    _, velocity = run(base, False)
    # 1 m/s^2 for 0.04 s
    assert velocity[0, 1] == pytest.approx(.04, abs=1e-9)
    position, velocity = drive(base, [[.5, 0.]], 2.)
    assert velocity[0, 1] == pytest.approx(.5)
    assert position[0, 0] == pytest.approx(1., abs=1e-9)
    # 2.04 s in all: the 0.5 s ramp covers 0.125 m, then 1.54 s at 0.5 m/s
    assert position[0, 1] == pytest.approx(2. + .125 + .77, abs=2e-3)
    wheels = velocity[0, 3:]
    np.testing.assert_allclose(wheels, [.5 / BASE.wheel_radius] * 2)


def test_base_turns_in_place_and_clips():
    base = c.BaseVelocity(BaseSpec(wheels=('l', 'r'), wheel_radius=.1, track=.5, max_speed=(.5, 1.),
                                   max_acceleration=(100., 100.)))
    base.reset([[0., 0., 0.]])
    position, velocity = drive(base, [[0., 5.]], 1.)
    assert velocity[0, 2] == pytest.approx(1.)
    assert position[0, 2] == pytest.approx(1., abs=.02)
    np.testing.assert_allclose(position[0, :2], [0., 0.])
    assert velocity[0, 3] == pytest.approx(-2.5) and velocity[0, 4] == pytest.approx(2.5)


def test_base_stands_still_without_drift():
    base = c.BaseVelocity(BASE)
    base.reset([[0., 0., 0.]])
    drive(base, [[.3, .2]], 1.)
    position, _ = drive(base, [[0., 0.]], 2.)
    later, velocity = drive(base, [[0., 0.]], 1.)
    np.testing.assert_allclose(later, position)
    np.testing.assert_allclose(velocity, 0.)


def test_fixed_robot_controller_layout(arm_model):
    """Without a mobile base, lift or head there are only the arm and its gripper."""
    controller = c.RobotController(ARM, [arm_model])
    assert controller.action_space() == {'arm': 6, 'arm_gripper': 1}
    assert controller.joints == list(ARM.body_joints)
    q = {name: np.array([value]) for name, value in ARM.keyframe('ready').items()}
    controller.reset(q, [[0., 0., 0.]])
    controller.measure([[0., 0., 0.]])
    controller.set_action({'arm_gripper': [[.04]]})
    positions, _ = controller.substep(c.SUBSTEPS)
    row = dict(zip(controller.joints, positions[0]))
    assert row['finger_left_joint'] == pytest.approx(.02) and row['joint2'] == pytest.approx(.5)
    with pytest.raises(KeyError):
        controller.set_action({'base': [[.1, 0.]]})


def test_end_effector_delta_moves_the_tcp(arm_model):
    controller = c.RobotController(ARM, [arm_model], modes={'arm': 'ee_delta'})
    q = {name: np.array([value]) for name, value in ARM.keyframe('ready').items()}
    controller.reset(q, [[0., 0., 0.]])
    start = arm_model.tcp('arm')
    for _ in range(4):
        controller.set_action({'arm': [[.01, 0., -.01, 0., 0., .02]]})
    assert not controller.groups['arm'].failed.any()
    positions, _ = controller.substep(c.SUBSTEPS)
    reached = arm_model.tcp('arm', arm_model.vector(dict(zip(controller.joints, positions[0]))))
    np.testing.assert_allclose(reached[:3, 3] - start[:3, 3], [.04, 0., -.04], atol=1e-3)


def test_blocked_base_target_stays_near_the_measured_pose():
    base = c.BaseVelocity(BASE)
    base.reset([[0., 0., 0.]])
    for _ in range(50):                       # 2 s at 0.5 m/s against a wall at x = 0
        base.measure([[0., 0., 0.]])
        base.set_action([[.5, 0.]])
        run(base, False)
    base.measure([[0., 0., 0.]])
    assert base.pose[0, 0] == pytest.approx(BASE.max_lag[0])
