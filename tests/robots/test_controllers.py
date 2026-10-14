"""Controllers turn 25 Hz actions into per-physics-step joint targets (no simulator)."""

import numpy as np
import pytest

from table_1000.robots import controllers as c
from table_1000.robots.agent import BaseSpec
from table_1000.robots.x1pro import SPEC


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
    arm = SPEC.arms['right']
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
    base = c.BaseVelocity(SPEC.base)
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
    np.testing.assert_allclose(wheels, [.5 / SPEC.base.wheel_radius] * 2)


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
    base = c.BaseVelocity(SPEC.base)
    base.reset([[0., 0., 0.]])
    drive(base, [[.3, .2]], 1.)
    position, _ = drive(base, [[0., 0.]], 2.)
    later, velocity = drive(base, [[0., 0.]], 1.)
    np.testing.assert_allclose(later, position)
    np.testing.assert_allclose(velocity, 0.)


mplib = pytest.importorskip('mplib')


@pytest.fixture(scope='module')
def model(tmp_path_factory):
    from table_1000.robots.kinematics import RobotModel
    model = RobotModel(SPEC, tmp_path_factory.mktemp('planning'))
    model.set_state(SPEC.keyframe('ready'))
    return model


def test_robot_controller_layout(model):
    controller = c.RobotController(SPEC, [model])
    assert controller.action_space() == {'base': 2, 'lift': 1, 'head': 2, 'right': 6, 'right_gripper': 1,
                                         'left': 6, 'left_gripper': 1}
    assert len(controller.joints) == len(set(controller.joints)) == 3 + 2 + len(SPEC.body_joints)
    q = {name: np.array([value]) for name, value in SPEC.keyframe('ready').items()}
    controller.reset(q, [[0., 0., 0.]])
    controller.set_action({'lift': [[.5]], 'right_gripper': [[.04]]})
    positions, _ = controller.substep(c.SUBSTEPS)
    row = dict(zip(controller.joints, positions[0]))
    assert row['lift_joint'] == pytest.approx(.5)
    assert row['right_arm_gripper_left_joint'] == pytest.approx(.02)
    assert row['left_arm_joint2'] == pytest.approx(.4)
    with pytest.raises(KeyError):
        controller.set_action({'wings': [[0.]]})


def test_end_effector_delta_moves_the_tcp(model):
    controller = c.RobotController(SPEC, [model], modes={'right': 'ee_delta'})
    q = {name: np.array([value]) for name, value in SPEC.keyframe('ready').items()}
    controller.reset(q, [[0., 0., 0.]])
    start = model.tcp('right')
    for _ in range(5):
        controller.set_action({'right': [[.01, 0., -.01, 0., 0., .02]]})
    assert not controller.groups['right'].failed.any()
    positions, _ = controller.substep(c.SUBSTEPS)
    reached = model.tcp('right', model.vector(dict(zip(controller.joints, positions[0]))))
    np.testing.assert_allclose(reached[:3, 3] - start[:3, 3], [.05, 0., -.05], atol=1e-3)


def test_blocked_base_target_stays_near_the_measured_pose():
    base = c.BaseVelocity(SPEC.base)
    base.reset([[0., 0., 0.]])
    for _ in range(50):                       # 2 s at 0.5 m/s against a wall at x = 0
        base.measure([[0., 0., 0.]])
        base.set_action([[.5, 0.]])
        run(base, False)
    base.measure([[0., 0., 0.]])
    assert base.pose[0, 0] == pytest.approx(SPEC.base.max_lag[0])
