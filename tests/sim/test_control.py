"""Control arithmetic of the action terms: 25 Hz actions to per-physics-step targets (no simulator)."""

import numpy as np
import pytest
import torch

from table_1000.robots.spec import BaseSpec
from table_1000.sim import CONTROL_HZ, PHYSICS_DT, SUBSTEPS, control

# A differential base like X1Pro's.
BASE = BaseSpec(wheels=('left_wheel_joint', 'right_wheel_joint'), wheel_radius=.086, track=.48,
                max_speed=(.5, 1.), max_acceleration=(1., 2.), max_force=(116., 28.))


def tensor(value):
    return torch.tensor(value, dtype=torch.float64)


def test_rate():
    assert SUBSTEPS == 10 and CONTROL_HZ * SUBSTEPS * PHYSICS_DT == pytest.approx(1.)


def test_joint_targets_interpolate_with_feed_forward():
    start, target = tensor([[0., 0.]]), tensor([[.4, 1.]])
    first, velocity = control.interpolate(start, target, 1, SUBSTEPS, PHYSICS_DT)
    np.testing.assert_allclose(first, [[.04, .1]])
    np.testing.assert_allclose(velocity, [[10., 25.]])
    last, _ = control.interpolate(start, target, SUBSTEPS, SUBSTEPS, PHYSICS_DT)
    np.testing.assert_allclose(last, target)
    held, velocity = control.interpolate(target, target, 3, SUBSTEPS, PHYSICS_DT)
    np.testing.assert_allclose(held, target)
    np.testing.assert_allclose(velocity, 0.)


def test_gripper_gap():
    signs = tensor([1., -1.])
    np.testing.assert_allclose(control.finger_targets(tensor([[.06]]), signs, .1), [[.03, -.03]])
    np.testing.assert_allclose(control.finger_targets(tensor([[1.]]), signs, .1), [[.05, -.05]])


class Base:
    """The base term's state, stepped through the control functions."""

    def __init__(self, spec, pose):
        self.spec = spec
        self.pose, self.speed = tensor([pose]), tensor([[0., 0.]])
        self.max_speed, self.max_acceleration = tensor(spec.max_speed), tensor(spec.max_acceleration)

    def drive(self, command, seconds, measured=None):
        command = torch.clamp(tensor([command]), -self.max_speed, self.max_speed)
        for _ in range(round(seconds * CONTROL_HZ)):
            if measured is not None:
                self.pose = control.clamp_lag(self.pose, tensor([measured]), tensor(self.spec.max_lag))
            for _ in range(SUBSTEPS):
                self.pose, self.speed, self.velocity = control.base_substep(self.pose, self.speed, command,
                                                                           self.max_acceleration, PHYSICS_DT)
        return self.pose[0].numpy(), self.velocity[0].numpy()


def test_base_drives_straight_and_ramps():
    base = Base(BASE, [1., 2., np.pi / 2])
    _, velocity = base.drive([.5, 0.], 1 / CONTROL_HZ)
    # 1 m/s^2 for 0.04 s
    assert velocity[1] == pytest.approx(.04, abs=1e-9)
    position, velocity = base.drive([.5, 0.], 2.)
    assert velocity[1] == pytest.approx(.5)
    assert position[0] == pytest.approx(1., abs=1e-9)
    # 2.04 s in all: the 0.5 s ramp covers 0.125 m, then 1.54 s at 0.5 m/s
    assert position[1] == pytest.approx(2. + .125 + .77, abs=2e-3)
    np.testing.assert_allclose(control.wheel_speeds(base.speed, BASE.track, BASE.wheel_radius)[0],
                               [.5 / BASE.wheel_radius] * 2)


def test_base_turns_in_place_and_clips():
    spec = BaseSpec(wheels=('l', 'r'), wheel_radius=.1, track=.5, max_speed=(.5, 1.), max_acceleration=(100., 100.))
    base = Base(spec, [0., 0., 0.])
    position, velocity = base.drive([0., 5.], 1.)
    assert velocity[2] == pytest.approx(1.)
    assert position[2] == pytest.approx(1., abs=.02)
    np.testing.assert_allclose(position[:2], [0., 0.])
    np.testing.assert_allclose(control.wheel_speeds(base.speed, spec.track, spec.wheel_radius)[0], [-2.5, 2.5])


def test_base_stands_still_without_drift():
    base = Base(BASE, [0., 0., 0.])
    base.drive([.3, .2], 1.)
    position, _ = base.drive([0., 0.], 2.)
    later, velocity = base.drive([0., 0.], 1.)
    np.testing.assert_allclose(later, position)
    np.testing.assert_allclose(velocity, 0.)


def test_blocked_base_target_stays_near_the_measured_pose():
    base = Base(BASE, [0., 0., 0.])
    base.drive([.5, 0.], 2., measured=[0., 0., 0.])      # against a wall at x = 0
    pose = control.clamp_lag(base.pose, tensor([[0., 0., 0.]]), tensor(BASE.max_lag))
    assert pose[0, 0] == pytest.approx(BASE.max_lag[0])


def test_yaw_compares_across_turns():
    """PhysX reports a turning joint in (-2 pi, 2 pi]: one heading has two values there."""
    # Integrated yaw just past pi, measured the same heading written 2 pi lower.
    pose = control.clamp_lag(tensor([[0., 0., 3.2]]), tensor([[0., 0., 3.19 - 2 * np.pi]]), tensor(BASE.max_lag))
    assert float(control.wrap_angle(pose[0, 2] - 3.2)) == pytest.approx(0., abs=1e-9)
    # The drive target goes in the measured value's representation, inside [-2 pi, 2 pi].
    assert float(control.near_angle(tensor(.03), tensor(-6.25))) == pytest.approx(.03 - 2 * np.pi)
    assert float(control.near_angle(tensor(-3.13), tensor(3.15))) == pytest.approx(2 * np.pi - 3.13)
    assert float(control.near_angle(tensor(.02), tensor(6.27))) == pytest.approx(2 * np.pi)


def test_base_turns_through_whole_turns():
    base = Base(BASE, [0., 0., 3.])
    position, velocity = base.drive([0., 1.], 8.)
    assert velocity[2] == pytest.approx(1.)
    assert -np.pi <= position[2] < np.pi
    # 2 rad/s^2: the 0.5 s ramp loses 0.25 rad (less a step-integration rest).
    assert float(control.wrap_angle(tensor(position[2] - (3. + 8. - .25)))) == pytest.approx(0., abs=5e-3)
