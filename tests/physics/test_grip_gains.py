"""Grip gain reference timing and axis controls, without launching Isaac."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from table_1000.physics.testing import TestContext as Context
from table_1000.sim.isaaclab import fixtures


def make_grip(monkeypatch, dt=.004, reference=None):
    fixture = Mock(released=False, origin=np.zeros(3))
    fixture.positions.return_value = np.zeros(6)
    fixture.environment_positions.return_value = np.zeros((4, 6))
    fixture.pose_coordinates.return_value = np.zeros(6)
    fixture.velocity_coordinates.return_value = np.zeros(6)
    monkeypatch.setattr(fixtures, 'Fixture', Mock(return_value=fixture))
    ctx = SimpleNamespace(dt=dt, initialized=False, physics='gpu', grips=[],
                          gravity=(0, 0, -9.81), observe=Mock(),
                          body_pose=lambda body: (np.zeros(3), np.array([1., 0, 0, 0])))
    body = SimpleNamespace(name='rubber', instance=SimpleNamespace(name='asset'))
    grip = Context.grip(ctx, body, name='guide', gain_reference_dt=reference)
    grip.tree_state = Mock(return_value={'mass': .014, 'inertia': np.diag([1e-6, 2e-6, 4e-6]),
                                        'com': np.zeros(3)})
    grip.measured_velocities = Mock(return_value=np.zeros((4, 6)))
    return grip, ctx


def test_default_uses_actual_step_at_initialization(monkeypatch):
    grip, ctx = make_grip(monkeypatch)
    ctx.dt = .002
    grip.initialize()
    assert grip.gains['gain_reference_dt_s'] == ctx.dt
    assert grip.gains['linear_stiffness_N_per_m'] == .014*(10/.002)**2
    assert grip.gains['angular_damping_N_m_s_per_rad'] == 2*4e-6*(3/.002)


def test_one_ms_can_preserve_four_ms_gains_and_report_both(monkeypatch):
    baseline, _ = make_grip(monkeypatch)
    baseline.initialize()
    grip, ctx = make_grip(monkeypatch, dt=.001, reference=.004)
    grip.initialize()
    for key, value in baseline.gains.items():
        if key != 'physics_dt_s':
            assert grip.gains[key] == value
    assert ctx.dt == .001
    report = Context.grip_details(ctx)['guide']
    assert report['physics_dt_s'] == .001
    assert report['gain_reference_dt_s'] == .004
    assert report['angular_stiffness_N_m_per_rad'] == 4e-6*(3/.004)**2


@pytest.mark.parametrize('reference', [0, -1, float('nan'), float('inf'), -float('inf'),
                                       True, np.bool_(False), '0.004', [0.004]])
def test_invalid_reference_rejected_before_creating_fixture(monkeypatch, reference):
    create = Mock()
    monkeypatch.setattr(fixtures, 'Fixture', create)
    with pytest.raises(ValueError, match='finite positive'):
        fixtures.Grip(SimpleNamespace(initialized=False), None, name='bad', gain_reference_dt=reference)
    create.assert_not_called()


def test_grip_cannot_be_declared_after_initialization(monkeypatch):
    create = Mock()
    monkeypatch.setattr(fixtures, 'Fixture', create)
    with pytest.raises(AssertionError, match='before the first'):
        fixtures.Grip(SimpleNamespace(initialized=True), None, name='late', gain_reference_dt=.004)
    create.assert_not_called()


def test_reference_gains_leave_translation_disabled_in_all_environments(monkeypatch):
    grip, _ = make_grip(monkeypatch, dt=.001, reference=.004)
    grip.axes(linear=(0, 0, 0), angular=True)
    grip.initialize()
    grip.update(0, .001)
    for values in (grip.fixture.stiffness, grip.fixture.damping, grip.fixture.actuation):
        assert values.shape == (4, 6)
        np.testing.assert_array_equal(values[:, :3], 0.)
    np.testing.assert_array_equal(grip.fixture.stiffness[:, 3:], 4e-6*(3/.004)**2)
    np.testing.assert_array_equal(grip.fixture.damping[:, 3:], 2*4e-6*(3/.004))
    grip.fixture.apply.assert_called_once()
