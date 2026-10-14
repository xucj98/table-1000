"""Whole-body planning model of X1Pro (no simulator)."""

import numpy as np
import pytest

mplib = pytest.importorskip('mplib')

from table_1000.robots import x1pro
from table_1000.robots.kinematics import ROOT, RobotModel, pose_error

SPEC = x1pro.SPEC
CAMERA = x1pro.HEAD_CAMERA


@pytest.fixture(scope='module')
def model(tmp_path_factory):
    model = RobotModel(SPEC, tmp_path_factory.mktemp('planning'))
    model.set_state(SPEC.keyframe('ready'))
    return model


def free_samples(model, arm, count, seed=0):
    rng = np.random.default_rng(seed)
    indices = model.arm_indices(arm)
    low, high = model.limits[indices].T
    samples = []
    while len(samples) < count:
        q = model.q.copy()
        q[indices] = rng.uniform(low, high)
        if not model.self_collisions(q):
            samples.append(q)
    return samples


def test_joints_and_keyframes(model):
    assert sorted(model.joints) == sorted(SPEC.body_joints)
    for name in SPEC.keyframes:
        assert model.self_collisions(model.vector(SPEC.keyframe(name))) == []


def test_tcp_matches_the_task_planner(model, tmp_path):
    """The right TCP agrees with the single-arm planner the drawer task uses."""
    frozen = {name: value for name, value in SPEC.keyframe('ready').items() if name not in x1pro.ARM}
    planner = x1pro.build_planner(ROOT / SPEC.urdf, tmp_path, frozen)
    for q in free_samples(model, 'right', 20, seed=1):
        arm = q[model.arm_indices('right')]
        planner.pinocchio_model.compute_forward_kinematics(np.r_[arm, 0., 0.])
        legacy = planner.pinocchio_model.get_link_pose(planner.link_name_2_idx[x1pro.TCP_LINK])
        error = pose_error(legacy.to_transformation_matrix(), model.tcp('right', q))
        assert error[0] < 1e-6 and error[1] < 1e-6


@pytest.mark.parametrize('arm', ['right', 'left'])
def test_ik_round_trip(model, arm):
    """IK reaches poses of random collision-free configurations from the ready pose."""
    samples = free_samples(model, arm, 50, seed=2)
    solved = 0
    for q in samples:
        target = model.tcp(arm, q)
        status, solution = model.ik(arm, target)
        if status == 'Success':
            solved += 1
            position, rotation = pose_error(model.tcp(arm, solution), target)
            assert position < 1e-3 and rotation < 1e-2
            others = [i for i in range(len(model.joints)) if i not in model.arm_indices(arm)]
            np.testing.assert_allclose(solution[others], model.q[others])
    assert solved / len(samples) >= .95


def test_rrt_moves_only_the_arm(model):
    target = model.tcp('right')
    target[:3, 3] += [.10, -.05, .05]
    result = model.plan_rrt('right', target)
    assert result['status'] == 'Success'
    assert result['position'].shape[1] == 6
    end = model.vector(dict(zip(result['joints'], result['position'][-1])))
    assert pose_error(model.tcp('right', end), target)[0] < 1e-3
    for row in result['position']:
        assert model.self_collisions(model.vector(dict(zip(result['joints'], row)))) == []


def test_screw_is_straight(model):
    start = model.tcp('right')
    target = start.copy()
    target[:3, 3] += [.08, .0, -.06]
    result = model.plan_screw('right', target)
    assert result['status'] == 'Success'
    direction = (target[:3, 3] - start[:3, 3]) / np.linalg.norm(target[:3, 3] - start[:3, 3])
    deviations = []
    for row in result['position']:
        point = model.tcp('right', model.vector(dict(zip(result['joints'], row))))[:3, 3] - start[:3, 3]
        deviations.append(np.linalg.norm(point - (point @ direction) * direction))
    assert max(deviations) < 5e-3


def test_look_at(model):
    point = model.tcp('right', world=True)[:3, 3] - [0., 0., .4]
    head = model.look_at(point, CAMERA)
    frame = model.fk(CAMERA, model.vector(head), world=True)
    direction = point - frame[:3, 3]
    assert frame[:3, 2] @ direction / np.linalg.norm(direction) > .999
