"""Kinematics and self-collision planning of a fixed single arm (no simulator)."""

import numpy as np
import pytest

from table_1000.robots.kinematics import pose_error


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


def test_joints_and_keyframes(arm_model):
    spec = arm_model.spec
    assert sorted(arm_model.joints) == sorted(spec.body_joints)
    assert spec.base is None and spec.keyframe_names == ['zero', 'ready']
    for name in spec.keyframe_names:
        assert arm_model.self_collisions(arm_model.vector(spec.keyframe(name))) == []


def test_ik_round_trip(arm_model):
    """IK reaches the TCP poses of random collision-free configurations."""
    samples = free_samples(arm_model, 'arm', 40, seed=2)
    solved = 0
    for q in samples:
        target = arm_model.tcp('arm', q)
        status, solution = arm_model.ik('arm', target)
        if status == 'Success':
            solved += 1
            position, rotation = pose_error(arm_model.tcp('arm', solution), target)
            assert position < 1e-3 and rotation < 1e-2
    assert solved / len(samples) >= .9


def test_world_targets_follow_the_base_pose(arm_model):
    base = np.eye(4)
    base[:3, 3] = [1., 2., .7]
    local = arm_model.tcp('arm')
    arm_model.set_state(base=base)
    try:
        np.testing.assert_allclose(arm_model.tcp('arm', world=True), base @ local)
    finally:
        arm_model.set_state(base=np.eye(4))


def test_rrt_moves_only_the_arm(arm_model):
    goal = free_samples(arm_model, 'arm', 1, seed=3)[0]
    result = arm_model.plan_rrt('arm', None, goals=[goal])
    assert result['status'] == 'Success'
    assert result['position'].shape[1] == 6
    np.testing.assert_allclose(result['position'][-1], goal[arm_model.arm_indices('arm')], atol=1e-3)
    for row in result['position']:
        assert arm_model.self_collisions(arm_model.vector(dict(zip(result['joints'], row)))) == []


def test_screw_is_straight(arm_model):
    start = arm_model.tcp('arm')
    target = start.copy()
    target[:3, 3] += [.05, 0., -.04]
    result = arm_model.plan_screw('arm', target)
    assert result['status'] == 'Success'
    direction = (target[:3, 3] - start[:3, 3]) / np.linalg.norm(target[:3, 3] - start[:3, 3])
    deviations = []
    for row in result['position']:
        point = arm_model.tcp('arm', arm_model.vector(dict(zip(result['joints'], row))))[:3, 3] - start[:3, 3]
        deviations.append(np.linalg.norm(point - (point @ direction) * direction))
    assert max(deviations) < 5e-3


def test_look_at_needs_a_head(arm_model):
    with pytest.raises(Exception):
        arm_model.look_at(np.zeros(3), 'link6')


def test_check_path_catches_a_collision_between_waypoints(arm_model):
    """Two collision-free waypoints whose straight joint path sweeps the hand through an obstacle."""
    from table_1000.planning.world import PlanningScene, box_object
    middle = arm_model.vector(arm_model.spec.keyframe('ready'))
    joint = arm_model.arm_indices('arm')[0]
    start, end = middle.copy(), middle.copy()
    start[joint], end[joint] = -1., 1.
    scene = PlanningScene(arm_model.planners['arm'])
    scene.add(*box_object('obstacle', (.025, .025, .025), arm_model.fk('link6', middle)))
    try:
        indices = arm_model.arm_indices('arm')
        assert arm_model.check_path('arm', [start[indices]])[1] == []
        assert arm_model.check_path('arm', [end[indices]])[1] == []
        checks, pairs = arm_model.check_path('arm', [start[indices], end[indices]])
        assert checks > 2 and any('obstacle' in pair for pair in pairs)
    finally:
        scene.clear()
