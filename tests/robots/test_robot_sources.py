"""Every robot source in asset_sources/robots/ (no simulator): description, keyframes, IK, settings."""

import numpy as np
import pytest

from table_1000.robots import ROOT, load_robot_test, load_spec, robot_names
from table_1000.robots.checks import select_cases
from table_1000.robots.kinematics import pose_error

NAMES = robot_names()


@pytest.fixture(scope='module', params=NAMES)
def robot(request, tmp_path_factory):
    pytest.importorskip('mplib')
    from table_1000.robots.kinematics import RobotModel
    spec = load_spec(request.param)
    return spec, RobotModel(spec, tmp_path_factory.mktemp(f'{request.param}-planning'))


def test_there_are_robots():
    assert 'x1pro' in NAMES


@pytest.mark.parametrize('name', NAMES)
def test_description_and_settings(name):
    import xml.etree.ElementTree as ET
    spec = load_spec(name)
    assert spec.name == name and (ROOT / spec.urdf).exists()
    joints = {joint.get('name') for joint in ET.parse(ROOT / spec.urdf).getroot().findall('joint')}
    assert set(spec.body_joints) <= joints
    assert spec.base is None or set(spec.base.wheels) <= joints
    settings = load_robot_test(name)
    assert settings is not None, 'robot_test.py with the acceptance settings'
    select_cases(spec, settings)
    links = {link.get('name') for link in ET.parse(ROOT / spec.urdf).getroot().findall('link')}
    assert set(getattr(settings, 'DROP_LINKS', {}).values()) <= links
    for camera in spec.cameras.values():
        mount = camera.mount if isinstance(camera.mount, str) else camera.mount[0]
        assert mount in links and camera.sensor.kind in ('rgb', 'depth') and camera.sensor.model in ('pinhole', 'fisheye')


def test_keyframes_free_of_self_collision(robot):
    spec, model = robot
    for name in spec.keyframe_names:
        assert model.self_collisions(model.vector(spec.keyframe(name))) == [], name


def test_ik_round_trip(robot):
    spec, model = robot
    model.set_state(spec.keyframe(spec.keyframe_names[-1]))
    rng = np.random.default_rng(0)
    for arm in spec.arms:
        indices = model.arm_indices(arm)
        solved = tried = 0
        while tried < 20:
            q = model.q.copy()
            q[indices] = rng.uniform(*model.limits[indices].T)
            if model.self_collisions(q):
                continue
            tried += 1
            target = model.tcp(arm, q)
            status, solution = model.ik(arm, target)
            if status == 'Success':
                solved += 1
                assert pose_error(model.tcp(arm, solution), target)[0] < 1e-3
        assert solved >= 18, arm


def test_cameras_and_look_at(robot):
    spec, model = robot
    if not (spec.head and spec.cameras):
        pytest.skip('no head camera')
    camera = next(camera.mount for camera in spec.cameras.values() if isinstance(camera.mount, str))
    model.set_state(spec.keyframe(spec.keyframe_names[-1]))
    point = model.tcp(spec.primary_arm, world=True)[:3, 3] - [0., 0., .4]
    frame = model.fk(camera, model.vector(model.look_at(point, camera)), world=True)
    direction = point - frame[:3, 3]
    assert frame[:3, 2] @ direction / np.linalg.norm(direction) > .999
