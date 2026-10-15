"""Camera projection against OpenCV."""

import numpy as np
import pytest

from table_1000.robots.cameras import direction, project
from table_1000.robots.spec import Sensor

CAMERAS = [
    Sensor((1280, 720), (722.9, 722.7, 643.3, 366.9), 'pinhole', (-.0204, .0034, .00058, -.00004)),
    Sensor((640, 480), (280., 280., 320., 240.), 'fisheye', (.05, -.01, .002, 0.)),
]


@pytest.mark.parametrize('camera', CAMERAS, ids=lambda camera: camera.model)
def test_projection_matches_opencv(camera):
    cv2 = pytest.importorskip('cv2')
    rng = np.random.default_rng(0)
    points = np.c_[rng.uniform(-.4, .4, (50, 2)), rng.uniform(.3, 1., 50)]
    K = np.array([[camera.intrinsics[0], 0, camera.intrinsics[2]], [0, camera.intrinsics[1], camera.intrinsics[3]], [0, 0, 1]])
    if camera.model == 'fisheye':
        expected = cv2.fisheye.projectPoints(points[:, None], np.zeros(3), np.zeros(3), K, np.array(camera.distortion))[0]
    else:
        expected = cv2.projectPoints(points, np.zeros(3), np.zeros(3), K, np.array(camera.distortion))[0]
    np.testing.assert_allclose(project(camera, points), expected[:, 0], atol=1e-6)


@pytest.mark.parametrize('camera', CAMERAS, ids=lambda camera: camera.model)
def test_direction_points_at_the_pixel(camera):
    """Without distortion the ray through a pixel projects back onto it."""
    ideal = Sensor(camera.resolution, camera.intrinsics, camera.model)
    for pixel in [(100., 80.), (camera.intrinsics[2], camera.intrinsics[3]), (500., 400.)]:
        np.testing.assert_allclose(project(ideal, direction(ideal, pixel)[None])[0], pixel, atol=1e-6)


def test_override_cameras():
    from table_1000.robots.spec import CameraSpec, override_cameras
    cameras = {'head': CameraSpec(CAMERAS[0], 'head_optical_frame'), 'wrist': CameraSpec(CAMERAS[1], 'wrist_frame')}
    changed = override_cameras(cameras, {'resize': (320, 180), 'near': .02, 'wrist': None})
    assert list(changed) == ['head']
    head = changed['head'].sensor
    assert head.resolution == (320, 180) and head.near == .02
    np.testing.assert_allclose(head.intrinsics, np.array(CAMERAS[0].intrinsics) / 4)
    assert override_cameras(cameras, {'wrist': {'self_view': .5}})['wrist'].self_view == .5
    with pytest.raises(KeyError):
        override_cameras(cameras, {'fps': 15})


def test_mount_folds_fixed_joints(tmp_path):
    from table_1000.robots.cameras import mount_on_body
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('''<robot name="r">
      <link name="base"/><link name="head"/><link name="camera"/><link name="optical"/>
      <joint name="pan" type="revolute"><parent link="base"/><child link="head"/><origin xyz="0 0 1"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <joint name="fix1" type="fixed"><parent link="head"/><child link="camera"/><origin xyz="0.1 0 0"/></joint>
      <joint name="fix2" type="fixed"><parent link="camera"/><child link="optical"/>
        <origin xyz="0 0 0.05" rpy="-1.5707963 0 -1.5707963"/></joint></robot>''')
    link, transform = mount_on_body(urdf, 'optical')
    assert link == 'head'
    np.testing.assert_allclose(transform[:3, 3], [.1, 0., .05], atol=1e-9)
    np.testing.assert_allclose(transform[:3, 2], [1., 0., 0.], atol=1e-6)      # optical +Z looks along +X
    link, transform = mount_on_body(urdf, ('camera', (0., 0., .2), (0., 0., 0.)))
    assert link == 'head'
    np.testing.assert_allclose(transform[:3, 3], [.1, 0., .2], atol=1e-9)
