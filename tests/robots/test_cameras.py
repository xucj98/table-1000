"""Camera projection against OpenCV."""

import numpy as np
import pytest

from table_1000.robots.cameras import direction, project
from table_1000.robots.spec import CameraSpec

CAMERAS = [
    CameraSpec('head', (1280, 720), (722.9, 722.7, 643.3, 366.9), 'pinhole', (-.0204, .0034, .00058, -.00004)),
    CameraSpec('wrist', (640, 480), (280., 280., 320., 240.), 'fisheye', (.05, -.01, .002, 0.)),
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
    ideal = CameraSpec(camera.frame, camera.resolution, camera.intrinsics, camera.model)
    for pixel in [(100., 80.), (camera.intrinsics[2], camera.intrinsics[3]), (500., 400.)]:
        np.testing.assert_allclose(project(ideal, direction(ideal, pixel)[None])[0], pixel, atol=1e-6)
