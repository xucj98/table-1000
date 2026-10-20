import numpy as np
import pytest

from table_1000.experts.grasps import build_grasp_pose, compute_grasp_info_by_obb


def test_grasp_pose_axes():
    pose = build_grasp_pose([0., 0., -1.], [0., 1., .3], [1., 2., 3.])
    assert np.allclose(pose[:3, 0], [0., 0., -1.])
    assert np.allclose(pose[:3, 1], [0., 1., 0.])        # made orthogonal to the approach
    assert np.allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3))
    assert np.allclose(pose[:3, 3], [1., 2., 3.])


def test_obb_grasp_closes_across_the_short_side_from_the_top():
    box = np.eye(4)
    box[:3, 3] = [0., 0., .05]
    info = compute_grasp_info_by_obb((box, np.array([.2, .04, .1])), depth=.02)
    assert np.allclose(np.abs(info['closing']), [0., 1., 0.])
    # The top face is at z = 0.10; the TCP goes 2 cm into the box.
    assert info['center'][2] == pytest.approx(.08)
    assert np.allclose(info['extents'], [.1, .04, .2])
