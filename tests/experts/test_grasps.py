import numpy as np
import pytest

from table_1000.experts.grasps import Box, Cylinder, build_grasp_pose, compute_grasp_info_by_obb, lengthwise


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


def test_only_the_length_of_an_elongated_shape_is_not_squeezed():
    step = np.radians(15)
    assert lengthwise(list(Box((.02, .02, .179)).closings(step))) == .179        # a pen, not by its ends
    assert lengthwise(list(Cylinder(.01, .18).closings(step))) == .18
    assert lengthwise(list(Box((.12, .048, .015)).closings(step))) is None       # a flat handle, every way
    assert lengthwise(list(Cylinder(.04, .05).closings(step))) is None
    assert lengthwise(list(Box((.048, .01, .012)).closings(step))) is None       # a drawer handle, by its ends
