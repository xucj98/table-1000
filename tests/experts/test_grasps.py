import numpy as np
import pytest

from table_1000.experts.grasps import Box, Cylinder, build_grasp_pose, compute_grasp_info_by_obb, fitted, lengthwise


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


def _prism(section, length=.15):
    """Vertices of a prism of the 2-D ``section`` points along Z."""
    section = np.asarray(section, dtype=float)
    return np.vstack([np.c_[section, np.full(len(section), z)] for z in (-length / 2, length / 2)])


def _rounded_square(side, corner, count=6):
    centres = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    return [(sx * (side / 2 - corner) + corner * np.cos(a), sy * (side / 2 - corner) + corner * np.sin(a))
            for k, (sx, sy) in enumerate(centres) for a in np.radians(np.linspace(90 * k, 90 * k + 90, count))]


def test_fitted_takes_a_polygonal_barrel_for_a_cylinder_and_a_rounded_square_for_a_box():
    turns = np.linspace(0., 2 * np.pi, 12, endpoint=False)
    shape, _ = fitted(_prism(np.c_[np.cos(turns), np.sin(turns)] * .009))
    assert isinstance(shape, Cylinder) and shape.radius == pytest.approx(.009)
    shape, _ = fitted(_prism(_rounded_square(.02, .006)))
    assert isinstance(shape, Box)
