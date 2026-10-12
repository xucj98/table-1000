"""Grasp and placement frames of the drawer-and-pen task."""

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.robots.x1pro import TIP_BEYOND_TCP
from table_1000.tasks import drawer_pen_geometry as g


def random_pose(seed):
    rng = np.random.default_rng(seed)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('z', rng.uniform(-np.pi, np.pi)).as_matrix()
    pose[:3, 3] = rng.uniform(-1, 1, 3)
    return pose


def is_rotation(matrix):
    return np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(matrix), 1.)


class HandleGraspTest(unittest.TestCase):
    def test_frame_follows_drawer_pose(self):
        drawer = random_pose(1)
        centre = np.array([0., -.009, -.006])
        frame = g.handle_frame(drawer, centre)
        np.testing.assert_allclose(frame[:3, :3], drawer[:3, :3])
        np.testing.assert_allclose(frame[:3, 3], drawer[:3, :3] @ centre + drawer[:3, 3])

    def test_fingertips_stop_before_the_drawer_face(self):
        face = .007
        tcp = g.handle_tcp(face)
        self.assertTrue(is_rotation(tcp[:3, :3]))
        approach, opening = tcp[:3, 0], tcp[:3, 1]
        tip = tcp[:3, 3] + approach * TIP_BEYOND_TCP
        self.assertAlmostEqual(tip[1], face - g.HANDLE_TIP_CLEARANCE)
        # Fingers close along the handle, the approach leans down into the cabinet.
        np.testing.assert_allclose(np.abs(opening), [1, 0, 0], atol=1e-12)
        self.assertGreater(approach[1], 0)
        self.assertAlmostEqual(np.degrees(np.arcsin(-approach[2])), np.degrees(g.HANDLE_PITCH))
        # The approach line passes through the handle centre.
        self.assertAlmostEqual(np.linalg.norm(np.cross(tip, approach)), 0.)

    def test_standoff_moves_back_along_approach(self):
        tcp = random_pose(2)
        back = g.standoff(tcp, .08)
        np.testing.assert_allclose(back[:3, 3], tcp[:3, 3] - .08 * tcp[:3, 0])
        np.testing.assert_allclose(back[:3, :3], tcp[:3, :3])


class PenGraspTest(unittest.TestCase):
    def test_top_down_grasp_across_the_axis(self):
        pen = random_pose(3)
        pen[2, 3] = .765
        tcp = g.pen_tcp(pen, .755, -.006)
        self.assertTrue(is_rotation(tcp[:3, :3]))
        np.testing.assert_allclose(tcp[:3, 0], [0, 0, -1], atol=1e-12)
        self.assertAlmostEqual(abs(tcp[:3, 1] @ pen[:3, 0]), 0.)
        expected = pen[:3, :3] @ [-.006, 0, 0] + pen[:3, 3]
        np.testing.assert_allclose(tcp[:2, 3], expected[:2])
        self.assertAlmostEqual(tcp[2, 3], .755 + g.PEN_TIP_CLEARANCE + TIP_BEYOND_TCP)

    def test_grasp_point_is_kept_on_the_barrel(self):
        pen = np.eye(4)
        tcp = g.pen_tcp(pen, -1., -.09)
        self.assertAlmostEqual(tcp[0, 3], g.PEN_BARREL[0])


class PlacementTest(unittest.TestCase):
    def test_release_point_is_over_the_exposed_interior(self):
        drawer = random_pose(4)
        interior = ((-.126, .002), (.126, .258))
        point = g.drawer_release_point(drawer, interior, .115, .04)
        local = np.linalg.inv(drawer) @ np.r_[point, 1.]
        np.testing.assert_allclose(local[:3], [0., (.002 + .115) / 2, .04 + g.PEN_DROP + g.PEN_RADIUS], atol=1e-12)

    def test_place_tcp_puts_the_held_pen_at_the_point(self):
        tcp_pen = np.eye(4)
        # Pen axis along the TCP's Z (across the fingers), rolled about itself.
        tcp_pen[:3, :3] = Rotation.from_euler('y', -np.pi / 2).as_matrix() @ Rotation.from_euler('x', .3).as_matrix()
        tcp_pen[:3, 3] = [.01, 0., .002]
        point, axis = np.array([.3, -.2, .9]), np.array([1., 0., 0.])
        for tilt in (0., np.radians(40)):
            tcp = g.place_tcp(point, tcp_pen, axis, tilt, (0., 1., 0.))
            pen = tcp @ tcp_pen
            np.testing.assert_allclose(pen[:3, 3], point, atol=1e-12)
            self.assertAlmostEqual(abs(pen[:3, 0] @ axis), 1.)
            self.assertAlmostEqual(np.degrees(np.arccos(-tcp[2, 0])), np.degrees(tilt))
            if tilt:
                self.assertGreater(tcp[1, 0], 0.)


if __name__ == '__main__':
    unittest.main()
