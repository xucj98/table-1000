"""Screw planning reaches small-rotation goals and keeps its failure gates.

Needs ``mplib`` (0.2.1) in the running Python environment.
"""

from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

from table_1000.planning.screw import _skew, exp_coordinates, plan_screw

AXES = ['0 0 1', '0 1 0', '0 1 0', '1 0 0', '0 1 0', '1 0 0']


def arm_urdf(path):
    """Six revolute joints, 0.2 m apart, with a ``tcp`` link 0.1 m past the last."""
    links = ['<link name="base"/>'] + [f'<link name="l{i}"/>' for i in range(1, 7)] + ['<link name="tcp"/>']
    joints = []
    for i, axis in enumerate(AXES, 1):
        parent = 'base' if i == 1 else f'l{i - 1}'
        joints.append(f'<joint name="j{i}" type="revolute"><parent link="{parent}"/><child link="l{i}"/>'
                      f'<origin xyz="0 0 {.1 if i == 1 else .2}"/><axis xyz="{axis}"/>'
                      '<limit lower="-3" upper="3" effort="10" velocity="1"/></joint>')
    joints.append('<joint name="tcp_fixed" type="fixed"><parent link="l6"/><child link="tcp"/>'
                  '<origin xyz=".1 0 0"/></joint>')
    path.write_text('<robot name="arm">' + ''.join(links + joints) + '</robot>')
    return path


def pose(transform):
    import mplib
    return mplib.Pose(transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]])


class ExpCoordinatesTest(unittest.TestCase):
    def test_matches_matrix_exponential(self):
        rng = np.random.default_rng(0)
        for angle in (0., 1e-9, 2.3295e-3, .3, 2., np.pi - 1e-3):
            axis = rng.normal(size=3)
            phi = angle * axis / np.linalg.norm(axis)
            rho = rng.normal(size=3)
            twist = np.zeros((4, 4))
            twist[:3, :3], twist[:3, 3] = _skew(phi), rho
            with self.subTest(angle=angle):
                np.testing.assert_allclose(exp_coordinates(expm(twist)), np.r_[rho, phi], atol=1e-9)


class ScrewPlanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mplib
        cls.directory = tempfile.TemporaryDirectory()
        urdf = arm_urdf(Path(cls.directory.name) / 'arm.urdf')
        cls.planner = mplib.Planner(str(urdf), 'tcp', joint_vel_limits=np.ones(6), joint_acc_limits=np.ones(6))
        cls.start = np.array([.1, .7, -1.4, .2, .6, -.1])

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def tcp(self, qpos):
        index = self.planner.link_name_2_idx['tcp']
        self.planner.pinocchio_model.compute_forward_kinematics(self.planner.pad_move_group_qpos(qpos))
        return self.planner.pinocchio_model.get_link_pose(index).to_transformation_matrix()

    def goal(self, rotation_rad, translation_m):
        target = self.tcp(self.start)
        target[:3, :3] = Rotation.from_rotvec([0., rotation_rad, 0.]).as_matrix() @ target[:3, :3]
        target[:3, 3] += translation_m
        return target

    def test_goal_is_reached_along_the_screw(self):
        # The r9 seat target differed from the reached pose by 2.33 mrad; native
        # MPlib 0.2.1 drops that rotation and also drifts on larger screws.
        start_inverse = np.linalg.inv(self.tcp(self.start))
        for rotation in (2.3295e-3, .2, .4):
            target = self.goal(rotation, [.003, -.002, .01])
            with self.subTest(rotation=rotation):
                result = plan_screw(self.planner, pose(target), self.start, wrt_world=False)
                self.assertEqual(result['status'], 'Success', result.get('reason'))
                reached = self.tcp(result['position'][-1])
                self.assertLess(np.linalg.norm(reached[:3, 3] - target[:3, 3]), 1e-3)
                self.assertLess(Rotation.from_matrix(target[:3, :3].T @ reached[:3, :3]).magnitude(), 1e-3)
                screw = exp_coordinates(target @ start_inverse)
                screw /= np.linalg.norm(screw)
                for qpos in result['position']:
                    xi = exp_coordinates(self.tcp(qpos) @ start_inverse)
                    self.assertLess(np.linalg.norm(xi - screw * (screw @ xi)), 2e-3)

    def test_joint_limit_is_a_failure(self):
        limits = self.planner.joint_limits.copy()
        try:
            self.planner.joint_limits[1, 1] = self.start[1] + 1e-3
            result = plan_screw(self.planner, pose(self.goal(0., [.05, 0., -.05])), self.start, wrt_world=False)
        finally:
            self.planner.joint_limits[:] = limits
        self.assertEqual(result['reason'], 'joint_limit')


if __name__ == '__main__':
    unittest.main()
