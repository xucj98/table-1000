"""Screw-motion planning on an MPlib planner.

Adapted from MPlib 0.2.1 ``Planner.plan_screw`` (MIT, Copyright (c) 2021 Hao Su's
Lab, UCSD). That release has two defects corrected here:

- its SE(3) logarithm treats every rotation whose trace is ``np.isclose`` to 3 as
  zero, so a goal rotated by a few mrad is planned as a pure translation;
- it subtracts each linearized step from a fixed initial residual, so the end
  pose drifts from the goal on curved paths without being detected.

Here the residual is recomputed from forward kinematics at every step, and the
plan succeeds only once the planner FK is within tolerance of the goal. Joint
limit, collision and time-parameterization failures are returned, never relaxed.
"""

import numpy as np
from scipy.spatial.transform import Rotation


def _skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_coordinates(transform):
    """Return ``(rho, phi)`` with ``expm([[phi]x, rho], [0, 0]) == transform``."""
    transform = np.asarray(transform, dtype=float)
    phi = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    theta = float(np.linalg.norm(phi))
    # Coefficient of [phi]x^2 in the inverse left Jacobian; its limit is 1/12.
    if theta < 1e-4:
        coefficient = 1. / 12. + theta ** 2 / 720.
    else:
        coefficient = (1. - theta / (2. * np.tan(theta / 2.))) / theta ** 2
    skew = _skew(phi)
    rho = (np.eye(3) - .5 * skew + coefficient * skew @ skew) @ transform[:3, 3]
    return np.r_[rho, phi]


def endpoint_error(planner, ee_index, goal, qpos):
    """Position (m) and rotation (rad) error of the planner FK from ``goal``."""
    planner.pinocchio_model.compute_forward_kinematics(qpos)
    actual = planner.pinocchio_model.get_link_pose(ee_index).to_transformation_matrix()
    target = goal.to_transformation_matrix()
    if not np.isfinite(actual).all():
        return float('inf'), float('inf')
    rotation = Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude()
    return float(np.linalg.norm(actual[:3, 3] - target[:3, 3])), float(rotation)


LIMIT_GAIN = .01   # null-space pull of redundant joints toward their mid-range, per step


def plan_screw(planner, goal, qpos, *, qpos_step=.03, time_step=.004,
               position_tolerance=1e-3, rotation_tolerance=1e-3, wrt_world=True,
               max_iterations=2000, locked=()):
    """Move along the screw from the current to the ``goal`` pose.

    ``goal`` is an ``mplib.Pose`` of the planner's move-group link. Each step
    moves at most ``qpos_step`` (joint-space norm) toward the remaining screw
    residual; joints at the full-configuration indices ``locked`` stay put.
    Returns MPlib's result dictionary; a failure has ``status != 'Success'``
    and a ``reason``.
    """
    # Measured joints can sit a few mrad past a limit; like MPlib's plan_pose,
    # start from the clipped state.
    qpos = np.clip(planner.pad_move_group_qpos(np.asarray(qpos, dtype=float).copy()),
                   planner.joint_limits[:, 0], planner.joint_limits[:, 1])
    planner.robot.set_qpos(qpos, True)
    if wrt_world:
        goal = planner._transform_goal_to_wrt_base(goal)
    ee_index = planner.link_name_2_idx[planner.move_group]
    joints = planner.move_group_joint_indices
    limits = planner.joint_limits
    path = [qpos[joints].copy()]

    for iteration in range(max_iterations + 1):
        position_error, rotation_error = endpoint_error(planner, ee_index, goal, qpos)
        state = {'iterations': iteration, 'q': qpos[joints].tolist(),
                 'position_error_m': position_error, 'rotation_error_rad': rotation_error}
        if position_error <= position_tolerance and rotation_error <= rotation_tolerance:
            break
        current = planner.pinocchio_model.get_link_pose(ee_index)
        residual = exp_coordinates((goal * current.inv()).to_transformation_matrix())
        planner.pinocchio_model.compute_full_jacobian(qpos)
        jacobian = planner.pinocchio_model.get_link_jacobian(ee_index, local=False)
        jacobian[:, list(locked)] = 0.
        inverse = np.linalg.pinv(jacobian)
        delta_q = inverse @ residual
        # Redundant joints (a free torso lift) keep away from the limits in the null space.
        free = [i for i in joints if i not in set(locked)]
        if len(free) > 6:
            middle, half = limits.mean(1), np.maximum(np.ptp(limits, 1) / 2, 1e-6)
            away = np.zeros_like(qpos)
            away[free] = ((middle - qpos) / half ** 2)[free] * LIMIT_GAIN
            delta_q += (np.eye(len(qpos)) - inverse @ jacobian) @ away
        norm = float(np.linalg.norm(delta_q))
        if not np.isfinite(norm) or norm < 1e-7:
            return {'status': 'screw plan failed', 'reason': 'no_progress', **state}
        qpos += delta_q * min(1., qpos_step / norm)
        state['q'] = qpos[joints].tolist()
        if not np.all((qpos >= limits[:, 0] - 1e-3) & (qpos <= limits[:, 1] + 1e-3)):
            return {'status': 'screw plan failed', 'reason': 'joint_limit', **state}
        planner.planning_world.set_qpos_all(qpos[joints])
        if planner.planning_world.is_state_colliding():
            pairs = sorted({(c.link_name1, c.object_name2, c.link_name2)
                            for c in planner.planning_world.check_collision()})
            return {'status': 'screw plan failed', 'reason': 'collision', 'collisions': pairs, **state}
        path.append(qpos[joints].copy())
    else:
        return {'status': 'screw plan failed', 'reason': 'iteration_limit', **state}

    if len(path) == 1:
        path.append(path[0])
    times, positions, velocities, accelerations, duration = planner.TOPP(np.vstack(path), time_step)
    positions = np.asarray(positions)
    if len(positions) < 2 or not np.isfinite(positions).all():
        return {'status': 'screw plan failed', 'reason': 'time_parameterization_samples', **state}
    # MPlib's pad_move_group_qpos assumes the move group leads the full configuration;
    # here it need not (a second arm's chain), so the end is written at the group's indices.
    end = qpos.copy()
    end[joints] = positions[-1]
    timed = endpoint_error(planner, ee_index, goal, end)
    if timed[0] > position_tolerance or timed[1] > rotation_tolerance:
        return {'status': 'screw plan failed', 'reason': 'time_parameterization_endpoint',
                'position_error_m': timed[0], 'rotation_error_rad': timed[1]}
    return {'status': 'Success', 'position': positions, 'velocity': velocities,
            'acceleration': accelerations, 'time': times, 'duration': duration, **state}
