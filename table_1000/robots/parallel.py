"""GPU robot environments in parallel: clones given the same inputs agree, distinct inputs are tracked
independently, resetting one clone leaves the others, and clone zero matches a single environment."""

import numpy as np


def check_parallel(spec, settings, assets, root, num_envs=4):
    """Run the parallel-environment checks on CUDA PhysX (in a started app); returns the result with
    ``status`` ``'PASS'`` (a failed check raises)."""
    import torch
    from table_1000.robots.checks import check_scene
    from table_1000.sim.isaaclab.env import RobotEnv, apply_layout
    scene = check_scene(spec, settings)
    result = {'num_envs': num_envs}
    opened = []

    def open_env(count):
        opened.append(RobotEnv(scene, spec, assets, device='cuda:0', num_envs=count, root=root))
        return opened[-1]

    def close(env):
        opened.remove(env)
        env.close()

    try:
        env = open_env(num_envs)
        obs = env.reset(scene.placement())
        assert obs['agent']['qpos'].shape == (num_envs, len(env.joints))
        for _ in range(50):
            env.step()
        # All clones start from the same local scene and receive the same inputs.
        q = env.obs['agent']['qpos']
        result['same_input_joint_spread'] = float(np.max(np.abs(q - q[0])))
        held_q = q[0].copy()
        assert result['same_input_joint_spread'] < .001
        for name, poses in env.privileged()['poses'].items():
            np.testing.assert_allclose(poses, np.broadcast_to(poses[0], poses.shape), atol=.001, rtol=0)
        arm = spec.primary_arm
        joints = spec.arms[arm].joints
        initial = env.qpos(joints).copy()
        target = initial.copy()
        offsets = np.linspace(-.12, .12, num_envs)
        target[:, 0] += offsets
        for step in range(1, 101):
            env.step({arm: initial + (target - initial) * min(step / 50, 1.)})
        measured = env.qpos(joints)[:, 0] - initial[:, 0]
        result['requested_offsets_rad'] = offsets.tolist()
        result['measured_offsets_rad'] = measured.tolist()
        result['tracking_error_rad'] = float(np.max(np.abs(measured - offsets)))
        assert result['tracking_error_rad'] < .02
        moved_q = env.obs['agent']['qpos'][0].copy()
        # Reset only environment zero. Other clones must retain state AND targets.
        before = env.robot.data.joint_pos[1:].clone()
        targets_before = env.robot.data.joint_pos_target[1:].clone()
        poses_before = env.privileged()['poses']
        block = np.eye(4)
        block[:3, 3] = (4., 4., 2.)
        env.env.pending_reset = (scene.placement(block=block), env.initial_robot_q(), {})
        apply_layout(env.env, torch.tensor([0], device=env.device))
        torch.testing.assert_close(env.robot.data.joint_pos[1:], before, rtol=0, atol=0)
        torch.testing.assert_close(env.robot.data.joint_pos_target[1:], targets_before, rtol=0, atol=0)
        poses = env.privileged()['poses']
        for name in poses:
            np.testing.assert_allclose(poses[name][1:], poses_before[name][1:], atol=1e-6, rtol=0)
        for _ in range(5):
            env.step()
        np.testing.assert_allclose(env.privileged()['poses']['block'][0], block, atol=1e-5, rtol=0)
        result['partial_reset_preserved_other_states_and_targets'] = True
        result['kinematic_reset_persisted_after_steps'] = True
        close(env)
        # Compare an independent N=1 simulation against clone zero under the
        # same layout and action sequence, rather than only comparing clones.
        env = open_env(1)
        env.reset(scene.placement())
        for _ in range(50):
            env.step()
        hold_error = float(np.max(np.abs(env.obs['agent']['qpos'][0] - held_q)))
        single_initial = env.qpos(joints).copy()
        single_target = single_initial.copy()
        single_target[:, 0] += offsets[0]
        for step in range(1, 101):
            env.step({arm: single_initial + (single_target - single_initial) * min(step / 50, 1.)})
        move_error = float(np.max(np.abs(env.obs['agent']['qpos'][0] - moved_q)))
        result['single_vs_multiple_max_joint_error_rad'] = max(hold_error, move_error)
        assert result['single_vs_multiple_max_joint_error_rad'] < .001
        close(env)
    finally:
        # A failed check leaves no environment behind for the next case of the process.
        for env in list(opened):
            close(env)
    # Every check above raises when it fails.
    result['checks'] = {'same_inputs_agree': True, 'distinct_inputs_tracked': True, 'partial_reset_keeps_others': True,
                        'matches_single_environment': True}
    result['status'] = 'PASS'
    return result
