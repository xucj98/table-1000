"""Check independent GPU robot environments, distinct actions, and partial reset."""

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot')
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--num-envs', type=int, default=4)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.num_envs < 2:
        parser.error('--num-envs must be at least 2')
    args.output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    app = start_app(gpu=args.gpu, device='cuda:0', cameras=False)
    result = {'robot': args.robot, 'device': 'cuda:0', 'num_envs': args.num_envs}
    env = None
    try:
        import numpy as np
        import torch
        from table_1000.robots import load_spec, load_robot_test
        from table_1000.robots.checks import check_scene
        from table_1000.sim.isaaclab.env import RobotEnv, apply_layout
        spec = load_spec(args.robot, ROOT)
        scene = check_scene(spec, load_robot_test(args.robot, ROOT))
        env = RobotEnv(scene, spec, ROOT / 'assets', device='cuda:0', num_envs=args.num_envs)
        obs = env.reset(scene.placement())
        assert obs['agent']['qpos'].shape == (args.num_envs, len(env.joints))
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
        offsets = np.linspace(-.12, .12, args.num_envs)
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
        env.env.pending_reset = (scene.placement(block=block), env.initial_robot_q())
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
        env.close()
        env = None
        import omni.usd
        omni.usd.get_context().new_stage()
        # Compare an independent N=1 simulation against clone zero under the
        # same layout and action sequence, rather than only comparing clones.
        env = RobotEnv(scene, spec, ROOT / 'assets', device='cuda:0', num_envs=1)
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
        result['status'] = 'PASS'
    except BaseException:
        result.update(status='ERROR', error=traceback.format_exc())
    finally:
        (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
        if env is not None:
            env.close()
        app.close(exit_code=int(result['status'] != 'PASS'))


if __name__ == '__main__':
    main()
