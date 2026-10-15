"""Control arithmetic of the action terms, on ``[N, ...]`` torch tensors (no simulator).

One action per control step becomes targets at each of ``substeps`` physics
steps (see :mod:`table_1000.sim.isaaclab.actions`).
"""

import torch


def interpolate(start, target, substep, substeps, dt):
    """Joint targets after physics step ``substep`` (1..substeps) of a control step moving
    linearly from ``start`` to ``target``, and the matching velocity as feed-forward."""
    alpha = min(substep, substeps) / substeps
    return start + alpha * (target - start), (target - start) / (substeps * dt)


def finger_targets(gap, signs, max_gap):
    """``[N, 2]`` finger targets ``sign * g / 2`` of pad gaps ``[N, 1]``, clipped to ``[0, max_gap]``."""
    return torch.clamp(gap, 0., max_gap) * signs / 2


def clamp_lag(pose, measured, lag):
    """Integrated base ``pose`` ``[N, 3]`` (x, y, yaw) kept within ``lag`` (m, rad) of ``measured``."""
    pose = pose.clone()
    offset = pose[:, :2] - measured[:, :2]
    distance = torch.linalg.norm(offset, dim=1, keepdim=True)
    scale = torch.clamp(lag[0] / torch.clamp(distance, min=1e-12), max=1.)
    pose[:, :2] = measured[:, :2] + offset * scale
    pose[:, 2] = measured[:, 2] + torch.clamp(pose[:, 2] - measured[:, 2], -lag[1], lag[1])
    return pose


def base_substep(pose, speed, command, max_acceleration, dt):
    """One physics step of a differential base: ``speed`` ``(v, w)`` approaches ``command`` at most
    at ``max_acceleration``; returns the new pose, speed and world velocity ``(vx, vy, w)``."""
    speed = speed + torch.clamp(command - speed, -max_acceleration * dt, max_acceleration * dt)
    v, w = speed[:, 0], speed[:, 1]
    yaw = pose[:, 2]
    velocity = torch.stack([v * torch.cos(yaw), v * torch.sin(yaw), w], dim=1)
    return pose + velocity * dt, speed, velocity


def wheel_speeds(speed, track, radius):
    """Left and right wheel angular speeds ``[N, 2]`` of a differential drive at ``(v, w)``."""
    v, w = speed[:, 0], speed[:, 1]
    return torch.stack([v - w * track / 2, v + w * track / 2], dim=1) / radius
