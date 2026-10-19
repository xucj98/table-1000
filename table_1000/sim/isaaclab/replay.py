"""Replaying recorded episodes (:mod:`.recording`) by their states, for rendering.

Restoring the recorded state of a step (``InteractiveScene.reset_to``) puts
every body exactly where it was, without simulating: images rendered from a
trajectory file match the episode that was recorded, whatever the physics
would do on a second run.
"""

import json

import h5py
import torch

from table_1000.scenes.recordings import subtask


def read_tree(group, index=None):
    """Nested dict of ``[1, ...]`` tensors from an HDF5 group (row ``index`` of each dataset, or the
    whole dataset when it holds one row)."""
    out = {}
    for key, value in group.items():
        if isinstance(value, h5py.Group):
            out[key] = read_tree(value, index)
        else:
            row = value[()] if index is None else value[index:index + 1]
            out[key] = torch.as_tensor(row)
    return out


class Episode:
    """One ``demo_<id>`` group: its attributes and the state before each step."""

    def __init__(self, group):
        self.group, self.name = group, group.name.rsplit('/', 1)[-1]
        self.attrs = {key: json.loads(value) if isinstance(value, str) and value[:1] in '[{' else value
                      for key, value in group.attrs.items()}
        self.length = int(group.attrs['num_samples'])

    def state(self, t):
        """Scene state before step ``t`` (``t = length``: after the last step)."""
        return read_tree(self.group['initial_state']) if t == 0 else read_tree(self.group['states'], t - 1)

    def subtask(self, t):
        """Instruction of the subtask step ``t`` belongs to, or None."""
        return subtask(self.attrs, t, self.length)


def episodes(path):
    """``(file, {name: Episode})`` of a trajectory file, in demo order (keep the file open while reading)."""
    file = h5py.File(path, 'r')
    names = sorted(file['data'], key=lambda name: int(name.split('_')[1]))
    return file, {name: Episode(file['data'][name]) for name in names}


def env_args(file):
    return json.loads(file['data'].attrs['env_args'])


def set_state(env, state):
    """Put ``env`` (a :class:`~table_1000.sim.isaaclab.env.RobotEnv`, environment 0) in ``state``."""
    device = env.device

    def to_device(tree):
        return {key: to_device(value) if isinstance(value, dict) else value.to(device) for key, value in tree.items()}

    from omni.physx import get_physx_interface
    env.env.scene.reset_to(to_device(state), is_relative=True)
    env.env.sim.forward()
    # Without a physics step nothing writes the new poses to USD, which the renderer reads.
    get_physx_interface().update_transformations(False, True)
