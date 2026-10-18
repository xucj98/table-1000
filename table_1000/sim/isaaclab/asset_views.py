"""NumPy observations over PhysX torch views; all simulator writes use tensors.

Asset test functions use SI NumPy values. Conversion belongs at this boundary,
including CPU tensors for PhysX property setters and device tensors for state.
Import after starting the Isaac Lab application.
"""

import numpy as np
import torch


class TensorView:
    """Preserve the small asset-test view API on CPU and CUDA."""

    def __init__(self, view, device):
        self.view, self.device = view, device

    def __getattr__(self, name):
        attr = getattr(self.view, name)
        if not callable(attr):
            return attr
        if name.startswith('get_'):
            def get(*args, **kwargs):
                return attr(*args, **kwargs).detach().cpu().numpy().copy()
            return get
        if name.startswith('set_'):
            def set(data, indices):
                # Properties (unlike state and targets) are CPU buffers in PhysX.
                getter = getattr(self.view, name.replace('set_', 'get_', 1), None)
                device = getter().device if getter is not None else self.device
                return attr(torch.as_tensor(data, dtype=torch.float32, device=device),
                            torch.as_tensor(indices, dtype=torch.int32, device=device))
            return set
        return attr


class ReplicatedView(TensorView):
    """One articulation per environment; reads environment 0, writes all.

    Environments replicate environment 0 with identical inputs, so every
    write is broadcast; root transforms are shifted by each environment offset.
    """

    def __init__(self, view, device, offsets):
        super().__init__(view, device)
        self.offsets = np.asarray(offsets, dtype=np.float32)

    def __getattr__(self, name):
        attr = TensorView.__getattr__(self, name)
        if name.startswith('get_') and callable(attr):
            return lambda *args, **kwargs: attr(*args, **kwargs)[:1]
        if name.startswith('set_') and callable(attr):
            def set(data, indices):
                assert list(np.asarray(indices)) == [0], 'Write environment 0; replicas follow'
                data = np.repeat(np.asarray(data, dtype=np.float32)[:1], len(self.offsets), axis=0)
                if name == 'set_root_transforms':
                    data[:, :3] += self.offsets
                return attr(data, np.arange(len(self.offsets)))
            return set
        return attr


class RigidView(TensorView):
    """Rigid bodies of environment 0, replicated in every environment.

    ``replicas`` lists the same bodies' paths per environment (environment 0
    first) with each environment's world offset. Getters return environment 0;
    setters and forces apply to every environment, shifted by its offset.
    """

    def __init__(self, paths, device, replicas=None, offsets=None):
        self.paths, self.device = paths, device
        self.replicas = replicas or [paths]
        self.offsets = np.zeros((1, 3), np.float32) if offsets is None else np.asarray(offsets, dtype=np.float32)
        self.view = None

    def __getattr__(self, name):
        attr = TensorView.__getattr__(self, name)
        if name.startswith('get_') and callable(attr):
            return lambda *args, **kwargs: attr(*args, **kwargs)[:len(self.paths)]
        return attr

    def initialize(self):
        from isaacsim.core.simulation_manager import SimulationManager
        self.view = SimulationManager.get_physics_sim_view().create_rigid_body_view(
            [path for paths in self.replicas for path in paths])
        self.subsets = {}

    def subset(self, indices):
        # GPU setters reject a whole view containing non-root articulation
        # links, even when the supplied indices select independent bodies.
        from isaacsim.core.simulation_manager import SimulationManager
        key = tuple(map(int, indices))
        if key not in self.subsets:
            self.subsets[key] = TensorView(
                SimulationManager.get_physics_sim_view().create_rigid_body_view(
                    [paths[i] for paths in self.replicas for i in key]), self.device)
        return self.subsets[key]

    def tile(self, values, shift=False):
        """Environment-major copies of per-body values, optionally shifted by the offsets."""
        values = np.asarray(values, dtype=np.float32)
        tiled = np.concatenate([values + (offset if shift else 0) for offset in self.offsets])
        return tiled

    def is_physics_handle_valid(self):
        return self.view is not None and self.view.check()

    def get_world_poses(self):
        pose = self.get_transforms()
        return pose[:, :3], pose[:, [6, 3, 4, 5]]

    def get_coms(self):
        pose = self.view.get_coms().detach().cpu().numpy().copy()[:len(self.paths)]
        return pose[:, :3], pose[:, [6, 3, 4, 5]]

    def replica_deviation(self):
        """Largest position difference (m) of any replica from environment 0."""
        pose = self.view.get_transforms().detach().cpu().numpy().reshape(len(self.offsets), len(self.paths), 7)
        return float(np.abs(pose[:, :, :3] - self.offsets[:, None] - pose[:1, :, :3]).max())

    def set_world_poses(self, positions, quaternions, indices=None):
        indices = np.arange(len(self.paths)) if indices is None else indices
        if not len(indices):
            return
        pose = np.c_[self.tile(positions, shift=True), self.tile(np.asarray(quaternions)[:, [1, 2, 3, 0]])]
        self.subset(indices).set_transforms(pose, np.arange(len(pose)))

    def set_velocities(self, velocities, indices=None):
        indices = np.arange(len(self.paths)) if indices is None else indices
        if len(indices):
            values = self.tile(np.asarray(velocities)[indices])
            self.subset(indices).set_velocities(values, np.arange(len(values)))

    def apply_forces_and_torques_at_pos(self, forces, torques, positions, is_global=True):
        """Apply per-body forces in every environment at ``positions[environment, body]`` (local)."""
        convert = lambda value: torch.as_tensor(value, dtype=torch.float32, device=self.device)
        count = len(self.paths) * len(self.offsets)
        points = (np.asarray(positions, dtype=np.float32) + self.offsets[:, None]).reshape(-1, 3)
        self.view.apply_forces_and_torques_at_position(
            convert(self.tile(forces)), convert(self.tile(torques)), convert(points),
            torch.arange(count, dtype=torch.int32, device=self.device), is_global)
