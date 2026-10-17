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


class RigidView(TensorView):
    def __init__(self, paths, device):
        self.paths, self.device = paths, device
        self.view = None

    def initialize(self):
        from isaacsim.core.simulation_manager import SimulationManager
        self.view = SimulationManager.get_physics_sim_view().create_rigid_body_view(self.paths)
        self.subsets = {}

    def subset(self, indices):
        # GPU setters reject a whole view containing non-root articulation
        # links, even when the supplied indices select independent bodies.
        from isaacsim.core.simulation_manager import SimulationManager
        key = tuple(map(int, indices))
        if key not in self.subsets:
            self.subsets[key] = TensorView(
                SimulationManager.get_physics_sim_view().create_rigid_body_view([self.paths[i] for i in key]),
                self.device)
        return self.subsets[key]

    def is_physics_handle_valid(self):
        return self.view is not None and self.view.check()

    def get_world_poses(self):
        pose = self.get_transforms()
        return pose[:, :3], pose[:, [6, 3, 4, 5]]

    def get_coms(self):
        pose = self.view.get_coms().detach().cpu().numpy().copy()
        return pose[:, :3], pose[:, [6, 3, 4, 5]]

    def set_world_poses(self, positions, quaternions, indices=None):
        indices = np.arange(len(self.paths)) if indices is None else indices
        if not len(indices):
            return
        pose = np.c_[positions, np.asarray(quaternions)[:, [1, 2, 3, 0]]]
        self.subset(indices).set_transforms(pose, np.arange(len(indices)))

    def set_velocities(self, velocities, indices=None):
        indices = np.arange(len(self.paths)) if indices is None else indices
        if len(indices):
            self.subset(indices).set_velocities(np.asarray(velocities)[indices], np.arange(len(indices)))

    def apply_forces_and_torques_at_pos(self, forces, torques, positions, is_global=True):
        convert = lambda value: torch.as_tensor(value, dtype=torch.float32, device=self.device)
        self.view.apply_forces_and_torques_at_position(
            convert(forces), convert(torques), convert(positions),
            torch.arange(len(self.paths), dtype=torch.int32, device=self.device), is_global)
