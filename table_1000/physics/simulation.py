"""Rigid state and force batching for official PhysX step subscriptions."""

from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def rotation(quaternion):
    w, x, y, z = quaternion
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def enable_scripting():
    """Enable the official component once for this application."""
    import carb
    import omni.kit.app
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('omni.kit.scripting', True)
    carb.settings.get_settings().set_bool('/app/scripting/ignoreWarningDialog', True)


class Body:
    def __init__(self, instance, name):
        self.instance, self.name = instance, name
        self.key = (instance.name, name)
        self.path = instance.paths[name]

    @property
    def state(self):
        return self.instance.session.states[self.key]


class Asset:
    def __init__(self, session, path, name):
        from pxr import Usd, UsdPhysics
        from isaacsim.core.prims import RigidPrim
        self.session, self.path, self.name = session, path, name
        self.paths = {prim.GetCustomDataByKey('table1000:name'): str(prim.GetPath())
                      for prim in Usd.PrimRange(session.stage.GetPrimAtPath(path)) if prim.GetCustomDataByKey('table1000:name')}
        self.body_names = [prim.GetCustomDataByKey('table1000:name') for prim in Usd.PrimRange(session.stage.GetPrimAtPath(path))
                           if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.joint_names = [prim.GetCustomDataByKey('table1000:name') for prim in Usd.PrimRange(session.stage.GetPrimAtPath(path))
                            if prim.IsA(UsdPhysics.Joint)]
        self.indices = {name: i for i, name in enumerate(self.body_names)}
        self.view = RigidPrim([self.paths[name] for name in self.body_names], name=name, reset_xform_properties=False)
        roots = [prim for prim in Usd.PrimRange(session.stage.GetPrimAtPath(path))
                 if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
        self.articulation_root = str(roots[0].GetPath()) if roots else None
        self.articulation = None
        self.root_index = self.indices[roots[0].GetCustomDataByKey('table1000:name')] if roots else None

    def body(self, name):
        from table_1000.assets.references import expand_names
        names = expand_names(name, self.body_names)
        assert len(names) == 1
        return Body(self, names[0])

    def joint(self, name):
        from table_1000.assets.references import expand_names
        names = expand_names(name, self.joint_names)
        assert len(names) == 1
        return self.session.stage.GetPrimAtPath(self.paths[names[0]])

    def behavior(self):
        # The official manager owns these instances; this only reads them.
        from omni.kit.scripting import ScriptManager
        return next(iter(ScriptManager.get_instance()._prim_to_scripts[self.path].values()))

    def initialize(self):
        if self.articulation_root:
            from isaacsim.core.simulation_manager import SimulationManager
            self.articulation = SimulationManager.get_physics_sim_view().create_articulation_view(self.articulation_root)
        self.view.initialize()
        self.local_com = np.asarray(self.view.get_coms()[0]).reshape(-1, 3)
        self.local_inertia = np.asarray(self.view.get_inertias()).reshape(-1, 3, 3)
        self.mass = np.asarray(self.view.get_masses()).reshape(-1)

    def restore_body_poses(self, positions, quaternions):
        if self.articulation:
            # Reduced coordinates forbid writes to non-root link transforms.
            i = self.root_index
            transforms = np.c_[positions[i:i+1], quaternions[i:i+1, [1, 2, 3, 0]]].astype(np.float32)
            self.articulation.set_root_transforms(transforms, np.array([0], dtype=np.int32))
        else:
            self.view.set_world_poses(positions, quaternions)

    def restore_body_velocities(self, velocities):
        if self.articulation:
            i = self.root_index
            self.articulation.set_root_velocities(velocities[i:i+1], np.array([0], dtype=np.int32))
        else:
            self.view.set_velocities(velocities)

    def snapshot(self):
        state = (*self.view.get_world_poses(), self.view.get_velocities())
        if self.articulation:
            state += (self.articulation.get_dof_positions().copy(), self.articulation.get_dof_velocities().copy())
        return state

    def restore(self, state):
        self.restore_body_poses(state[0], state[1])
        self.restore_body_velocities(state[2])
        if self.articulation:
            indices = np.array([0], dtype=np.int32)
            self.articulation.set_dof_positions(state[3], indices)
            self.articulation.set_dof_velocities(state[4], indices)


_sessions = {}


def session_for(stage):
    key = stage.GetRootLayer().identifier
    if key not in _sessions:
        _sessions[key] = PhysicsStep(stage, key)
    return _sessions[key]


class PhysicsStep:
    """Actions, official asset callbacks, then one batched force application.

    This helper handles physical state, not script discovery or assembly rules.
    Official BehaviorScript instances own their own middle-step subscriptions.
    """
    def __init__(self, stage, key):
        from omni.physx import get_physx_interface
        self.stage, self.key = stage, key
        self.instances, self.states = {}, {}
        self.actions = None
        self.time, self.steps = 0., 0
        self.callback_steps = 0
        self.ready = False
        self.active = True
        self.error = None
        physx = get_physx_interface()
        self.prepare_subscription = physx.subscribe_physics_on_step_events(self.prepare, True, -100)
        self.flush_subscription = physx.subscribe_physics_on_step_events(self.flush, True, 100)

    def register(self, path, name=None):
        found = next((asset for asset in self.instances.values() if asset.path == path), None)
        if found:
            return found
        name = name or path.rsplit('/', 1)[-1]
        asset = Asset(self, path, name)
        self.instances[name] = asset
        self.ready = False
        return asset

    def load(self, asset, name='asset', position=(0, 0, 0), quaternion=(1, 0, 0, 0)):
        from pxr import Gf, UsdGeom
        asset = Path(asset).resolve()
        if asset.is_dir():
            asset = asset / ('object.usda' if (asset / 'object.usda').exists() else 'object.usdz')
        path = '/World/Assets/' + name
        prim = UsdGeom.Xform.Define(self.stage, path)
        prim.GetPrim().GetReferences().AddReference(str(asset))
        transform = UsdGeom.Xformable(prim)
        transform.AddTranslateOp().Set(Gf.Vec3d(*position))
        transform.AddOrientOp().Set(Gf.Quatf(quaternion[0], Gf.Vec3f(*quaternion[1:])))
        return self.register(path, name)

    def initialize(self):
        for instance in self.instances.values():
            instance.initialize()
        self.refresh()
        self.ready = True

    def refresh(self):
        from pxr import UsdPhysics
        self.states = {}
        for instance in self.instances.values():
            positions, quaternions = instance.view.get_world_poses()
            velocities = instance.view.get_velocities()
            for i, name in enumerate(instance.body_names):
                r = rotation(quaternions[i])
                self.states[(instance.name, name)] = {
                    'position': positions[i].astype(float), 'quaternion': quaternions[i].astype(float),
                    'rotation': r, 'velocity': velocities[i].astype(float),
                    'com': positions[i] + r @ instance.local_com[i],
                    'inverse_mass': 1 / float(instance.mass[i]),
                    'inverse_inertia': np.linalg.inv(r @ instance.local_inertia[i] @ r.T),
                    'external_force': np.zeros(3), 'external_torque': np.zeros(3)}
        # Native world-fixed constraints determine mobility for supplemental
        # forces. Refresh restores finite mass/inertia immediately on release.
        by_path = {instance.paths[name]:(instance.name,name) for instance in self.instances.values() for name in instance.body_names}
        for prim in self.stage.Traverse():
            if prim.IsA(UsdPhysics.FixedJoint):
                joint = UsdPhysics.FixedJoint(prim)
                if not joint.GetJointEnabledAttr().Get():
                    continue
                paths = [joint.GetBody0Rel().GetTargets(),joint.GetBody1Rel().GetTargets()]
                for a,b in [paths,paths[::-1]]:
                    anchor = self.stage.GetPrimAtPath(a[0]) if a else None
                    fixed_anchor = not a or (anchor and (not anchor.HasAPI(UsdPhysics.RigidBodyAPI)
                        or UsdPhysics.RigidBodyAPI(anchor).GetKinematicEnabledAttr().Get()))
                    if b and str(b[0]) in by_path and fixed_anchor:
                        state = self.states[by_path[str(b[0])]]
                        state['inverse_mass'] = 0
                        state['inverse_inertia'] = np.zeros((3,3))

    def force(self, body, force, point=None, torque=None, external=True):
        key = body.key
        state = self.states[key]
        force = np.asarray(force, dtype=float)
        moment = np.zeros(3) if torque is None else np.asarray(torque, dtype=float)
        if point is not None:
            moment = moment + np.cross(np.asarray(point) - state['com'], force)
        self.forces[key] += force
        self.torques[key] += moment
        if external:
            state['external_force'] += force
            state['external_torque'] += moment

    def prepare(self, dt):
        from isaacsim.core.simulation_manager import SimulationManager
        # World.reset emits startup steps before Isaac has created its tensor
        # simulation view. Those are initialization, not experiment steps.
        if not self.active or SimulationManager.get_physics_sim_view() is None:
            return
        try:
            if not self.instances:
                return
            if not self.ready or not all(a.view.is_physics_handle_valid() for a in self.instances.values()):
                self.initialize()
            self.refresh()
            self.forces = {key: np.zeros(3) for key in self.states}
            self.torques = {key: np.zeros(3) for key in self.states}
            if self.actions:
                self.actions(self.time, dt)
            self.callback_steps += 1
        except Exception as error:
            self.error = error
            self.ready = False

    def flush(self, dt):
        if not self.active or not self.ready:
            return
        for asset in self.instances.values():
            keys = [(asset.name, name) for name in asset.body_names]
            asset.view.apply_forces_and_torques_at_pos(
                np.array([self.forces[key] for key in keys], dtype=np.float32),
                np.array([self.torques[key] for key in keys], dtype=np.float32),
                positions=np.array([self.states[key]['com'] for key in keys], dtype=np.float32), is_global=True)

    def step(self, world):
        self.active = True
        count = self.callback_steps
        world.step(render=False)
        if self.error:
            raise self.error
        assert self.callback_steps == count + 1, 'Expected one callback per physical step'
        self.steps += 1
        self.time = self.steps * world.get_physics_dt()
        self.refresh()

    def unregister(self, path):
        for name, asset in list(self.instances.items()):
            if asset.path == path:
                del self.instances[name]
        self.ready = False

    def close(self):
        self.prepare_subscription = self.flush_subscription = None
        self.instances.clear()
        _sessions.pop(self.key, None)
