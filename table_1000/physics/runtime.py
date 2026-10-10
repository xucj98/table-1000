"""Isaac asset loader: discover portable manifests and own per-instance callbacks.

Create this shared runtime once per world. Loading an asset requires only its
USDZ path and placement; scenes do not import asset behavior modules.
"""

from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def rotation(quaternion):
    w, x, y, z = quaternion
    return Rotation.from_quat([x, y, z, w]).as_matrix()


class Context:
    def __init__(self, runtime, instance):
        self.runtime, self.instance = runtime, instance

    def key(self, name):
        return name if isinstance(name, tuple) else (self.instance.name, name)

    def get_interface(self, name):
        key = self.key(name)
        instance = self.runtime.instances[key[0]]
        definition = instance.interfaces[key[1]]
        body = instance.body_key(definition['body'])
        state = self.runtime.state(body)
        pose = definition['pose']
        return {'key': key, 'body': body, 'role': definition['role'], 'compatible': definition['compatible'],
                'position': state['position'] + state['rotation'] @ pose[:3],
                'rotation': state['rotation'] @ rotation(pose[3:]), 'state': state}

    def interface_exists(self, name):
        key = self.key(name)
        instance = self.runtime.instances.get(key[0])
        return instance is not None and key[1] in instance.interfaces

    def match_interface(self, name, parameters):
        female = self.get_interface(name)
        female_key = female['key']
        candidates = []
        old = self.runtime.pairs.get(female_key)
        if old is not None:
            candidates.append(old)
        candidates += [key for key in self.runtime.interface_keys() if key != old]
        for key in candidates:
            male = self.get_interface(key)
            if male['role'] != 'male' or male['compatible'] != female['compatible']:
                continue
            if key in self.runtime.pairs.values() and self.runtime.pairs.get(female_key) != key:
                continue
            relative = male['rotation'].T @ (female['position'] - male['position'])
            opening = -relative[0]
            if (-.001 < opening < parameters['engagement_length_m']
                    and np.linalg.norm(relative[1:]) < parameters['radial_tolerance_m']
                    and male['rotation'][:, 0] @ female['rotation'][:, 0] > parameters['axis_cosine_min']):
                self.runtime.pairs[female_key] = key
                return female, male
        self.release_interface(name)
        return None

    def release_interface(self, name):
        self.runtime.pairs.pop(self.key(name), None)

    def apply_force(self, body, force, point=None, torque=None):
        self.runtime.apply_force(body, force, point, torque, external=False)


class Instance:
    def __init__(self, runtime, name, asset, position, quaternion):
        from pxr import Gf, Usd, UsdGeom
        from isaacsim.core.prims import RigidPrim
        self.runtime, self.name, self.asset = runtime, name, asset
        self.path = '/World/Assets/' + name
        prim = UsdGeom.Xform.Define(runtime.stage, self.path)
        prim.GetPrim().GetReferences().AddReference(str(asset))
        transform = UsdGeom.Xformable(prim)
        transform.AddTranslateOp().Set(Gf.Vec3d(*position))
        transform.AddOrientOp().Set(Gf.Quatf(quaternion[0], Gf.Vec3f(*quaternion[1:])))
        self.paths = {child.GetCustomDataByKey('table1000:name'): str(child.GetPath())
                      for child in Usd.PrimRange(prim.GetPrim()) if child.GetCustomDataByKey('table1000:name')}
        bodies = [child for child in Usd.PrimRange(prim.GetPrim()) if child.GetCustomDataByKey('table1000:role') == 'rigid']
        self.body_names = [child.GetCustomDataByKey('table1000:name') for child in bodies]
        self.indices = {body: i for i, body in enumerate(self.body_names)}
        self.view = RigidPrim([str(child.GetPath()) for child in bodies], name=name, reset_xform_properties=False)
        runtime_entry = prim.GetPrim().GetCustomDataByKey('table1000:runtime')
        self.interfaces, self.behaviors, self.load_evidence = {}, {}, []
        self.manifest_path = asset.parent / runtime_entry if runtime_entry else None
        if self.manifest_path:
            manifest = json.loads(self.manifest_path.read_text())
            if 'isaac' not in manifest['supported_backends']:
                raise ValueError('Asset runtime does not support Isaac')
            physics = json.loads((self.manifest_path.parent / manifest['physics']).read_text())
            from table_1000.physics.config import physical_config
            model = {'nodes': [{'name': child.GetCustomDataByKey('table1000:name'),
                               'rigid': child.GetCustomDataByKey('table1000:role') == 'rigid',
                               'part': child.GetCustomDataByKey('table1000:role') == 'part'}
                              for child in Usd.PrimRange(prim.GetPrim()) if child.GetCustomDataByKey('table1000:name')],
                     'joints': [{'name': child.GetCustomDataByKey('table1000:name')} for child in Usd.PrimRange(prim.GetPrim())
                                if child.GetCustomDataByKey('table1000:role') == 'joint']}
            physics = physical_config(physics, model)
            self.interfaces = physics.get('interfaces', {})
            context = Context(runtime, self)
            for behavior_name, definition in physics.get('behaviors', {}).items():
                module_name, class_name = definition['entry'].split(':')
                module_path = self.manifest_path.parent / manifest['modules'][module_name]
                spec = spec_from_file_location(f'table1000_asset_{name}_{behavior_name}', module_path)
                module = module_from_spec(spec)
                spec.loader.exec_module(module)
                self.behaviors[behavior_name] = getattr(module, class_name)(context, definition.get('bindings', {}), definition.get('parameters', {}))
                self.load_evidence.append({'behavior': behavior_name, 'module': str(module_path), 'class': class_name})
        cache = UsdGeom.XformCache()
        self.initial_positions, self.initial_quaternions = [], []
        for body in bodies:
            saved = Gf.Transform(cache.GetLocalToWorldTransform(body))
            q = saved.GetRotation().GetQuat()
            self.initial_positions.append(list(saved.GetTranslation()))
            self.initial_quaternions.append([q.GetReal(), *q.GetImaginary()])

    def body_key(self, name):
        return self.name, name

    def initialize(self):
        self.view.initialize()
        self.local_com = np.asarray(self.view.get_coms()[0]).reshape(-1, 3)
        self.local_inertia = np.asarray(self.view.get_inertias()).reshape(-1, 3, 3)
        self.mass = np.asarray(self.view.get_masses()).reshape(-1)

    def reset(self):
        self.view.set_world_poses(np.array(self.initial_positions, dtype=np.float32), np.array(self.initial_quaternions, dtype=np.float32))
        self.view.set_velocities(np.zeros((len(self.body_names), 6), dtype=np.float32))
        for behavior in self.behaviors.values():
            behavior.reset()

    def close(self):
        for behavior in self.behaviors.values():
            behavior.close()
        self.behaviors.clear()


class Runtime:
    """Global opt-in runtime; use load/reset/step/unload for ordinary consumption."""
    def __init__(self, world):
        import omni.usd
        self.world, self.stage = world, omni.usd.get_context().get_stage()
        self.instances, self.pairs = {}, {}
        self.external_actions = None
        self.observer = None
        self.ready = False
        self.callback_error = None
        self.time = 0
        self.steps = 0
        self.callback_steps = 0
        self._states, self._forces, self._torques = {}, {}, {}
        from omni.physx import get_physx_interface
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(self.callback, True, 0)

    def load(self, asset, name, position=(0, 0, 0), rotation=(1, 0, 0, 0)):
        instance = Instance(self, name, Path(asset).resolve(), position, rotation)
        self.instances[name] = instance
        return instance

    def initialize(self):
        self.world.reset()
        for instance in self.instances.values():
            instance.initialize()
            instance.reset()
        self.time = 0
        self.steps = 0
        self.refresh_states()
        self.ready = True

    def interface_keys(self):
        return [(name, key) for name, instance in self.instances.items() for key in instance.interfaces]

    def state(self, key):
        return self._states[key]

    def refresh_states(self):
        self._states = {}
        for name, instance in self.instances.items():
            positions, quaternions = instance.view.get_world_poses()
            velocities = instance.view.get_velocities()
            for i, body in enumerate(instance.body_names):
                r = rotation(quaternions[i]);key = (name, body)
                self._states[key] = {'position': positions[i].astype(float), 'quaternion': quaternions[i].astype(float),
                    'rotation': r, 'velocity': velocities[i].astype(float), 'com': positions[i] + r @ instance.local_com[i],
                    'inverse_mass': 1 / float(instance.mass[i]), 'inverse_inertia': np.linalg.inv(r @ instance.local_inertia[i] @ r.T),
                    'external_force': np.zeros(3), 'external_torque': np.zeros(3)}

    def apply_force(self, key, force, point=None, torque=None, external=True):
        state = self.state(key)
        force = np.asarray(force, dtype=float)
        moment = np.zeros(3) if torque is None else np.asarray(torque, dtype=float)
        if point is not None:
            moment = moment + np.cross(np.asarray(point) - state['com'], force)
        self._forces[key] += force
        self._torques[key] += moment
        if external:
            state['external_force'] += force
            state['external_torque'] += moment

    def before_step(self, dt):
        if not self.ready or not self.instances or not all(instance.view.is_physics_handle_valid() for instance in self.instances.values()):
            return
        self.refresh_states()
        self._forces = {key: np.zeros(3) for key in self._states}
        self._torques = {key: np.zeros(3) for key in self._states}
        if self.external_actions:
            self.external_actions(self.time, dt)
        for instance in self.instances.values():
            for behavior in instance.behaviors.values():
                behavior.before_step(dt)
        for name, instance in self.instances.items():
            keys = [instance.body_key(body) for body in instance.body_names]
            instance.view.apply_forces_and_torques_at_pos(
                np.array([self._forces[key] for key in keys], dtype=np.float32),
                np.array([self._torques[key] for key in keys], dtype=np.float32),
                positions=np.array([self._states[key]['com'] for key in keys], dtype=np.float32), is_global=True)
        if self.observer:
            self.observer()

    def callback(self, dt):
        # Kit logs callback exceptions and continues. Propagate the actual
        # failure to the synchronous consumer rather than report completion.
        try:
            self.before_step(dt)
            if self.ready:
                self.callback_steps += 1
        except Exception as error:
            self.callback_error = error
            self.ready = False

    def step(self, render=False):
        dt = self.world.get_physics_dt()
        callbacks = self.callback_steps
        self.world.step(render=render)
        if self.callback_error:
            raise self.callback_error
        if self.callback_steps != callbacks + 1:
            raise RuntimeError('Expected exactly one asset callback per physical step')
        self.steps += 1
        self.time = self.steps * dt
        self.refresh_states()
        for instance in self.instances.values():
            for behavior in instance.behaviors.values():
                behavior.after_step(dt)

    def reset(self):
        self.ready = False
        self.callback_error = None
        self.pairs.clear()
        self.world.reset()
        for instance in self.instances.values():
            instance.initialize()
            instance.reset()
        self.time = 0
        self.steps = 0
        self.refresh_states()
        self.ready = True

    def unload(self, name):
        # Removing a shape invalidates Isaac's global tensor view. Stop before
        # changing the stage, then rebuild handles and retain the other assets'
        # current states instead of silently resetting them to their saved pose.
        remaining = {key: (*item.view.get_world_poses(), item.view.get_velocities())
                     for key, item in self.instances.items() if key != name}
        self.ready = False
        self.world.stop()
        instance = self.instances.pop(name)
        instance.close()
        self.pairs = {female: male for female, male in self.pairs.items() if female[0] != name and male[0] != name}
        self.stage.RemovePrim(instance.path)
        if self.instances:
            self.world.play()
            for key, item in self.instances.items():
                item.initialize()
                positions, quaternions, velocities = remaining[key]
                item.view.set_world_poses(positions, quaternions)
                item.view.set_velocities(velocities)
            self.refresh_states()
            self.ready = True

    def close(self):
        for name in list(self.instances):
            self.unload(name)
        self.subscription = None
        self.world.stop()
