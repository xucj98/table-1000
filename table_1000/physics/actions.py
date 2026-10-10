"""Named test actions applied through the same runtime force accumulator."""

import numpy as np
from copy import deepcopy
from scipy.spatial.transform import Rotation, Slerp
from table_1000.physics.runtime import rotation


def interpolate(frames, time):
    for a, b in zip(frames, frames[1:]):
        if a['time'] <= time < b['time']:
            u = (time - a['time']) / (b['time'] - a['time'])
            result = {}
            for key, value in a.items():
                if key == 'time':
                    continue
                if key == 'rotation':
                    q = np.array([value, b[key]])[:, [1, 2, 3, 0]]
                    out = Slerp([0, 1], Rotation.from_quat(q))([u]).as_quat()[0]
                    result[key] = out[[3, 0, 1, 2]]
                else:
                    result[key] = np.asarray(value) * (1 - u) + np.asarray(b[key]) * u
            return result
    return None


class Actions:
    def __init__(self, runtime, instance, definitions):
        self.runtime, self.instance, self.definitions = runtime, instance, deepcopy(definitions)
        self.fixtures, self.records = {}, {}
        self.joint_actions, self.activated = {}, set()

    def activate(self, name, action):
        if name in self.activated:
            return
        kind = action['type']
        initial = {}
        if kind == 'fixture':
            state = self.runtime.state(self.instance.body_key(action['body']))
            initial = {'position': state['position'].copy(), 'rotation': state['quaternion'].copy()}
        elif kind.startswith('joint_'):
            from table_1000.physics.joints import frames
            initial = {'target_position': frames(self.instance, action['joint'])['position']}
        for frame in action['keyframes']:
            initial.update(frame)
            frame.update(deepcopy(initial))
        self.activated.add(name)

    def joint_action(self, name, action, target):
        from pxr import UsdPhysics
        from table_1000.physics.joints import frames
        info = frames(self.instance, action['joint'])
        prim, factor = info['prim'], info['factor']
        joint = (UsdPhysics.PrismaticJoint if info['slider'] else UsdPhysics.RevoluteJoint)(prim)
        lower, upper = joint.GetLowerLimitAttr(), joint.GetUpperLimitAttr()
        if not lower.Get() / factor <= target['target_position'] <= upper.Get() / factor:
            raise ValueError('Joint target outside authored limits')
        if name not in self.joint_actions:
            saved = {'joint': joint, 'limits': (lower.Get(), upper.Get())}
            if action['type'] == 'joint_lock':
                if abs(target['target_position'] - info['position']) > 1e-6:
                    raise ValueError('Joint lock must match the enabling coordinate')
                saved['target'] = target['target_position']
            else:
                axis = 'linear' if info['slider'] else 'angular'
                api = UsdPhysics.DriveAPI(prim, axis)
                saved['drive_was_present'] = bool(api)
                saved['axis'] = axis
                api = UsdPhysics.DriveAPI.Apply(prim, axis)
                attributes = [api.GetTypeAttr(), api.GetTargetPositionAttr(), api.GetStiffnessAttr(), api.GetDampingAttr(), api.GetMaxForceAttr()]
                saved['drive_attributes'] = [(attr, attr.Get(), attr.HasAuthoredValueOpinion()) for attr in attributes]
                for field, method in [('stiffness', 'CreateStiffnessAttr'), ('damping', 'CreateDampingAttr'), ('max_force', 'CreateMaxForceAttr')]:
                    if field in action:
                        getattr(api, method)(action[field] / factor if field != 'max_force' else action[field])
                    elif not saved['drive_was_present']:
                        raise ValueError('A new joint drive needs stiffness, damping and max_force')
                api.CreateTypeAttr('force')
                saved['drive'] = api
            self.joint_actions[name] = saved
        saved = self.joint_actions[name]
        if action['type'] == 'joint_lock':
            if abs(target['target_position'] - saved['target']) > 1e-6:
                raise ValueError('Joint lock target must remain constant')
            lower.Set(target['target_position'] * factor);upper.Set(target['target_position'] * factor)
        else:
            saved['drive'].CreateTargetPositionAttr(float(target['target_position']) * factor)

    def restore_joint(self, name):
        from pxr import UsdPhysics
        saved = self.joint_actions.pop(name)
        saved['joint'].GetLowerLimitAttr().Set(saved['limits'][0]);saved['joint'].GetUpperLimitAttr().Set(saved['limits'][1])
        if 'drive' in saved:
            for attr, value, authored in saved['drive_attributes']:
                attr.Set(value) if authored else attr.Clear()
            if not saved['drive_was_present']:
                saved['joint'].GetPrim().RemoveAPI(UsdPhysics.DriveAPI, saved['axis'])

    def fixed(self, name, action, state, target):
        from pxr import Gf, UsdPhysics
        if name not in self.fixtures:
            pos = target.setdefault('position', state['position'].copy())
            q = target.setdefault('rotation', state['quaternion'].copy())
            if not np.allclose(pos, state['position'], atol=1e-6) or not np.allclose(rotation(q), state['rotation'], atol=1e-6):
                raise ValueError('Fixed fixture target must match the enabling pose')
            path = '/World/Fixtures/fixture_' + str(len(self.fixtures))
            joint = UsdPhysics.FixedJoint.Define(self.runtime.stage, path)
            joint.CreateBody1Rel().SetTargets([self.instance.paths[action['body']]])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*pos))
            joint.CreateLocalRot0Attr(Gf.Quatf(q[0], Gf.Vec3f(*q[1:])))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1))
            self.fixtures[name] = {'path': path, 'position': np.asarray(pos), 'rotation': np.asarray(q)}
        fixed = self.fixtures[name]
        if any(key in target and not np.allclose(target[key], fixed[key], atol=1e-6) for key in ('position', 'rotation')):
            raise ValueError('Fixed fixture target must remain constant')
        state['inverse_mass'] = 0
        state['inverse_inertia'] = np.zeros((3, 3))
        return {'position': fixed['position'], 'rotation': fixed['rotation']}

    def __call__(self, time, dt):
        self.records = {}
        # Constraints change the mobility seen by the asset's internal forces.
        ordered = sorted(self.definitions.items(), key=lambda item: item[1]['type'] != 'fixture')
        for name, action in ordered:
            if action['keyframes'][0]['time'] <= time < action['keyframes'][-1]['time']:
                self.activate(name, action)
            target = interpolate(action['keyframes'], time)
            record = {'active': int(target is not None)}
            self.records[name] = record
            if target is None:
                if name in self.fixtures:
                    fixture = self.fixtures.pop(name)
                    if 'path' in fixture:
                        self.runtime.stage.RemovePrim(fixture['path'])
                if action['type'] in ('force', 'torque'):
                    record['value'] = np.zeros(3)
                if name in self.joint_actions:
                    self.restore_joint(name)
                continue
            kind = action['type']
            if kind.startswith('joint_'):
                self.joint_action(name, action, target)
                record['target_position'] = target['target_position']
                continue
            key = self.instance.body_key(action['body'])
            state = self.runtime.state(key)
            if kind in ('force', 'torque'):
                if action['frame'] not in ('world', 'body'):
                    raise ValueError('Force frame must be world or body')
                value = np.asarray(target['value'])
                if action['frame'] == 'body':
                    value = state['rotation'] @ value
                point = state['com']
                if 'point' in target:
                    point = np.asarray(target['point'])
                    if action['point_frame'] == 'body':
                        point = state['position'] + state['rotation'] @ point
                    elif action['point_frame'] != 'world':
                        raise ValueError('Point frame must be world or body')
                if kind == 'force':
                    self.runtime.apply_force(key, value, point)
                    record['point'] = point
                else:
                    self.runtime.apply_force(key, np.zeros(3), torque=value)
                record['value'] = value
            elif kind == 'fixture':
                if action.get('mode', 'fixed') == 'fixed':
                    target = self.fixed(name, action, state, target)
                elif action['mode'] == 'spring':
                    if name not in self.fixtures:
                        self.fixtures[name] = {'position': state['position'].copy(), 'rotation': state['quaternion'].copy()}
                    target.setdefault('position', self.fixtures[name]['position'])
                    target.setdefault('rotation', self.fixtures[name]['rotation'])
                    linear, angular = action['linear'], action['angular']
                    for a, b in zip(action['keyframes'], action['keyframes'][1:]):
                        if a['time'] <= time < b['time']:
                            target_v = (np.asarray(b['position']) - a['position']) / (b['time'] - a['time'])
                            target_w = Rotation.from_matrix(rotation(b['rotation']) @ rotation(a['rotation']).T).as_rotvec() / (b['time'] - a['time'])
                            break
                    gain = linear['damping'] + dt * linear['stiffness']
                    root_velocity = state['velocity'][:3] + np.cross(state['velocity'][3:], state['position'] - state['com'])
                    force = (linear['stiffness'] * (target['position'] - state['position']) + gain * (target_v - root_velocity)) / (1 + dt * gain * state['inverse_mass'])
                    error = Rotation.from_matrix(rotation(target['rotation']) @ state['rotation'].T).as_rotvec()
                    gain = angular['damping'] + dt * angular['stiffness']
                    torque = np.linalg.solve(np.eye(3) + dt * gain * state['inverse_inertia'],
                                             angular['stiffness'] * error + gain * (target_w - state['velocity'][3:]))
                    self.runtime.apply_force(key, force, torque=torque)
                    record.update(force=force, torque=torque)
                else:
                    raise ValueError('Unknown fixture mode')
                record.update(target)
                record['position_error'] = target['position'] - state['position']
                record['rotation_error'] = Rotation.from_matrix(rotation(target['rotation']) @ state['rotation'].T).as_rotvec()

    def close(self):
        for fixture in self.fixtures.values():
            if 'path' in fixture:
                self.runtime.stage.RemovePrim(fixture['path'])
        self.fixtures.clear()
        for name in list(self.joint_actions):
            self.restore_joint(name)
