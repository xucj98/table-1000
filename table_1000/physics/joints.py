"""Read native joint coordinates and apply initial joint poses in SI units."""

import numpy as np
from scipy.spatial.transform import Rotation
from table_1000.physics.runtime import rotation


def frames(instance, name):
    from pxr import UsdPhysics
    prim = instance.runtime.stage.GetPrimAtPath(instance.paths[name])
    joint = UsdPhysics.Joint(prim)
    anchors = []
    for i in (0, 1):
        path = str(getattr(joint, f'GetBody{i}Rel')().GetTargets()[0])
        body = instance.runtime.stage.GetPrimAtPath(path).GetCustomDataByKey('table1000:name')
        state = instance.runtime.state(instance.body_key(body))
        offset = np.asarray(getattr(joint, f'GetLocalPos{i}Attr')().Get())
        q = getattr(joint, f'GetLocalRot{i}Attr')().Get()
        local_r = rotation([q.GetReal(), *q.GetImaginary()])
        point = state['position'] + state['rotation'] @ offset
        anchors.append({'body': body, 'state': state, 'position': point,
                        'rotation': state['rotation'] @ local_r,
                        'velocity': state['velocity'][:3] + np.cross(state['velocity'][3:], point - state['com'])})
    slider = prim.IsA(UsdPhysics.PrismaticJoint)
    axis_label = (UsdPhysics.PrismaticJoint(prim) if slider else UsdPhysics.RevoluteJoint(prim)).GetAxisAttr().Get()
    axis_index = 'XYZ'.index(axis_label)
    a, b = anchors
    axis = a['rotation'][:, axis_index]
    position = float((b['position'] - a['position']) @ axis) if slider else float(Rotation.from_matrix(a['rotation'].T @ b['rotation']).as_rotvec()[axis_index])
    velocity = float((b['velocity'] - a['velocity']) @ axis) if slider else float((b['state']['velocity'][3:] - a['state']['velocity'][3:]) @ axis)
    return {'prim': prim, 'anchors': anchors, 'axis': axis, 'position': position,
            'velocity': velocity, 'factor': 1 if slider else 180 / np.pi, 'slider': slider}


def set_initial(instance, targets):
    for name, target in targets.items():
        info = frames(instance, name)
        a, b = info['anchors']
        delta = target - info['position']
        position = b['state']['position']
        matrix = b['state']['rotation']
        if info['slider']:
            position = position + delta * info['axis']
        else:
            change = Rotation.from_rotvec(info['axis'] * delta).as_matrix()
            position = a['position'] + change @ (position - a['position'])
            matrix = change @ matrix
        q = Rotation.from_matrix(matrix).as_quat()[[3, 0, 1, 2]]
        instance.view.set_world_poses(np.asarray([position], dtype=np.float32), np.asarray([q], dtype=np.float32), indices=np.array([instance.indices[b['body']]]))
        instance.runtime.refresh_states()
