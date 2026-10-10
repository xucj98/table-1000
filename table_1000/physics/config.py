"""Resolve physical configuration references against an exported model."""

from copy import deepcopy
from table_1000.assets.references import expand_mapping, expand_names
from table_1000.physics.plots import source_parts


def merge(base, override):
    """Recursive dictionary inheritance; arrays, scalars and null replace."""
    result = deepcopy(base)
    for key, value in override.items():
        result[key] = merge(result[key], value) if isinstance(value, dict) and isinstance(result.get(key), dict) else deepcopy(value)
    return result


def keyframes(frames, duration, dt):
    if len(frames) < 2:
        raise ValueError('An action needs at least two keyframes')
    result, previous = [], {}
    last = -1
    for frame in frames:
        time = frame['time']
        if not 0 <= time <= duration or time <= last or abs(time / dt - round(time / dt)) > 1e-7:
            raise ValueError('Action times must increase, lie in the test and align with dt')
        previous = merge(previous, frame)
        result.append(previous)
        last = time
    return result


def test_configs(config, model):
    bodies = {node['name'] for node in model['nodes'] if node['rigid']}
    joints = {joint['name'] for joint in model['joints']}
    result = {}
    for name, definition in config['tests'].items():
        test = merge(config.get('defaults', {}), definition)
        dt, duration = test['simulation']['dt'], test['duration']
        if test['simulation']['backend'] != 'isaac':
            raise ValueError('Only the Isaac backend is implemented')
        if dt <= 0 or duration <= 0 or abs(duration / dt - round(duration / dt)) > 1e-7:
            raise ValueError('Duration must be a positive integer number of physical steps')
        stride = 1 / (dt * test['camera']['fps'])
        if abs(stride - round(stride)) > 1e-7:
            raise ValueError('Camera frame period must be an integer number of physical steps')
        initial = test.setdefault('initial', {})
        initial['rigid_bodies'] = expand_mapping(initial.get('rigid_bodies', {}), bodies)
        initial['joints'] = expand_mapping(initial.get('joints', {}), joints)
        actions, action_names = {}, {}
        for action_name, action in test.get('actions', {}).items():
            kind = action['type']
            target_field = 'joint' if kind in ('joint_lock', 'joint_drive') else 'body'
            if kind not in ('force', 'torque', 'fixture', 'joint_lock', 'joint_drive'):
                raise ValueError(f'Unsupported action type: {kind}')
            frames = keyframes(action['keyframes'], duration, dt)
            if kind in ('force', 'torque') and kind not in frames[0]:
                raise ValueError(f'{kind} action needs {kind} in its first keyframe')
            targets = expand_names(action[target_field], joints if target_field == 'joint' else bodies)
            action_names[action_name] = []
            for target in targets:
                resolved = deepcopy(action)
                resolved[target_field] = target
                resolved['keyframes'] = deepcopy(frames)
                label = action_name if len(targets) == 1 else f'{action_name}[{target}]'
                if label in actions:
                    raise ValueError(f'Duplicate action label: {label}')
                actions[label] = resolved
                action_names[action_name].append(label)
        test['actions'] = actions
        observe = test.get('observe')
        requested_plots = observe.get('plots', []) if observe is not None else []
        if observe is None:
            observe = {'rigid_bodies': sorted(bodies), 'joints': sorted(joints), 'actions': list(actions)}
        else:
            requested = observe.get('actions', [])
            if len(set(requested)) != len(requested) or set(requested) - action_names.keys():
                raise ValueError('Unknown or duplicate observed action')
            observe = {'rigid_bodies': expand_names(observe.get('rigid_bodies', []), bodies),
                       'joints': expand_names(observe.get('joints', []), joints),
                       'actions': [label for key in requested for label in action_names[key]]}
        observe['plots'] = []
        for source in requested_plots:
            category, target, field, component = source_parts(source)
            if category in ('rigid_bodies', 'joints'):
                targets = expand_names(target, bodies if category == 'rigid_bodies' else joints)
            elif category == 'actions':
                if target not in action_names and target not in actions:
                    raise ValueError(f'Unknown plotted action: {target}')
                targets = action_names.get(target, [target])
            else:
                targets = [target]
            for target in targets:
                if category == 'actions':
                    action = actions[target]
                    fields = {'force': {'active', 'force', 'point'}, 'torque': {'active', 'torque'},
                              'fixture': {'active', 'position', 'rotation', 'position_error', 'rotation_error'},
                              'joint_lock': {'active', 'target_position'}, 'joint_drive': {'active', 'target_position'}}[action['type']]
                    if action['type'] == 'fixture' and action.get('mode') == 'spring':
                        fields |= {'force', 'torque'}
                    if field not in fields:
                        raise ValueError(f'Unknown action plot quantity: {source}')
                resolved = f'{category}.{target}.{field}' + (f'.{component}' if component else '')
                if resolved in observe['plots']:
                    raise ValueError(f'Duplicate plot source: {resolved}')
                observe['plots'].append(resolved)
                if category in observe and target not in observe[category]:
                    observe[category].append(target)
        test['observe'] = observe
        result[name] = test
    return result


def physical_config(config, model):
    result = deepcopy(config)
    bodies = {node['name'] for node in model['nodes'] if node['rigid']}
    parts = {node['name'] for node in model['nodes'] if node['part']}
    joints = {joint['name'] for joint in model['joints']}
    for field, available in [('rigid_bodies', bodies), ('colliders', parts), ('joints', joints)]:
        result[field] = expand_mapping(result.get(field, {}), available)
    if result['rigid_bodies'].keys() != bodies:
        raise ValueError('physics.rigid_bodies must give a mass for every independent rigid body')
    for name, body in result['rigid_bodies'].items():
        if body['mass'] <= 0:
            raise ValueError(f'{name}: mass must be positive')
        if ('center_of_mass' in body) != ('inertia' in body):
            raise ValueError(f'{name}: center_of_mass and inertia must be supplied together')
    for interface in result.get('interfaces', {}).values():
        targets = expand_names(interface['body'], bodies)
        if len(targets) != 1:
            raise ValueError('An assembly interface belongs to one rigid body')
        interface['body'] = targets[0]
    references = bodies | joints | result.get('interfaces', {}).keys()
    for behavior in result.get('behaviors', {}).values():
        bindings = {}
        for key, value in behavior.get('bindings', {}).items():
            targets = expand_names(value, references)
            if len(targets) != 1:
                raise ValueError('Each behavior binding must identify one target')
            bindings[key] = targets[0]
        behavior['bindings'] = bindings
    return result
