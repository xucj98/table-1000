"""Expand explicit integer ranges only where a configuration references objects."""

from copy import deepcopy
import re


def expand_name(name):
    """Expand name1..N.suffix; preserve ordinary complete names."""
    if '..' not in name:
        return [name]
    match = re.fullmatch(r'(.*?)([0-9]+)\.\.([0-9]+)(.*)', name)
    if not match or '..' in match[1] or '..' in match[4]:
        raise ValueError(f'Invalid object range: {name}')
    prefix, first, last, suffix = match.groups()
    start, stop = int(first), int(last)
    padded = any(len(value) > 1 and value.startswith('0') for value in (first, last))
    width = len(first) if padded else 0
    if start > stop or padded and (len(last) != width or first != str(start).zfill(width) or last != str(stop).zfill(width)):
        raise ValueError(f'Invalid object range: {name}')
    return [prefix + str(index).zfill(width) + suffix for index in range(start, stop + 1)]


def expand_names(names, available=None):
    """Expand a name or name list and reject missing or repeated targets."""
    result = []
    for name in [names] if isinstance(names, str) else names:
        for target in expand_name(name):
            if target in result:
                raise ValueError(f'Duplicate object reference: {target}')
            if available is not None and target not in available:
                raise ValueError(f'Unknown object reference: {target}')
            result.append(target)
    return result


def expand_mapping(mapping, available=None):
    """Give each expanded target its own copy of the authored configuration."""
    result = {}
    for name, value in mapping.items():
        for target in expand_names(name, available):
            if target in result:
                raise ValueError(f'Duplicate object configuration: {target}')
            result[target] = deepcopy(value)
    return result
