"""MPlib planning model derived from a robot's URDF source.

The planning URDF is the robot source without visuals, with every joint
outside the planned chain frozen at a given value and a named TCP frame added.
Collision meshes are copied beside it, since MPlib resolves mesh names relative
to the URDF; with ``use_convex`` it reads ``<mesh>.convex.stl`` beside each
mesh, written here as the convex hull of the source mesh. Its SRDF disables collision checks only between bodies joined by a
joint and between pairs already in contact in the frozen reference pose (for
example, a shoulder housing seated in the torso), so every other pair stays
checked.
"""

from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh


def _origin(element):
    origin = element.find('origin')
    if origin is None:
        origin = ET.SubElement(element, 'origin', xyz='0 0 0', rpy='0 0 0')
    return origin


def _freeze(joint, value):
    origin = _origin(joint)
    xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    rotation = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
    axis = np.fromstring(joint.find('axis').get('xyz') if joint.find('axis') is not None else '1 0 0', sep=' ')
    if joint.get('type') == 'prismatic':
        xyz = xyz + rotation.apply(axis * value)
    else:
        rotation = rotation * Rotation.from_rotvec(axis * value)
    origin.set('xyz', ' '.join(f'{v:.9g}' for v in xyz))
    origin.set('rpy', ' '.join(f'{v:.9g}' for v in rotation.as_euler('xyz')))
    joint.set('type', 'fixed')
    for tag in ('axis', 'limit', 'mimic', 'dynamics'):
        if joint.find(tag) is not None:
            joint.remove(joint.find(tag))


def write_planning_urdf(source, output, active, frozen, tcp, padding=0., unpadded=()):
    """Write ``output`` and return it.

    ``active`` joint names stay movable; every other movable joint is fixed at
    ``frozen.get(name, 0)``. ``tcp`` is ``(parent_link, name, xyz, rpy)``, or a
    list of them.
    The convex hulls are grown by ``padding`` on every side, except those of
    the ``unpadded`` links (meshes shared with padded links are not allowed).
    """
    source, output = Path(source), Path(output)
    root = ET.parse(source).getroot()
    output.parent.mkdir(parents=True, exist_ok=True)
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    grown = {}
    for link in root.findall('link'):
        for mesh in link.iter('mesh'):
            amount = 0. if link.get('name') in unpadded else padding
            if grown.setdefault(mesh.get('filename'), amount) != amount:
                raise ValueError(f'{mesh.get("filename")} is shared by padded and unpadded links')
    for mesh in root.iter('mesh'):
        name = Path(mesh.get('filename'))
        (output.parent / name).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source.parent / name, output.parent / name)
        hull = trimesh.load(source.parent / name, force='mesh').convex_hull
        if grown[mesh.get('filename')]:
            from table_1000.planning.world import grow
            hull = trimesh.Trimesh(grow(hull.vertices, grown[mesh.get('filename')]), hull.faces)
        hull.export(output.parent / name.with_name(name.name + '.convex.stl'))
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed' and joint.get('name') not in active:
            _freeze(joint, float(frozen.get(joint.get('name'), 0.)))
    missing = set(active) - {joint.get('name') for joint in root.findall('joint') if joint.get('type') != 'fixed'}
    if missing:
        raise ValueError(f'active joints not in URDF: {sorted(missing)}')
    for parent, name, xyz, rpy in ([tcp] if isinstance(tcp[0], str) else tcp):
        ET.SubElement(root, 'link', name=name)
        joint = ET.SubElement(root, 'joint', name=name + '_joint', type='fixed')
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
    ET.ElementTree(root).write(output, encoding='unicode')
    return output


def write_srdf(urdf, output, touching=()):
    """Disable adjacent pairs (bodies with geometry joined through empty links)
    plus the given ``touching`` pairs."""
    root = ET.parse(urdf).getroot()
    geometric = {link.get('name') for link in root.findall('link') if link.find('collision') is not None}
    parents = {joint.find('child').get('link'): joint.find('parent').get('link') for joint in root.findall('joint')}
    pairs = set()
    for name in geometric:
        parent = parents.get(name)
        while parent is not None and parent not in geometric:
            parent = parents.get(parent)
        if parent is not None:
            pairs.add(tuple(sorted((name, parent))))
    pairs |= {tuple(sorted(pair)) for pair in touching}
    srdf = ET.Element('robot', name=root.get('name'))
    for first, second in sorted(pairs):
        ET.SubElement(srdf, 'disable_collisions', link1=first, link2=second, reason='adjacent or seated')
    ET.ElementTree(srdf).write(output, encoding='unicode')
    return sorted(pairs)


def touching_pairs(planner):
    """Body pairs of ``planner`` in collision at its current configuration."""
    return sorted({tuple(sorted((c.link_name1, c.link_name2))) for c in planner.check_for_self_collision()})
