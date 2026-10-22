"""MPlib planning model derived from a robot's URDF source.

The planning URDF is the robot source without visuals, with every joint
outside the planned chain frozen at a given value and a named TCP frame added.
Collision meshes are copied beside it, since MPlib resolves mesh names relative
to the URDF; with ``use_convex`` it reads ``<mesh>.convex.stl`` beside each
mesh, written here as the convex hull of the source mesh (simplified, see
:func:`~table_1000.planning.world.simplify_hull`). Its SRDF disables collision checks only between bodies joined by a
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


def _grow_primitive(geometry, padding):
    """Grow a box, cylinder or sphere collision by ``padding`` on every side."""
    if not padding:
        return
    box, cylinder, sphere = geometry.find('box'), geometry.find('cylinder'), geometry.find('sphere')
    if box is not None:
        box.set('size', ' '.join(f'{float(v) + 2 * padding:.6g}' for v in box.get('size').split()))
    if cylinder is not None:
        cylinder.set('radius', f'{float(cylinder.get("radius")) + padding:.6g}')
        cylinder.set('length', f'{float(cylinder.get("length")) + 2 * padding:.6g}')
    if sphere is not None:
        sphere.set('radius', f'{float(sphere.get("radius")) + padding:.6g}')


def _transform(element):
    origin = element.find('origin')
    transform = np.eye(4)
    if origin is not None:
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    return transform


def _merge_convex(link, directory, output, tolerance=1.2):
    """Replace ``link``'s mesh collisions by one mesh of their common convex hull (written under
    ``output``) when that hull is at most ``tolerance`` times their summed volume: checks cost
    per pair of shapes, and an arm segment or a finger made of a dozen convex pieces is about
    convex as a whole."""
    collisions = link.findall('collision')
    meshes = [collision.find('geometry/mesh') for collision in collisions]
    if len(collisions) < 2 or any(mesh is None or mesh.get('scale') for mesh in meshes):
        return
    points, volume = [], 0.
    for collision, mesh in zip(collisions, meshes):
        hull = trimesh.load(directory / mesh.get('filename'), force='mesh').convex_hull
        transform = _transform(collision)
        points.append(hull.vertices @ transform[:3, :3].T + transform[:3, 3])
        volume += hull.volume
    hull = trimesh.convex.convex_hull(np.vstack(points))
    if hull.volume > tolerance * volume:
        return
    name = Path('merged') / f'{link.get("name")}.stl'
    (output / name).parent.mkdir(parents=True, exist_ok=True)
    hull.export(output / name)
    for collision in collisions[1:]:
        link.remove(collision)
    collision = collisions[0]
    if collision.find('origin') is not None:
        collision.remove(collision.find('origin'))
    collision.find('geometry/mesh').set('filename', name.as_posix())


def write_planning_urdf(source, output, active, frozen, tcp, padding=0., unpadded=()):
    """Write ``output`` and return it.

    ``active`` joint names stay movable; every other movable joint is fixed at
    ``frozen.get(name, 0)``. ``tcp`` is ``(parent_link, name, xyz, rpy)``, or a
    list of them.
    The convex hulls, boxes, cylinders and spheres are grown by ``padding`` on
    every side, except those of the ``unpadded`` links (meshes shared with
    padded links are not allowed); the other links' near-convex mesh sets are
    merged (:func:`_merge_convex`).
    """
    source, output = Path(source), Path(output)
    root = ET.parse(source).getroot()
    output.parent.mkdir(parents=True, exist_ok=True)
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    for link in root.findall('link'):
        # The unpadded links touch objects (fingers): they keep their exact shapes.
        if link.get('name') not in unpadded:
            _merge_convex(link, source.parent, output.parent)
    grown = {}
    for link in root.findall('link'):
        for mesh in link.iter('mesh'):
            amount = 0. if link.get('name') in unpadded else padding
            if grown.setdefault(mesh.get('filename'), amount) != amount:
                raise ValueError(f'{mesh.get("filename")} is shared by padded and unpadded links')
    for link in root.findall('link'):
        amount = 0. if link.get('name') in unpadded else padding
        for geometry in link.findall('collision/geometry'):
            _grow_primitive(geometry, amount)
    for mesh in root.iter('mesh'):
        name = Path(mesh.get('filename'))
        # Meshes of a base robot (../<base>/meshes/...) are copied inside the output directory too.
        copy = Path(*['up' if part == '..' else part for part in name.parts])
        if name.parts[0] == 'merged':   # written by _merge_convex
            mesh_file = output.parent / name
        else:
            (output.parent / copy).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source.parent / name, output.parent / copy)
            mesh_file = source.parent / name
        from table_1000.planning.world import simplify_hull
        hull = trimesh.Trimesh(simplify_hull(trimesh.load(mesh_file, force='mesh').convex_hull.vertices)).convex_hull
        if grown[mesh.get('filename')]:
            from table_1000.planning.world import grow
            hull = trimesh.Trimesh(grow(hull.vertices, grown[mesh.get('filename')]), hull.faces)
        hull.export(output.parent / copy.with_name(copy.name + '.convex.stl'))
        mesh.set('filename', copy.as_posix())
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
