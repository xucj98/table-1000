"""Collision from boxes, cylinders and spheres fitted to regions of the visual meshes.

A robot's shape list (a ``SHAPES`` dict in a Python file) gives, per visual
mesh (named by its file stem), the primitives that replace its convex pieces::

    SHAPES = {
        'chassis_base_link': [
            {'type': 'box', 'region': {'z': (None, .246)}},           # axis-aligned box
            {'type': 'box', 'region': {'z': (.246, .43)}, 'fit': 'obb'},  # any orientation
            {'type': 'cylinder', 'axis': 'z', 'region': {'y': (.2, None)}},
        ],
    }
    KEEP = ('gripper_left_link',)   # parts that keep their convex pieces

A region is a set of ranges in the mesh frame (``None`` is open); the visual
mesh is cut to it by planes, so every point of its surface inside the region
is covered, also where a long triangle crosses the region's border. The
primitive is the smallest of its kind around those points: an axis-aligned or
minimum-volume box, a cylinder along ``axis`` (smallest circle around the
projected points), or the smallest sphere. Parts in neither ``SHAPES`` nor
``KEEP`` get one minimum-volume box. The regions of a part should cover its
whole visual; :func:`uncovered` measures what is left out.
"""

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

AXES = {'x': 0, 'y': 1, 'z': 2}
CYLINDER_SECTIONS = 64


def cut(mesh, region):
    """``mesh`` restricted to ``region`` (keeps n . x >= value for each bound)."""
    import trimesh
    part = mesh
    for name, (low, high) in region.items():
        axis = np.eye(3)[AXES[name]]
        bounds = [(axis, low)] if low is not None else []
        bounds += [(-axis, -high)] if high is not None else []
        for normal, value in bounds:
            if part is None or not len(part.faces):
                return None
            # The plane n . x = value passes through n * value (|n| = 1).
            part = trimesh.intersections.slice_mesh_plane(part, normal, normal * value, cap=False)
    return part


def _rotation_to(axis):
    """Rotation taking +z to ``axis``."""
    from scipy.spatial.transform import Rotation
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    z = np.array([0., 0., 1.])
    if np.allclose(axis, z):
        return np.eye(3)
    if np.allclose(axis, -z):
        return np.diag([1., -1., -1.])
    rotvec = np.cross(z, axis)
    return Rotation.from_rotvec(rotvec / np.linalg.norm(rotvec) * np.arccos(np.clip(z @ axis, -1, 1))).as_matrix()


def fit(points, shape):
    """``(kind, dimensions, 4x4 pose in the mesh frame)`` of the smallest primitive of ``shape`` around ``points``."""
    import trimesh
    kind = shape['type']
    pose = np.eye(4)
    if kind == 'box':
        if shape.get('fit') == 'obb':
            transform, extents = trimesh.bounds.oriented_bounds(points)
            pose = np.linalg.inv(transform)
            return 'box', tuple(float(e) for e in extents), pose
        low, high = points.min(0), points.max(0)
        pose[:3, 3] = (low + high) / 2
        return 'box', tuple(float(e) for e in high - low), pose
    if kind == 'cylinder':
        axis = AXES[shape['axis']]
        rotation = _rotation_to(np.eye(3)[axis])
        local = points @ rotation              # coordinates in a frame whose z is the axis
        center, radius = trimesh.nsphere.minimum_nsphere(local[:, :2])
        low, high = local[:, 2].min(), local[:, 2].max()
        pose[:3, :3] = rotation
        pose[:3, 3] = rotation @ np.array([center[0], center[1], (low + high) / 2])
        return 'cylinder', (float(radius), float(high - low)), pose
    if kind == 'sphere':
        center, radius = trimesh.nsphere.minimum_nsphere(points)
        pose[:3, 3] = center
        return 'sphere', (float(radius),), pose
    raise ValueError(f'unknown primitive {kind}')


def fit_part(mesh, shapes):
    """Primitives of one visual mesh."""
    result = []
    for shape in shapes or [{'type': 'box', 'fit': 'obb', 'region': {}}]:
        part = cut(mesh, shape.get('region', {}))
        if part is None or not len(part.vertices):
            raise ValueError(f'empty region {shape}')
        result.append(fit(np.asarray(part.vertices), shape))
    return result


def to_mesh(kind, dimensions, pose):
    """A triangle mesh of a primitive (cylinders with CYLINDER_SECTIONS sides, circumscribed)."""
    import trimesh
    if kind == 'box':
        return trimesh.creation.box(extents=dimensions, transform=pose)
    if kind == 'cylinder':
        radius, length = dimensions
        # Circumscribed polygon, so the mesh contains the true cylinder.
        return trimesh.creation.cylinder(radius=radius / np.cos(np.pi / CYLINDER_SECTIONS), height=length,
                                         sections=CYLINDER_SECTIONS, transform=pose)
    return trimesh.creation.icosphere(subdivisions=3, radius=dimensions[0]).apply_transform(pose)


def element_mesh(element):
    """Mesh of a URDF ``<collision>`` primitive in its link frame (``None`` for mesh geometry)."""
    geometry = element.find('geometry')
    for kind in ('box', 'cylinder', 'sphere'):
        node = geometry.find(kind)
        if node is not None:
            if kind == 'box':
                dimensions = tuple(map(float, node.get('size').split()))
            elif kind == 'cylinder':
                dimensions = (float(node.get('radius')), float(node.get('length')))
            else:
                dimensions = (float(node.get('radius')),)
            return to_mesh(kind, dimensions, origin(element))
    return None


def origin(element):
    from scipy.spatial.transform import Rotation
    transform = np.eye(4)
    node = element.find('origin')
    if node is not None:
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
    return transform


def _write_origin(element, transform):
    from scipy.spatial.transform import Rotation
    ET.SubElement(element, 'origin', xyz=' '.join(f'{v:.6g}' for v in transform[:3, 3]),
                  rpy=' '.join(f'{v:.9g}' for v in Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')))


def apply(source, shapes, keep=()):
    """Replace the convex pieces of every part of the robot source ``source`` by fitted primitives,
    except the ``keep`` parts. Returns ``{part: [(kind, dimensions)]}``."""
    from table_1000.robots import prepare
    source = Path(source)
    tree = ET.parse(source / 'robot.urdf')
    fitted = {}
    folder = source / 'meshes' / 'collision'
    for link in tree.getroot().findall('link'):
        visuals = {Path(v.find('geometry/mesh').get('filename')).stem: v for v in link.findall('visual')}
        pieces = {}
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is not None:
                pieces.setdefault(Path(mesh.get('filename')).stem.rsplit('_', 1)[0], []).append(collision)
        for part, collisions in pieces.items():
            if part in keep or part not in visuals:
                continue
            if part not in fitted:
                mesh = prepare.load_mesh(source / 'meshes' / 'visual' / f'{part}.ply')
                fitted[part] = fit_part(mesh, shapes.get(part))
            for collision in collisions:
                link.remove(collision)
            for kind, dimensions, pose in fitted[part]:
                collision = ET.SubElement(link, 'collision')
                _write_origin(collision, origin(visuals[part]) @ pose)
                geometry = ET.SubElement(ET.SubElement(collision, 'geometry'), kind)
                if kind == 'box':
                    geometry.set('size', ' '.join(f'{v:.6g}' for v in dimensions))
                elif kind == 'cylinder':
                    geometry.set('radius', f'{dimensions[0]:.6g}')
                    geometry.set('length', f'{dimensions[1]:.6g}')
                else:
                    geometry.set('radius', f'{dimensions[0]:.6g}')
    for part in fitted:
        for path in folder.glob(f'{part}_*.ply'):
            if path.stem.rsplit('_', 1)[0] == part:
                path.unlink()
    ET.indent(tree.getroot())
    tree.write(source / 'robot.urdf', encoding='unicode')
    return {part: [(kind, dimensions) for kind, dimensions, _ in items] for part, items in fitted.items()}


def uncovered(mesh, primitives, samples=20000):
    """Largest distance of a surface point of ``mesh`` outside all ``primitives`` (``(kind, dimensions, pose)``)."""
    import trimesh
    points, _ = trimesh.sample.sample_surface(mesh, samples, seed=0)
    points = np.vstack([points, mesh.vertices])
    outside = np.full(len(points), np.inf)
    for kind, dimensions, pose in primitives:
        local = (points - pose[:3, 3]) @ pose[:3, :3]
        if kind == 'box':
            distance = np.linalg.norm(np.maximum(np.abs(local) - np.asarray(dimensions) / 2, 0), axis=1)
        elif kind == 'cylinder':
            radial = np.maximum(np.linalg.norm(local[:, :2], axis=1) - dimensions[0], 0)
            axial = np.maximum(np.abs(local[:, 2]) - dimensions[1] / 2, 0)
            distance = np.hypot(radial, axial)
        else:
            distance = np.maximum(np.linalg.norm(local, axis=1) - dimensions[0], 0)
        outside = np.minimum(outside, distance)
    return float(outside.max())


def _slab_cost(points):
    """``(volume, shape)`` of the smallest axis-aligned box or axis-aligned cylinder around ``points``."""
    import trimesh
    low, high = points.min(0), points.max(0)
    best = (float(np.prod(high - low)), {'type': 'box'})
    for name, axis in AXES.items():
        others = [i for i in range(3) if i != axis]
        _, radius = trimesh.nsphere.minimum_nsphere(points[:, others])
        volume = np.pi * radius ** 2 * (high[axis] - low[axis])
        if volume < best[0]:
            best = (float(volume), {'type': 'cylinder', 'axis': name})
    return best


def design(mesh, count, region=None, candidates=40, samples=20000):
    """Shapes (``SHAPES`` entries) of ``count`` slabs of ``mesh`` along the best of x, y, z.

    For each axis, cut positions are chosen among ``candidates`` evenly spaced
    ones by dynamic programming so that the summed volume of the slabs'
    primitives (box or cylinder, whichever is smaller) is least; the axis with
    the least total wins. ``region`` limits the part considered (e.g. a clip).
    Returns ``(volume, shapes)``.
    """
    import trimesh
    region = region or {}
    part = cut(mesh, region)
    points, _ = trimesh.sample.sample_surface(part, samples, seed=0)
    points = np.vstack([points, part.vertices])
    best = (np.inf, None)
    for name, axis in AXES.items():
        edges = np.linspace(points[:, axis].min(), points[:, axis].max(), candidates + 1)
        edges[0], edges[-1] = -np.inf, np.inf
        cost = {}
        for i in range(candidates):
            for j in range(i + 1, candidates + 1):
                inside = points[(points[:, axis] >= edges[i]) & (points[:, axis] <= edges[j])]
                cost[i, j] = _slab_cost(inside) if len(inside) >= 4 else (0., None)
        # table[k][j]: least volume covering edges[0]..edges[j] with k slabs
        table = [{0: (0., [])}]
        for k in range(1, count + 1):
            row = {}
            for j in range(1, candidates + 1):
                options = [(table[k - 1][i][0] + cost[i, j][0], table[k - 1][i][1] + [(i, j)])
                           for i in table[k - 1] if i < j]
                if options:
                    row[j] = min(options, key=lambda option: option[0])
            table.append(row)
        if candidates in table[count] and table[count][candidates][0] < best[0]:
            volume, slabs = table[count][candidates]
            shapes = []
            for i, j in slabs:
                shape = dict(cost[i, j][1] or {'type': 'box'})
                bounds = (None if i == 0 else float(edges[i]), None if j == candidates else float(edges[j]))
                shape['region'] = {**region, name: bounds} if name not in region else {
                    **region, name: (max(v for v in (bounds[0], region[name][0]) if v is not None)
                                     if bounds[0] is not None or region[name][0] is not None else None,
                                     min(v for v in (bounds[1], region[name][1]) if v is not None)
                                     if bounds[1] is not None or region[name][1] is not None else None)}
                shapes.append(shape)
            best = (volume, shapes)
    return best
