"""Turn a vendor URDF package into ``asset_sources/robots/<name>/`` and check it.

``prepare`` rewrites the source's ``robot.urdf`` and ``meshes/`` from the raw
package named by the source's ``prepare.py``:

1. drop placeholder visuals (1 mm boxes);
2. simplify every visual mesh by quadric decimation, doubling the face count
   until the two-sided 99th-percentile surface distance is within MAX_ERROR;
3. split each simplified visual into convex pieces with CoACD (``PIECES`` per
   link, one hull otherwise; ``NO_COLLISION`` links get none), or for links in
   ``SLABS`` into hulls of equal slabs along an axis of the link;
4. apply the robot's joint edits and merge fixed-joint subtrees into their
   moving body (the Isaac importer only merges massless links).

A robot that differs from another only in some parts (a gripper variant) sets
``BASE`` in its ``prepare.py`` to that robot's name: meshes of the raw package
that the base robot's URDF also uses are referenced from the base source
(``../<base>/meshes/...``, with its collision pieces) instead of written again;
budgets are shared out over all meshes as if every one were new.

``check`` measures the source as committed: every visual vertex inside its
collision pieces, and the directory within the size budget. Both write a JSON
report under ``assets/robots/<name>/``. Needs trimesh, open3d and coacd.
"""

import argparse
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

MAX_ERROR = .001          # m, two-sided 99th-percentile surface distance
INITIAL_FACES = 250000    # total visual budget before error-driven refinement
OUTSIDE_TOLERANCE = 5e-4  # m, visual vertices allowed outside their collision pieces
SIZE_BUDGET = 10e6        # bytes for the whole robot source directory
PLACEHOLDER = '0.001 0.001 0.001'


def load_prepare(source):
    spec = importlib.util.spec_from_file_location('robot_prepare', Path(source) / 'prepare.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_mesh(path):
    import trimesh
    mesh = trimesh.load(path, force='mesh')
    mesh.merge_vertices()
    return mesh


def surface_error(a, b, samples=20000):
    import trimesh
    errors = []
    for x, y in ((a, b), (b, a)):
        points, _ = trimesh.sample.sample_surface(x, samples, seed=0)
        errors.append(float(np.percentile(trimesh.proximity.closest_point(y, points)[1], 99)))
    return max(errors)


def simplify(original, faces):
    """Return ``(mesh, error)``; the original when no smaller mesh meets MAX_ERROR."""
    import open3d as o3d
    import trimesh
    while faces < len(original.faces):
        decimated = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(original.vertices),
                                              o3d.utility.Vector3iVector(original.faces))
        decimated = decimated.simplify_quadric_decimation(int(faces))
        mesh = trimesh.Trimesh(np.asarray(decimated.vertices), np.asarray(decimated.triangles))
        error = surface_error(original, mesh)
        if error <= MAX_ERROR:
            return mesh, error
        faces *= 2
    return original, 0.


def decompose(mesh, pieces):
    """Convex pieces of ``mesh``, ordered by centroid so files do not depend on
    CoACD's thread scheduling (its pieces are deterministic, their order is not)."""
    import coacd
    import trimesh
    if pieces == 1:
        return [mesh.convex_hull]
    coacd.set_log_level('error')
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=.3, max_convex_hull=pieces,
                            preprocess_mode='auto', resolution=2000, seed=0)
    hulls = [trimesh.Trimesh(v, f).convex_hull for v, f in parts]
    return sorted(hulls, key=lambda hull: tuple(np.round(hull.vertices.mean(0), 6)))


def slabs(mesh, axis, count):
    """Hulls of ``count`` equal slabs of ``mesh`` along ``axis`` (0, 1, 2), each over the vertices within
    it and where edges cross its two planes: together they cover the mesh exactly. A long part with a
    concave edge along the axis (a finger's curved inner face) keeps that edge to the sag of one slab,
    where CoACD's pieces bridge it."""
    import trimesh
    points, edges = mesh.vertices, mesh.edges_unique
    first, second = points[edges[:, 0], axis], points[edges[:, 1], axis]
    planes = np.linspace(points[:, axis].min(), points[:, axis].max(), count + 1)
    hulls = []
    for low, high in zip(planes[:-1], planes[1:]):
        parts = [points[(points[:, axis] >= low) & (points[:, axis] <= high)]]
        for plane in (low, high):
            cross = (first - plane) * (second - plane) < 0
            t = (plane - first[cross]) / (second[cross] - first[cross])
            parts.append(points[edges[cross, 0]] + t[:, None] * (points[edges[cross, 1]] - points[edges[cross, 0]]))
        # Crossings next to a vertex nearly coincide with it; once merged in float32 (the planning
        # model's STL) such a pair breaks the hull, so points are snapped to a micrometre grid.
        hulls.append(trimesh.convex.convex_hull(np.unique(np.round(np.concatenate(parts), 6), axis=0)))
    return hulls


# -- fixed-joint merge ---------------------------------------------------------

def read_origin(element):
    from trimesh.transformations import euler_matrix
    origin = element.find('origin') if element is not None else None
    matrix = np.eye(4)
    if origin is not None:
        matrix = euler_matrix(*map(float, origin.get('rpy', '0 0 0').split()), 'sxyz')
        matrix[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return matrix


def write_origin(element, matrix):
    from trimesh.transformations import euler_from_matrix
    origin = element.find('origin')
    if origin is None:
        origin = ET.Element('origin')
        element.insert(0, origin)
    origin.set('xyz', ' '.join(f'{v:.9g}' for v in matrix[:3, 3]))
    origin.set('rpy', ' '.join(f'{v:.9g}' for v in euler_from_matrix(matrix, 'sxyz')))


def inertia_terms(link, transform):
    inertial = link.find('inertial')
    if inertial is None:
        return None
    mass = float(inertial.find('mass').get('value'))
    frame = transform @ read_origin(inertial)
    i = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
    tensor = np.array([[i['ixx'], i['ixy'], i['ixz']], [i['ixy'], i['iyy'], i['iyz']], [i['ixz'], i['iyz'], i['izz']]])
    rotation = frame[:3, :3]
    return mass, frame[:3, 3], rotation @ tensor @ rotation.T


def combine(terms):
    mass = sum(m for m, _, _ in terms)
    center = sum(m * c for m, c, _ in terms) / mass
    tensor = sum(t + m * (d @ d * np.eye(3) - np.outer(d, d)) for m, c, t in terms for d in [c - center])
    return mass, center, tensor


def merge(root):
    """Move geometry and inertia of fixed-joint subtrees to their moving body.

    Merged links stay as empty frames fixed to the body; moving joints are
    re-parented to it. Returns the number of moving bodies. Idempotent.
    """
    links = {link.get('name'): link for link in root.findall('link')}
    joints = root.findall('joint')
    parent_joint = {joint.find('child').get('link'): joint for joint in joints}

    def body(name):  # (moving body root, transform root->link)
        joint = parent_joint.get(name)
        if joint is None or joint.get('type') != 'fixed':
            return name, np.eye(4)
        root_name, transform = body(joint.find('parent').get('link'))
        return root_name, transform @ read_origin(joint)

    placement = {name: body(name) for name in links}
    terms = {}
    for name, link in links.items():
        root_name, transform = placement[name]
        term = inertia_terms(link, transform)
        if term is not None and term[0] > 0:
            terms.setdefault(root_name, []).append(term)
        if root_name == name:
            continue
        for element in link.findall('visual') + link.findall('collision'):
            write_origin(element, transform @ read_origin(element))
            link.remove(element)
            links[root_name].append(element)
        if link.find('inertial') is not None:
            link.remove(link.find('inertial'))
    for name, link in links.items():
        if placement[name][0] != name or name not in terms:
            continue
        if link.find('inertial') is not None:
            link.remove(link.find('inertial'))
        mass, center, tensor = combine(terms[name])
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=' '.join(f'{v:.9g}' for v in center), rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value=f'{mass:.9g}')
        ET.SubElement(inertial, 'inertia', **{key: f'{tensor[a, b]:.9g}' for key, a, b in (
            ('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2), ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2))})
    for joint in joints:
        parent = joint.find('parent').get('link')
        root_name, transform = placement[parent]
        if root_name != parent:
            joint.find('parent').set('link', root_name)
            write_origin(joint, transform @ read_origin(joint))
    # Stable child order: inertial, visuals, collisions.
    for link in links.values():
        children = list(link)
        for child in children:
            link.remove(child)
        for tag in ('inertial', 'visual', 'collision'):
            link.extend(child for child in children if child.tag == tag)
    return sum(1 for name in links if placement[name][0] == name)


# -- prepare and check ---------------------------------------------------------

def raw_meshes(root):
    return {shape.get('filename') for shape in root.iter('mesh')}


def base_meshes(source, spec, raw):
    """``{raw mesh: (visual, [collision pieces])}`` paths relative to ``source`` for the meshes the
    base robot (``spec.BASE``) shares with this one; empty without a base."""
    name = getattr(spec, 'BASE', None)
    if name is None:
        return {}
    base = Path(source).parent / name
    used = raw_meshes(ET.parse(raw / load_prepare(base).URDF).getroot())
    pieces = {}
    for collision in ET.parse(base / 'robot.urdf').getroot().iter('collision'):
        mesh = collision.find('geometry/mesh').get('filename')
        pieces.setdefault(Path(mesh).stem.rsplit('_', 1)[0], set()).add(f'../{name}/{mesh}')
    return {mesh: (f'../{name}/meshes/visual/{Path(mesh).stem}.ply', sorted(pieces.get(Path(mesh).stem, ())))
            for mesh in used}


def prepare(source, raw):
    """Rewrite ``source/robot.urdf`` and ``source/meshes/`` from the raw package."""
    source, raw = Path(source), Path(raw)
    spec = load_prepare(source)
    root = ET.parse(raw / spec.URDF).getroot()
    shared = base_meshes(source, spec, raw)
    meshes = source / 'meshes'
    shutil.rmtree(meshes, ignore_errors=True)
    (meshes / 'visual').mkdir(parents=True)
    (meshes / 'collision').mkdir()

    shapes = {}
    for link in root.findall('link'):
        for visual in list(link.findall('visual')):
            shape = visual.find('geometry')[0]
            if shape.tag == 'box' and shape.get('size') == PLACEHOLDER:
                link.remove(visual)
            elif shape.tag == 'mesh':
                shapes.setdefault(shape.get('filename'), []).append(shape)
    originals = {name: load_mesh(raw / name) for name in shapes}
    weights = {name: np.sqrt(len(mesh.faces)) for name, mesh in originals.items()}
    total = sum(weights.values())
    report = {'raw': str(raw), 'urdf': spec.URDF, 'max_error_m': MAX_ERROR, 'visual': {}, 'collision': {},
              'shared': sorted(shared[name][0] for name in shapes if name in shared)}
    written = {}
    for name, references in shapes.items():
        if name in shared:
            visual, pieces = shared[name]
            written[visual] = pieces
            for shape in references:
                shape.set('filename', visual)
            continue
        budget = min(len(originals[name].faces), max(300, int(INITIAL_FACES * weights[name] / total)))
        mesh, error = simplify(originals[name], budget)
        output = f'meshes/visual/{Path(name).stem}.ply'
        if (source / output).exists():
            raise RuntimeError(f'duplicate mesh name {output}')
        mesh.export(source / output)
        report['visual'][Path(name).stem] = {'source_faces': len(originals[name].faces), 'faces': len(mesh.faces),
                                             'error_m': error}
        for shape in references:
            shape.set('filename', output)

    for link in root.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
        visuals = [v for v in link.findall('visual') if v.find('geometry/mesh') is not None]
        if not visuals or link.get('name').endswith(spec.NO_COLLISION):
            continue
        for visual in visuals:
            mesh_name = visual.find('geometry/mesh').get('filename')
            stem = Path(mesh_name).stem
            if mesh_name not in written:
                mesh = load_mesh(source / mesh_name)
                if link.get('name') in getattr(spec, 'SLABS', {}):
                    hulls = slabs(mesh, *spec.SLABS[link.get('name')])
                else:
                    hulls = decompose(mesh, spec.PIECES.get(link.get('name'), 1))
                written[mesh_name] = []
                for index, hull in enumerate(hulls):
                    output = f'meshes/collision/{stem}_{index}.ply'
                    hull.export(source / output)
                    written[mesh_name].append(output)
                report['collision'][stem] = len(hulls)
            for output in written[mesh_name]:
                collision = ET.SubElement(link, 'collision')
                if visual.find('origin') is not None:
                    collision.append(copy.deepcopy(visual.find('origin')))
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=output)

    spec.edit(root)
    report['bodies'] = merge(root)
    ET.indent(root)
    ET.ElementTree(root).write(source / 'robot.urdf', encoding='unicode', xml_declaration=False)
    return report


def outside_distance(points, hulls):
    """Per point, how far it lies outside the nearest-containing convex piece
    (0 inside any piece; otherwise the smallest over pieces of the largest
    face-plane violation, a lower bound of the true distance)."""
    best = np.full(len(points), np.inf)
    for hull in hulls:
        offsets = np.einsum('ij,ij->i', hull.face_normals, hull.triangles[:, 0])
        violation = (points @ hull.face_normals.T - offsets).max(axis=1)
        best = np.minimum(best, np.maximum(violation, 0.))
    return best


def check(source):
    """Coverage of every visual mesh by its collision pieces, and source size."""
    source = Path(source)
    root = ET.parse(source / 'robot.urdf').getroot()
    pieces = {}
    for collision in root.iter('collision'):
        name = Path(collision.find('geometry/mesh').get('filename'))
        visual = name.parent.parent / 'visual' / f'{name.stem.rsplit("_", 1)[0]}.ply'
        pieces.setdefault(visual.as_posix(), set()).add(name)
    coverage = {}
    for visual, names in sorted(pieces.items()):
        vertices = load_mesh(source / visual).vertices
        outside = outside_distance(vertices, [load_mesh(source / name) for name in sorted(names)])
        coverage[visual] = {'pieces': len(names), 'max_outside_m': float(outside.max())}
    size = sum(path.stat().st_size for path in source.rglob('*') if path.is_file())
    failing = sorted(stem for stem, entry in coverage.items() if entry['max_outside_m'] > OUTSIDE_TOLERANCE)
    return {'status': 'PASS' if not failing and size <= SIZE_BUDGET else 'FAIL', 'size_bytes': size,
            'size_budget_bytes': SIZE_BUDGET, 'outside_tolerance_m': OUTSIDE_TOLERANCE,
            'uncovered': failing, 'coverage': coverage}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('robot', help='robot name under asset_sources/robots')
    parser.add_argument('--source', type=Path, default=Path('asset_sources/robots'))
    parser.add_argument('--assets', type=Path, default=Path('assets'))
    parser.add_argument('--check', action='store_true', help='only check the committed source')
    args = parser.parse_args(argv)
    source = args.source / args.robot
    output = args.assets / 'robots' / args.robot
    output.mkdir(parents=True, exist_ok=True)
    if not args.check:
        report = prepare(source, args.assets / load_prepare(source).RAW)
        (output / 'prepare.json').write_text(json.dumps(report, indent=2) + '\n')
        worst = max(report['visual'].items(), key=lambda item: item[1]['error_m'])
        print(f"{len(report['visual'])} visual meshes, {sum(report['collision'].values())} collision pieces, "
              f"{report['bodies']} bodies; largest simplification error {worst[1]['error_m'] * 1000:.2f} mm ({worst[0]})")
    result = check(source)
    (output / 'check.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f"check {result['status']}: {result['size_bytes'] / 1e6:.2f} MB, uncovered {result['uncovered']}")
    return 0 if result['status'] == 'PASS' else 1
