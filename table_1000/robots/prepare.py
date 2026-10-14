"""Turn a vendor URDF package into ``asset_sources/robots/<name>/`` and check it.

``prepare`` rewrites the source's ``robot.urdf`` and ``meshes/`` from the raw
package named by the source's ``prepare.py``:

1. drop placeholder visuals (1 mm boxes);
2. simplify every visual mesh by quadric decimation, doubling the face count
   until the two-sided 99th-percentile surface distance is within MAX_ERROR;
3. split each simplified visual into convex pieces: one hull, then CoACD with
   2, 4, 8 and 16 pieces, until the collision surface lies within the link's
   ``EXCESS`` tolerance of the visual (99th percentile over the outer surface
   of the pieces). Every piece has at most MAX_VERTICES vertices (PhysX's
   default hull limit), reduced so that it still encloses its CoACD piece.
   ``NO_COLLISION`` links get none, nor do links whose pieces lie inside the
   pieces of another link of the same rigid body (sensors in a housing);
4. apply the robot's joint edits and merge fixed-joint subtrees into their
   moving body (the Isaac importer only merges massless links).

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
MAX_VERTICES = 64         # per convex piece; PhysX cooks convex hulls with at most 64 by default
PIECE_COUNTS = (1, 2, 4, 8, 16)
ENCLOSED_TOLERANCE = .001 # m, a link whose pieces are inside others' by this much gets no collision
PLACEHOLDER = '0.001 0.001 0.001'


def load_spec(source):
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


def excluded(points, boxes):
    """Mask of ``points`` inside any of ``boxes`` (``((min xyz), (max xyz))``)."""
    mask = np.zeros(len(points), dtype=bool)
    for low, high in boxes:
        mask |= np.all((points >= low) & (points <= high), axis=1)
    return mask


def collision_part(mesh, boxes):
    """``mesh`` without the faces whose centre lies in ``boxes``."""
    if not boxes:
        return mesh
    keep = ~excluded(mesh.triangles_center, boxes)
    part = mesh.submesh([np.flatnonzero(keep)], append=True)
    part.merge_vertices()
    return part


def enclose(hull, max_vertices=MAX_VERTICES):
    """A convex hull with at most ``max_vertices`` vertices that contains ``hull``.

    The hull is decimated, then scaled about its centroid just enough to
    contain every original vertex again (scaling keeps the vertex count).
    """
    import open3d as o3d
    import trimesh
    if len(hull.vertices) <= max_vertices:
        return hull
    target = 2 * max_vertices - 4
    while True:
        decimated = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(hull.vertices),
                                              o3d.utility.Vector3iVector(hull.faces))
        decimated = decimated.simplify_quadric_decimation(int(target))
        reduced = trimesh.Trimesh(np.asarray(decimated.vertices)).convex_hull
        if len(reduced.vertices) <= max_vertices:
            break
        target = int(target * .8)
    center = reduced.vertices.mean(0)
    normals = reduced.face_normals
    offsets = np.einsum('ij,ij->i', normals, reduced.triangles[:, 0]) - normals @ center
    scale = max(1., float(((hull.vertices - center) @ normals.T / offsets).max()))
    return trimesh.Trimesh(center + (reduced.vertices - center) * scale * (1 + 1e-6)).convex_hull


def excess(hulls, visual, samples=20000):
    """99th-percentile distance from the outer surface of ``hulls`` to ``visual``.

    Points of a piece that lie inside another piece are not on the outer
    surface and are skipped.
    """
    import trimesh
    union = trimesh.util.concatenate(hulls)
    points, faces = trimesh.sample.sample_surface(union, samples, seed=0)
    owner = np.repeat(np.arange(len(hulls)), [len(h.faces) for h in hulls])[faces]
    keep = np.ones(len(points), dtype=bool)
    for index, hull in enumerate(hulls):
        others = owner != index
        offsets = np.einsum('ij,ij->i', hull.face_normals, hull.triangles[:, 0])
        inside = ((points[others] @ hull.face_normals.T - offsets).max(axis=1) < -1e-4)
        keep[np.flatnonzero(others)[inside]] = False
    distances = trimesh.proximity.closest_point(visual, points[keep])[1]
    return float(np.percentile(distances, 99))


def decompose(mesh, tolerance):
    """``(pieces, excess)``: the fewest convex pieces (from PIECE_COUNTS) within
    ``tolerance`` of ``mesh``, or the last attempt. Pieces are ordered by
    centroid so files do not depend on CoACD's thread scheduling."""
    for count in PIECE_COUNTS:
        hulls = convex_pieces(mesh, count)
        error = excess(hulls, mesh)
        if error <= tolerance:
            break
    return hulls, error


def convex_pieces(mesh, count):
    """At most ``count`` convex pieces of ``mesh`` (one hull, else CoACD), each
    with at most MAX_VERTICES vertices, ordered by centroid."""
    import coacd
    import trimesh
    if count == 1:
        hulls = [mesh.convex_hull]
    else:
        coacd.set_log_level('error')
        parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=.05, max_convex_hull=count,
                                preprocess_mode='auto', resolution=2000, seed=0)
        hulls = [trimesh.Trimesh(v, f).convex_hull for v, f in parts]
    return sorted((enclose(hull) for hull in hulls), key=lambda hull: tuple(np.round(hull.vertices.mean(0), 6)))


def link_poses(root):
    """Pose of every link with all joints at zero, and the rigid body (fixed-joint root) it belongs to."""
    parent_joint = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
    poses, bodies = {}, {}

    def resolve(name):
        if name not in poses:
            joint = parent_joint.get(name)
            if joint is None:
                poses[name], bodies[name] = np.eye(4), name
            else:
                parent = joint.find('parent').get('link')
                resolve(parent)
                poses[name] = poses[parent] @ read_origin(joint)
                bodies[name] = bodies[parent] if joint.get('type') == 'fixed' else name
        return poses[name]

    for link in root.findall('link'):
        resolve(link.get('name'))
    return poses, bodies


def enclosed_links(root, source):
    """Links whose collision pieces lie inside other links' pieces of the same rigid body."""
    poses, bodies = link_poses(root)
    pieces = {}
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            mesh = load_mesh(source / collision.find('geometry/mesh').get('filename'))
            mesh.apply_transform(poses[link.get('name')] @ read_origin(collision))
            pieces.setdefault(link.get('name'), []).append(mesh)
    enclosed = []
    for name, own in pieces.items():
        others = [hull for other, hulls in pieces.items()
                  if other != name and other not in enclosed and bodies[other] == bodies[name] for hull in hulls]
        if not others:
            continue
        points = np.vstack([hull.vertices for hull in own])
        if outside_distance(points, others).max() <= ENCLOSED_TOLERANCE:
            enclosed.append(name)
    return enclosed


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

def prepare(source, raw):
    """Rewrite ``source/robot.urdf`` and ``source/meshes/`` from the raw package."""
    source, raw = Path(source), Path(raw)
    spec = load_spec(source)
    root = ET.parse(raw / spec.URDF).getroot()
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
    report = {'raw': str(raw), 'urdf': spec.URDF, 'max_error_m': MAX_ERROR, 'visual': {}, 'collision': {}}
    for name, references in shapes.items():
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

    written, tolerances = {}, {}
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
        visuals = [v for v in link.findall('visual') if v.find('geometry/mesh') is not None]
        if not visuals or link.get('name').endswith(spec.NO_COLLISION):
            continue
        for visual in visuals:
            mesh_name = visual.find('geometry/mesh').get('filename')
            stem = Path(mesh_name).stem
            if stem not in written:
                tolerance = spec.excess(link.get('name'))
                boxes = getattr(spec, 'COLLISION_EXCLUDE', {}).get(link.get('name'), [])
                hulls, error = decompose(collision_part(load_mesh(source / mesh_name), boxes), tolerance)
                written[stem] = []
                for index, hull in enumerate(hulls):
                    output = f'meshes/collision/{stem}_{index}.ply'
                    hull.export(source / output)
                    written[stem].append(output)
                report['collision'][stem] = {'pieces': len(hulls), 'excess_m': error, 'tolerance_m': tolerance,
                                             'vertices': max(len(hull.vertices) for hull in hulls)}
                print(f'{stem}: {len(hulls)} pieces, excess {error * 1000:.1f} mm (tolerance {tolerance * 1000:.0f})',
                      flush=True)
            for output in written[stem]:
                collision = ET.SubElement(link, 'collision')
                if visual.find('origin') is not None:
                    collision.append(copy.deepcopy(visual.find('origin')))
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=output)

    report['enclosed'] = enclosed_links(root, source)
    for link in root.findall('link'):
        if link.get('name') in report['enclosed']:
            for collision in link.findall('collision'):
                link.remove(collision)
    used = {c.find('geometry/mesh').get('filename') for c in root.iter('collision')}
    for path in (source / 'meshes' / 'collision').iterdir():
        if f'meshes/collision/{path.name}' not in used:
            path.unlink()
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
    """Coverage of every visual mesh by its collision pieces (outside the
    ``COLLISION_EXCLUDE`` boxes), and source size."""
    source = Path(source)
    exclude = getattr(load_spec(source), 'COLLISION_EXCLUDE', {})
    root = ET.parse(source / 'robot.urdf').getroot()
    pieces = {}
    for collision in root.iter('collision'):
        name = collision.find('geometry/mesh').get('filename')
        pieces.setdefault(Path(name).stem.rsplit('_', 1)[0], set()).add(name)
    coverage = {}
    for stem, names in sorted(pieces.items()):
        vertices = load_mesh(source / 'meshes' / 'visual' / f'{stem}.ply').vertices
        vertices = vertices[~excluded(vertices, exclude.get(stem, []))]
        outside = outside_distance(vertices, [load_mesh(source / name) for name in sorted(names)])
        coverage[stem] = {'pieces': len(names), 'max_outside_m': float(outside.max())}
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
        report = prepare(source, args.assets / load_spec(source).RAW)
        (output / 'prepare.json').write_text(json.dumps(report, indent=2) + '\n')
        worst = max(report['visual'].items(), key=lambda item: item[1]['error_m'])
        pieces = sum(entry['pieces'] for stem, entry in report['collision'].items())
        print(f"{len(report['visual'])} visual meshes, {pieces} collision pieces before dropping enclosed links "
              f"{report['enclosed']}, "
              f"{report['bodies']} bodies; largest simplification error {worst[1]['error_m'] * 1000:.2f} mm ({worst[0]})")
    result = check(source)
    (output / 'check.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f"check {result['status']}: {result['size_bytes'] / 1e6:.2f} MB, uncovered {result['uncovered']}")
    return 0 if result['status'] == 'PASS' else 1
