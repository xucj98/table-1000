"""Compare collision variants of a robot on accuracy, task cost, size and renders.

``scripts/robots/collision_variants.py x1pro baseline --source asset_sources/robots/x1pro --output outputs/robots/variants``
copies this checkout to ``<output>/<name>/checkout`` with the robot source
replaced by ``--source`` (code, scenes and object assets unchanged), builds the
robot there, and writes ``<output>/<name>/report.json`` with:

- ``accuracy``: distance from the outer collision surface to the visual
  surface, mean and 99th percentile, for the hand (fingers and the link
  carrying the gripper), the arm (the other arm links) and the body (the rest).
  Points are sampled uniformly over each group's collision surface.
- ``size``: collision files, pieces and vertices; source and built USD bytes.
- ``task``: the drawer-and-pen ``standard`` batch (19 cases, no rendering) run
  from the copy, summed over cases: total, physics stepping and planning time.
- ``renders``: collision of the whole robot, one arm and one hand in the
  ``ready`` pose (``collision_full.jpg``; ``collision_arm.jpg`` and ``collision_hand.jpg`` show only
  those links).

Task timings depend on the machine's load: compare variants run under the
same ``--jobs``/``--per-process``, preferably one after another.
"""

import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

from table_1000.robots import load_spec

ROOT = Path(__file__).resolve().parents[2]
COPIED = ('table_1000', 'scripts', 'asset_sources', 'pyproject.toml')
SAMPLES_PER_M2 = 2e5
MIN_SAMPLES = 2000
BUILD_ATTEMPTS = 3
# The hand as committed: the drawer-and-pen grasps fail on millimetre changes, so variants of the arm and
# body keep it to stay comparable.
HAND = ('gripper_left_link', 'gripper_right_link', 'gripper_base_link', 'gripper_camera_base_link')


def group(link):
    """``'hand'``, ``'arm'`` or ``'body'`` of a link by name."""
    if 'gripper' in link or link.endswith('arm_link6'):
        return 'hand'
    return 'arm' if 'arm_link' in link else 'body'


def _origin(element):
    from scipy.spatial.transform import Rotation
    transform = np.eye(4)
    node = element.find('origin')
    if node is not None:
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
    return transform


def _meshes(source, link, kind):
    import trimesh
    from table_1000.robots.primitives import element_mesh
    meshes = []
    for element in link.findall(kind):
        if element.find('geometry/mesh') is None:
            meshes.append(element_mesh(element))
            continue
        mesh = trimesh.load(source / element.find('geometry/mesh').get('filename'), force='mesh', process=False)
        mesh.apply_transform(_origin(element))
        meshes.append(mesh)
    return meshes


def surface_gaps(hulls, visual, count, seed=0):
    """Distances to ``visual`` of ``count`` points on the outer surface of ``hulls``
    (points inside another hull are not on the outer surface and are dropped)."""
    import trimesh
    union = trimesh.util.concatenate(hulls)
    points, faces = trimesh.sample.sample_surface(union, count, seed=seed)
    owner = np.repeat(np.arange(len(hulls)), [len(h.faces) for h in hulls])[faces]
    keep = np.ones(len(points), dtype=bool)
    for index, hull in enumerate(hulls):
        others = owner != index
        offsets = np.einsum('ij,ij->i', hull.face_normals, hull.triangles[:, 0])
        inside = (points[others] @ hull.face_normals.T - offsets).max(axis=1) < -1e-4
        keep[np.flatnonzero(others)[inside]] = False
    return distance(visual, points[keep])


def distance(mesh, points):
    """Unsigned distance from ``points`` to the surface of ``mesh`` (Open3D, much faster than trimesh)."""
    import open3d as o3d
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(mesh.vertices), o3d.utility.Vector3iVector(mesh.faces))))
    return scene.compute_distance(o3d.core.Tensor(np.asarray(points, dtype=np.float32))).numpy().astype(float)


def accuracy(source):
    """``{'hand' | 'arm' | 'body': {'mean_mm', 'p99_mm', 'links': {link: {...}}}}`` of a robot source."""
    import trimesh
    source = Path(source)
    distances, result = {}, {}
    for link in ET.parse(source / 'robot.urdf').getroot().findall('link'):
        collision = _meshes(source, link, 'collision')
        visual = _meshes(source, link, 'visual')
        if not collision or not visual:
            continue
        hulls = [mesh.convex_hull for mesh in collision]
        area = sum(hull.area for hull in hulls)
        gaps = surface_gaps(hulls, trimesh.util.concatenate(visual), max(MIN_SAMPLES, int(area * SAMPLES_PER_M2)))
        name = group(link.get('name'))
        # Every point stands for the same area, so pooled statistics weigh links by area.
        weight = area / len(gaps)
        distances.setdefault(name, []).append((gaps, weight))
        result.setdefault(name, {'links': {}})['links'][link.get('name')] = {
            'pieces': len(hulls), 'mean_mm': 1e3 * float(gaps.mean()),
            'p99_mm': 1e3 * float(np.percentile(gaps, 99))}
    for name, parts in distances.items():
        values = np.concatenate([gaps for gaps, _ in parts])
        weights = np.concatenate([np.full(len(gaps), weight) for gaps, weight in parts])
        order = np.argsort(values)
        cumulative = np.cumsum(weights[order]) / weights.sum()
        result[name].update(mean_mm=1e3 * float(np.average(values, weights=weights)),
                            p99_mm=1e3 * float(values[order][np.searchsorted(cumulative, .99)]),
                            worst_link=max(result[name]['links'], key=lambda k: result[name]['links'][k]['p99_mm']))
    return result


def parts(source):
    """``{part: [(link, visual element)]}``: each visual mesh (named by its file stem) and the links using it.
    Left and right arms share mesh files, so one part can be used by several links."""
    result = {}
    for link in ET.parse(Path(source) / 'robot.urdf').getroot().findall('link'):
        for visual in link.findall('visual'):
            result.setdefault(Path(visual.find('geometry/mesh').get('filename')).stem, []).append(link.get('name'))
    return result


def part_gaps(source):
    """``{part: {'group', 'pieces', 'p99_mm', 'mean_mm'}}`` with each part's own collision pieces."""
    import trimesh
    from table_1000.robots import prepare
    source = Path(source)
    result = {}
    for part, links in parts(source).items():
        files = sorted((f for f in (source / 'meshes' / 'collision').glob(f'{part}_*') if f.suffix in ('.ply', '.stl')),
                       key=lambda f: int(f.stem.rsplit('_', 1)[1]))
        files = [f for f in files if f.stem.rsplit('_', 1)[0] == part]
        if not files:
            continue
        hulls = [trimesh.load(f, force='mesh', process=False) for f in files]
        visual = prepare.load_mesh(source / 'meshes' / 'visual' / f'{part}.ply')
        area = sum(h.area for h in hulls)
        gaps = surface_gaps(hulls, visual, max(MIN_SAMPLES, int(area * SAMPLES_PER_M2)))
        result[part] = {'group': group(links[0]), 'links': links, 'pieces': len(hulls),
                        'p99_mm': 1e3 * float(np.percentile(gaps, 99)), 'mean_mm': 1e3 * float(gaps.mean())}
    return result


def edit_source(base, target, cap=False, pieces=None, keep=()):
    """Copy the robot source ``base`` to ``target``; with ``cap`` every collision piece is reduced to at
    most MAX_VERTICES vertices (still enclosing it), except the ``keep`` parts; ``pieces`` (``{part: count}``) re-decomposes those
    parts with CoACD (the robot's ``COLLISION_CLIP`` applies)."""
    import copy
    import trimesh
    from table_1000.robots import prepare
    base, target = Path(base), Path(target)
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(base, target, ignore=shutil.ignore_patterns('__pycache__'))
    folder = target / 'meshes' / 'collision'
    if cap:
        for path in [*folder.glob('*.ply'), *folder.glob('*.stl')]:
            if path.stem.rsplit('_', 1)[0] in keep:
                continue
            hull = trimesh.load(path, force='mesh', process=False).convex_hull
            if len(hull.vertices) > prepare.MAX_VERTICES:
                prepare.enclose(hull).export(path)
    clips = getattr(prepare.load_spec(base), 'COLLISION_CLIP', {})
    tree = ET.parse(target / 'robot.urdf')
    for part, count in (pieces or {}).items():
        mesh = prepare.load_mesh(target / 'meshes' / 'visual' / f'{part}.ply')
        hulls = prepare.convex_pieces(mesh, count, clips.get(part, []))
        for path in folder.glob(f'{part}_*'):
            if path.stem.rsplit('_', 1)[0] == part:
                path.unlink()
        for index, hull in enumerate(hulls):
            hull.export(folder / f'{part}_{index}.ply')
        for link in tree.getroot().findall('link'):
            visual = next((v for v in link.findall('visual')
                           if Path(v.find('geometry/mesh').get('filename')).stem == part), None)
            if visual is None:
                continue
            for collision in link.findall('collision'):
                if Path(collision.find('geometry/mesh').get('filename')).stem.rsplit('_', 1)[0] == part:
                    link.remove(collision)
            for index in range(len(hulls)):
                collision = ET.SubElement(link, 'collision')
                if visual.find('origin') is not None:
                    collision.append(copy.deepcopy(visual.find('origin')))
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh',
                              filename=f'meshes/collision/{part}_{index}.ply')
    ET.indent(tree.getroot())
    tree.write(target / 'robot.urdf', encoding='unicode')
    as_stl(target)
    return target


def as_stl(source):
    """Store the collision meshes of a robot source as STL. Isaac's mesh converter crashes (SIGSEGV, in a
    worker thread) on most builds whose collision is many small PLY hulls; the same hulls as STL import."""
    import trimesh
    source = Path(source)
    for path in (source / 'meshes' / 'collision').glob('*.ply'):
        trimesh.load(path, force='mesh', process=False).export(path.with_suffix('.stl'))
        path.unlink()
    tree = ET.parse(source / 'robot.urdf')
    for mesh in tree.getroot().iter('mesh'):
        name = mesh.get('filename')
        if name.startswith('meshes/collision/') and name.endswith('.ply'):
            mesh.set('filename', name[:-4] + '.stl')
    tree.write(source / 'robot.urdf', encoding='unicode')


def _bytes(path):
    path = Path(path)
    return sum(f.stat().st_size for f in path.rglob('*') if f.is_file()) if path.is_dir() else path.stat().st_size


def size(source, built):
    import trimesh
    source = Path(source)
    files = sorted((source / 'meshes' / 'collision').glob('*'))
    elements = ET.parse(source / 'robot.urdf').getroot().findall('.//collision/geometry/*')
    kinds = {kind: sum(element.tag == kind for element in elements) for kind in ('mesh', 'box', 'cylinder', 'sphere')}
    return {'collision_files': len(files), 'collision_bytes': sum(f.stat().st_size for f in files),
            'collision_vertices': int(sum(len(trimesh.load(f, force='mesh').vertices) for f in files)),
            'collision_elements': len(elements), 'collision_kinds': kinds,
            'source_bytes': _bytes(source), 'built_bytes': _bytes(built)}


def make_checkout(robot, source, folder):
    """Copy this checkout's code and sources to ``folder`` with the robot source replaced by ``source``.
    ``assets/`` links every object build of this checkout; the robot is built into the copy."""
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    ignore = shutil.ignore_patterns('__pycache__', '*.pyc')
    for name in COPIED:
        (shutil.copytree if (ROOT / name).is_dir() else shutil.copy2)(ROOT / name, folder / name, **(
            {'ignore': ignore} if (ROOT / name).is_dir() else {}))
    target = folder / 'asset_sources' / 'robots' / robot
    shutil.rmtree(target)
    shutil.copytree(source, target, ignore=ignore)
    (folder / 'assets' / 'robots').mkdir(parents=True)
    for entry in (ROOT / 'assets').iterdir():
        if entry.name != 'robots':
            (folder / 'assets' / entry.name).symlink_to(entry.resolve())
    return folder


def task_cost(summary):
    rows = summary['rows']
    total = lambda key: float(sum(row[key] or 0. for row in rows))
    return {'cases': len(rows), 'passed': summary['passed'], 'regrasps': int(sum(row['regrasps'] or 0 for row in rows)),
            'unexpected_contacts': int(sum(row['unexpected_contacts'] or 0 for row in rows)),
            'processes_ok': summary['processes_ok'], 'simulated_s': total('simulated_s'),
            'total_s': total('case_s'), 'physics_s': total('step_s'), 'planning_s': total('planning_s'),
            'other_s': total('case_s') - total('step_s') - total('planning_s'), 'batch_wall_s': summary['wall_s'],
            'jobs': summary['jobs'], 'per_process': summary['per_process']}


def renders(robot, checkout, output):
    """Collision renders in the ``ready`` pose: the whole robot, then one arm and its hand alone."""
    from table_1000.robots.preview import render_panels
    from PIL import Image
    spec = load_spec(robot)
    urdf = checkout / 'asset_sources' / 'robots' / robot / 'robot.urdf'
    links = [link.get('name') for link in ET.parse(urdf).getroot().findall('link')]
    side = sorted(spec.arms)[-1]
    arm = [link for link in links if link.startswith(side) and group(link) in ('arm', 'hand')]
    hand = [link for link in links if link.startswith(side) and group(link) == 'hand']
    views = {'full': None, 'arm': arm, 'hand': hand}
    panels = [{'kind': 'collision', 'path': str(output / f'collision_{name}.png'), 'focus': focus,
               'isolate': focus is not None} for name, focus in views.items()]
    render_panels(urdf, spec.keyframe('ready'), panels)
    paths = {}
    for name, panel in zip(views, panels):
        path = output / f'collision_{name}.jpg'
        Image.open(panel['path']).convert('RGB').save(path, quality=85)
        Path(panel['path']).unlink()
        paths[name] = path.name
    return paths


def run(robot, name, source, output, gpus, jobs=4, per_process=5, skip_task=False):
    folder = Path(output) / name
    folder.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    report = {'robot': robot, 'name': name, 'source': str(Path(source).resolve())}
    checkout = make_checkout(robot, Path(source).resolve(), folder / 'checkout')
    robot_source = checkout / 'asset_sources' / 'robots' / robot
    build = time.perf_counter()
    # Isaac's mesh converter sometimes crashes (SIGSEGV) on many small hulls; a new process usually succeeds.
    for attempt in range(BUILD_ATTEMPTS):
        completed = subprocess.run([sys.executable, str(checkout / 'scripts/assets/build_robot.py'), robot,
                                    '--source', str(robot_source.parent), '--output', str(checkout / 'assets/robots'),
                                    '--gpu', str(gpus[0])], cwd=checkout,
                                   stdout=(folder / f'build-{attempt}.log').open('w'), stderr=subprocess.STDOUT)
        if completed.returncode == 0:
            break
    else:
        raise RuntimeError(f'build failed {BUILD_ATTEMPTS} times, see {folder}/build-*.log')
    report['build_attempts'] = attempt + 1
    report['build_s'] = time.perf_counter() - build
    report['accuracy'] = accuracy(robot_source)
    report['size'] = size(robot_source, checkout / 'assets' / 'robots' / robot)
    report['renders'] = renders(robot, checkout, folder)
    write(folder, report)
    if not skip_task:
        batch = folder / 'standard'
        if batch.exists():
            shutil.rmtree(batch)
        subprocess.run([sys.executable, str(checkout / 'scripts/tasks/drawer_pen_batch.py'), 'standard',
                        '--output', str(batch), '--gpus', *map(str, gpus), '--jobs', str(jobs),
                        '--per-process', str(per_process)], cwd=checkout,
                       stdout=(folder / 'batch.log').open('w'), stderr=subprocess.STDOUT)
        report['task'] = task_cost(json.loads((batch / 'summary.json').read_text()))
    report['report_s'] = time.perf_counter() - started
    write(folder, report)
    return report


def write(folder, report):
    (folder / 'report.json').write_text(json.dumps(report, indent=1) + '\n')


def table(reports):
    """Markdown comparison of ``report.json`` contents."""
    lines = ['| 方案 | 手 99%/均值 | 臂 99%/均值 | 身体 99%/均值 | 总耗时 | 仿真 | 规划 | 其他 | 通过 | 凸块 | 顶点 | 碰撞文件 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in reports:
        a, s, t = r['accuracy'], r['size'], r.get('task') or {}
        cell = lambda g: f"{a[g]['p99_mm']:.1f} / {a[g]['mean_mm']:.1f} mm"
        sec = lambda k: f"{t[k]:.0f} s" if k in t else '-'
        lines.append(f"| {r['name']} | {cell('hand')} | {cell('arm')} | {cell('body')} | {sec('total_s')} | "
                     f"{sec('physics_s')} | {sec('planning_s')} | {sec('other_s')} | "
                     f"{t.get('passed', '-')}/{t.get('cases', '-')} | {s['collision_elements']} | "
                     f"{s['collision_vertices']} | {s['collision_bytes'] / 1e6:.2f} MB |")
    return '\n'.join(lines)
