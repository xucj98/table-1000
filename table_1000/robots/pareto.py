"""Accuracy against cost of one link's collision geometry, for choosing its piece count.

``scripts/robots/collision_pareto.py x1pro lift_link --counts 1 2 3 4 8 16 32 --output outputs/robots/pareto``
writes, per count, a robot source variant in which only that link's collision
is replaced (``COLLISION_CLIP`` applies), builds it, and runs the check
suite's benchmark on it. ``pareto.json`` and ``pareto.png`` plot the gap
between collision and visual surface against physics and planning cost.
Physics cost depends on the machine's load; the variants run one after
another, ``--rounds`` times in alternating order.
"""

import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

from table_1000.robots import prepare

ROOT = Path(__file__).resolve().parents[2]


def gap(hulls, visual):
    """Mean and 99th-percentile distance from the outer collision surface to the visual."""
    import trimesh
    union = trimesh.util.concatenate(hulls)
    points, _ = trimesh.sample.sample_surface(union, 20000, seed=0)
    return prepare.excess(hulls, visual), float(np.mean(trimesh.proximity.closest_point(visual, points)[1]))


def make_variant(robot, link, count, folder):
    """Copy the robot source under ``folder`` with ``link``'s collision split into ``count`` pieces."""
    source = ROOT / 'asset_sources' / 'robots' / robot
    target = folder / 'asset_sources' / 'robots' / robot
    shutil.rmtree(folder, ignore_errors=True)
    shutil.copytree(source, target)
    spec = prepare.load_spec(source)
    root = ET.parse(target / 'robot.urdf').getroot()
    element = next(l for l in root.findall('link') if l.get('name') == link)
    visual = next(v for v in element.findall('visual')
                  if Path(v.find('geometry/mesh').get('filename')).stem == link)
    mesh = prepare.load_mesh(target / visual.find('geometry/mesh').get('filename'))
    hulls = prepare.convex_pieces(mesh, count, getattr(spec, 'COLLISION_CLIP', {}).get(link, []))
    part = mesh
    for collision in element.findall('collision'):
        name = collision.find('geometry/mesh').get('filename')
        if Path(name).stem.rsplit('_', 1)[0] == link:
            element.remove(collision)
            (target / name).unlink()
    for index, hull in enumerate(hulls):
        name = f'meshes/collision/{link}_{index}.ply'
        hull.export(target / name)
        collision = ET.SubElement(element, 'collision')
        if visual.find('origin') is not None:
            collision.append(copy.deepcopy(visual.find('origin')))
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=name)
    ET.indent(root)
    ET.ElementTree(root).write(target / 'robot.urdf', encoding='unicode')
    p99, mean = gap(hulls, part)
    return {'count': count, 'pieces': len(hulls), 'gap_p99_mm': p99 * 1e3, 'gap_mean_mm': mean * 1e3,
            'vertices': int(sum(len(h.vertices) for h in hulls))}


def run(robot, link, counts, output, gpu, rounds=2):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    python = sys.executable
    rows = {}
    for count in counts:
        folder = output / f'pieces-{count}'
        rows[count] = make_variant(robot, link, count, folder)
        subprocess.run([python, str(ROOT / 'scripts/assets/build_robot.py'), robot,
                        '--source', str(folder / 'asset_sources/robots'), '--output', str(folder / 'assets/robots'),
                        '--gpu', str(gpu)], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # The check scene references the robot only; the rest of assets/ is not needed.
        print(json.dumps(rows[count]), flush=True)
    for round_index in range(rounds):
        for count in (counts if round_index % 2 == 0 else counts[::-1]):
            folder = output / f'pieces-{count}'
            bench = folder / f'bench-{round_index}'
            shutil.rmtree(bench, ignore_errors=True)
            subprocess.run([python, str(ROOT / 'scripts/robots/check_robot.py'), robot, '--benchmark',
                            '--root', str(folder), '--assets', str(folder / 'assets'), '--output', str(bench),
                            '--gpu', str(gpu)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            report = json.loads((bench / 'report.json').read_text())
            rows[count].setdefault('runs', []).append(report.get('benchmark') or {'error': report.get('error')})
            print(count, round_index, json.dumps(rows[count]['runs'][-1]), flush=True)
    for row in rows.values():
        runs = [r for r in row['runs'] if 'error' not in r]
        for key in ('physics_hold_ms', 'physics_self_contact_ms', 'self_collision_check_us', 'rrt_median_s'):
            row[key] = float(np.median([r[key] for r in runs])) if runs else None
    (output / 'pareto.json').write_text(json.dumps({'robot': robot, 'link': link, 'rows': list(rows.values())},
                                                   indent=1))
    plot(list(rows.values()), link, output / 'pareto.png')
    return rows


def pareto_front(points):
    """Indices of points not beaten in both coordinates (lower is better for both)."""
    return [i for i, (x, y) in enumerate(points)
            if not any(a <= x and b <= y and (a, b) != (x, y) for a, b in points)]


def plot(rows, link, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    metrics = [('physics_self_contact_ms', 'physics, arm swinging into torso (ms / 4 ms step)'),
               ('physics_hold_ms', 'physics, holding still (ms / step)'),
               ('self_collision_check_us', 'planning self-collision check (µs)')]
    figure, axes = plt.subplots(1, len(metrics), figsize=(5.2 * len(metrics), 4.4))
    for axis, (key, label) in zip(axes, metrics):
        valid = [r for r in rows if r.get(key) is not None]
        points = [(r['gap_p99_mm'], r[key]) for r in valid]
        front = pareto_front(points)
        axis.plot([points[i][0] for i in sorted(front, key=lambda i: points[i][0])],
                  [points[i][1] for i in sorted(front, key=lambda i: points[i][0])], color='#c44', lw=1.2, zorder=1)
        for i, (row, (x, y)) in enumerate(zip(valid, points)):
            axis.scatter(x, y, color='#c44' if i in front else '#888', zorder=2)
            axis.annotate(str(row['pieces']), (x, y), textcoords='offset points', xytext=(5, 4), fontsize=9)
        axis.set_xlabel(f'{link}: collision-to-visual gap, p99 (mm)')
        axis.set_ylabel(label)
        axis.grid(alpha=.3)
    figure.suptitle(f'{link} collision pieces: accuracy vs cost (labels: piece count; red: Pareto front)')
    figure.tight_layout()
    figure.savefig(path, dpi=130)
