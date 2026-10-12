"""Offline reachability of the drawer-and-pen key poses over region corners.

For every combination of cabinet and pen offsets on the 3 x 3 grid (centre,
edge midpoints, corners) the key TCP poses of the task are solved by MPlib IK
with collision checking against the scene at rest: handle pre-grasp, grasp,
the opened handle and its retreats, pen pre-grasp, grasp and lifts, and the
release pose above the opened drawer. Needs MPlib, not Isaac Sim; object poses
come from the scene placement and the assets' rest geometry.
"""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from table_1000.planning.world import PlanningScene, asset_objects, box_object, model_shapes, pose
from table_1000.robots import x1pro
from table_1000.scenes import load
from table_1000.scenes.stage import robot_base
from table_1000.tasks import drawer_pen_geometry as g
from table_1000.tasks.drawer_pen import PEN_LIFTS, PLACE_TILT, STANDOFFS, CabinetGeometry

ROOT = Path(__file__).resolve().parents[2]


def setup(scene, assets, output):
    frozen = {name: value for name, value in scene.ROBOT['joints'].items() if name not in x1pro.ARM}
    planner = x1pro.build_planner(ROOT / 'asset_sources' / 'robots' / 'x1pro' / 'robot.urdf', output, frozen)
    base = robot_base(scene)
    world = PlanningScene(planner, np.linalg.inv(base))
    table = scene.TABLE
    top = np.eye(4)
    top[:3, 3] = [*table['center'], table['height'] - table['top_thickness'] / 2]
    world.add(*box_object('table', (*table['size'], table['top_thickness']), top))
    cabinet_model = Path(assets) / scene.CABINET['asset'] / 'model.json'
    objects = {
        'cabinet': asset_objects(cabinet_model, 'cabinet:',
                                 lambda body, part: (part if part.endswith('.handle') else body, body)),
        'pen': asset_objects(Path(assets) / scene.PEN['asset'] / 'model.json', 'pen:',
                             lambda body, part: ('pen', 'body'), merged=lambda name: True)}
    rests = {'cabinet': model_shapes(cabinet_model)[1], 'pen': model_shapes(Path(assets) / scene.PEN['asset'] / 'model.json')[1]}
    cabinet = CabinetGeometry(cabinet_model)
    cabinet.pen_com_x = center_of_mass_x(Path(assets) / scene.PEN['asset'])
    return planner, base, world, objects, rests, cabinet


def center_of_mass_x(asset):
    """Axial centre of mass of an assembly as PhysX computes it: each body's
    mass (from the asset's physics.py) at its collision-volume centroid."""
    import importlib.util
    import trimesh
    spec = importlib.util.spec_from_file_location('asset_physics', Path(asset) / 'physics.py')
    physics = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(physics)
    shapes, rest = model_shapes(Path(asset) / 'model.json')
    total, moment = 0., np.zeros(3)
    for body, mass in physics.MASSES.items():
        volume, weighted = 0., np.zeros(3)
        for owner, _, transform, kind, data in shapes:
            if owner != body:
                continue
            if kind == 'sphere':
                v, centre = 4 / 3 * np.pi * data ** 3, np.zeros(3)
            else:
                hull = trimesh.Trimesh(data).convex_hull
                v, centre = hull.volume, hull.center_mass
            volume += v
            weighted += v * (rest[body] @ np.r_[transform[:3, :3] @ centre + transform[:3, 3], 1.])[:3]
        total += mass
        moment += mass * weighted / volume
    return float(moment[0] / total)


def solvable(planner, base, target, fingers):
    seed = np.zeros(len(planner.user_joint_names))
    seed[len(x1pro.ARM):] = [fingers / 2, -fingers / 2]
    status, _ = planner.IK(pose(np.linalg.inv(base) @ target), seed, mask=[False] * 6 + [True] * 2,
                           n_init_qpos=40)
    return status == 'Success'


def case(scene, layout, planner, base, world, objects, rests, cabinet):
    for name, group in objects.items():
        for obj in group:
            world.add(obj, layout[name] @ rests[name][obj.body[len(name) + 1:]])
    drawer = layout['cabinet'] @ rests['cabinet'][g.DRAWER]
    housing = layout['cabinet'] @ rests['cabinet']['cabinet.housing']
    world.allow(['cabinet:' + g.HANDLE], x1pro.FINGER_LINKS)
    grasp = g.handle_frame(drawer, cabinet.handle_center) @ g.handle_tcp(cabinet.face_distance)
    opened = g.translated(grasp, -drawer[:3, 1] * g.OPENING)
    open_drawer = drawer.copy()
    open_drawer[:3, 3] -= drawer[:3, 1] * g.OPENING
    result = {'handle pre-grasp': any(solvable(planner, base, g.standoff(grasp, d), .08) for d in STANDOFFS),
              'handle grasp': solvable(planner, base, grasp, .08)}
    world.set_pose('cabinet:' + g.DRAWER, open_drawer)
    world.set_pose('cabinet:' + g.HANDLE, open_drawer)
    result['opened handle'] = solvable(planner, base, opened, .05)
    result['opened retreat'] = any(solvable(planner, base, g.standoff(opened, d), .08) for d in STANDOFFS)
    world.allow(['pen:pen'], x1pro.FINGER_LINKS)
    pen = g.pen_tcp(layout['pen'], scene.TABLE['height'], cabinet.pen_com_x)
    result['pen pre-grasp'] = solvable(planner, base, g.standoff(pen, g.PEN_APPROACH), .05)
    result['pen grasp'] = solvable(planner, base, pen, .05)
    result['pen lift'] = any(solvable(planner, base, g.translated(pen, [0, 0, h]), .019) for h in PEN_LIFTS)
    face = cabinet.housing_face_in_drawer(housing, open_drawer)
    release = g.drawer_release_point(open_drawer, cabinet.interior, face, cabinet.wall_top)
    place = g.place_tcp(release, np.linalg.inv(pen) @ layout['pen'], open_drawer[:3, 0], PLACE_TILT, open_drawer[:3, 1])
    result['release'] = solvable(planner, base, place, .019)
    world.allow(['pen:pen'], x1pro.FINGER_LINKS, False)
    world.allow(['cabinet:' + g.HANDLE], x1pro.FINGER_LINKS, False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default='scene-000000')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    scene = load(args.scene)
    planner, base, world, objects, rests, cabinet = setup(scene, args.assets, args.output / 'planning')
    values = (-scene.HALF_RANGE, 0., scene.HALF_RANGE)
    grid = list(itertools.product(values, values))
    rows = []
    for cabinet_offset, pen_offset in itertools.product(grid, grid):
        layout = scene.placement(offsets={'cabinet': cabinet_offset, 'pen': pen_offset})
        result = case(scene, layout, planner, base, world, objects, rests, cabinet)
        rows.append({'cabinet': cabinet_offset, 'pen': pen_offset, **result})
    totals = {key: sum(row[key] for row in rows) for key in rows[0] if key not in ('cabinet', 'pen')}
    summary = {'cases': len(rows), 'reachable': totals,
               'failing': [row for row in rows if not all(v for k, v in row.items() if k not in ('cabinet', 'pen'))]}
    (args.output / 'reach.json').write_text(json.dumps({**summary, 'rows': rows}, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if not summary['failing'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
