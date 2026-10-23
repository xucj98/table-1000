import math
import types

import numpy as np
import pytest

pytest.importorskip('mplib')

from table_1000.scenes.geometry import AssetGeometry
from table_1000.scenes.layout import Layout, SamplingFailed


def prism_model(sides=6, radius=.01, length=.1):
    """A one-body asset: a ``sides``-gon prism along X with a vertex pointing down at rest."""
    angles = [-math.pi / 2 + 2 * math.pi * i / sides for i in range(sides)]
    ring = [(radius * math.cos(a), radius * math.sin(a)) for a in angles]
    vertices = [[x, y, z] for x in (-length / 2, length / 2) for y, z in ring]
    return {'nodes': [
        {'name': 'body', 'parent': None, 'matrix': np.eye(4).tolist(), 'rigid': True},
        {'name': 'body.collision', 'parent': 'body', 'matrix': np.eye(4).tolist(), 'rigid': False,
         'role': 'collision', 'shape': 'CONVEX_HULL', 'vertices': vertices}], 'joints': []}


def box_model(size):
    x, y, z = np.asarray(size) / 2
    vertices = [[sx * x, sy * y, sz * z + z] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return {'nodes': [
        {'name': 'box', 'parent': None, 'matrix': np.eye(4).tolist(), 'rigid': True},
        {'name': 'box.collision', 'parent': 'box', 'matrix': np.eye(4).tolist(), 'rigid': False,
         'role': 'collision', 'shape': 'CONVEX_HULL', 'vertices': vertices}], 'joints': []}


TABLE = {'size': (1., 1.), 'height': .7, 'top_thickness': .03, 'leg_size': .05, 'center': (0., 0.), 'friction': (.5, .4)}


def layout(models, seed=0):
    scene = types.SimpleNamespace(TABLE=TABLE, OBJECTS={name: {'asset': name} for name in models})
    result = Layout.__new__(Layout)
    Layout.__init__(result, types.SimpleNamespace(TABLE=TABLE, OBJECTS={}), None, np.random.default_rng(seed))
    result.scene = scene
    result.geometry = {name: AssetGeometry(model) for name, model in models.items()}
    return result


def test_prism_rolls_onto_a_face():
    geometry = AssetGeometry(prism_model())
    rotation = geometry.stable_rotation(np.eye(3))
    # A vertex points down at rest; the nearest face is half a side (30 degrees) away about X.
    angle = math.degrees(math.acos((np.trace(rotation) - 1) / 2))
    assert angle == pytest.approx(30., abs=.5)
    assert np.allclose(rotation @ [1., 0., 0.], [1., 0., 0.], atol=1e-6)


def test_authored_center_of_mass_keeps_a_leaning_body_upright():
    # A slab leaning out past its foot (a desk lamp's arm over its base): uniform density tips it over.
    vertices = [[x + lean, y, z] for z, lean in ((0., 0.), (.2, .12)) for x in (-.02, .02) for y in (-.05, .05)]
    model = {'nodes': [
        {'name': 'body', 'parent': None, 'matrix': np.eye(4).tolist(), 'rigid': True},
        {'name': 'body.collision', 'parent': 'body', 'matrix': np.eye(4).tolist(), 'rigid': False,
         'role': 'collision', 'shape': 'CONVEX_HULL', 'vertices': vertices}], 'joints': []}
    assert not np.allclose(AssetGeometry(model).stable_rotation(np.eye(3)), np.eye(3), atol=1e-6)
    heavy_foot = AssetGeometry(model, {'body': [0., 0., .01]})
    assert np.allclose(heavy_foot.stable_rotation(np.eye(3)), np.eye(3), atol=1e-6)


def test_resting_pose_touches_the_table_and_place_avoids_overlap():
    models = {'a': box_model((.2, .2, .1)), 'b': box_model((.2, .2, .1))}
    scene = layout(models)
    first = scene.place('a', region=((0., 0.), (0., 0.)))
    assert first[2, 3] == pytest.approx(TABLE['height'])
    assert scene.penetration('b', first) > .01
    second = scene.place('b', region=((-.3, -.3), (.3, .3)))
    assert scene.penetration('b', second) <= .001
    assert scene.penetrations() == {}


def test_place_gives_up_when_the_region_is_taken():
    models = {'a': box_model((.2, .2, .1)), 'b': box_model((.2, .2, .1))}
    scene = layout(models)
    scene.place('a', region=((0., 0.), (0., 0.)))
    with pytest.raises(SamplingFailed):
        scene.place('b', region=((-.01, -.01), (.01, .01)), tries=5)


def test_sampling_is_reproducible():
    models = {'a': box_model((.1, .1, .1))}
    poses = [layout(models, seed=3).place('a', region=((-.3, -.3), (.3, .3)), yaw=(-1., 1.)) for _ in range(2)]
    assert np.array_equal(*poses)


def cup_model(size=.1, wall=.004, height=.08):
    """An open box: a floor and four walls, each its own hull."""
    half = size / 2
    parts = {'floor': ((-half, -half, 0.), (half, half, wall)),
             'wall1': ((-half, -half, 0.), (-half + wall, half, height)),
             'wall2': ((half - wall, -half, 0.), (half, half, height)),
             'wall3': ((-half, -half, 0.), (half, -half + wall, height)),
             'wall4': ((-half, half - wall, 0.), (half, half, height))}
    nodes = [{'name': 'cup', 'parent': None, 'matrix': np.eye(4).tolist(), 'rigid': True}]
    for name, (low, high) in parts.items():
        vertices = [[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])]
        nodes.append({'name': f'cup.{name}', 'parent': 'cup', 'matrix': np.eye(4).tolist(), 'rigid': False,
                      'role': 'collision', 'shape': 'CONVEX_HULL', 'vertices': vertices})
    return {'nodes': nodes, 'joints': []}


def test_hull_overlap_is_exact_for_thin_hulls():
    from table_1000.scenes.geometry import Hull, overlap
    # Two thin rods 7 mm apart: FCL's convex test reported such wire loops as colliding.
    rod = Hull.of([[x, y, z] for x in (-.01, .01) for y in (-.0008, .0008) for z in (-.0008, .0008)])
    turned = np.eye(4)
    turned[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    turned[2, 3] = .0086
    assert overlap(rod, rod.posed(turned)) < 0
    turned[2, 3] = .001
    assert overlap(rod, rod.posed(turned)) == pytest.approx(.0006, abs=1e-6)


def test_drop_lands_inside_a_container_and_records_its_support():
    scene = layout({'cup': cup_model(), 'box': box_model((.04, .04, .02))})
    scene.place('cup', region=((0., 0.), (0., 0.)))
    pose, support = scene.drop('box', (0., 0.))
    assert support == 'cup' and 'box' in scene.loose
    assert pose[2, 3] == pytest.approx(TABLE['height'] + .004, abs=.0003)
    assert scene.penetrations() == {}
    pose, support = scene.drop('box', (.3, .3))
    assert support == 'table' and pose[2, 3] == pytest.approx(TABLE['height'], abs=.0003)


def test_open_joint_moves_the_geometry():
    model = box_model((.1, .1, .1))
    model['nodes'].append({'name': 'lid', 'parent': None, 'matrix': np.eye(4).tolist(), 'rigid': True})
    model['nodes'].append({'name': 'lid.collision', 'parent': 'lid', 'matrix': np.eye(4).tolist(), 'rigid': False,
                           'role': 'collision', 'shape': 'CONVEX_HULL',
                           'vertices': [[x, y, z] for x in (-.05, .05) for y in (-.05, .05) for z in (.1, .11)]})
    frame = np.eye(4)
    frame[:3, :3] = [[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]]     # slides toward -Y
    model['joints'] = [{'name': 'lid.slide', 'type': 'SLIDER', 'body1': 'box', 'body2': 'lid',
                        'matrix': frame.tolist(), 'limits': [0., .1]}]
    geometry = AssetGeometry(model)
    assert np.allclose(geometry.body_poses(np.eye(4), {'lid_slide': .03})['lid'][:3, 3], [0., -.03, 0.])
