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
