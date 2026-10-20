"""Every scene source initializes from its built assets without penetration, reproducibly."""

import numpy as np
import pytest

pytest.importorskip('mplib')

from table_1000.scenes import ROOT, load, scene_names
from table_1000.scenes.layout import Layout

ASSETS = ROOT / 'assets'


@pytest.mark.parametrize('name', scene_names())
def test_scene_initializes(name):
    scene = load(name)
    missing = [spec['asset'] for spec in scene.OBJECTS.values() if not (ASSETS / spec['asset'] / 'model.json').is_file()]
    if missing:
        pytest.skip(f'assets not built: {missing}')
    assert hasattr(load(name, 'expert'), 'solve')
    for seed in range(3):
        layouts = []
        for _ in range(2):
            layout = Layout(scene, ASSETS, np.random.default_rng([seed, 0]))
            scene.initialize(layout)
            layouts.append(layout)
        assert set(layouts[0].poses) == set(scene.OBJECTS)
        assert layouts[0].penetrations() == {}
        for key in layouts[0].poses:
            assert np.array_equal(layouts[0].poses[key], layouts[1].poses[key])
