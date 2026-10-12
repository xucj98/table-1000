"""scene-000000 placement: independent seeded offsets and layout constraints."""

import itertools
import unittest

import numpy as np

from table_1000.scenes import load

SCENE = load('scene-000000')


class PlacementTest(unittest.TestCase):
    def test_seed_is_reproducible_and_independent(self):
        first, again = SCENE.placement(7), SCENE.placement(7)
        self.assertEqual(first['offsets'], again['offsets'])
        np.testing.assert_array_equal(first['pen'], again['pen'])
        self.assertNotEqual(first['offsets']['cabinet'], first['offsets']['pen'])

    def test_offsets_stay_in_region_and_keep_orientation(self):
        for seed in range(200):
            layout = SCENE.placement(seed)
            for name, spec in (('cabinet', SCENE.CABINET), ('pen', SCENE.PEN)):
                offset = np.asarray(layout['offsets'][name])
                self.assertTrue(np.all(np.abs(offset) <= SCENE.HALF_RANGE))
                np.testing.assert_allclose(layout[name][:2, 3], np.asarray(spec['center']) + offset)
                np.testing.assert_allclose(layout[name][:3, :3], SCENE.pose(spec['center'], 0, spec['yaw'])[:3, :3])

    def test_out_of_region_offsets_are_rejected(self):
        with self.assertRaises(ValueError):
            SCENE.placement(offsets={'cabinet': (.06, 0.)})
        with self.assertRaises(ValueError):
            SCENE.placement(1, offsets={})

    def test_every_corner_combination_is_valid(self):
        values = (-SCENE.HALF_RANGE, 0., SCENE.HALF_RANGE)
        grid = list(itertools.product(values, values))
        for cabinet, pen in itertools.product(grid, grid):
            with self.subTest(cabinet=cabinet, pen=pen):
                self.assertEqual(SCENE.check(SCENE.placement(offsets={'cabinet': cabinet, 'pen': pen})), [])

    def test_check_reports_overlap_and_missing_support(self):
        layout = SCENE.placement()
        layout['pen'] = layout['cabinet'].copy()
        self.assertIn('pen inside the cabinet or its fully opened drawer space', SCENE.check(layout))
        layout = SCENE.placement()
        layout['cabinet'][0, 3] = SCENE.TABLE['size'][0] / 2
        self.assertIn('cabinet not fully supported by the table', SCENE.check(layout))


if __name__ == '__main__':
    unittest.main()
