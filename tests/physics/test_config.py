"""Keyframe checks do not require starting Isaac."""
import unittest
from table_1000.physics.testing import keyframes, interpolate


class KeyframeTests(unittest.TestCase):
    def test_inheritance_interpolation_and_end_exclusive(self):
        frames=keyframes([{'time':0,'force':[0,0,0]},{'time':.004,'force':[2,0,0]},{'time':.008}],.004)
        self.assertEqual(frames[-1]['force'],[2,0,0])
        self.assertEqual(interpolate(frames,.002)['force'].tolist(),[1,0,0])
        self.assertIsNone(interpolate(frames,.008))

    def test_times_align_with_the_actual_physical_step(self):
        with self.assertRaises(ValueError):
            keyframes([{'time':0,'force':[0,0,0]},{'time':.005,'force':[1,0,0]}],.004)
