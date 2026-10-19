"""Six native cap contracts; applied loads are not sampled reaction forces."""
from table_1000.physics.pen_tests import detent_tests

MASSES = {'body': 0.007, 'cap': 0.002}
DETENT = {'seat': -0.0275, 'neck': 0.0038, 'segments': 12, 'balls': 3, 'radius': 0.00035, 'height': 0.000196875, 'travel': 0.00035, 'mass': 8.3740234375e-05, 'stiffness': 765.625, 'damping': 0.13125, 'preload': 0.00013125, 'flat': False, 'dt': 0.001}
PROTOCOL = {'max_pull_N': 1.0, 'max_push_N': 1.0, 'weak_pull_N': 0.08, 'light_push_N': 0.04, 'cap_half_height_m': 0.0055}

TESTS = detent_tests(masses=MASSES, detent=DETENT, protocol=PROTOCOL)
