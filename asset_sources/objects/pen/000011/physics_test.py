"""Six native cap contracts; applied loads are not sampled reaction forces."""
from table_1000.physics.pen_tests import detent_tests

MASSES = {'body': 0.011, 'cap': 0.002}
DETENT = {'seat': -0.025, 'neck': 0.0056, 'segments': 12, 'balls': 3, 'radius': 0.00045, 'height': 0.000253125, 'travel': 0.00045000000000000004, 'mass': 0.000177978515625, 'stiffness': 984.375, 'damping': 0.16874999999999998, 'preload': 0.00016874999999999998, 'flat': False, 'dt': 0.001}
PROTOCOL = {'max_pull_N': 1.5, 'max_push_N': 1.5, 'weak_pull_N': 0.15, 'light_push_N': 0.04, 'cap_half_height_m': 0.007}

TESTS = detent_tests(masses=MASSES, detent=DETENT, protocol=PROTOCOL)
