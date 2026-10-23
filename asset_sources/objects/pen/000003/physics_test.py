"""Six native cap contracts; applied loads are not sampled reaction forces."""
from table_1000.physics.pen_tests import detent_tests

MASSES = {'body': 0.012, 'cap': 0.003}
DETENT = {'seat': -0.032, 'neck': 0.0078, 'segments': 16, 'balls': 3, 'radius': 0.0006, 'height': 0.00033749999999999996, 'travel': 0.0006, 'mass': 0.00042187499999999983, 'stiffness': 600.0, 'damping': 0.22499999999999995, 'preload': 0.0001, 'flat': False, 'dt': 0.001}
PROTOCOL = {'max_pull_N': 2.0, 'max_push_N': 2.0, 'weak_pull_N': 0.15, 'light_push_N': 0.04, 'cap_half_height_m': 0.01,
            'support_face_placement': True, 'gpu_free_reaction_solver': 'PGS'}

TESTS = detent_tests(masses=MASSES, detent=DETENT, protocol=PROTOCOL)
