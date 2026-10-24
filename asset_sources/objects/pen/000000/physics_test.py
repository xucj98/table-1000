"""Four independent force/release checks of the approved click approximation."""
from table_1000.physics.pen_tests import click_tests
MASSES = {'pen.shell': 0.01, 'pen.button': 0.001, 'pen.refill': 0.002}
TESTS = click_tests(masses=MASSES, drop_dt=.001)
