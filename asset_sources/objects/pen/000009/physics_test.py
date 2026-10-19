"""Approved independent-button click contracts, native finite-force drives."""
from table_1000.physics.pen_tests import click_tests
MASSES = {'pen.shell': .006, 'pen.button': .0004, 'pen.refill': .0016}
TESTS = click_tests(masses=MASSES, drop_dt=.001)
