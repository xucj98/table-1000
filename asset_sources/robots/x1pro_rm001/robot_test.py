"""X1Pro (rm001 gripper) acceptance settings: those of x1pro (../x1pro/robot_test.py)."""

from table_1000.robots import load_robot_test

_X1PRO = load_robot_test('x1pro')

TABLE, ROBOT, WORK, DROP_LINKS, CASES = _X1PRO.TABLE, _X1PRO.ROBOT, _X1PRO.WORK, _X1PRO.DROP_LINKS, _X1PRO.CASES
