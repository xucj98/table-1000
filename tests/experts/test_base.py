import numpy as np

from table_1000.experts.skills.base import drive, drive_path


def test_drive_turns_drives_and_turns_to_the_goal():
    start = np.array([0., 0., np.pi / 2])
    for goal in [(.3, .1, np.pi / 2), (-.2, -.4, 0.), (0., .5, -np.pi)]:
        first, distance, last = drive(start, goal)
        heading = start[2] + first
        end = start[:2] + distance * np.array([np.cos(heading), np.sin(heading)])
        assert np.allclose(end, goal[:2])
        assert np.isclose(np.cos(heading + last), np.cos(goal[2])) and np.isclose(np.sin(heading + last), np.sin(goal[2]))
        assert abs(first) <= np.pi / 2 + 1e-9        # drives back rather than turning round
        assert np.allclose(drive_path(start, goal)[-1][:2], goal[:2])


def test_drive_backs_off_straight_without_turning():
    assert np.allclose(drive(np.array([0., 0., np.pi / 2]), (0., -.2, np.pi / 2)), (0., -.2, 0.))
