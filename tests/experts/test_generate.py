import types

from table_1000.experts.generate import contact_summary, summarize

ARM = types.SimpleNamespace(finger_links=('l_left', 'l_right'))
SPEC = types.SimpleNamespace(arms={'right': ARM})


def test_unexpected_contacts_leave_out_fingers_on_objects():
    intervals = [
        {'pair': ('pen:body.barrel', 'robot.l_left'), 'start': 0., 'end': 1., 'max_penetration_m': .001, 'phase': 'a'},
        {'pair': ('robot.l_left', 'robot.l_right'), 'start': 0., 'end': 1., 'max_penetration_m': 0., 'phase': 'a'},
        {'pair': ('cabinet:cabinet.housing.shell', 'robot.link4'), 'start': 1., 'end': 1.5,
         'max_penetration_m': .002, 'phase': 'b'},
        {'pair': ('robot.l_left', 'robot.link2'), 'start': 2., 'end': None, 'max_penetration_m': .0, 'phase': 'c'},
    ]
    summary = contact_summary(intervals, SPEC)
    assert set(summary) == {'cabinet:cabinet.housing.shell / robot.link4', 'robot.l_left / robot.link2'}
    assert summary['cabinet:cabinet.housing.shell / robot.link4']['seconds'] == .5


def test_summary_counts_initial_failures_apart():
    reports = [
        {'seed': 0, 'status': 'success', 'initial': {'attempt': 0}, 'steps': 10, 'duration_s': .4, 'wall_s': 5.},
        {'seed': 1, 'status': 'failed', 'initial': {'attempt': 1}, 'failure': {'subtask': 'pick up the pen',
                                                                             'reason': 'pen not held'},
         'steps': 5, 'duration_s': .2, 'wall_s': 4.},
        {'seed': 2, 'status': 'init_failed', 'initial': {'attempts': []}, 'wall_s': 1.},
    ]
    summary = summarize(reports)
    assert summary['episodes'] == 2 and summary['init_failed'] == 1
    assert summary['success_rate'] == .5
    assert summary['failures'] == {'pick up the pen: pen not held': 1}
    assert summary['mean_init_attempts'] == 1.5
