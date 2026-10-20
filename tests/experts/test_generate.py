import types

from table_1000.experts.generate import finger_contacts

ARM = types.SimpleNamespace(finger_links=('l_left', 'l_right'))
SPEC = types.SimpleNamespace(arms={'right': ARM})


def test_finger_contacts_keep_only_fingers_on_objects():
    intervals = [
        {'pair': ('pen:body.barrel', 'robot.l_left'), 'start': 0., 'end': 1., 'max_penetration_m': .001, 'phase': 'a'},
        {'pair': ('pen:body.barrel', 'robot.l_left'), 'start': 2., 'end': 2.5, 'max_penetration_m': .002, 'phase': 'b'},
        {'pair': ('robot.l_left', 'robot.l_right'), 'start': 0., 'end': 1., 'max_penetration_m': 0., 'phase': 'a'},
        {'pair': ('cabinet:cabinet.housing.shell', 'robot.link4'), 'start': 1., 'end': 1.5,
         'max_penetration_m': .002, 'phase': 'b'},
        {'pair': ('robot.l_left', 'robot.link2'), 'start': 2., 'end': None, 'max_penetration_m': .0, 'phase': 'c'},
    ]
    contacts = finger_contacts(intervals, SPEC)
    assert list(contacts) == ['pen:body.barrel / robot.l_left']
    assert contacts['pen:body.barrel / robot.l_left'] == {'max_penetration_m': .002, 'seconds': 1.5,
                                                          'subtasks': ['a', 'b']}
