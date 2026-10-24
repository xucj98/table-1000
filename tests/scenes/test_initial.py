"""Settling includes loose caps/balls and measures motion relative to supports."""

from types import SimpleNamespace

import numpy as np
import pytest

from table_1000.scenes.initial import problems_after_settling, settle_report


@pytest.mark.parametrize('body', ['pen:cap', 'pen:cap_ball1', 'cabinet:drawer'])
@pytest.mark.parametrize('rotation', [False, True])
def test_settling_checks_every_body(body, rotation):
    poses = {key: np.eye(4)[None].copy() for key in
             ['pen:body', 'pen:cap', 'pen:cap_ball1', 'cabinet:housing', 'cabinet:drawer']}

    def step():
        if rotation:
            poses[body][0, :3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        else:
            poses[body][0, 0, 3] = .2

    env = SimpleNamespace(privileged=lambda: {'poses': poses, 'joints': {}}, step=step,
                          contacts=SimpleNamespace(clear=lambda: None, intervals=lambda: []))
    layout = SimpleNamespace(supports={'pen': 'table', 'cabinet': 'table'}, scene=SimpleNamespace(TABLE={'height': -1.}))
    report = settle_report(env, layout, seconds=.04)
    assert set(report['objects']) == set(poses)
    problems = problems_after_settling(report)
    assert len(problems) == 1 and problems[0].startswith(body + ' moved')


def test_settling_uses_the_support_frame():
    poses = {key: np.eye(4)[None].copy() for key in ['pen:body', 'pen:cap', 'cabinet:housing']}

    def step():
        for pose in poses.values():
            pose[0, 0, 3] += .2

    env = SimpleNamespace(privileged=lambda: {'poses': poses, 'joints': {}}, step=step,
                          object_pose=lambda name: poses['cabinet:housing'],
                          contacts=SimpleNamespace(clear=lambda: None, intervals=lambda: []))
    layout = SimpleNamespace(supports={'pen': 'cabinet', 'cabinet': 'table'}, scene=SimpleNamespace(TABLE={'height': -1.}))
    problems = problems_after_settling(settle_report(env, layout, seconds=.04))
    assert len(problems) == 1 and problems[0].startswith('cabinet:housing moved')


def test_a_loose_object_may_move_within_what_holds_it():
    from table_1000.scenes.initial import strayed

    def at(x):
        pose = np.eye(4)[None].copy()
        pose[0, 0, 3] = x
        return pose

    poses = {'cup': at(0.), 'clip': at(.02), 'clip2': at(.2)}
    env = SimpleNamespace(object_pose=lambda name: poses[name])
    # clip2 was dropped onto clip, which lies in the cup: both are held by the cup.
    # Each clip is a 1 cm cube around its origin; a notebook leaning out of the cup keeps its foot in it.
    def box(half, height):
        return np.array([[x, y, z] for x in (-half, half) for y in (-half, half) for z in (0., height)])

    cube, cup = box(.005, .01), box(.05, .1)
    poses['notebook'] = at(.06)          # its origin is past the cup's wall, its foot is not
    leaning = np.array([[-.03, 0., 0.], [-.01, 0., 0.], [.09, 0., .2]])

    def world_points(name, transform):
        local = {'notebook': leaning, 'cup': cup}.get(name, cube)
        return local @ transform[:3, :3].T + transform[:3, 3]

    layout = SimpleNamespace(loose={'clip', 'clip2', 'notebook'},
                             supports={'clip': 'cup', 'clip2': 'clip', 'cup': 'table', 'notebook': 'cup'},
                             world_points=world_points,
                             scene=SimpleNamespace(TABLE={'center': (0., 0.), 'size': (1., 1.)}))
    assert strayed(env, layout) == ['clip2 left cup']
