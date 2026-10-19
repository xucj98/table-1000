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
    layout = SimpleNamespace(supports={'pen': 'table', 'cabinet': 'table'})
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
    layout = SimpleNamespace(supports={'pen': 'cabinet', 'cabinet': 'table'})
    problems = problems_after_settling(settle_report(env, layout, seconds=.04))
    assert len(problems) == 1 and problems[0].startswith('cabinet:housing moved')
