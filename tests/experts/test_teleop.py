"""Teleoperating cases without a simulator: the cases of batches, drags in a view, the operated step."""

import json
import types

import numpy as np

from table_1000.experts.context import ExpertFailed, Step
from table_1000.experts.teleop import HumanExpert, cases, check, drag, failed_step, view_camera

NAMES = ['open', 'move', 'close']


def report(seed, done, status='failed', steps_run=None, reason='no grasp'):
    return {'seed': seed, 'status': status, 'step_names': NAMES, 'steps_run': steps_run,
            'steps': [{'step': step, 'name': NAMES[step - 1], 'at': 10 * step} for step in done],
            **({'failure': {'subtask': 'x', 'reason': reason}} if status != 'success' else {})}


def write(batch, summary, reports):
    (batch / 'episodes').mkdir(parents=True)
    (batch / 'summary.json').write_text(json.dumps(summary))
    for name, entry in reports.items():
        (batch / 'episodes' / f'{name}.json').write_text(json.dumps(entry))


def test_the_failed_step_is_the_first_not_completed():
    assert failed_step(report(0, [1])) == 2
    assert failed_step(report(0, [], steps_run=[2, 3])) == 2
    assert failed_step(report(0, [1, 2, 3], status='success')) is None


def test_cases_come_by_success_rate_and_start_before_the_failed_step(tmp_path):
    whole = tmp_path / 'expert' / '000002' / 'a-whole'
    write(whole, {'run': 'a-whole', 'scene': '000002'},
          {'0': report(0, [1, 2, 3], status='success'), '1': report(1, [1])})
    repeated = tmp_path / 'expert' / '000002' / 'b-repeats'
    write(repeated, {'run': 'b-repeats', 'scene': '000002', 'steps_run': [2, 2], 'from': str(whole),
                     'repeats': {'count': 3, 'succeeded': {'4': 1, '5': 2, '6': 3}}},
          {'4': report(4, [], steps_run=[2, 2]), '4-1': report(4, [2], status='success', steps_run=[2, 2]),
           '5': report(5, [2], status='success', steps_run=[2, 2]), '5-2': report(5, [], steps_run=[2, 2], reason='drop'),
           '6': report(6, [2], status='success', steps_run=[2, 2])})
    found = cases(tmp_path / 'expert', '000002')
    assert [(case['run'], case['seed'], case['successes']) for case in found] == \
        [('b-repeats', 4, 1), ('b-repeats', 5, 2), ('a-whole', 1, None)]
    # A repeated seed whose first run succeeded is a case from the run that failed.
    assert found[1]['failure'] == 'drop' and found[1]['rate'] == 2 / 3
    # The batch's first step starts from the checkpoints of the batch it ran from; a later one from its own.
    assert found[0]['start'] == str(whole / 'checkpoints' / 'step1.hdf5')
    assert found[2]['start'] == str(whole / 'checkpoints' / 'step1.hdf5') and found[2]['step'] == 2


def test_a_drag_moves_in_the_image_plane_at_the_point_depth():
    view = {'position': (0., -2., 1.), 'target': (0., 0., 1.), 'focal_length_mm': 20.955, 'resolution': (100, 50)}
    camera = view_camera(view)          # fx = 100 px, looking along +y
    point = np.array([0., 0., 1.])      # 2 m ahead
    np.testing.assert_allclose(drag(camera, point, 10., 0.), [.2, 0., 0.], atol=1e-12)     # right: +x
    np.testing.assert_allclose(drag(camera, point, 0., 10.), [0., 0., -.2], atol=1e-12)    # down: -z
    assert drag(camera, point, 0., 0., wheel=1.)[1] < 0.                                   # toward the camera


def test_the_operated_step_replaces_the_expert_step_unchecked():
    calls = []
    expert = types.SimpleNamespace(STEPS=[Step('a', lambda ctx: calls.append('a'), lambda ctx: None),
                                          Step('b', lambda ctx: calls.append('b'), lambda ctx: ctx.fail('no'))])
    human = HumanExpert(expert, 2, lambda ctx, step: calls.append(('operated', step.name)))
    assert human.STEPS[0] is expert.STEPS[0]
    assert human.STEPS[1].check is None
    human.STEPS[1].run(None)
    assert calls == [('operated', 'b')]


def test_a_check_reports_its_verdict_and_keeps_the_subtask():
    class Ctx:
        current = 'take the pen'

        def fail(self, reason):
            raise ExpertFailed(self.current, reason)
    ctx = Ctx()
    assert check(ctx, Step('b', None, lambda ctx: ctx.fail('pen not on the table'))) == \
        {'passed': False, 'reason': 'pen not on the table'}
    assert check(ctx, Step('b', None, lambda ctx: None)) == {'passed': True, 'reason': None}
    assert ctx.current == 'take the pen'
