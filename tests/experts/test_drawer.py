"""A small stationary drawer gap must not hang the rest of the seed batch."""

from types import SimpleNamespace

import numpy as np
import pytest

from table_1000.scenes import load


@pytest.mark.parametrize('moves_to_target', [None, 3, 9])
def test_drawer_slide_is_bounded(moves_to_target):
    module = load('000000', 'expert')
    calls = []

    def fail(reason, **details):
        raise RuntimeError(reason)

    expert = module.Expert.__new__(module.Expert)
    expert.arm = SimpleNamespace(carry=lambda *a: None, release=lambda *a: None,
                                tcp=lambda: np.eye(4), move_to_pose_with_screw=calls.append)
    expert.ctx = SimpleNamespace(
        joint=lambda *a: 0. if moves_to_target and len(calls) >= moves_to_target else .010,
        pose=lambda *a: np.eye(4), fail=fail)
    expert.cabinet_xy, expert.regrasps = np.zeros(2), 0
    if moves_to_target:
        expert.slide(0.)
        assert len(calls) == moves_to_target
    else:
        with pytest.raises(RuntimeError, match='drawer motion limit reached'):
            expert.slide(0.)
        assert len(calls) == module.MAX_SLIDE_MOVES
    assert expert.regrasps == 0
