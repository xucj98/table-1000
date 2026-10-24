"""Accounting must survive nesting, lazy consumers and failed searches."""

import pytest

from table_1000.common import timing
from table_1000.common.timing import WallProfile, measure, timed


def test_nested_exception_and_disabled(monkeypatch):
    clock = iter([0., 1., 3., 5., 10., 12.])
    monkeypatch.setattr(timing, 'perf_counter', lambda: next(clock))

    def child():
        with measure('child'):
            raise ValueError('unreachable')

    @timed('parent')
    def parent():
        with pytest.raises(ValueError):
            child()

    profile = WallProfile()
    with profile.record():
        parent()
        before = profile.snapshot()
        with profile.span('child'):
            pass
    assert before['parent'] == {'calls': 1, 'wall_s': 5., 'self_s': 3.}
    assert before['child'] == {'calls': 1, 'wall_s': 2., 'self_s': 2.}
    assert profile.since(before)['child'] == {'calls': 1, 'wall_s': 2., 'self_s': 2.}
    # The recording context is reset, and disabled profiling does not read the clock.
    parent()
    with profile.record(False):
        parent()
    assert profile.snapshot()['parent'] == before['parent']


def test_lazy_candidates_exclude_consumer(monkeypatch):
    clock = iter([0., 1., 20., 22., 40., 40.])
    monkeypatch.setattr(timing, 'perf_counter', lambda: next(clock))

    @timed('candidates')
    def candidates():
        yield 1
        yield 2

    profile = WallProfile()
    with profile.record():
        assert list(candidates()) == [1, 2]
    assert profile.snapshot()['candidates'] == {'calls': 3, 'wall_s': 3., 'self_s': 3.}
