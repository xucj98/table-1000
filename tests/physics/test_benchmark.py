"""Complete short experiments warm up independently without padding their duration."""
from itertools import count
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from table_1000.physics.test import benchmark


@pytest.mark.parametrize('duration,warm_repeats,round_repeats',
                         [(.8, 2, 13), (.1, 10, 100), (1., 1, 10), (2.4, 1, 5)])
def test_complete_fresh_repeats_cover_warmup_and_three_measured_rounds(duration, warm_repeats, round_repeats):
    contexts = []

    def create(*args, **kwargs):
        if contexts:
            contexts[-1].close.assert_called_once()
        ctx = SimpleNamespace(session=SimpleNamespace(time=0.), rows=[],
                              preparation_seconds=.02, close=Mock())
        contexts.append(ctx)
        return ctx

    def experiment(ctx):
        assert ctx.session.time == 0. and ctx.rows == []
        ctx.experiment_started = 0.
        ctx.session.time = duration
        ctx.rows.append({'time': duration, 'q': 42.})

    with (patch('table_1000.physics.testing.TestContext', side_effect=create),
          patch('table_1000.physics.test.time.perf_counter', side_effect=count())):
        result = benchmark('object.usdz', 'short', experiment, None)
    assert len(contexts) == warm_repeats + 3*round_repeats
    for ctx in contexts:
        ctx.close.assert_called_once()
        assert ctx.session.time == duration
    assert result['warmup']['independent_repeats'] == warm_repeats
    assert result['warmup']['simulated_seconds'] == pytest.approx(warm_repeats*duration)
    assert result['warmup']['wall_seconds'] == warm_repeats
    assert len(result['rounds']) == 3
    for measured in result['rounds']:
        assert measured['independent_repeats'] == round_repeats
        assert measured['simulated_seconds'] == pytest.approx(round_repeats*duration)
        assert measured['final_states'] == [{'time': duration, 'q': 42.}]*round_repeats
        assert measured['RTF'] > 0


def test_failure_in_second_warmup_retains_that_context_and_original_assertion():
    contexts = [SimpleNamespace(session=SimpleNamespace(time=.8), close=Mock()),
                SimpleNamespace(session=SimpleNamespace(time=.2), close=Mock(side_effect=RuntimeError('cleanup')))]
    experiment = Mock(side_effect=[None, AssertionError('physical criterion')])
    with (patch('table_1000.physics.testing.TestContext', side_effect=contexts) as create,
          patch('table_1000.physics.test.capture_failure') as capture, patch('sys.stderr')):
        with pytest.raises(AssertionError, match='physical criterion'):
            benchmark('object.usdz', 'short', experiment, None, diagnostics_directory='evidence')
    assert create.call_count == 2
    capture.assert_called_once_with(contexts[1], 'object.usdz', 'short', 'evidence',
                                    physics='cpu', num_envs=1, phase='benchmark_warmup')
    for ctx in contexts:
        ctx.close.assert_called_once()


def test_warmup_cleanup_failure_stops_before_next_repeat():
    ctx = SimpleNamespace(session=SimpleNamespace(time=.8), close=Mock(side_effect=RuntimeError('cleanup')))
    with patch('table_1000.physics.testing.TestContext', return_value=ctx) as create:
        with pytest.raises(RuntimeError, match='cleanup'):
            benchmark('object.usdz', 'short', lambda ctx: None, None)
    create.assert_called_once()
    ctx.close.assert_called_once()


@pytest.mark.parametrize('duration', [0., -.8, float('nan'), float('inf')])
def test_nonprogressing_warmup_fails_instead_of_repeating_forever(duration):
    ctx = SimpleNamespace(session=SimpleNamespace(time=duration), close=Mock())
    with patch('table_1000.physics.testing.TestContext', return_value=ctx) as create:
        with pytest.raises(AssertionError, match='finite positive experiment duration'):
            benchmark('object.usdz', 'short', lambda ctx: None, None)
    create.assert_called_once()
    ctx.close.assert_called_once()
