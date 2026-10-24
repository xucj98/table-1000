"""Optional wall profiling of nested planning operations, including lazy candidate generation.

``wall_s`` includes measured children; ``self_s`` excludes them and can be summed across
categories. Native-library collision checks inside IK or RRT remain in their parent's time.
Generator calls measure each resume only, never the work a consumer does between yields.
"""

from contextlib import contextmanager
from contextvars import ContextVar
from functools import wraps
from inspect import isgeneratorfunction
from time import perf_counter


_active = ContextVar('wall_profile', default=None)


class WallProfile:
    def __init__(self):
        self.totals = {}
        self.stack = []

    @contextmanager
    def record(self, enabled=True):
        if not enabled:
            yield self
            return
        token = _active.set(self)
        try:
            yield self
        finally:
            _active.reset(token)

    @contextmanager
    def span(self, name):
        frame = [perf_counter(), 0.]
        self.stack.append(frame)
        try:
            yield
        finally:
            elapsed = perf_counter() - frame[0]
            self.stack.pop()
            row = self.totals.setdefault(name, {'calls': 0, 'wall_s': 0., 'self_s': 0.})
            row['calls'] += 1
            row['wall_s'] += elapsed
            row['self_s'] += elapsed - frame[1]
            if self.stack:
                self.stack[-1][1] += elapsed

    def snapshot(self):
        return {name: dict(row) for name, row in self.totals.items()}

    def since(self, before):
        return {name: {key: value - before.get(name, {}).get(key, 0) for key, value in row.items()}
                for name, row in self.totals.items()}


def timed(name):
    """Measure a function (or each resume of a generator) while a profile is recording."""
    def decorate(function):
        if isgeneratorfunction(function):
            @wraps(function)
            def generator(*args, **kwargs):
                source = function(*args, **kwargs)
                try:
                    while True:
                        profile = _active.get()
                        try:
                            if profile is None:
                                value = next(source)
                            else:
                                with profile.span(name):
                                    value = next(source)
                        except StopIteration:
                            return
                        yield value
                finally:
                    source.close()
            return generator

        @wraps(function)
        def measured(*args, **kwargs):
            profile = _active.get()
            if profile is None:
                return function(*args, **kwargs)
            with profile.span(name):
                return function(*args, **kwargs)
        return measured
    return decorate
