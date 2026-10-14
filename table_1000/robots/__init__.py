"""Robot assets built from URDF sources."""

import importlib


def load_spec(name):
    """The :class:`~table_1000.robots.agent.RobotSpec` of ``table_1000.robots.<name>``, or None."""
    try:
        module = importlib.import_module(f'table_1000.robots.{name}')
    except ModuleNotFoundError:
        return None
    return getattr(module, 'SPEC', None)
