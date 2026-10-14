"""Robots built from URDF sources: ``asset_sources/robots/<name>/``."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def source_module(source, module):
    """Module ``module`` (``'robot'``, ``'robot_test'``) of the robot source directory ``source``, or None."""
    path = Path(source) / f'{module}.py'
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location(f'robot_{Path(source).name}_{module}', path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def source_dir(name, root=ROOT):
    return Path(root) / 'asset_sources' / 'robots' / name


def load_spec(name, root=ROOT):
    """The :class:`~table_1000.robots.agent.RobotSpec` in ``asset_sources/robots/<name>/robot.py``, or None."""
    module = source_module(source_dir(name, root), 'robot')
    return getattr(module, 'SPEC', None) if module else None


def load_robot_test(name, root=ROOT):
    """The robot's acceptance settings, ``asset_sources/robots/<name>/robot_test.py``, or None."""
    return source_module(source_dir(name, root), 'robot_test')


def robot_names(root=ROOT):
    return sorted(path.parent.name for path in (Path(root) / 'asset_sources' / 'robots').glob('*/robot.py'))
