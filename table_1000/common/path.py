"""Locate Isaac Sim resources in the active Python environment."""

import importlib.util
import os
from pathlib import Path


def configure_isaac_paths():
    """Set Isaac Sim resource paths before importing Isaac Sim or Isaac Lab.

    uv's symlink mode assembles multiple wheels under site-packages. Resolving
    isaacsim/__init__.py to its cache target loses that combined resource tree.
    Explicit environment settings take precedence; repeated calls are harmless.
    """
    spec = importlib.util.find_spec('isaacsim')
    if spec is None:
        raise ModuleNotFoundError("No module named 'isaacsim'", name='isaacsim')
    root = Path(spec.origin).parent
    for name, path in (('ISAAC_PATH', root), ('EXP_PATH', root / 'apps'),
                       ('CARB_APP_PATH', root / 'kit')):
        os.environ.setdefault(name, str(path))
