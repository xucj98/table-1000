"""Scene sources: ``asset_sources/scenes/<name>/scene.py`` modules."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load(name, sources=ROOT / 'asset_sources' / 'scenes'):
    """Import ``<sources>/<name>/scene.py`` as a module."""
    path = Path(sources) / name / 'scene.py'
    spec = importlib.util.spec_from_file_location(f'scene_{name.replace("-", "_")}', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.path = path
    return module
