"""Scenes: ``asset_sources/scenes/<name>/`` holds ``scene.py`` (what is in the scene and how it
starts) and ``expert.py`` (how the expert tidies it). Simulator-independent helpers."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# The one instruction of every Table-1000 scene; experts divide it into subtasks.
INSTRUCTION = 'tidy up the table'


def source_dir(name, root=ROOT):
    return Path(root) / 'asset_sources' / 'scenes' / name


def load(name, module='scene', root=ROOT):
    """Module ``module`` (``'scene'`` or ``'expert'``) of scene ``name``."""
    path = source_dir(name, root) / f'{module}.py'
    spec = importlib.util.spec_from_file_location(f'scene_{name.replace("-", "_")}_{module}', path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def scene_names(root=ROOT):
    return sorted(path.parent.name for path in (Path(root) / 'asset_sources' / 'scenes').glob('*/scene.py'))


def primitive_surface(spec, pose, count=400):
    """Points on the surface of a primitive (``{'shape', 'size'}``, as for :func:`add_primitive`) posed in the world."""
    import trimesh
    if spec['shape'] == 'sphere':
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=spec['size'])
    elif spec['shape'] == 'box':
        mesh = trimesh.creation.box(spec['size'])
    else:
        mesh = trimesh.creation.cylinder(radius=spec['size'][0], height=spec['size'][1])
    points, _ = trimesh.sample.sample_surface_even(mesh, count, seed=0)
    return points @ pose[:3, :3].T + pose[:3, 3]
