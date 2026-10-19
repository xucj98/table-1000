"""Regression coverage for uv's split-wheel symlink installation."""

from importlib.machinery import ModuleSpec
import os
import sys

from table_1000.common.path import configure_isaac_paths


def test_symlink_keeps_merged_environment_paths(tmp_path, monkeypatch):
    cached = tmp_path / 'cache' / 'isaacsim'
    cached.mkdir(parents=True)
    (cached / '__init__.py').write_text("raise RuntimeError('bootstrap must not run during path setup')\n")
    installed = tmp_path / 'site-packages' / 'isaacsim'
    installed.mkdir(parents=True)
    (installed / '__init__.py').symlink_to(cached / '__init__.py')
    for directory in ('apps', 'kit', 'exts'):
        (installed / directory).mkdir()
    monkeypatch.syspath_prepend(str(installed.parent))
    monkeypatch.delitem(sys.modules, 'isaacsim', raising=False)
    for name in ('ISAAC_PATH', 'EXP_PATH', 'CARB_APP_PATH'):
        monkeypatch.delenv(name, raising=False)

    configure_isaac_paths()

    assert os.environ['ISAAC_PATH'] == str(installed)
    assert os.environ['EXP_PATH'] == str(installed / 'apps')
    assert os.environ['CARB_APP_PATH'] == str(installed / 'kit')


def test_explicit_paths_survive_repeated_configuration(monkeypatch):
    spec = ModuleSpec('isaacsim', loader=None, origin='/environment/isaacsim/__init__.py')
    monkeypatch.setattr('table_1000.common.path.importlib.util.find_spec', lambda name: spec)
    paths = {'ISAAC_PATH': '/custom/isaac', 'EXP_PATH': '/custom/apps',
             'CARB_APP_PATH': '/custom/kit'}
    for name, value in paths.items():
        monkeypatch.setenv(name, value)

    configure_isaac_paths()
    configure_isaac_paths()

    assert {name: os.environ[name] for name in paths} == paths
