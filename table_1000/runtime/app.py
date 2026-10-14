"""Start Isaac Sim with run-local Kit settings and a verified PhysX worker count.

Kit normally reads and writes a shared ``user.config.json``; a run that changes
persistent settings there silently affects every later run. Here the portable
root, user config and caches all live under the run's output directory, and the
PhysX worker count is requested on the command line and read back.
"""

import json
import os
from pathlib import Path

PHYSICS_WORKERS = '/persistent/physics/numThreads'


def isolated_environment(output, environ=None):
    """Environment for a child process whose temporary files and caches stay in ``output``."""
    output = Path(output).resolve()
    env = dict(os.environ if environ is None else environ)
    env.update(OMNI_KIT_ACCEPT_EULA='YES', PYTHONUNBUFFERED='1', TMPDIR=str(output / 'tmp'),
               XDG_CACHE_HOME=str(output / 'cache'), CUDA_CACHE_PATH=str(output / 'cache/cuda'))
    for name in ('tmp', 'cache/cuda'):
        (output / name).mkdir(parents=True, exist_ok=True)
    return env


def start_app(output, *, gpu, workers=8, headless=True, extra_args=(), **config):
    """Return a ``SimulationApp`` whose Kit settings stay in ``output/kit``.

    ``gpu`` is the physical device index. The readback is written to
    ``output/kit-settings.json``; a worker count or config path that differs from
    the request raises before any stage is built.
    """
    from isaacsim import SimulationApp

    portable = (Path(output) / 'kit').resolve()
    portable.mkdir(parents=True, exist_ok=True)
    user_config = portable / 'user.config.json'
    app = SimulationApp({
        'headless': headless, 'active_gpu': gpu, 'physics_gpu': gpu, 'multi_gpu': False,
        'extra_args': [f'--portable-root={portable}', f'--/app/userConfigPath={user_config}',
                       f'--{PHYSICS_WORKERS}={int(workers)}', *extra_args],
        **config})
    import carb
    settings = carb.settings.get_settings()
    readback = {'requested_workers': int(workers), 'physics_workers': settings.get(PHYSICS_WORKERS),
                'user_config_path': settings.get('/app/userConfigPath'), 'portable_root': str(portable),
                'active_gpu': gpu}
    (Path(output) / 'kit-settings.json').write_text(json.dumps(readback, indent=2) + '\n')
    if readback['physics_workers'] != int(workers) or Path(readback['user_config_path'] or '') != user_config:
        app.close()
        raise RuntimeError(f'Kit settings differ from the request: {readback}')
    return app
