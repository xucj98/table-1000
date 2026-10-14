"""Start Isaac Sim without touching the shared Kit user config, and create the physics world.

Kit reads ``/persistent/...`` settings from a ``user.config.json`` shared by
every run of the installation and may write changed ones back on exit, so one
run could silently change later ones. Here each run points Kit at an empty
config in a temporary directory (removed when the app closes), requests the
PhysX worker count on the command line and reads it back. Shader and texture caches stay
shared: they only speed up start-up.
"""

import os
from pathlib import Path
import shutil
import tempfile

PHYSICS_WORKERS = '/persistent/physics/numThreads'
DT = .004


def start_app(*, gpu, workers=4, headless=True, extra_args=(), **config):
    """Return a ``SimulationApp`` on physical GPU ``gpu`` with ``workers`` PhysX threads.

    With ``CUDA_VISIBLE_DEVICES`` set to that GPU alone, PhysX sees it as device 0.
    A worker count or config path that differs from the request raises before
    any stage is built.
    """
    from isaacsim import SimulationApp

    portable = Path(tempfile.mkdtemp(prefix='kit-'))
    user_config = portable / 'user.config.json'
    physics_gpu = 0 if os.getenv('CUDA_VISIBLE_DEVICES') == str(gpu) else gpu
    app = SimulationApp({
        'headless': headless, 'active_gpu': gpu, 'physics_gpu': physics_gpu, 'multi_gpu': False,
        'extra_args': [f'--portable-root={portable}', f'--/app/userConfigPath={user_config}',
                       f'--{PHYSICS_WORKERS}={int(workers)}', *extra_args],
        **config})
    # app.close() may end the process before atexit handlers run, so the
    # (empty) directory goes first.
    close = app.close

    def close_and_clean(*args, **kwargs):
        shutil.rmtree(portable, ignore_errors=True)
        return close(*args, **kwargs)

    app.close = close_and_clean
    import carb
    settings = carb.settings.get_settings()
    readback = {'physics_workers': settings.get(PHYSICS_WORKERS), 'user_config_path': settings.get('/app/userConfigPath')}
    if readback['physics_workers'] != int(workers) or Path(readback['user_config_path'] or '') != user_config:
        app.close()
        raise RuntimeError(f'Kit settings differ from the request ({workers} workers, {user_config}): {readback}')
    return app


def new_world(rendering_dt=DT * 10):
    """A new stage and an Isaac ``World`` on CPU PhysX: dt 4 ms, TGS, MBP broadphase, CCD on."""
    import omni.usd
    from isaacsim.core.api import World
    World.clear_instance()
    omni.usd.get_context().new_stage()
    world = World(physics_dt=DT, rendering_dt=rendering_dt, stage_units_in_meters=1, backend='numpy', device='cpu')
    context = world.get_physics_context()
    context.enable_gpu_dynamics(False)
    context.set_solver_type('TGS')
    context.set_broadphase_type('MBP')
    context.enable_ccd(True)
    return world
