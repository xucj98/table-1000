"""Start Isaac Sim through Isaac Lab's ``AppLauncher``, without touching the shared Kit user config.

Kit reads ``/persistent/...`` settings from a ``user.config.json`` shared by
every run of the installation and may write changed ones back on exit, so one
run could silently change later ones. Here each run points Kit at an empty
config in a temporary directory (removed when the app closes), requests the
PhysX worker count on the command line and reads it back. Isaac's online assets
(the ground plane) load from the local mirror ``assets/isaac`` when it exists. Shader and texture caches stay
shared: they only speed up start-up.

The process sees only the requested GPU (``CUDA_VISIBLE_DEVICES``). CUDA uses
logical device 0, while the Vulkan renderer uses the physical index.
``isaaclab.sim``, ``isaaclab.envs``
and the rest import only after :func:`start_app`.
"""

import os
from pathlib import Path
import shutil
import tempfile

PHYSICS_WORKERS = '/persistent/physics/numThreads'
# Isaac's cloud asset root (Isaac Lab's ISAAC_NUCLEUS_DIR is its Isaac/), mirrored by
# scripts/assets/download_isaac_assets.py; used when present, so runs need no network.
ISAAC_ASSET_ROOT = '/persistent/isaac/asset_root/cloud'
ISAAC_ASSETS = Path(__file__).resolve().parents[3] / 'assets' / 'isaac'
# Extensions beyond Isaac Lab's experience: the RTX lens-distortion schemas of robot cameras.
EXTENSIONS = ('omni.usd.schema.omni_lens_distortion',)


def start_app(*, gpu, workers=4, device='cpu', cameras=True, extra_args=()):
    """Return a headless ``SimulationApp`` on physical GPU ``gpu`` with ``workers`` PhysX threads.

    ``device`` is the physics device (``'cpu'`` or ``'cuda'``); ``cameras`` enables
    offscreen rendering for camera sensors. A worker count or config path that
    differs from the request raises before any stage is built.
    """
    visible = os.environ.get('CUDA_VISIBLE_DEVICES')
    if visible not in (None, str(gpu)):
        raise RuntimeError(f'CUDA_VISIBLE_DEVICES={visible} conflicts with gpu={gpu}')
    os.environ['CUDA_VISIBLE_DEVICES'] = str(gpu)
    from table_1000.common.path import configure_isaac_paths
    configure_isaac_paths()
    from isaaclab.app import AppLauncher

    portable = Path(tempfile.mkdtemp(prefix='kit-'))
    user_config = portable / 'user.config.json'
    # Vulkan renderer indices remain physical when CUDA_VISIBLE_DEVICES masks
    # CUDA. AppLauncher otherwise uses logical device zero for both APIs.
    kit_args = [f'--portable-root={portable}', f'--/app/userConfigPath={user_config}',
                f'--/renderer/activeGpu={gpu}',
                f'--{PHYSICS_WORKERS}={int(workers)}', *(arg for name in EXTENSIONS for arg in ('--enable', name)),
                *extra_args]
    if ISAAC_ASSETS.is_dir():
        kit_args.append(f'--{ISAAC_ASSET_ROOT}={ISAAC_ASSETS}')
    launcher = AppLauncher(headless=True, enable_cameras=cameras, multi_gpu=False,
                           device='cpu' if device == 'cpu' else 'cuda:0',
                           kit_args=' '.join(kit_args))
    app = launcher.app
    # Batch runs own their simulation lifecycle, including repeated contexts.
    # Lab's interactive STOP callback otherwise waits forever for a user to
    # press Play, even during construction of a subsequent context.
    import omni.timeline
    from isaaclab.sim import SimulationContext

    def disable_interactive_stop(event):
        context = SimulationContext.instance()
        if context is not None:
            context._disable_app_control_on_stop_handle = True

    app._batch_stop_subscription = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop_by_type(
        int(omni.timeline.TimelineEventType.STOP), disable_interactive_stop, order=0)
    # app.close() ends the process (with Kit's return code, 0 unless posted)
    # before atexit handlers run, so the (empty) directory goes first. Isaac
    # Lab's simulation context renders until the timeline plays again once it
    # stops (for an interactive window); closing stops the timeline, so that
    # handler is turned off first.
    close = app.close

    def close_and_clean(*args, exit_code=0, **kwargs):
        import omni.kit.app
        from isaaclab.sim import SimulationContext
        context = SimulationContext.instance()
        if context is not None:
            context._disable_app_control_on_stop_handle = True
        shutil.rmtree(portable, ignore_errors=True)
        if exit_code:
            omni.kit.app.get_app().post_quit(exit_code)
        return close(*args, **kwargs)

    app.close = close_and_clean
    import carb
    settings = carb.settings.get_settings()
    readback = {'physics_workers': settings.get(PHYSICS_WORKERS), 'user_config_path': settings.get('/app/userConfigPath')}
    if readback['physics_workers'] != int(workers) or Path(readback['user_config_path'] or '') != user_config:
        app.close(exit_code=1)
        raise RuntimeError(f'Kit settings differ from the request ({workers} workers, {user_config}): {readback}')
    return app
