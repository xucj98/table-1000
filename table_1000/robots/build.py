"""Build a robot USD from its URDF source and a native physics layer.

``asset_sources/robots/<name>/`` holds ``robot.urdf`` with meshes referenced by
relative path, and ``physics.py`` whose ``author(import_path, output_path)``
writes a USD layer over the imported robot. The build writes, under
``assets/robots/<name>/``: ``import/`` (the Isaac URDF importer output),
``robot.usda`` (the entry to reference) and snapshots of both source files.
"""

import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import time

IMPORT_SETTINGS = {
    'distance_scale': 1., 'make_default_prim': True, 'create_physics_scene': False,
    'fix_base': True, 'merge_fixed_joints': True,
    'import_inertia_tensor': True, 'self_collision': True, 'convex_decomp': False,
    'collision_from_visuals': False, 'parse_mimic': False, 'replace_cylinders_with_capsules': False,
    'density': 0.,
}


def import_urdf(urdf, destination):
    """Convert ``urdf`` to ``destination`` with the Isaac Sim URDF importer."""
    import omni.kit.app
    import omni.kit.commands
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
        'isaacsim.asset.importer.urdf', True)
    _, config = omni.kit.commands.execute('URDFCreateImportConfig')
    for key, value in IMPORT_SETTINGS.items():
        getattr(config, 'set_' + key)(value)
    parsed, model = omni.kit.commands.execute('URDFParseFile', urdf_path=str(urdf), import_config=config)
    if not parsed:
        raise RuntimeError(f'URDF parse failed: {urdf}')
    omni.kit.commands.execute('URDFImportRobot', urdf_path=str(urdf), urdf_robot=model,
                              import_config=config, dest_path=str(destination))
    if not Path(destination).is_file():
        raise RuntimeError(f'URDF import wrote no USD: {destination}')


def author_physics(source, output):
    spec = importlib.util.spec_from_file_location('robot_physics', source / 'physics.py')
    physics = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(physics)
    physics.author(output / 'import' / 'robot.usd', output / 'robot.usda')


def build(source, output):
    started = time.perf_counter()
    shutil.rmtree(output / 'import', ignore_errors=True)
    output.mkdir(parents=True, exist_ok=True)
    import_urdf(source / 'robot.urdf', output / 'import' / 'robot.usd')
    imported = time.perf_counter()
    author_physics(source, output)
    for name in ('robot.urdf', 'physics.py'):
        shutil.copyfile(source / name, output / name)
    report = {'source': str(source), 'import_settings': IMPORT_SETTINGS,
              'import_seconds': imported - started, 'total_seconds': time.perf_counter() - started}
    (output / 'build.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robots', nargs='+', help='robot names under asset_sources/robots')
    parser.add_argument('--source', type=Path, default=Path('asset_sources/robots'))
    parser.add_argument('--output', type=Path, default=Path('assets/robots'))
    parser.add_argument('--gpu', type=int, default=0)
    args = parser.parse_args(argv)
    from table_1000.runtime.app import start_app
    app = start_app(args.output.resolve(), gpu=args.gpu, workers=4)
    # Kit owns stdout/stderr and app.close() may end the process, so failures
    # are written next to the output before closing.
    try:
        for name in args.robots:
            print(json.dumps(build((args.source / name).resolve(), (args.output / name).resolve())), flush=True)
    except Exception:
        import traceback
        (args.output / 'build-error.txt').write_text(traceback.format_exc())
        raise
    finally:
        app.close()
