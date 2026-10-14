"""Build a robot USD from its URDF source and a native physics layer.

``asset_sources/robots/<name>/`` holds ``robot.urdf`` with meshes referenced by
relative path, and ``physics.py`` whose ``author(import_path, output_path)``
writes a USD layer over the imported robot. The build writes, under
``assets/robots/<name>/``: ``import/`` (the Isaac URDF importer output),
``robot.usda`` (the entry to reference) and snapshots of both source files.

A robot whose spec (``table_1000.robots.<name>.SPEC``) has a mobile base is
imported from ``mobile.urdf``: the source with three virtual joints (world x,
world y, yaw) between a fixed root and ``base_link``. The simulator moves the
base through them; the robot prim itself stays at the world origin.
"""

import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

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


# Virtual base links: small, so they barely change the robot's inertia.
VIRTUAL_MASS, VIRTUAL_INERTIA = .5, 1e-3


def mobile_urdf(source, output, joints, max_force=(1e6, 1e6), base='base_link'):
    """Write ``output``: ``source`` with ``mobile_root -x-> -y-> -yaw-> base``.

    ``max_force`` limits the x and y drives (N) and the yaw drive (N m).

    Mesh paths become absolute, so the file can live outside the source.
    """
    root = ET.parse(source).getroot()
    for mesh in root.iter('mesh'):
        mesh.set('filename', str((Path(source).parent / mesh.get('filename')).resolve()))
    names = ['mobile_root', 'base_x_link', 'base_y_link']
    for name in names:
        link = ET.Element('link', name=name)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', value=str(VIRTUAL_MASS))
        ET.SubElement(inertial, 'inertia', ixx=str(VIRTUAL_INERTIA), iyy=str(VIRTUAL_INERTIA),
                      izz=str(VIRTUAL_INERTIA), ixy='0', ixz='0', iyz='0')
        root.insert(0, link)
    children = names[1:] + [base]
    efforts = (max_force[0], max_force[0], max_force[1])
    for (name, kind, axis), parent, child, effort in zip(
            zip(joints, ('prismatic', 'prismatic', 'continuous'), ('1 0 0', '0 1 0', '0 0 1')), names, children,
            efforts):
        joint = ET.SubElement(root, 'joint', name=name, type=kind)
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(joint, 'axis', xyz=axis)
        limit = {'effort': str(effort), 'velocity': '10'}
        if kind == 'prismatic':
            limit.update(lower='-100', upper='100')
        ET.SubElement(joint, 'limit', **limit)
    ET.ElementTree(root).write(output, encoding='unicode')
    return output


def author_physics(source, output):
    spec = importlib.util.spec_from_file_location('robot_physics', source / 'physics.py')
    physics = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(physics)
    physics.author(output / 'import' / 'robot.usd', output / 'robot.usda')


def build(source, output):
    started = time.perf_counter()
    shutil.rmtree(output / 'import', ignore_errors=True)
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.robots import source_module
    module = source_module(source, 'robot')
    spec = getattr(module, 'SPEC', None)
    urdf = source / 'robot.urdf'
    if spec is not None and spec.base is not None:
        urdf = mobile_urdf(urdf, output / 'mobile.urdf', spec.base.virtual_joints, spec.base.max_force)
    import_urdf(urdf, output / 'import' / 'robot.usd')
    imported = time.perf_counter()
    author_physics(source, output)
    for name in ('robot.urdf', 'physics.py', 'robot.py'):
        if (source / name).exists():
            shutil.copyfile(source / name, output / name)
    report = {'source': str(source), 'urdf': str(urdf), 'import_settings': IMPORT_SETTINGS,
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
    from table_1000.sim.isaac.app import start_app
    app = start_app(gpu=args.gpu, workers=4)
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
