"""Three-quarter thumbnail of a URDF robot: visual meshes left, collision right
(one colour per rigid body).

``python -m table_1000.robots.preview asset_sources/robots/x1pro`` runs Blender
in the background on this file and writes ``three_quarter.jpg`` (1920 x 960)
next to the URDF, posed by ``--joints`` (JSON, missing joints at zero).
This file is also the script Blender runs, so its top level imports only what
Blender's Python has.
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

PANEL = 960
VIEW = (1., -.75, .55)  # camera direction from the robot, robot frame


def origin(element):
    from scipy.spatial.transform import Rotation
    transform = np.eye(4)
    node = element.find('origin') if element is not None else None
    if node is not None:
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
    return transform


def link_poses(urdf, joints):
    """World pose of every link for joint values ``joints`` (missing = 0)."""
    from scipy.spatial.transform import Rotation
    root = ET.parse(urdf).getroot()
    children = {}
    for joint in root.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    names = {link.get('name') for link in root.findall('link')}
    base = (names - {joint.find('child').get('link') for joint in root.findall('joint')}).pop()
    poses, stack = {base: np.eye(4)}, [base]
    while stack:
        parent = stack.pop()
        for joint in children.get(parent, []):
            motion = np.eye(4)
            value = joints.get(joint.get('name'), 0.)
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ') if joint.find('axis') is not None else np.zeros(3)
            if joint.get('type') in ('revolute', 'continuous'):
                motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
            elif joint.get('type') == 'prismatic':
                motion[:3, 3] = axis * value
            child = joint.find('child').get('link')
            poses[child] = poses[parent] @ origin(joint) @ motion
            stack.append(child)
    return root, poses


def parts(urdf, joints, scratch=None):
    """``{'visual': [(mesh path, 4x4, link)], 'collision': [...]}`` in the robot frame.
    Box, cylinder and sphere collisions are written as meshes to ``scratch``."""
    root, poses = link_poses(urdf, joints)
    result = {'visual': [], 'collision': []}
    count = 0
    for link in root.findall('link'):
        for kind in result:
            for element in link.findall(kind):
                mesh = element.find('geometry/mesh')
                if mesh is not None:
                    path = (Path(urdf).parent / mesh.get('filename')).resolve()
                    result[kind].append((str(path), (poses[link.get('name')] @ origin(element)).tolist(),
                                         link.get('name')))
                elif scratch is not None and element.find('geometry') is not None:
                    from table_1000.robots.primitives import element_mesh
                    primitive = element_mesh(element)
                    if primitive is None:
                        continue
                    path = Path(scratch) / f'primitive_{count}.ply'
                    count += 1
                    primitive.export(path)
                    result[kind].append((str(path), poses[link.get('name')].tolist(), link.get('name')))
    return result


def render(spec):
    """Blender side: import meshes and render ``spec['panels']`` (by default visual and collision, whole robot)."""
    import bpy
    from mathutils import Matrix, Vector
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    import colorsys
    groups, materials = {}, {}

    def material(name, colour):
        if name not in materials:
            materials[name] = bpy.data.materials.new(name)
            materials[name].use_nodes = True
            materials[name].node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = colour
        return materials[name]

    # Collision bodies get hues spaced by the golden angle, so neighbours differ.
    bodies = list(dict.fromkeys(link for _, _, link in spec['parts']['collision']))
    hues = {link: (i * .618034) % 1. for i, link in enumerate(bodies)}
    for kind, entries in spec['parts'].items():
        groups[kind] = []
        for path, transform, link in entries:
            (bpy.ops.wm.stl_import if path.endswith('.stl') else bpy.ops.wm.ply_import)(filepath=path)
            obj = bpy.context.selected_objects[0]
            obj.matrix_world = Matrix(transform)
            if kind == 'visual':
                obj.data.materials.append(material('visual', (.78, .78, .8, 1.)))
            else:
                obj.data.materials.append(material(link, (*colorsys.hsv_to_rgb(hues[link], .85, .8), 1.)))
            groups[kind].append(obj)
    bpy.context.view_layer.update()
    camera = bpy.data.objects.new('camera', bpy.data.cameras.new('camera'))
    camera.data.type = 'ORTHO'
    scene.collection.objects.link(camera)
    scene.camera = camera
    lights = []
    for name, offset, power in (('key', (1, -1.5, 2), 120), ('fill', (-1.5, -.5, 1), 50), ('rim', (0, 1.5, 1.5), 60)):
        light = bpy.data.objects.new(name, bpy.data.lights.new(name, 'AREA'))
        scene.collection.objects.link(light)
        lights.append((light, Vector(offset), power))
    scene.world = bpy.data.worlds.new('world')
    scene.world.use_nodes = True
    scene.world.node_tree.nodes['Background'].inputs['Color'].default_value = (.35, .35, .35, 1)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 16
    scene.cycles.use_denoising = True
    scene.render.resolution_x = scene.render.resolution_y = PANEL
    scene.render.image_settings.file_format = 'PNG'
    links = {kind: [link for _, _, link in entries] for kind, entries in spec['parts'].items()}
    default = [{'kind': kind, 'path': spec['output'] + f'.{kind}.png'} for kind in ('visual', 'collision')]
    panels = []
    for panel in spec.get('panels') or default:
        # Framed on the visual meshes of the panel's links (all links by default).
        focus = set(panel.get('focus') or links['visual'])
        framed = [obj for obj, link in zip(groups['visual'], links['visual']) if link in focus]
        points = [obj.matrix_world @ Vector(corner) for obj in framed for corner in obj.bound_box]
        low = Vector([min(p[i] for p in points) for i in range(3)])
        high = Vector([max(p[i] for p in points) for i in range(3)])
        center, span = (low + high) / 2, max(high - low)
        direction = Vector(panel.get('view') or VIEW).normalized()
        right = Vector((0, 0, 1)).cross(direction).normalized()
        up = direction.cross(right)
        projected = [((p - center).dot(right), (p - center).dot(up)) for p in points]
        camera.data.ortho_scale = 1.1 * max(max(abs(x) for x, _ in projected), max(abs(y) for _, y in projected)) * 2
        camera.data.clip_end = span * 40
        # The camera stays outside the whole robot so near links do not cut the view.
        camera.matrix_world = (Matrix.Translation(center + direction * max(span * 3, 4.))
                               @ Matrix((right, up, direction)).transposed().to_4x4())
        for light, offset, power in lights:
            light.data.energy = power * span * span
            light.data.size = span * 1.5
            light.location = center + offset * span
            light.rotation_euler = (-offset).to_track_quat('-Z', 'Y').to_euler()
        # ``isolate`` hides every link outside the focus (close-ups of one arm or hand).
        for name, objects in groups.items():
            for obj, link in zip(objects, links[name]):
                obj.hide_render = name != panel['kind'] or (panel.get('isolate') and link not in focus)
        scene.render.filepath = panel['path']
        bpy.ops.render.render(write_still=True)
        panels.append(scene.render.filepath)
    print('PANELS', json.dumps(panels), flush=True)


def render_panels(urdf, joints, panels):
    """Render ``[{'kind': 'visual' | 'collision', 'path', 'focus': [links] | None, 'isolate': bool, 'view': xyz}]``
    with Blender."""
    import tempfile
    output = Path(panels[0]['path']).with_suffix('')
    with tempfile.TemporaryDirectory() as scratch:
        spec = {'parts': parts(urdf, joints, scratch), 'output': str(output), 'panels': panels}
        spec_path = output.with_suffix('.preview.json')
        spec_path.write_text(json.dumps(spec))
        try:
            subprocess.run(['blender', '-b', '--python-exit-code', '1', '-P', __file__, '--', str(spec_path)],
                           check=True, stdout=subprocess.DEVNULL)
        finally:
            spec_path.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot', type=Path, help='robot source directory with robot.urdf')
    parser.add_argument('--joints', type=json.loads, default=None, help='JSON joint values')
    args = parser.parse_args(argv)
    from PIL import Image
    urdf = (args.robot / 'robot.urdf').resolve()
    joints = args.joints or {}
    output = args.robot.resolve() / 'three_quarter.jpg'
    spec = {'parts': parts(urdf, joints), 'output': str(output.with_suffix(''))}
    spec_path = output.with_suffix('.preview.json')
    spec_path.write_text(json.dumps(spec))
    try:
        subprocess.run(['blender', '-b', '--python-exit-code', '1', '-P', __file__, '--', str(spec_path)], check=True,
                       stdout=subprocess.DEVNULL)
        panels = [Image.open(f'{spec["output"]}.{kind}.png').convert('RGB') for kind in ('visual', 'collision')]
        pair = Image.new('RGB', (2 * PANEL, PANEL))
        for i, panel in enumerate(panels):
            pair.paste(panel, (i * PANEL, 0))
        pair.save(output, quality=80, optimize=True)
    finally:
        spec_path.unlink(missing_ok=True)
        for kind in ('visual', 'collision'):
            Path(f'{spec["output"]}.{kind}.png').unlink(missing_ok=True)
    print(output, output.stat().st_size)


if __name__ == '__main__':
    if '--' in sys.argv:
        render(json.loads(Path(sys.argv[sys.argv.index('--') + 1]).read_text()))
    else:
        main()
