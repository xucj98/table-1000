"""Planning URDF/SRDF derivation and planning-world objects from asset models.

Needs ``mplib`` (0.2.1) and ``trimesh`` in the running Python environment.
"""

from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from table_1000.robots.planning_model import write_planning_urdf, write_srdf
from table_1000.planning.world import PlanningScene, asset_objects, model_shapes


def robot(directory):
    """base -(revolute j1)- l1 -(fixed)- frame ; l1 -(prismatic j2)- l2 ; l2 -(revolute j3)- l3."""
    mesh = directory / 'meshes' / 'box.ply'
    mesh.parent.mkdir()
    trimesh.creation.box((.1, .1, .1)).export(mesh)
    collision = '<collision><geometry><mesh filename="meshes/box.ply"/></geometry></collision>'
    links = ''.join(f'<link name="{name}">{collision if name != "frame" else ""}</link>'
                    for name in ('base', 'l1', 'frame', 'l2', 'l3'))
    joints = ('<joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 .2"/>'
              '<axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>'
              '<joint name="f" type="fixed"><parent link="l1"/><child link="frame"/><origin xyz=".1 0 0"/></joint>'
              '<joint name="j2" type="prismatic"><parent link="frame"/><child link="l2"/><origin xyz="0 0 .2"/>'
              '<axis xyz="0 0 1"/><limit lower="0" upper=".5" effort="1" velocity="1"/></joint>'
              '<joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz=".3 0 0"/>'
              '<axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>')
    path = directory / 'robot.urdf'
    path.write_text(f'<robot name="r">{links}{joints}</robot>')
    return path


class PlanningModelTest(unittest.TestCase):
    def test_frozen_joints_become_fixed_at_their_value(self):
        with tempfile.TemporaryDirectory() as temporary:
            temporary = Path(temporary)
            source = robot(temporary / 'source' if (temporary / 'source').mkdir() is None else None)
            urdf = write_planning_urdf(source, temporary / 'plan' / 'robot.urdf', ['j1', 'j3'], {'j2': .25},
                                       ('l3', 'tcp', (.1, 0, 0), (0, 0, 0)))
            root = ET.parse(urdf).getroot()
            joints = {joint.get('name'): joint for joint in root.findall('joint')}
            self.assertEqual(joints['j2'].get('type'), 'fixed')
            np.testing.assert_allclose(np.fromstring(joints['j2'].find('origin').get('xyz'), sep=' '), [0, 0, .45])
            self.assertEqual(joints['j3'].get('type'), 'revolute')
            self.assertIn('tcp', {link.get('name') for link in root.findall('link')})
            self.assertTrue((temporary / 'plan' / 'meshes' / 'box.ply.convex.stl').is_file())
            with self.assertRaises(ValueError):
                write_planning_urdf(source, temporary / 'x' / 'robot.urdf', ['missing'], {}, ('l3', 't', (0, 0, 0), (0, 0, 0)))

    def test_srdf_disables_adjacent_bodies_through_empty_links(self):
        with tempfile.TemporaryDirectory() as temporary:
            temporary = Path(temporary)
            (temporary / 'source').mkdir()
            urdf = robot(temporary / 'source')
            pairs = write_srdf(urdf, temporary / 'robot.srdf', [('base', 'l3')])
            self.assertEqual(pairs, [('base', 'l1'), ('base', 'l3'), ('l1', 'l2'), ('l2', 'l3')])


MODEL = {'nodes': [
    {'name': 'box', 'parent': None, 'matrix': np.eye(4).tolist(), 'rigid': True},
    {'name': 'box.lid', 'parent': 'box', 'matrix': np.eye(4).tolist(), 'rigid': True},
    {'name': 'box.wall', 'parent': 'box', 'matrix': np.eye(4).tolist(), 'rigid': False},
    {'name': 'box.wall.collision', 'parent': 'box.wall', 'role': 'collision', 'shape': 'BOX', 'rigid': False,
     'matrix': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [.2, 0, 0, 1]],
     'vertices': (trimesh.creation.box((.1, .1, .1)).vertices).tolist()},
    {'name': 'box.lid.knob', 'parent': 'box.lid', 'rigid': False, 'matrix': np.eye(4).tolist()},
    {'name': 'box.lid.knob.collision', 'parent': 'box.lid.knob', 'role': 'collision', 'shape': 'SPHERE',
     'rigid': False, 'radius': .08, 'matrix': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, .3, 1]]},
]}
for node in MODEL['nodes']:
    node['matrix'] = np.asarray(node['matrix'], dtype=float).T.tolist() if node['name'].endswith('collision') else node['matrix']


class PlanningWorldTest(unittest.TestCase):
    def test_shapes_are_placed_relative_to_their_body(self):
        shapes, rest = model_shapes(MODEL)
        self.assertEqual(sorted(rest), ['box', 'box.lid'])
        by_part = {part: (body, transform, kind) for body, part, transform, kind, _ in shapes}
        self.assertEqual(by_part['box.wall'][0], 'box')
        np.testing.assert_allclose(by_part['box.wall'][1][:3, 3], [.2, 0, 0])
        self.assertEqual(by_part['box.lid.knob'][2], 'sphere')

    def test_merged_object_is_one_hull_covering_all_shapes(self):
        (merged,) = asset_objects(MODEL, '', lambda body, part: ('all', 'box'), merged=lambda name: True)
        ((transform, kind, points),) = merged.raw
        self.assertEqual(kind, 'convex')
        np.testing.assert_allclose(points.max(0), [.25, .08, .38])
        np.testing.assert_allclose(points.min(0), [-.08, -.08, -.05])

    def test_padding_grows_every_side(self):
        (wall,) = [o for o in asset_objects(MODEL, '', lambda body, part: (part, body)) if o.name == 'box.wall']
        wall.fcl(np.eye(4))
        wall.fcl(np.eye(4), .01)
        bare, padded = (np.asarray(wall.cache[p][0][1].get_vertices()) for p in (0., .01))
        np.testing.assert_allclose(padded.max(0), bare.max(0) + .01)
        np.testing.assert_allclose(padded.min(0), bare.min(0) - .01)

    def test_grouping_and_permissions_survive_pose_updates(self):
        import mplib
        objects = asset_objects(MODEL, 'a:', lambda body, part: (part, body))
        self.assertEqual(sorted(o.name for o in objects), ['a:box.lid.knob', 'a:box.wall'])
        with tempfile.TemporaryDirectory() as temporary:
            (Path(temporary) / 'source').mkdir()
            urdf = robot(Path(temporary) / 'source')
            write_srdf(urdf, Path(temporary) / 'source' / 'robot.srdf')
            planner = mplib.Planner(str(urdf), 'l3', srdf=str(Path(temporary) / 'source' / 'robot.srdf'))
            scene = PlanningScene(planner)
            far = np.eye(4)
            far[:3, 3] = [5, 5, 5]
            for obj in objects:
                scene.add(obj, far)
            near = np.eye(4)
            near[:3, 3] = [.4, 0, .1]
            scene.update({'a:box.lid': near})
            # The knob sphere moved into l3 (at 0.4, 0, 0.4); the wall stayed away.
            self.assertTrue(planner.planning_world.check_collision())
            scene.allow(['a:box.lid.knob'], ['l2', 'l3'])
            scene.update({'a:box.lid': near})
            self.assertFalse(planner.planning_world.check_collision())


if __name__ == '__main__':
    unittest.main()
