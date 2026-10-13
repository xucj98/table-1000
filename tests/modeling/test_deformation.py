"""Blender integration: live collision refresh, saved callbacks and moving hinges.

Run: blender -b --python-exit-code 1 -P tests/modeling/test_deformation.py
"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import bpy
import numpy as np
from mathutils import Matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from table_1000.modeling.asset_builders import asset, body, deformable, part, box, joint, material
from table_1000.modeling.geometry_checks import GeometryChecks
from table_1000.modeling.geometry_pose import GeometryPose
from table_1000.modeling.preview import run_worker
from table_1000.physics.blender_model import extract


CALLBACK = '''
import bpy
from mathutils import Matrix
def apply_geometry_pose(controls):
    q = controls['bag.open']
    for name in ['film.visual', 'Collision_film.visual']:
        obj = bpy.data.objects[name]
        for vertex in obj.data.vertices:
            vertex.co.x += q
        obj.data.update()
    if 'slider' in bpy.data.objects:
        bpy.data.objects['slider'].matrix_world = Matrix.Translation((q, 0, 2))
        bpy.data.objects['ring'].matrix_world = Matrix.Translation((q, 0, 3))
        bpy.data.objects['swing'].matrix_world = Matrix.Translation((q, 0, 2.5))
'''


class DeformationTests(unittest.TestCase):
    def build(self, directory, target_x=2, hardware=False):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        scene = bpy.context.scene
        scene.unit_settings.system = 'METRIC'
        scene.unit_settings.scale_length = 1
        root = asset('bag', 'test', 'test', 'test')
        owner = deformable('bag.film', root)
        surface = material('test', (.2,.4,.6))
        box('film.visual', (.2,.2,.2), (0,0,0), surface, part('bag.film.surface', owner))
        target = body('target', root)
        box('target.visual', (.2,.2,.2), (target_x,0,0), surface, part('target.shell', target))
        if hardware:
            slider, ring = body('slider', root), body('ring', root)
            box('slider.visual', (.1,.1,.1), (0,0,0), surface, part('slider.shell', slider))
            box('ring.visual', (.1,.1,.1), (0,0,0), surface, part('ring.shell', ring))
            slider.location.z, ring.location.z = 2, 3
            bpy.context.view_layer.update()
            joint('swing', 'HINGE', slider, ring, (0,0,2.5), (-1,1))
        scene.rigidbody_world.enabled = False
        scene['geometry_controls'] = json.dumps({'bag.open': {'owner':'bag.film','limits':[0,2],'unit':'m'}})
        bpy.context.view_layer.update()
        (directory/'object.py').write_text(CALLBACK)
        path = directory/'object.blend'
        bpy.ops.wm.save_as_mainfile(filepath=str(path))
        return path

    def test_dynamic_data_avoids_both_false_negative_and_false_positive(self):
        for target_x in (0,2):
            with self.subTest(target_x=target_x), tempfile.TemporaryDirectory() as tmp:
                path = self.build(Path(tmp), target_x)
                checks = GeometryChecks();pose = GeometryPose(path, checks)
                self.assertEqual(len(checks.bodies), 1)
                self.assertEqual(len(checks.deformables), 1)
                baseline = np.array([tuple(v.co) for v in bpy.data.objects['film.visual'].data.vertices])
                for q in (0,2,2,0):
                    controls=pose.apply({'bag.open':q})
                    hits=checks.sample('probe',0,{},controls=controls)
                    self.assertEqual(bool(hits), q==target_x)
                    visual=bpy.data.objects['film.visual'];proxy=bpy.data.objects['Collision_film.visual']
                    np.testing.assert_allclose([tuple(v.co) for v in visual.data.vertices],baseline+[q,0,0],atol=1e-7)
                    np.testing.assert_allclose([tuple(v.co) for v in visual.data.vertices],
                                               [tuple(v.co) for v in proxy.data.vertices],atol=1e-7)

    def test_controls_and_hinge_share_moving_anchor_and_reset(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);path=self.build(directory,target_x=10,hardware=True)
            cam=[1,-1,1,0,0,1]
            views={'moved.jpg':[{'frame':0,'camera':cam,'controls':{'bag.open':1},'joints':{'swing':.5}}],
                   'transformed.jpg':[{'frame':0,'camera':cam,'controls':{'bag.open':1},
                       'joints':{'swing':.5},'transforms':{'slider':[.5,0,0,1,0,0,0]}}],
                   'reset.jpg':[{'frame':0,'camera':cam,'joints':{}}]}
            original=GeometryChecks.sample
            def capture(checks,output,frame,values,transforms=None,controls=None):
                q=controls['bag.open']
                x=q+(transforms or {}).get('slider',[0])[0]
                np.testing.assert_allclose(bpy.data.objects['slider'].matrix_world.translation,(x,0,2),atol=1e-6)
                np.testing.assert_allclose(bpy.data.objects['swing'].matrix_world.translation,(q,0,2.5),atol=1e-6)
                expected=Matrix.Translation((x,0,2.5))@Matrix.Rotation(values.get('swing',0),4,'Z')@Matrix.Translation((0,0,.5))
                np.testing.assert_allclose(bpy.data.objects['ring'].matrix_world,expected,atol=1e-6)
                return original(checks,output,frame,values,transforms,controls)
            with patch.object(GeometryChecks,'sample',capture):
                run_worker(path,views,directory/'preview',check_only=True)
            report=json.loads((directory/'preview/acceptance.json').read_text())
            self.assertEqual(report['samples'][-1]['controls'],{'bag.open':0})
            self.assertEqual(report['status'],'passed')
            with self.assertRaisesRegex(ValueError,'no physical export'):
                extract(path)

    def test_invalid_controls_and_deformed_nonconvex_collision_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=self.build(Path(tmp));checks=GeometryChecks();pose=GeometryPose(path,checks)
            for invalid in ({'unknown':0},{'bag.open':3},{'bag.open':float('nan')}):
                with self.assertRaises(ValueError):pose.apply(invalid)
            pose.apply({})
            obj=checks.dynamic_colliders[0]
            obj.rigid_body.collision_shape='CONVEX_HULL'
            obj.data.vertices[0].co=(0,0,0);obj.data.update()
            with self.assertRaisesRegex(ValueError,'Nonconvex'):
                checks.sample('bad',0,{})
            owner=bpy.data.objects['bag.film'];owner['rigid_body_root']=True
            with self.assertRaises(ValueError):GeometryChecks()

    def test_unknown_controls_rejected_for_rigid_only_asset(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=self.build(Path(tmp));checks=GeometryChecks()
            checks.deformables=[]
            del bpy.context.scene['geometry_controls']
            pose=GeometryPose(path,checks)
            with self.assertRaisesRegex(ValueError,'Unknown geometry controls'):
                pose.apply({'bag.open':1})


result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(DeformationTests))
if not result.wasSuccessful():raise RuntimeError('Deformation regression failed')
