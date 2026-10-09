"""Regression checks for geometric rejection, joint poses and sparse keyframes.

Run inside Blender; requires --assets-root pointing to generated example assets.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import unittest
import bpy
import numpy as np
from mathutils import Matrix, Vector
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from table_1000.modeling.geometry_checks import GeometryChecks, convex_data, sat_depth
from table_1000.modeling.preview import read_views, interpolate, run_worker
args = argparse.ArgumentParser()
args.add_argument('--assets-root',type=Path,required=True)
args.add_argument('--output',type=Path,required=True)
options = args.parse_args(sys.argv[sys.argv.index('--')+1:])
options.output.mkdir(parents=True,exist_ok=True)


class AcceptanceTests(unittest.TestCase):
    def load(self):
        bpy.ops.wm.open_mainfile(filepath=str(options.assets_root/'cabinet/000000/object.blend'))
        bpy.context.scene.rigidbody_world.enabled=False
        bpy.context.view_layer.update()

    def test_sparse_merge_and_interpolation(self):
        data={'move.mp4':{'fps':24,'frames':[
            {'frame':0,'camera':[1,-1,1,0,0,1],'joints':{'a':.1}},
            {'frame':10,'joints':{'b':.2}}, {'frame':20,'joints':{'a':0}}]}}
        with tempfile.TemporaryDirectory(dir=options.output) as tmp:
            p=Path(tmp)/'views.json';p.write_text(json.dumps(data))
            _,keys=read_views(p)['move.mp4']
            self.assertEqual(keys[1]['joints'],{'a':.1,'b':.2})
            self.assertEqual(interpolate(keys,5)['joints'],{'a':.1,'b':.1})
            self.assertEqual(interpolate(keys,20)['joints'],{'a':0,'b':.2})
            data['move.mp4']['frames'][1]['frame']=0;p.write_text(json.dumps(data))
            with self.assertRaises(ValueError):read_views(p)

    def test_collision_and_filter(self):
        self.load();checks=GeometryChecks()
        self.assertFalse(checks.sample('closed',0,{}))
        first=bpy.data.objects['drawer1_open'].rigid_body_constraint.object2
        first.location.z-=.06
        bpy.context.view_layer.update()
        self.assertTrue(checks.sample('intentional_overlap',1,{}))
        checks.ignored={tuple(sorted((a.name,b.name))) for a in checks.bodies for b in checks.bodies if a!=b}
        self.assertFalse(checks.sample('explicitly_filtered',1,{}))

    def test_containment_and_touch(self):
        normals=np.eye(3);edges=np.eye(3)
        import itertools
        a=np.array(list(itertools.product([-1,1],repeat=3)),dtype=float)
        b=a*.1
        self.assertGreater(sat_depth((a,normals,edges),(b,normals,edges)),1)
        self.assertAlmostEqual(sat_depth((a,normals,edges),(a+[2,0,0],normals,edges)),0)

    def test_open_and_concave_rejected(self):
        self.load();checks=GeometryChecks();o=checks.colliders[0]
        import bmesh
        bm=bmesh.new();bm.from_mesh(o.data);bm.faces.ensure_lookup_table()
        bmesh.ops.delete(bm,geom=[bm.faces[0]],context='FACES_ONLY');bm.to_mesh(o.data);bm.free()
        with self.assertRaises(ValueError):convex_data(o)
        self.load();checks=GeometryChecks()
        o=next(o for o in checks.colliders if o.rigid_body.collision_shape=='BOX')
        o.rigid_body.collision_shape='CONVEX_HULL'
        o.data.vertices[0].co=sum((v.co for v in o.data.vertices),Vector())/len(o.data.vertices)
        o.data.update()
        with self.assertRaises(ValueError):convex_data(o)

    def test_pose_and_invalid_joint(self):
        path=options.assets_root/'cabinet/000000/object.blend'
        with tempfile.TemporaryDirectory(dir=options.output) as tmp:
            entry=lambda q:{'pose.jpg':[{'frame':0,'camera':[1,-1,1,0,0,1],'joints':q}]}
            run_worker(path,entry({'drawer1_open':.08}),Path(tmp),True)
            body=bpy.data.objects['drawer1_open'].rigid_body_constraint.object2
            self.assertAlmostEqual(body.matrix_world.translation.y,-.213,places=5)
            other=bpy.data.objects['drawer2_open'].rigid_body_constraint.object2
            self.assertAlmostEqual(other.matrix_world.translation.y,-.133,places=5)
            for q in ({'unknown':0},{'drawer1_open':1}):
                with self.assertRaises(ValueError):run_worker(path,entry(q),Path(tmp),True)

    def test_complexity_totals(self):
        self.load()
        self.assertEqual(GeometryChecks().report['complexity'], {
            'visual_triangles': 412, 'colliders': 29,
            'primitive_colliders': 21, 'convex_colliders': 8,
            'convex_vertices': 64, 'convex_faces': 48,
        })

    def test_pen_button_and_refill_share_slider(self):
        path=options.assets_root/'pen/000000/object.blend'
        camera=[.82,-.82,.72,0,0,1]
        with tempfile.TemporaryDirectory(dir=options.output) as tmp:
            for values in ({'button_press':-.003}, {'button_press':0}):
                views={'pose.jpg':[{'frame':0,'camera':camera,'joints':values}]}
                run_worker(path,views,Path(tmp),True)
                checks=GeometryChecks()
                self.assertEqual(len(checks.bodies),2)
                self.assertEqual(len(checks.report['joints']),1)
                self.assertFalse(checks.ignored)
                c=bpy.data.objects['button_press'].rigid_body_constraint
                self.assertEqual(c.type,'SLIDER')
                self.assertAlmostEqual(c.object2.matrix_world.translation.x,values['button_press'],places=6)
                self.assertAlmostEqual(c.object1.matrix_world.translation.x,0,places=6)
                for name in ('Silver push button','Refill shaft','Fine ballpoint'):
                    self.assertEqual(bpy.data.objects[name].parent,c.object2)


result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(AcceptanceTests))
if not result.wasSuccessful():raise RuntimeError('Acceptance regression failed')
