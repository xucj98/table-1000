"""The build entry uses the author hook and preserves its source snapshot."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

from table_1000.physics.build import author_geometry, author_physics, build, main


class PhysicsAuthorTest(unittest.TestCase):
    def test_source_generates_layer_and_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory) / 'source', Path(directory) / 'asset'
            source.mkdir()
            output.mkdir()
            script = "def author(geometry_path, output_path):\n    output_path.write_text(geometry_path.read_text() + ':physics')\n"
            (source / 'physics.py').write_text(script)
            (source / 'physics.usda').write_text('unused old layer')
            (output / 'geometry.usdc').write_text('geometry')
            author_physics(source, output)
            self.assertEqual((output / 'physics.usda').read_text(), 'geometry:physics')
            self.assertEqual((output / 'physics.py').read_bytes(), (source / 'physics.py').read_bytes())


class PhysicsBatchTest(unittest.TestCase):
    def test_discovery_and_selection_share_one_application(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory) / 'source', Path(directory) / 'assets'
            for reference in ['cabinet/000000', 'pen/000002']:
                path = source / reference
                path.mkdir(parents=True)
                (path / 'physics.py').write_text('')
            for selected in [None, ['pen/000002', 'cabinet/000000']]:
                arguments = ['--source', str(source), '--output', str(output), '--gpu', '3']
                references = ['cabinet/000000', 'pen/000002']
                if selected:
                    arguments += ['--assets', *selected]
                    references = selected
                app = Mock()
                simulation_app = Mock(return_value=app)
                with self.subTest(selected=selected), patch.dict(sys.modules, {'isaacsim': SimpleNamespace(SimulationApp=simulation_app)}), \
                        patch('table_1000.physics.build.build') as build:
                    main(arguments)
                    self.assertEqual(build.call_args_list, [call(output/ref, source/ref, output/ref) for ref in references])
                    simulation_app.assert_called_once()
                    self.assertEqual(simulation_app.call_args.args[0]['active_gpu'], 3)
                    app.close.assert_called_once_with(wait_for_replicator=False)


class VisualMaterialTest(unittest.TestCase):
    def test_transmission_is_independent_of_cutout_alpha(self):
        try:
            from pxr import Usd, UsdShade
        except ImportError:
            self.skipTest('requires the Isaac/OpenUSD Python environment')
        surfaces = {
            'smoke': {'transmission': .73, 'metallic': 0., 'alpha': 1.},
            'white': {'transmission': 0., 'metallic': 0., 'alpha': 1.},
            'metal': {'transmission': 0., 'metallic': 1., 'alpha': 1.},
            'cutout': {'transmission': 0., 'metallic': 0., 'alpha': .4},
        }
        nodes = [{'name': name, 'parent': None, 'matrix': [[int(i == j) for j in range(4)] for i in range(4)],
                  'rigid': False, 'part': False, 'role': 'visual',
                  'vertices': [(0, 0, 0), (1, 0, 0), (0, 1, 0)], 'faces': [(0, 1, 2)],
                  'normals': [(0, 0, 1)] * 3,
                  'material': {'color': (.5, .55, .6), 'roughness': .22, 'ior': 1.46, **surface}}
                 for name, surface in surfaces.items()]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'geometry.usda'
            author_geometry({'nodes': nodes, 'joints': []}, path)
            stage = Usd.Stage.Open(str(path))
            for name, surface in surfaces.items():
                material, _ = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath('/Asset/' + name)).ComputeBoundMaterial()
                preview = material.ComputeSurfaceSource()[0]
                self.assertAlmostEqual(preview.GetInput('opacity').Get(), surface['alpha'])
                self.assertAlmostEqual(preview.GetInput('metallic').Get(), surface['metallic'])
                self.assertAlmostEqual(preview.GetInput('ior').Get(), 1.46)
                if surface['transmission']:
                    mdl = material.ComputeSurfaceSource('mdl')[0]
                    self.assertEqual(mdl.GetSourceAssetSubIdentifier('mdl'), 'OmniSurface')
                    self.assertAlmostEqual(mdl.GetInput('specular_transmission_weight').Get(), .73)
                    self.assertAlmostEqual(mdl.GetInput('specular_reflection_ior').Get(), 1.46)
                    for actual, base in zip(mdl.GetInput('specular_transmission_color').Get(), (.5, .55, .6)):
                        self.assertAlmostEqual(actual, base ** .5)
                    self.assertFalse(mdl.GetInput('enable_opacity').Get())
                else:
                    self.assertFalse(material.GetSurfaceOutput('mdl'))


class StageMetadataTest(unittest.TestCase):
    def test_packaged_and_behavior_entries_use_metres_and_z_up(self):
        try:
            from omni.kit.material.library import get_mdl_lib_paths  # noqa: F401
            from pxr import Gf, Usd, UsdGeom
        except ImportError:
            self.skipTest('requires a running Isaac/OpenUSD Python environment')
        identity = [[int(i == j) for j in range(4)] for i in range(4)]
        body = {'name': 'body', 'parent': None, 'matrix': identity, 'rigid': True, 'part': False}
        mesh = {'parent': 'body', 'matrix': identity, 'rigid': False, 'part': False,
                'vertices': [(0, 0, 0), (.02, 0, 0), (0, .03, 0), (0, 0, .05)],
                'faces': [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)], 'shape': 'CONVEX_HULL'}
        model = {'nodes': [body, dict(mesh, name='body.collision', role='collision')], 'joints': []}
        author = '''from pxr import Sdf, Usd, UsdPhysics, UsdShade
def author(geometry_path, output_path):
    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [layer.identifier, str(geometry_path)]
    stage.SetEditTarget(layer)
    UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath('/Asset/body')).CreateMassAttr(.012)
    material = UsdShade.Material.Define(stage, '/Asset/ContactMaterial')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateStaticFrictionAttr(.4)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/Asset/body/collision')).Bind(material, materialPurpose='physics')
    layer.Save()
'''
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'source'
            source.mkdir()
            (source / 'physics.py').write_text(author)
            for behavior in [False, True]:
                with self.subTest(behavior=behavior):
                    if behavior:
                        (source / 'behavior.py').write_text('# metadata regression fixture\n')
                    output = Path(directory) / str(behavior)

                    def extract(command, **kwargs):
                        # Only Blender extraction is replaced; compose, flatten,
                        # package, reopen and reference use real USD operations.
                        Path(command[-1]).write_text(json.dumps(model))

                    with patch('table_1000.physics.build.subprocess.run', side_effect=extract):
                        build(output, source, output)
                    entries = ['object.usdz', 'object.usda'] if behavior else ['object.usdz']
                    for entry in entries:
                        stage = Usd.Stage.Open(str(output / entry))
                        self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.)
                        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)
                        world = Usd.Stage.CreateInMemory()
                        UsdGeom.SetStageMetersPerUnit(world, 1.)
                        UsdGeom.SetStageUpAxis(world, UsdGeom.Tokens.z)
                        item = UsdGeom.Xform.Define(world, '/World/Item')
                        item.GetPrim().GetReferences().AddReference(str(output / entry))
                        item.AddTranslateOp().Set(Gf.Vec3d(.1, .2, .3))
                        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'], False, True).ComputeWorldBound(item.GetPrim()).ComputeAlignedRange()
                        for actual, expected in zip(bounds.GetMin(), (.1, .2, .3)):
                            self.assertAlmostEqual(actual, expected, places=7)
                        for actual, expected in zip(bounds.GetMax(), (.12, .23, .35)):
                            self.assertAlmostEqual(actual, expected, places=7)


if __name__ == '__main__':
    unittest.main()
