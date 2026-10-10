"""Export Blender geometry and compose a native USD physics layer."""

import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time


def author_physics(source, output):
    spec = importlib.util.spec_from_file_location('asset_physics', source / 'physics.py')
    physics = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(physics)
    physics.author(output / 'geometry.usdc', output / 'physics.usda')
    shutil.copyfile(source / 'physics.py', output / 'physics.py')


def author_geometry(model, destination):
    import numpy as np
    from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade
    stage = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, '/Asset')
    stage.SetDefaultPrim(root.GetPrim())
    paths, worlds = {}, {}
    for node in model['nodes']:
        name, parent = node['name'], node['parent']
        local_name = name[len(parent) + 1:] if parent and name.startswith(parent + '.') else name
        path = paths.get(parent, '/Asset') + '/' + Tf.MakeValidIdentifier(local_name)
        if path in paths.values():
            raise ValueError(f'USD name collision: {name}')
        paths[name] = path
        matrix = np.array(node['matrix'])
        worlds[name] = worlds[parent] @ matrix if parent else matrix
        role = node.get('role')
        if role:
            obj = UsdGeom.Mesh.Define(stage, path)
            obj.CreatePointsAttr([Gf.Vec3f(*value) for value in node['vertices']])
            obj.CreateFaceVertexCountsAttr([len(face) for face in node['faces']])
            obj.CreateFaceVertexIndicesAttr([index for face in node['faces'] for index in face])
            obj.CreateSubdivisionSchemeAttr('none')
        else:
            obj = UsdGeom.Xform.Define(stage, path)
        prim = obj.GetPrim()
        prim.SetCustomDataByKey('table1000:name', name)
        prim.SetCustomDataByKey('table1000:role', 'rigid' if node['rigid'] else 'part' if node['part'] else role or 'organization')
        UsdGeom.Xformable(prim).AddTransformOp().Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()))
        if node['rigid']:
            UsdPhysics.RigidBodyAPI.Apply(prim).CreateRigidBodyEnabledAttr(True)
        elif role == 'collision':
            obj.CreateVisibilityAttr('invisible')
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('boundingCube' if node['shape'] == 'BOX' else 'convexHull')
        elif role == 'visual':
            obj.CreateNormalsAttr([Gf.Vec3f(*value) for value in node['normals']])
            obj.SetNormalsInterpolation('faceVarying')
            settings = node['material']
            obj.CreateDisplayColorAttr([Gf.Vec3f(*settings['color'])])
            material = UsdShade.Material.Define(stage, path + '/Material')
            shader = UsdShade.Shader.Define(stage, path + '/Material/Shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            for key, field, kind in [('diffuseColor', 'color', Sdf.ValueTypeNames.Color3f),
                                     ('roughness', 'roughness', Sdf.ValueTypeNames.Float),
                                     ('metallic', 'metallic', Sdf.ValueTypeNames.Float)]:
                value = Gf.Vec3f(*settings[field]) if field == 'color' else settings[field]
                shader.CreateInput(key, kind).Set(value)
            shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(1 - settings['transmission'])
            shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    for row in model['joints']:
        path = '/Asset/Joints/' + Tf.MakeValidIdentifier(row['name'])
        cls = UsdPhysics.PrismaticJoint if row['type'] == 'SLIDER' else UsdPhysics.RevoluteJoint
        joint = cls.Define(stage, path)
        joint.GetPrim().SetCustomDataByKey('table1000:name', row['name'])
        joint.GetPrim().SetCustomDataByKey('table1000:role', 'joint')
        joint.CreateBody0Rel().SetTargets([paths[row['body1']]])
        joint.CreateBody1Rel().SetTargets([paths[row['body2']]])
        joint.CreateAxisAttr('X' if row['type'] == 'SLIDER' else 'Z')
        for index, body in enumerate([row['body1'], row['body2']]):
            local = np.linalg.inv(worlds[body]) @ np.array(row['matrix'])
            transform = Gf.Transform(Gf.Matrix4d(*local.T.reshape(-1).tolist()))
            q = transform.GetRotation().GetQuat()
            getattr(joint, f'CreateLocalPos{index}Attr')(Gf.Vec3f(*transform.GetTranslation()))
            getattr(joint, f'CreateLocalRot{index}Attr')(Gf.Quatf(q.GetReal(), Gf.Vec3f(*q.GetImaginary())))
        factor = 1 if row['type'] == 'SLIDER' else 180 / np.pi
        if row['limits']:
            joint.CreateLowerLimitAttr(row['limits'][0] * factor)
            joint.CreateUpperLimitAttr(row['limits'][1] * factor)
        joint.CreateCollisionEnabledAttr(not row['disable_collisions'])
    stage.GetRootLayer().Save()
    return paths


def build(asset, source, output):
    from pxr import Sdf, Usd, UsdPhysics, UsdShade, UsdUtils
    started = time.perf_counter()
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temporary:
        temporary = Path(temporary)
        model_path = temporary / 'model.json'
        subprocess.run(['blender', '-b', '--python-exit-code', '1', '-P', str(Path(__file__).with_name('blender_model.py')),
                        '--', '--blend', str(asset / 'object.blend'), '--output', str(model_path)], check=True)
        model = json.loads(model_path.read_text())
        paths = author_geometry(model, output / 'geometry.usdc')
        author_physics(source, output)
        composed = Usd.Stage.CreateInMemory()
        composed.GetRootLayer().subLayerPaths = [str(output / 'physics.usda'), str(output / 'geometry.usdc')]
        composed.SetDefaultPrim(composed.GetPrimAtPath('/Asset'))
        scripts = [Sdf.AssetPath('behavior.py')] if (source / 'behavior.py').exists() else []
        # Standard USDZ contains the physical asset. Executable scripts remain
        # beside it, with their official component carried by an outer layer.
        flattened = composed.Flatten()
        packaged_stage = Usd.Stage.Open(flattened)
        root = packaged_stage.GetPrimAtPath('/Asset')
        root.RemoveProperty('omni:scripting:scripts')
        root.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit([name for name in root.GetAppliedSchemas() if name != 'OmniScriptingAPI']))
        flattened.Export(str(temporary / 'object.usdc'))
        if not UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(temporary / 'object.usdc')), str(output / 'object.usdz')):
            raise RuntimeError('USDZ packaging failed')
        carrier = Usd.Stage.CreateNew(str(output / 'object.usda'))
        from pxr import UsdGeom
        root = UsdGeom.Xform.Define(carrier, '/Asset').GetPrim()
        root.GetReferences().AddReference('object.usdz')
        carrier.SetDefaultPrim(root)
        if scripts:
            root.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit(['OmniScriptingAPI']))
            root.CreateAttribute('omni:scripting:scripts', Sdf.ValueTypeNames.AssetArray).Set([Sdf.AssetPath(path.path) for path in scripts])
            for path in scripts:
                shutil.copyfile(source / path.path, output / path.path)
        carrier.GetRootLayer().Save()
        packaged = Usd.Stage.Open(str(output / 'object.usdz'))
        rigid = [prim for prim in packaged.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        assert len(rigid) == sum(node['rigid'] for node in model['nodes'])
        assert all(UsdPhysics.MassAPI(prim).GetMassAttr().Get() > 0 for prim in rigid)
        materials = {}
        for prim in packaged.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                material, rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
                assert material and material.GetPrim().HasAPI(UsdPhysics.MaterialAPI)
                materials[str(prim.GetPath())] = str(material.GetPath())
    if (source / 'physics_test.py').exists():
        shutil.copyfile(source / 'physics_test.py', output / 'physics_test.py')
    (output / 'model.json').write_text(json.dumps(model) + '\n')
    report = {'rigid_bodies': len(rigid), 'parts': sum(node['part'] for node in model['nodes']),
              'colliders': len(materials), 'physics_materials': materials, 'paths': paths,
              'geometry_layer': 'geometry.usdc', 'physics_source': 'physics.py', 'physics_layer': 'physics.usda',
              'entry': 'object.usda' if scripts else 'object.usdz',
              'behavior_scripts': [path.path for path in scripts],
              'build_wall_seconds': time.perf_counter() - started}
    (output / 'physics_build.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PHYSICAL_ASSET_BUILT', output / report['entry'], flush=True)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('asset', type=Path, help='Generated asset directory containing object.blend')
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--gpu', type=int, default=0)
    args = parser.parse_args(argv)
    source = args.source or Path('asset_sources/objects').joinpath(*args.asset.parts[-2:])
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'active_gpu': args.gpu, 'physics_gpu': args.gpu,
                         'multi_gpu': False, 'create_new_stage': False,
                         'disable_viewport_updates': True, 'limit_cpu_threads': 4})
    try:
        build(args.asset.resolve(), source.resolve(), (args.output or args.asset).resolve())
    finally:
        app.close()
