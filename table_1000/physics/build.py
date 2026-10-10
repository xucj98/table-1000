"""Build a real USDZ and its portable runtime from a saved Blender object."""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

from table_1000.physics.config import physical_config


def mass_properties(model, config):
    """Uniform density over authored collision components, using trimesh."""
    import numpy as np
    import trimesh
    from scipy.spatial.transform import Rotation
    worlds = {}
    for node in model['nodes']:
        matrix = np.array(node['matrix'])
        worlds[node['name']] = worlds[node['parent']] @ matrix if node['parent'] else matrix
    result = {}
    for name, settings in config['rigid_bodies'].items():
        if 'inertia' in settings:
            result[name] = settings.copy()
            continue
        shapes = []
        for node in model['nodes']:
            if node.get('role') == 'collision' and node['body'] == name:
                shape = trimesh.Trimesh(node['vertices'], node['triangles'], process=False)
                shape.apply_transform(np.linalg.inv(worlds[name]) @ worlds[node['name']])
                shapes.append(shape)
        props = trimesh.util.concatenate(shapes).mass_properties
        values, axes = np.linalg.eigh(props.inertia * settings['mass'] / props.volume)
        if np.linalg.det(axes) < 0:
            axes[:, 0] *= -1
        xyzw = Rotation.from_matrix(axes).as_quat()
        result[name] = {'mass': settings['mass'], 'center_of_mass': props.center_mass.tolist(),
                        'inertia': {'diagonal_inertia': values.tolist(),
                                    'principal_axes': [float(xyzw[3]), *xyzw[:3].tolist()]}}
    return result


def author_usd(model, config, mass, destination, runtime):
    import numpy as np
    from pxr import Gf, PhysxSchema, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade
    stage = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, '/Asset')
    stage.SetDefaultPrim(root.GetPrim())
    if runtime:
        root.GetPrim().SetCustomDataByKey('table1000:runtime', 'runtime/manifest.json')
    materials = {}
    for name, settings in config['materials'].items():
        material = UsdShade.Material.Define(stage, '/Asset/Materials/' + Tf.MakeValidIdentifier(name))
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(settings['static_friction'])
        api.CreateDynamicFrictionAttr(settings['dynamic_friction'])
        api.CreateRestitutionAttr(settings['restitution'])
        physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx.CreateFrictionCombineModeAttr('average')
        physx.CreateRestitutionCombineModeAttr('average')
        materials[name] = material
    paths, worlds = {}, {}
    options = config.get('backends', {}).get('isaac', {})
    setters = {'linear_damping': 'CreateLinearDampingAttr', 'angular_damping': 'CreateAngularDampingAttr',
               'solver_position_iterations': 'CreateSolverPositionIterationCountAttr',
               'solver_velocity_iterations': 'CreateSolverVelocityIterationCountAttr',
               'enable_ccd': 'CreateEnableCCDAttr', 'max_depenetration_velocity': 'CreateMaxDepenetrationVelocityAttr'}
    allowed = setters.keys() | {'contact_offset', 'rest_offset'}
    if options.keys() - allowed:
        raise ValueError(f'Unsupported Isaac asset options: {sorted(options.keys() - allowed)}')
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
            api = UsdPhysics.MassAPI.Apply(prim)
            props = mass[name]
            api.CreateMassAttr(props['mass'])
            api.CreateCenterOfMassAttr(Gf.Vec3f(*props['center_of_mass']))
            api.CreateDiagonalInertiaAttr(Gf.Vec3f(*props['inertia']['diagonal_inertia']))
            q = props['inertia'].get('principal_axes', [1, 0, 0, 0])
            api.CreatePrincipalAxesAttr(Gf.Quatf(q[0], Gf.Vec3f(*q[1:])))
            physx = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            for key, setter in setters.items():
                if key in options:
                    getattr(physx, setter)(options[key])
        elif role == 'collision':
            obj.CreateVisibilityAttr('invisible')
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('boundingCube' if node['shape'] == 'BOX' else 'convexHull')
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(options.get('contact_offset', .00005))
            api.CreateRestOffsetAttr(options.get('rest_offset', 0))
            material_name = config['colliders'].get(parent, {}).get('material', config['defaults']['material'])
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[material_name], materialPurpose='physics')
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
        if 'drive' in config['joints'].get(row['name'], {}):
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear' if row['type'] == 'SLIDER' else 'angular')
            settings = config['joints'][row['name']]['drive']
            drive.CreateTypeAttr('force')
            drive.CreateTargetPositionAttr(settings['target_position'] * factor)
            drive.CreateStiffnessAttr(settings['stiffness'] / factor)
            drive.CreateDampingAttr(settings['damping'] / factor)
            drive.CreateMaxForceAttr(settings['max_force'])
    stage.GetRootLayer().Save()
    return paths


def build(asset, source, output):
    from pxr import Sdf, Usd, UsdPhysics, UsdUtils
    started = time.perf_counter()
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temporary:
        temporary = Path(temporary)
        model_path = temporary / 'model.json'
        subprocess.run(['blender', '-b', '--python-exit-code', '1', '-P', str(Path(__file__).with_name('blender_model.py')),
                        '--', '--blend', str(asset / 'object.blend'), '--output', str(model_path)], check=True)
        model = json.loads(model_path.read_text())
        config = physical_config(json.loads((source / 'physics.json').read_text()), model)
        mass = mass_properties(model, config)
        modules = {entry['entry'].split(':')[0] for entry in config.get('behaviors', {}).values()}
        paths = author_usd(model, config, mass, temporary / 'object.usdc', bool(modules))
        if not UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(temporary / 'object.usdc')), str(output / 'object.usdz')):
            raise RuntimeError('USDZ packaging failed')
        packaged = Usd.Stage.Open(str(output / 'object.usdz'))
        if not packaged or len([prim for prim in packaged.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]) != len(mass):
            raise ValueError('Packaged USDZ rigid body count differs from source')
    for name in ['physics.json', 'physics_test.json']:
        if (source / name).exists():
            shutil.copyfile(source / name, output / name)
    if modules:
        runtime = output / 'runtime'
        runtime.mkdir(exist_ok=True)
        for name in modules:
            shutil.copyfile(source / name, runtime / name)
        manifest = {'physics': '../physics.json', 'modules': {name: name for name in sorted(modules)}, 'supported_backends': ['isaac']}
        (runtime / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (output / 'model.json').write_text(json.dumps(model) + '\n')
    report = {'rigid_bodies': len(mass), 'parts': sum(node['part'] for node in model['nodes']),
              'colliders': sum(node.get('role') == 'collision' for node in model['nodes']),
              'mass_properties': mass, 'mass_calculation': 'trimesh uniform density over authored collision components',
              'material_combine': 'average', 'resolved_physics': config, 'paths': paths,
              'build_wall_seconds': time.perf_counter() - started}
    (output / 'physics_build.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PHYSICAL_ASSET_BUILT', output / 'object.usdz', flush=True)
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
