"""Author a scene source's layout into the open Isaac Sim stage.

Run inside a started ``SimulationApp``. The table is authored from primitives;
the robot and the scene's ``OBJECTS`` (names of module-level dicts with an
``asset``) reference their built USD entries unchanged, posed by a transform on
the referencing prim. A scene may also list ``PRIMITIVES``: simple rigid shapes
for tests (``{name: {'shape': 'box' | 'sphere' | 'cylinder', 'size': ...,
'mass': kg or None}}``); one without mass is kinematic, so it stays where it is
put. Each object and primitive is posed by ``layout[name]``.
"""

from pathlib import Path

import numpy as np

from table_1000.runtime import render

ROBOT_PATH, TABLE_PATH, PRIMITIVES_PATH = '/World/Robot', '/World/Table', '/World/Primitives'


def object_paths(scene):
    """``{name: prim path}`` of the scene's asset objects and primitives."""
    paths = {name: f'/World/{name.capitalize()}' for name in getattr(scene, 'OBJECTS', ())}
    paths.update({name: f'{PRIMITIVES_PATH}/{name}' for name in getattr(scene, 'PRIMITIVES', {})})
    return paths


def asset_entry(assets, relative):
    directory = Path(assets) / relative
    for name in ('object.usda', 'object.usdz', 'robot.usda'):
        if (directory / name).is_file():
            return directory / name
    raise FileNotFoundError(f'no built USD entry in {directory}')


def set_transform(prim, transform):
    from pxr import Gf, UsdGeom
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(Gf.Matrix4d(np.asarray(transform, dtype=float).T.tolist()))


def robot_base(scene):
    transform = np.eye(4)
    c, s = np.cos(scene.ROBOT['yaw']), np.sin(scene.ROBOT['yaw'])
    transform[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    transform[:3, 3] = scene.ROBOT['position']
    return transform


def add_table(stage, table):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    root = UsdGeom.Xform.Define(stage, TABLE_PATH)
    (width, depth), height, thickness, leg = table['size'], table['height'], table['top_thickness'], table['leg_size']
    cx, cy = table['center']
    parts = {'top': ((cx, cy, height - thickness / 2), (width, depth, thickness))}
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        parts[f'leg{i + 1}'] = ((cx + sx * (width - leg) / 2, cy + sy * (depth - leg) / 2, (height - thickness) / 2),
                                (leg, leg, height - thickness))
    material = UsdShade.Material.Define(stage, TABLE_PATH + '/PhysicsMaterial')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(table['friction'][0])
    api.CreateDynamicFrictionAttr(table['friction'][1])
    api.CreateRestitutionAttr(0.)
    for name, (center, size) in parts.items():
        cube = UsdGeom.Cube.Define(stage, f'{TABLE_PATH}/{name}')
        cube.CreateSizeAttr(1.)
        cube.CreateDisplayColorAttr([Gf.Vec3f(.55, .42, .3)])
        xform = UsdGeom.Xformable(cube)
        xform.AddTranslateOp().Set(Gf.Vec3d(*center))
        xform.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim()).CreateContactOffsetAttr(.002)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material, materialPurpose='physics')
    return root


def add_reference(stage, path, entry, transform):
    prim = stage.DefinePrim(path, 'Xform')
    prim.GetReferences().AddReference(str(entry))
    set_transform(prim, transform)
    return prim


def populate(stage, scene, layout, assets, robot_transform=None):
    """Add the table, robot and placed objects to ``stage`` (no ground or physics scene).

    The robot prim is posed at ``robot_transform`` (default: the scene's base
    pose); a robot with a virtual mobile base stays at the origin and is moved
    by its base joints instead.
    """
    from pxr import UsdGeom
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    add_table(stage, scene.TABLE)
    add_reference(stage, ROBOT_PATH, asset_entry(assets, scene.ROBOT['asset']),
                  robot_base(scene) if robot_transform is None else robot_transform)
    for name in getattr(scene, 'OBJECTS', ()):
        add_reference(stage, object_paths(scene)[name], asset_entry(assets, getattr(scene, name.upper())['asset']),
                      layout[name])
    for name, spec in getattr(scene, 'PRIMITIVES', {}).items():
        add_primitive(stage, f'{PRIMITIVES_PATH}/{name}', spec, layout[name])
    render.add_lights(stage)


def add_primitive(stage, path, spec, transform):
    """A rigid shape named ``body`` (``table1000:name``), dynamic or kinematic."""
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    root = stage.DefinePrim(path, 'Xform')
    set_transform(root, transform)
    root.SetCustomDataByKey('table1000:name', 'body')
    shape, size = spec['shape'], spec['size']
    if shape == 'box':
        geometry = UsdGeom.Cube.Define(stage, path + '/geometry')
        geometry.CreateSizeAttr(1.)
        UsdGeom.Xformable(geometry).AddScaleOp().Set(Gf.Vec3f(*size))
    elif shape == 'sphere':
        geometry = UsdGeom.Sphere.Define(stage, path + '/geometry')
        geometry.CreateRadiusAttr(float(size))
    elif shape == 'cylinder':
        geometry = UsdGeom.Cylinder.Define(stage, path + '/geometry')
        geometry.CreateRadiusAttr(float(size[0]))
        geometry.CreateHeightAttr(float(size[1]))
        geometry.CreateAxisAttr('Z')
    else:
        raise ValueError(f'unknown primitive shape {shape}')
    geometry.CreateDisplayColorAttr([Gf.Vec3f(*spec.get('color', (.2, .45, .8)))])
    UsdPhysics.CollisionAPI.Apply(geometry.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(geometry.GetPrim()).CreateContactOffsetAttr(.002)
    body = UsdPhysics.RigidBodyAPI.Apply(root)
    if spec.get('mass') is None:
        body.CreateKinematicEnabledAttr(True)
    else:
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(spec['mass']))
        PhysxSchema.PhysxRigidBodyAPI.Apply(root).CreateEnableCCDAttr(True)
    material = UsdShade.Material.Define(stage, path + '/PhysicsMaterial')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction = spec.get('friction', (.6, .5))
    api.CreateStaticFrictionAttr(friction[0])
    api.CreateDynamicFrictionAttr(friction[1])
    api.CreateRestitutionAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(geometry.GetPrim()).Bind(material, materialPurpose='physics')
    return root


def named_bodies(stage, root):
    """``{table1000:name: prim path}`` of the rigid bodies under ``root``."""
    from pxr import Usd, UsdPhysics
    return {prim.GetCustomDataByKey('table1000:name'): str(prim.GetPath())
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root))
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetCustomDataByKey('table1000:name')}
