"""Author a scene source's layout into the open Isaac Sim stage.

Run inside a started ``SimulationApp``. The table is authored from primitives;
the robot and objects reference their built USD entries unchanged, posed by a
transform on the referencing prim.
"""

from pathlib import Path

import numpy as np

ROBOT_PATH, TABLE_PATH = '/World/Robot', '/World/Table'
OBJECT_PATHS = {'cabinet': '/World/Cabinet', 'pen': '/World/Pen'}


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


def add_lights(stage):
    from pxr import Gf, UsdGeom, UsdLux
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(900)
    sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
    sun.CreateIntensityAttr(2500)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))


def populate(stage, scene, layout, assets):
    """Add the table, robot and placed objects to ``stage`` (no ground or physics scene)."""
    from pxr import UsdGeom
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    add_table(stage, scene.TABLE)
    add_reference(stage, ROBOT_PATH, asset_entry(assets, scene.ROBOT['asset']), robot_base(scene))
    for name, path in OBJECT_PATHS.items():
        add_reference(stage, path, asset_entry(assets, getattr(scene, name.upper())['asset']), layout[name])
    add_lights(stage)


def named_bodies(stage, root):
    """``{table1000:name: prim path}`` of the rigid bodies under ``root``."""
    from pxr import Usd, UsdPhysics
    return {prim.GetCustomDataByKey('table1000:name'): str(prim.GetPath())
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root))
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetCustomDataByKey('table1000:name')}
