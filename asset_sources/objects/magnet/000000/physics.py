"""Single 8 g rigid magnet; contact parameters are engineering estimates.

The inherited mass is not a measurement. PhysX estimates COM and inertia
from the two overlapping collision proxies with a uniform mass distribution.
Magnetic forces are supplied by the companion official behavior, not friction.
"""

MASS_KG = .008
CONTACT_MATERIALS = {'magnetic_base': (.40, .30), 'plastic_cap': (.35, .25)}
CONTACT_OFFSET_M = .0001


def author(geometry_path, output_path):
    """Write a strong USD physics layer; leave the input geometry untouched."""
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
    materials = {}
    for name, (static, dynamic) in CONTACT_MATERIALS.items():
        material = UsdShade.Material.Define(stage, f'/Asset/Materials/{name}')
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0)
        physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx.CreateFrictionCombineModeAttr('average')
        physx.CreateRestitutionCombineModeAttr('average')
        materials[name] = material
    for prim in stage.Traverse():
        role = prim.GetCustomDataByKey('table1000:role')
        name = prim.GetCustomDataByKey('table1000:name')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG)
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0)
            api.CreateAngularDampingAttr(0)
            api.CreateSolverPositionIterationCountAttr(16)
            api.CreateSolverVelocityIterationCountAttr(4)
            api.CreateEnableCCDAttr(True)
        elif role == 'part':
            material = materials[name.rsplit('.', 1)[-1]]
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
            # Cook small bevels at a useful local coordinate scale. The
            # inverse Xform scale preserves every world-space input vertex;
            # only PhysX's local cooking precision changes, not the geometry.
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            mesh.GetPointsAttr().Set([Gf.Vec3f(*(float(v)*100 for v in point)) for point in points])
            UsdGeom.Xformable(prim).AddScaleOp(opSuffix='cooking').Set(Gf.Vec3f(.01))
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateHullVertexLimitAttr(128)
    stage.GetPrimAtPath('/Asset').CreateAttribute('table1000:magnet:moment', Sdf.ValueTypeNames.Double).Set(.035)
    layer.Save()
