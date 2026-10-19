"""Rigid USB drive; uncalibrated engineering mass and contact materials.

12 g candidate: 6 g housing, 4 g PCB/packages, 2 g connector. COM/inertia
are estimated by PhysX from the uniform collision volume, not those component
masses. The connector is fixed and the solid proxy does not model insertion.
"""

MASS_KG = .012
CONTACT_MATERIALS = {'housing': (.40, .30), 'connector': (.25, .18)}
CONTACT_OFFSET_M = .0001


def author(geometry_path, output_path):
    """Author a strong USD physics layer over unchanged geometry."""
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

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
    layer.Save()
