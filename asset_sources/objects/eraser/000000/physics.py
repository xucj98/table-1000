"""Fixed paper sleeve and rubber, one 14 g rigid body; uncalibrated estimates.

Rubber volume ~8.41 cm3 at 1.60 g/cm3 plus ~0.484 cm3 of sleeve at
0.9 g/cm3 gives ~13.9 g, rounded to 14 g. Inertia/COM use the collision
volume estimate, not a measured paper/rubber density distribution.
"""

MASS_KG = .014
CONTACT_MATERIALS = {'rubber': (.85, .70), 'paper': (.45, .35)}
CONTACT_OFFSET_M = .00005


def author(geometry_path, output_path):
    """Author a strong physics layer, leaving input geometry unchanged."""
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
            api.CreateEnableSpeculativeCCDAttr(True)
        elif role == 'part':
            material = materials['rubber' if name.endswith('.rubber') else 'paper']
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
    layer.Save()
