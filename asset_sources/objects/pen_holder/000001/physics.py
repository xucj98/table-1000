"""Empty moulded-plastic holder: uncalibrated engineering mass/contact.

Approx. 149.35 cm³ shell at 1050 kg/m³, rounded to 160 g.
COM and inertia are estimated from the existing hollow collision geometry.
"""
MASS_KG = .160


def author(geometry_path, output_path):
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade
    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
    material = UsdShade.Material.Define(stage, '/Asset/Materials/plastic')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.65)
    api.CreateDynamicFrictionAttr(.50)
    api.CreateRestitutionAttr(0)
    api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    api.CreateFrictionCombineModeAttr('average')
    api.CreateRestitutionCombineModeAttr('average')
    for prim in stage.Traverse():
        role = prim.GetCustomDataByKey('table1000:role')
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
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(.0001)
            api.CreateRestOffsetAttr(0)
    layer.Save()
