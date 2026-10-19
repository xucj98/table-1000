"""Uncalibrated clear-polymer shell mass at 1180 kg/m³; SI units.

Mass uses existing 24-sided floor/wall and lid geometry, labels neglected.
COM/inertia from hollow collision shapes. Native unlatched contact only.
"""
MASSES = {'storage_box.container': 0.015172}


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
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASSES[prim.GetCustomDataByKey('table1000:name')])
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
