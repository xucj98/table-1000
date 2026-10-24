"""Rigid PP basket; uncalibrated engineering mass and contact estimates.

280 g: approximately 164 g solid 3 mm floor plus 100--130 g perforated
walls/thick rim (PP density about 900 kg/m³). Filled collision perforations
bias the collision-derived COM/inertia; no calibrated mass distribution.
"""
MASS_KG = .280
CONTACT_OFFSET_M = .0001


def author(geometry_path, output_path):
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade
    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
    material = UsdShade.Material.Define(stage, '/Asset/Materials/polypropylene')
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(.40)
    api.CreateDynamicFrictionAttr(.30)
    api.CreateRestitutionAttr(0)
    mat = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    mat.CreateFrictionCombineModeAttr('average')
    mat.CreateRestitutionCombineModeAttr('average')
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
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
    layer.Save()
