"""Whole-object contact physics for scene initialization; uncalibrated estimates."""

MASS_KG = 0.025
FRICTION = (.45, .35)


def author(geometry_path, output_path):
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
    material = UsdShade.Material.Define(stage, '/Asset/Materials/plastic')
    contact = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    contact.CreateStaticFrictionAttr(FRICTION[0])
    contact.CreateDynamicFrictionAttr(FRICTION[1])
    contact.CreateRestitutionAttr(0.)
    for prim in stage.Traverse():
        role = prim.GetCustomDataByKey('table1000:role')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG)
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rigid.CreateSolverPositionIterationCountAttr(16)
            rigid.CreateSolverVelocityIterationCountAttr(4)
            rigid.CreateEnableCCDAttr(True)
        elif role == 'part':
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(.0001)
            collision.CreateRestOffsetAttr(0.)
    layer.Save()
