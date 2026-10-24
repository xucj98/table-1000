"""Single rigid board eraser; engineering estimates, not measured materials.

50 g = approximately 25 g ABS shell, 20 g base/support, 5 g felt. PhysX
estimates COM/inertia from the two solid collision proxies; the hollow shell
and low-density felt do not have a calibrated mass distribution.
"""

MASS_KG = .050
CONTACT_MATERIALS = {'felt': (.70, .55), 'shell': (.35, .25)}
CONTACT_OFFSET_M = .0001


def author(geometry_path, output_path):
    """Author a strong physics layer without changing the geometry layer."""
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
            material = materials['felt' if name.endswith('.felt_pad') else 'shell']
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
    layer.Save()
