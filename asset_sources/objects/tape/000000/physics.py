"""Rigid tape roll physics; engineering estimates for the complete roll."""

# Winding ~39.7 cm3 at 1.2 g/cm3 + core ~3.18 cm3 at 0.7 g/cm3.
# Rounded ~49.9 g; inertia still uses uniform collision-volume density.
MASS_KG = .050
CONTACT_MATERIALS = {
    "winding": (0.38, 0.28),
    "core": (0.45, 0.32),
}
CONTACT_OFFSET_M = 0.0001


def author(geometry_path, output_path):
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
    materials = {}
    for name, (static, dynamic) in CONTACT_MATERIALS.items():
        material = UsdShade.Material.Define(stage, f"/Asset/Materials/{name}")
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx.CreateFrictionCombineModeAttr("average")
        physx.CreateRestitutionCombineModeAttr("average")
        materials[name] = material
    for prim in stage.Traverse():
        role = prim.GetCustomDataByKey("table1000:role")
        name = prim.GetCustomDataByKey("table1000:name")
        if role == "rigid":
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG)
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rigid.CreateLinearDampingAttr(0.0)
            rigid.CreateAngularDampingAttr(0.0)
            rigid.CreateSolverPositionIterationCountAttr(16)
            rigid.CreateSolverVelocityIterationCountAttr(4)
            rigid.CreateEnableCCDAttr(True)
            rigid.CreateEnableSpeculativeCCDAttr(True)
        elif role == "part":
            material = materials["core" if name.endswith(".core") else "winding"]
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
        elif role == "collision":
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            collision.CreateRestOffsetAttr(0.0)
    layer.Save()
