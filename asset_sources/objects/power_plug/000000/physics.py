"""Rigid power-plug mass and part-specific contact materials.

Mass and friction values are uncalibrated engineering estimates. The single
rigid body uses collision-volume-derived center of mass and inertia.
"""

MASS_KG = 0.065
CONTACT_MATERIALS = {
    "shell": (0.55, 0.42),
    "left_pin": (0.32, 0.24),
    "right_pin": (0.32, 0.24),
    "top_pin": (0.32, 0.24),
}
CONTACT_OFFSET_M = 0.0001


def author(geometry_path, output_path):
    """Author a physics layer over the unchanged exported geometry."""
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
        elif role == "part":
            part_name = name.rsplit(".", 1)[-1]
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                materials[part_name], materialPurpose="physics"
            )
        elif role == "collision":
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            collision.CreateRestOffsetAttr(0.0)

    layer.Save()
