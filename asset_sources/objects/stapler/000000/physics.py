"""Native two-body stapler physics; values are engineering estimates.

The upper assembly remains one rigid body.  The approved return approximation
is an angular force drive on the existing hinge; it is not a calibrated leaf
spring or a model of stapling force.
"""

MASS_KG = {'stapler.base': .035, 'stapler.upper': .025}
CONTACT_MATERIALS = {'plastic': (.40, .30), 'steel': (.25, .18)}
PLASTIC_PARTS = {
    'stapler.base.shell', 'stapler.base.hinge_cheek1',
    'stapler.base.hinge_cheek2', 'stapler.upper.cover',
    'stapler.upper.rear_bridge',
}
CONTACT_OFFSET_M = .00005
RETURN_STIFFNESS_N_M_PER_RAD = .40
RETURN_DAMPING_N_M_S_PER_RAD = .006
RETURN_TARGET_RAD = .05
RETURN_MAX_TORQUE_N_M = .25


def author(geometry_path, output_path):
    """Author masses and contact properties over the unchanged geometry layer."""
    from pathlib import Path
    from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(Path(output_path).resolve()))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [layer.identifier, str(Path(geometry_path).resolve())]
    stage.SetEditTarget(layer)
    UsdGeom.Scope.Define(stage, '/Asset/Materials')
    materials = {}
    for name, (static, dynamic) in CONTACT_MATERIALS.items():
        material = UsdShade.Material.Define(stage, '/Asset/Materials/' + name)
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0)
        physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx.CreateFrictionCombineModeAttr('average')
        physx.CreateRestitutionCombineModeAttr('average')
        materials[name] = material
    for prim in stage.Traverse():
        name = prim.GetCustomDataByKey('table1000:name')
        role = prim.GetCustomDataByKey('table1000:role')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG[name])
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0)
            api.CreateAngularDampingAttr(0)
            # With TGS external forces applied each iteration (a scene setting),
            # 64/4 preserves both velocity consistency and the return-speed gate.
            api.CreateSolverPositionIterationCountAttr(64)
            api.CreateSolverVelocityIterationCountAttr(4)
            api.CreateEnableCCDAttr(True)
        elif role == 'part':
            material = materials['plastic' if name in PLASTIC_PARTS else 'steel']
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
        elif role == 'joint':
            # USD rotational drive attributes use degrees while the public
            # contract and test scripts use radians and SI torque units.
            import math
            drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
            drive.CreateTypeAttr('force')
            drive.CreateTargetPositionAttr(math.degrees(RETURN_TARGET_RAD))
            drive.CreateStiffnessAttr(RETURN_STIFFNESS_N_M_PER_RAD * math.pi / 180)
            drive.CreateDampingAttr(RETURN_DAMPING_N_M_S_PER_RAD * math.pi / 180)
            drive.CreateMaxForceAttr(RETURN_MAX_TORQUE_N_M)
    layer.Save()
