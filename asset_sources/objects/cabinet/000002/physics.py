"""Native physics for the three-drawer cabinet; values are not real calibration.

Each link has an explicit total mass. PhysX estimates COM and inertia from its
unchanged collision geometry. Contact material does not change mass distribution.
Articulation friction is load dependent; the zero-target drive adds viscous
resistance, which must not be described as static friction.
"""

MASS_KG = {'cabinet.housing': .75, **{f'cabinet.drawer{i}': .16 for i in (1, 2, 3)}}
CONTACT_MATERIALS = {'plastic': (.30, .22), 'rubber': (.85, .70)}
JOINT_FRICTION = .25
DRIVE_DAMPING_N_S_PER_M = 3
CONTACT_OFFSET_M = .0001


def author(geometry_path, output_path):
    """Write one independent native physics layer over exported geometry."""
    from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
    UsdGeom.Scope.Define(stage, '/Asset/Materials')

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
        name = prim.GetCustomDataByKey('table1000:name')
        role = prim.GetCustomDataByKey('table1000:role')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG[name])
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0)
            api.CreateAngularDampingAttr(0)
            api.CreateSolverPositionIterationCountAttr(16)
            api.CreateSolverVelocityIterationCountAttr(4)
            api.CreateEnableCCDAttr(True)
            if name == 'cabinet.housing':
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                articulation.CreateEnabledSelfCollisionsAttr(True)
                articulation.CreateSolverPositionIterationCountAttr(16)
                articulation.CreateSolverVelocityIterationCountAttr(4)
        elif role == 'part':
            material = materials['rubber' if '.rubber_pad' in name else 'plastic']
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
        elif role == 'joint':
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateJointFrictionAttr(JOINT_FRICTION)
            drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(0)
            drive.CreateDampingAttr(DRIVE_DAMPING_N_S_PER_M)
            drive.CreateTargetVelocityAttr(0)
            drive.CreateMaxForceAttr(20)
    layer.Save()
