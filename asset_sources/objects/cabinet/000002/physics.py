"""Native physics for the five-body pegboard cabinet; values are not real calibration.

Each link has an explicit total mass. PhysX estimates COM and inertia from its
current collision geometry. Contact material does not change mass distribution.
Native joint-axis friction supplies constant static/dynamic sliding resistance;
the zero-target drive adds viscous resistance. These are design values.
"""

# Actual compound hull-volume sums after the book-clearance correction:
# housing ~2.657 L after the 40 mm board extension (adds 102.4 mL);
# each drawer 180.4 mL, tray 118.6 mL, hook 2.62 mL (including the taller mounting peg).
# ABS ~1050 kg/m3, PET ~1380 kg/m3. Rounded engineering estimates;
# compound intersections overcount material slightly; no real calibration.
MASS_KG = {'pegboard_cabinet.housing': 2.76,
           'pegboard_cabinet.drawer_left': .25,
           'pegboard_cabinet.drawer_right': .25,
           'pegboard_cabinet.tray': .16, 'pegboard_cabinet.hook': .003}
CONTACT_MATERIALS = {'ABS': (.35, .25), 'PET': (.32, .23)}
# Light molded plastic sliding rails, engineering estimates; nominal gravity
# onset asin(.45/(.25*9.81)) = 10.6 deg before geometric/contact effects.
STATIC_FRICTION_N = .45
DYNAMIC_FRICTION_N = .32
DRIVE_DAMPING_N_S_PER_M = 2.5
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
            if name == 'pegboard_cabinet.housing':
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                articulation.CreateEnabledSelfCollisionsAttr(True)
                articulation.CreateSolverPositionIterationCountAttr(16)
                articulation.CreateSolverVelocityIterationCountAttr(4)
        elif role == 'part':
            material = materials['PET' if any(k in name for k in ('.drawer_', '.tray.')) else 'ABS']
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
        elif role == 'joint':
            # Native joint-axis resistance, not a measured rail coefficient.
            prim.ApplyAPI('PhysxJointAxisAPI', 'linear')
            prim.GetAttribute('physxJointAxis:linear:staticFrictionEffort').Set(STATIC_FRICTION_N)
            prim.GetAttribute('physxJointAxis:linear:dynamicFrictionEffort').Set(DYNAMIC_FRICTION_N)
            prim.GetAttribute('physxJointAxis:linear:viscousFrictionCoefficient').Set(0)
            drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(0)
            drive.CreateDampingAttr(DRIVE_DAMPING_N_S_PER_M)
            drive.CreateTargetVelocityAttr(0)
            drive.CreateMaxForceAttr(20)
    layer.Save()
