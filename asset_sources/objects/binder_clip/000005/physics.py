"""Native preloaded hinge configuration for the 22 mm binder clip.

Approved native approximation: passive preloaded jaw torsion springs, free
handle hinges with their original 0-degree inward stops, and a free-root
articulation. Four directly connected pairs are explicitly filtered: the two
jaw/handle pairs and the two back/jaw clearance seams. All other self contacts
remain enabled. Functional acceptance is still pending; see README.

Mass: steel density 7800 kg/m^3 times current collision solid volume, with
hexagonal wire/octagonal lip sections corrected to circular sections. Paint
mass is omitted, sheet/lip overlap retained. Values are engineering estimates.
PhysX estimates COM/inertia from collision shapes, including polygon error.
"""

MASS_KG = {
    "binder_clip.back": 0.0010570560,
    "binder_clip.front_handle": 0.0006837938,
    "binder_clip.front_jaw": 0.0019827771,
    "binder_clip.rear_handle": 0.0006837938,
    "binder_clip.rear_jaw": 0.0019827771,
}
CONTACT_MATERIALS = {"paint": (0.35, 0.25), "wire": (0.30, 0.20)}
CONTACT_OFFSET_M = 0.0001
JAW_STIFFNESS_NM_RAD = 0.08
JAW_DAMPING_NMS_RAD = 0.0008
JAW_REST_DEGREES = -5


def author(geometry_path, output_path):
    """Author native mass, contacts, articulation and passive jaw springs."""
    import math

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
        api.CreateRestitutionAttr(0)
        physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx.CreateFrictionCombineModeAttr("average")
        physx.CreateRestitutionCombineModeAttr("average")
        materials[name] = material

    for prim in stage.Traverse():
        name = prim.GetCustomDataByKey("table1000:name")
        role = prim.GetCustomDataByKey("table1000:role")
        if role == "rigid":
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG[name])
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0)
            api.CreateAngularDampingAttr(0)
            api.CreateSolverPositionIterationCountAttr(16)
            api.CreateSolverVelocityIterationCountAttr(4)
            api.CreateEnableCCDAttr(True)
            if name == "binder_clip.back":
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                articulation.CreateEnabledSelfCollisionsAttr(True)
                articulation.CreateSolverPositionIterationCountAttr(16)
                articulation.CreateSolverVelocityIterationCountAttr(4)
        elif role == "part":
            surface = "wire" if name.endswith(".wire_loop") else "paint"
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                materials[surface], materialPurpose="physics"
            )
        elif role == "collision":
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
        elif role == "joint":
            joint = UsdPhysics.Joint(prim)
            joint.CreateCollisionEnabledAttr(False)
            a, b = (
                joint.GetBody0Rel().GetTargets()[0],
                joint.GetBody1Rel().GetTargets()[0],
            )
            UsdPhysics.FilteredPairsAPI.Apply(
                stage.GetPrimAtPath(a)
            ).CreateFilteredPairsRel().AddTarget(b)
            if name.endswith("_jaw_open"):
                # USD angular drive uses degrees; engine tensors report radians.
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(JAW_STIFFNESS_NM_RAD * math.pi / 180)
                drive.CreateDampingAttr(JAW_DAMPING_NMS_RAD * math.pi / 180)
                drive.CreateTargetPositionAttr(JAW_REST_DEGREES)
                drive.CreateTargetVelocityAttr(0)
                drive.CreateMaxForceAttr(1)
    layer.Save()
