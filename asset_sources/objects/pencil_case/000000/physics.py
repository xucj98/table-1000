"""Native rigid-body properties for the approved pencil-case approximation.

The first chapter deliberately models the bag shell and its two lips as five
rigid bodies.  This layer assigns masses, contact materials, and passive joint
resistance to that geometry.  The companion behavior.py implements the approved PC-01 native-limit
interlock; this layer does not add zipper teeth or cloth deformation.

All values below are engineering estimates for the 240 mm empty stationery
case, not measurements.  The collision geometry determines centre of mass and
inertia, so the mass budget should be revisited if the representation changes.
"""

import math
from pathlib import Path


MASS_KG = {
    "pencil_case.housing": 0.050,
    "pencil_case.front_lip": 0.010,
    "pencil_case.back_lip": 0.010,
    "pencil_case.slider": 0.007,
    "pencil_case.pull_ring": 0.003,
}

# (static friction, dynamic friction), estimated by material family.
CONTACT_MATERIALS = {
    "cloth": (0.55, 0.40),
    "metal": (0.30, 0.22),
}

CONTACT_OFFSET_M = 0.00005
SLIDER_STATIC_FRICTION_N = 0.35
SLIDER_DYNAMIC_FRICTION_N = 0.25
SLIDER_DAMPING_N_S_PER_M = 2.0
LIP_DAMPING_N_M_S_PER_RAD = 0.001
RING_DAMPING_N_M_S_PER_RAD = 0.00002


def _part_material(name):
    """Return the contact material for a semantic part name."""
    # The zipper carriage, attachment bridge and pull ring are the only metal
    # parts.  The string is textile and all shell/lip/lining parts are cloth.
    if any(token in name for token in ("pull_ring", "slider.carriage", "slider.attachment")):
        return "metal"
    return "cloth"


def author(geometry_path, output_path):
    """Write one independent native USD physics layer over ``geometry_path``."""
    from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    output_path = Path(output_path).resolve()
    geometry_path = Path(geometry_path).resolve()
    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(layer.identifier), str(geometry_path)]
    stage.SetEditTarget(layer)
    UsdGeom.Scope.Define(stage, "/Asset/Materials")

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
        name = prim.GetCustomDataByKey("table1000:name")
        role = prim.GetCustomDataByKey("table1000:role")
        if role == "rigid":
            if name not in MASS_KG:
                raise KeyError(f"Missing engineering mass for rigid body {name!r}")
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG[name])
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rigid.CreateLinearDampingAttr(0.0)
            rigid.CreateAngularDampingAttr(0.0)
            rigid.CreateSolverPositionIterationCountAttr(16)
            rigid.CreateSolverVelocityIterationCountAttr(4)
            rigid.CreateEnableCCDAttr(True)
            rigid.CreateSleepThresholdAttr(0.0)
            if name == "pencil_case.housing":
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                articulation.CreateEnabledSelfCollisionsAttr(True)
                articulation.CreateSolverPositionIterationCountAttr(16)
                articulation.CreateSolverVelocityIterationCountAttr(4)
        elif role == "part":
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                materials[_part_material(name)], materialPurpose="physics"
            )
        elif role == "collision":
            collider = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collider.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            collider.CreateRestOffsetAttr(0.0)
        elif role == "joint":
            if name == "pencil_case.unzip":
                # The USD linear axis is metres; resistance is expressed as an
                # effort so a low pull can hold the slider while a larger pull
                # moves it.  These are deliberately diagnostic values.
                prim.ApplyAPI("PhysxJointAxisAPI", "linear")
                prim.GetAttribute(
                    "physxJointAxis:linear:staticFrictionEffort"
                ).Set(SLIDER_STATIC_FRICTION_N)
                prim.GetAttribute(
                    "physxJointAxis:linear:dynamicFrictionEffort"
                ).Set(SLIDER_DYNAMIC_FRICTION_N)
                prim.GetAttribute(
                    "physxJointAxis:linear:viscousFrictionCoefficient"
                ).Set(0.0)
                drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(SLIDER_DAMPING_N_S_PER_M)
                drive.CreateTargetVelocityAttr(0.0)
                drive.CreateMaxForceAttr(4.0)
            else:
                # USD angular drives use degrees; convert the SI viscous gain.
                damping = (RING_DAMPING_N_M_S_PER_RAD if name.endswith('ring_swing')
                           else LIP_DAMPING_N_M_S_PER_RAD)
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(damping * math.pi / 180)
                drive.CreateTargetVelocityAttr(0.0)
                drive.CreateMaxForceAttr(0.15)

    layer.Save()
    return layer
