"""Pen-only mass, contact and native drive authoring; no simulator startup."""
from pathlib import Path

MATERIALS = {'plastic': (.35, .30), 'metal': (.20, .15), 'rubber': (.70, .55)}
CONTACT_OFFSET_M = .0001


def author(geometry_path, output_path, *, masses, metal_parts=(), rubber_parts=(), detent=None, articulation=False, click=None):
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
    layer = Sdf.Layer.CreateNew(str(Path(output_path).resolve()))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [layer.identifier, str(Path(geometry_path).resolve())]
    stage.SetEditTarget(layer)
    materials = {}
    for name, (static, dynamic) in MATERIALS.items():
        material = UsdShade.Material.Define(stage, '/Asset/Materials/' + name)
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0)
        px = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        px.CreateFrictionCombineModeAttr('average')
        px.CreateRestitutionCombineModeAttr('average')
        materials[name] = material
    masses = dict(masses)
    if detent:
        masses.update({f'cap_ball{i}': detent['mass'] for i in range(1, detent['balls']+1)})
    for prim in stage.Traverse():
        role = prim.GetCustomDataByKey('table1000:role')
        name = prim.GetCustomDataByKey('table1000:name')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(masses[name])
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0)
            api.CreateAngularDampingAttr(0)
            api.CreateSolverPositionIterationCountAttr(32 if detent else 16)
            api.CreateSolverVelocityIterationCountAttr(8 if detent else 4)
            api.CreateEnableCCDAttr(True)
            api.CreateSleepThresholdAttr(0)
            if detent:
                api.CreateStabilizationThresholdAttr(0)
                api.CreateMaxDepenetrationVelocityAttr(1)
            if articulation and name == 'pen.shell':
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                root = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                root.CreateEnabledSelfCollisionsAttr(True)
                root.CreateSolverPositionIterationCountAttr(16)
                root.CreateSolverVelocityIterationCountAttr(4)
        elif role == 'part':
            kind = 'metal' if name in metal_parts or name.startswith('cap_ball') else 'rubber' if name in rubber_parts else 'plastic'
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[kind], materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
            if detent and prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
                if approximation == 'convexHull':
                    # PhysX's absolute cooking tolerance removes sub-mm ridge
                    # features in metre-sized local coordinates. Cook a larger
                    # local mesh and compensate with native shape scale. World
                    # geometry is unchanged; no simulator-wide setting needed.
                    mesh = UsdGeom.Mesh(prim)
                    mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(p)*100 for p in mesh.GetPointsAttr().Get()]))
                    UsdGeom.Xformable(prim).AddScaleOp(opSuffix='penCooking').Set(Gf.Vec3f(.01))
        elif role == 'joint' and detent:
            drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
            drive.CreateTypeAttr('force')
            drive.CreateTargetPositionAttr(-detent['preload'])
            drive.CreateStiffnessAttr(detent['stiffness'])
            drive.CreateDampingAttr(detent['damping'])
        elif role == 'joint' and click:
            values = click['button' if name == 'button_press' else 'refill']
            drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
            drive.CreateTypeAttr('force')
            drive.CreateTargetPositionAttr(0)
            drive.CreateTargetVelocityAttr(0)
            drive.CreateStiffnessAttr(values['stiffness'])
            drive.CreateDampingAttr(values['damping'])
            drive.CreateMaxForceAttr(values['max_force'])
    if click:
        root = stage.GetPrimAtPath('/Asset')
        root.CreateAttribute('table1000:click:initialExtended', Sdf.ValueTypeNames.Bool).Set(False)
    layer.Save()
    return layer
