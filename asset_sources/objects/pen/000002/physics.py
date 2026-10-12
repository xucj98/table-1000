"""Author this pen's native layer from the input geometry's names and roles."""
from pathlib import Path


MASSES = {'body': 0.012, 'cap': 0.003}
CAP_FIT = {
    'static_retention_N': 2,
    'dynamic_retention_N': 1.4,
    'engagement_length_m': 0.026,
    'radial_tolerance_m': 0.002,
    'axis_cosine_min': 0.98,
    'seating_barrier_N': 3,
    'seating_width_m': 0.002,
    'radial_stiffness': 3000,
    'radial_damping': 2,
    'angular_stiffness': 0.015,
    'angular_damping': 0.0002,
}


def author(geometry_path, output_path):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

    layer = Sdf.Layer.CreateNew(str(Path(output_path).resolve()))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [layer.identifier, str(Path(geometry_path).resolve())]
    stage.SetEditTarget(layer)

    root = stage.GetPrimAtPath('/Asset')
    root.CreateAttribute('table1000:capFit:compatible', Sdf.ValueTypeNames.String, custom=True).Set('marker_18mm')
    for name, value in CAP_FIT.items():
        root.CreateAttribute('table1000:capFit:' + name, Sdf.ValueTypeNames.Double, custom=True).Set(value)

    UsdGeom.Scope.Define(stage, '/Asset/Materials')
    plastic = UsdShade.Material.Define(stage, '/Asset/Materials/plastic')
    material = UsdPhysics.MaterialAPI.Apply(plastic.GetPrim())
    material.CreateStaticFrictionAttr(0.35)
    material.CreateDynamicFrictionAttr(0.35)
    material.CreateRestitutionAttr(0)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(plastic.GetPrim())
    physx_material.CreateFrictionCombineModeAttr('average')
    physx_material.CreateRestitutionCombineModeAttr('average')

    for prim in Usd.PrimRange(root):
        name = prim.GetCustomDataByKey('table1000:name')
        role = prim.GetCustomDataByKey('table1000:role')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASSES[name])
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateLinearDampingAttr(0)
            body.CreateAngularDampingAttr(0)
            body.CreateSolverPositionIterationCountAttr(16)
            body.CreateSolverVelocityIterationCountAttr(4)
            body.CreateEnableCCDAttr(True)
        elif role == 'part':
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(plastic, materialPurpose='physics')
        elif role == 'collision':
            collider = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collider.CreateContactOffsetAttr(0.00005)
            collider.CreateRestOffsetAttr(0)

    layer.Save()
    return layer
