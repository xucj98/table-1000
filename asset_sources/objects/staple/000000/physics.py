"""Physical properties for the complete 24/6 staple strip.

The strip is deliberately one rigid body.  The mass is an engineering estimate
from 50 galvanized-steel round wires (0.51 mm diameter and roughly 24.7 mm of
wire per staple); it is not a measured value.  The seven native box
colliders preserve the U opening while omitting sub-pitch grooves.
"""

MASS_KG = 0.0020
CONTACT_MATERIAL = (0.35, 0.25)
CONTACT_OFFSET_M = 0.00025


def author(geometry_path, output_path):
    """Write the USD physics layer without changing the geometry layer."""
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)

    material = UsdShade.Material.Define(stage, '/Asset/Materials/galvanized_steel')
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr(CONTACT_MATERIAL[0])
    material_api.CreateDynamicFrictionAttr(CONTACT_MATERIAL[1])
    material_api.CreateRestitutionAttr(0.05)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr('average')
    physx_material.CreateRestitutionCombineModeAttr('average')

    for prim in stage.Traverse():
        role = prim.GetCustomDataByKey('table1000:role')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG)
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rigid.CreateLinearDampingAttr(0.0)
            rigid.CreateAngularDampingAttr(0.0)
            rigid.CreateSolverPositionIterationCountAttr(16)
            rigid.CreateSolverVelocityIterationCountAttr(4)
            rigid.CreateEnableCCDAttr(True)
            rigid.CreateEnableSpeculativeCCDAttr(False)
        elif role == 'part':
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            # Preserve the Blender BOX bounds as an analytic native shape;
            # mesh boundingCube cooking distorts these sub-millimetre sections.
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            low = [min(p[i] for p in points) for i in range(3)]
            high = [max(p[i] for p in points) for i in range(3)]
            prim.SetTypeName('Cube')
            cube = UsdGeom.Cube(prim)
            cube.CreateSizeAttr(1)
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*[(a+b)/2 for a,b in zip(low,high)]))
            xform.AddScaleOp().Set(Gf.Vec3f(*[b-a for a,b in zip(low,high)]))
            prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            collision.CreateRestOffsetAttr(0.0)

    layer.Save()
