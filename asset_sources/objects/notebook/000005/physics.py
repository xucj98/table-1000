"""Closed 320 g notebook, native single rigid body; engineering estimates.

Page block .143*.204*.0096 m3 at 800 kg/m3 plus two covers
.148*.210*.0024 m3 at 1100 kg/m3 gives 306.1 g; rounded to 320 g
for the fixed spine, ribbon and glue.
No measured page count or material calibration. COM/inertia follow collision
volume rather than material density weighting.
"""

MASS_KG = .320
CONTACT_MATERIALS = {'cover': (.55, .40), 'paper': (.45, .30), 'fabric': (.65, .50)}
CONTACT_OFFSET_M = .00005


def author(geometry_path, output_path):
    """Write physics overrides using the input geometry as the weak layer."""
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
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
        role = prim.GetCustomDataByKey('table1000:role')
        name = prim.GetCustomDataByKey('table1000:name')
        if role == 'rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG)
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0)
            api.CreateAngularDampingAttr(0)
            api.CreateSolverPositionIterationCountAttr(16)
            api.CreateSolverVelocityIterationCountAttr(4)
            api.CreateEnableCCDAttr(True)
            api.CreateEnableSpeculativeCCDAttr(True)
        elif role == 'part':
            part = name.rsplit('.', 1)[-1]
            kind = 'paper' if part in ('pages', 'page_edge_detail') else (
                'fabric' if part in ('strap', 'bookmark', 'side_loop') else 'cover')
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[kind], materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
    layer.Save()
