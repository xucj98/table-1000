"""Single rigid screwdriver with an explicit, uncalibrated composite mass model.

Coaxial cylinder estimates include an embedded steel tang; they do not add
geometry or joints. All lengths are metres, masses kg, inertias kg m^2.
"""

# name, mass, axial center, length, radius. Axis is X, common YZ center (0,.011).
MASS_COMPONENTS = (
    ('molded_grip', .026, -.0275, .095, .009),
    ('neck', .002, .024, .012, .0065),
    ('exposed_steel', .0055, .051, .048, .0022),
    ('embedded_tang', .0045, .009, .036, .0022),
)
MASS_KG = sum(c[1] for c in MASS_COMPONENTS)
CONTACT_MATERIALS = {'grip': (.55, .40), 'neck': (.45, .32), 'shaft': (.30, .22)}


def mass_properties():
    """Return the composite COM and diagonal inertia in the rigid local frame."""
    x = sum(m * x for _, m, x, _, _ in MASS_COMPONENTS) / MASS_KG
    axial = sum(m * r*r / 2 for _, m, _, _, r in MASS_COMPONENTS)
    transverse = sum(m * ((3*r*r + length*length)/12 + (cx-x)**2)
                     for _, m, cx, length, r in MASS_COMPONENTS)
    return (x, 0., .011), (axial, transverse, transverse)


def author(geometry_path, output_path):
    """Write only the strong physics layer, leaving the input geometry intact."""
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade
    layer = Sdf.Layer.CreateNew(str(output_path))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [str(output_path), str(geometry_path)]
    stage.SetEditTarget(layer)
    materials = {}
    for name, (static, dynamic) in CONTACT_MATERIALS.items():
        material = UsdShade.Material.Define(stage, '/Asset/Materials/' + name)
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.)
        px = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        px.CreateFrictionCombineModeAttr('average')
        px.CreateRestitutionCombineModeAttr('average')
        materials[name] = material
    center, inertia = mass_properties()
    for prim in stage.Traverse():
        role = prim.GetCustomDataByKey('table1000:role')
        if role == 'rigid':
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.CreateMassAttr(MASS_KG)
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*center))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
            mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateLinearDampingAttr(0.)
            body.CreateAngularDampingAttr(0.)
            body.CreateSolverPositionIterationCountAttr(16)
            body.CreateSolverVelocityIterationCountAttr(4)
            body.CreateEnableCCDAttr(True)
        elif role == 'part':
            name = prim.GetCustomDataByKey('table1000:name').rsplit('.', 1)[-1]
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[name], materialPurpose='physics')
        elif role == 'collision':
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(.0001)
            collision.CreateRestOffsetAttr(0.)
    layer.Save()
