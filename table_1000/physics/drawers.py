"""Physics layer of a drawer unit (see ``table_1000.modeling.drawer_unit_assets``): an articulation
rooted at the housing, drawers sliding with joint-axis friction and a viscous drive, one contact
material for the body and another for the feet.

An asset's ``physics.py`` calls :func:`author` with its values::

    from table_1000.physics.drawers import author as author_drawers

    def author(geometry_path, output_path):
        author_drawers(geometry_path, output_path, housing_kg=1.2, drawer_kg=.25,
                       friction=(.35, .25), foot_friction=(.85, .7))

Like objects/cabinet/000000: the joint-axis friction gives a breakaway force so an open drawer stays
where it is left; the zero-target drive adds viscous resistance. Design values, not calibration.
"""

from pathlib import Path

CONTACT_OFFSET_M = .0001


def author(geometry_path, output_path, *, housing_kg, drawer_kg, friction, foot_friction,
           static_friction_n=.5, dynamic_friction_n=.35, damping_n_s_per_m=3., max_force_n=20.):
    """Write the physics layer over ``geometry_path`` into ``output_path``: ``housing_kg`` and
    ``drawer_kg`` (each drawer), ``friction`` ``(static, dynamic)`` on every collider except the
    feet (``foot_friction``), joint friction forces in N and the drive damping in N s/m."""
    from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    layer = Sdf.Layer.CreateNew(str(Path(output_path).resolve()))
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths = [layer.identifier, str(Path(geometry_path).resolve())]
    stage.SetEditTarget(layer)
    UsdGeom.Scope.Define(stage, '/Asset/Materials')

    materials = {}
    for name, (static, dynamic) in {'body': friction, 'foot': foot_friction}.items():
        material = UsdShade.Material.Define(stage, f'/Asset/Materials/{name}')
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(float(static))
        api.CreateDynamicFrictionAttr(float(dynamic))
        api.CreateRestitutionAttr(0.)
        physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        physx.CreateFrictionCombineModeAttr('average')
        physx.CreateRestitutionCombineModeAttr('average')
        materials[name] = material

    for prim in stage.Traverse():
        name = prim.GetCustomDataByKey('table1000:name') or ''
        role = prim.GetCustomDataByKey('table1000:role')
        if role == 'rigid':
            housing = name.endswith('.housing')
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(housing_kg if housing else drawer_kg))
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateLinearDampingAttr(0)
            api.CreateAngularDampingAttr(0)
            api.CreateSolverPositionIterationCountAttr(16)
            api.CreateSolverVelocityIterationCountAttr(4)
            api.CreateEnableCCDAttr(True)
            if housing:
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                articulation.CreateEnabledSelfCollisionsAttr(True)
                articulation.CreateSolverPositionIterationCountAttr(16)
                articulation.CreateSolverVelocityIterationCountAttr(4)
        elif role == 'part':
            material = materials['foot' if '.foot' in name else 'body']
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')
        elif role == 'collision':
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(CONTACT_OFFSET_M)
            api.CreateRestOffsetAttr(0)
        elif role == 'joint':
            prim.ApplyAPI('PhysxJointAxisAPI', 'linear')
            prim.GetAttribute('physxJointAxis:linear:staticFrictionEffort').Set(float(static_friction_n))
            prim.GetAttribute('physxJointAxis:linear:dynamicFrictionEffort').Set(float(dynamic_friction_n))
            prim.GetAttribute('physxJointAxis:linear:viscousFrictionCoefficient').Set(0.)
            drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(0.)
            drive.CreateDampingAttr(float(damping_n_s_per_m))
            drive.CreateTargetVelocityAttr(0.)
            drive.CreateMaxForceAttr(float(max_force_n))
    layer.Save()
    return layer
