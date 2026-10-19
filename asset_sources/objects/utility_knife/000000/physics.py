"""Native 3-body press slider; proposed engineering parameters, uncalibrated."""
MASS_KG={'utility_knife.body':.016,'utility_knife.carriage':.003,'utility_knife.button':.001}


def author(geometry_path,output_path):
    from pxr import Sdf,Usd,UsdPhysics,UsdShade,PhysxSchema,UsdGeom,Gf
    layer=Sdf.Layer.CreateNew(str(output_path));stage=Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths=[str(output_path),str(geometry_path)];stage.SetEditTarget(layer)
    mats={}
    for name,static,dynamic in [('plastic',.35,.25),('steel',.20,.15)]:
        m=UsdShade.Material.Define(stage,'/Asset/Materials/'+name)
        api=UsdPhysics.MaterialAPI.Apply(m.GetPrim());api.CreateStaticFrictionAttr(static);api.CreateDynamicFrictionAttr(dynamic);api.CreateRestitutionAttr(0)
        p=PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim());p.CreateFrictionCombineModeAttr('average');p.CreateRestitutionCombineModeAttr('average')
        mats[name]=m
    for prim in stage.Traverse():
        name=prim.GetCustomDataByKey('table1000:name');role=prim.GetCustomDataByKey('table1000:role')
        if role=='rigid':
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_KG[name])
            p=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            p.CreateEnableCCDAttr(True);p.CreateSolverPositionIterationCountAttr(16);p.CreateSolverVelocityIterationCountAttr(4)
            p.CreateLinearDampingAttr(0);p.CreateAngularDampingAttr(0)
            if name=='utility_knife.body':
                UsdPhysics.ArticulationRootAPI.Apply(prim)
                a=PhysxSchema.PhysxArticulationAPI.Apply(prim);a.CreateEnabledSelfCollisionsAttr(True);a.CreateSolverPositionIterationCountAttr(16);a.CreateSolverVelocityIterationCountAttr(4)
        elif role=='part':
            steel=any(name.endswith('.'+k) for k in ['blade','latch','rack'])
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mats['steel' if steel else 'plastic'],materialPurpose='physics')
        elif role=='collision':
            # Exact authored BOX extents, expressed as native USD cubes. Mesh
            # boundingCube cooking blocked the sub-mm latch in this version.
            mesh_api=UsdPhysics.MeshCollisionAPI(prim)
            if mesh_api and mesh_api.GetApproximationAttr().Get()=='boundingCube':
                points=UsdGeom.Mesh(prim).GetPointsAttr().Get()
                lo=[min(float(v[i]) for v in points) for i in range(3)]
                hi=[max(float(v[i]) for v in points) for i in range(3)]
                matrix=UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Get()
                cube=UsdGeom.Cube.Define(stage,prim.GetPath());cube.CreateSizeAttr(1)
                prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
                transform=UsdGeom.Xformable(cube);transform.ClearXformOpOrder()
                scale=Gf.Matrix4d().SetScale(Gf.Vec3d(*[b-a for a,b in zip(lo,hi)]))
                center=Gf.Matrix4d().SetTranslate(Gf.Vec3d(*[(a+b)/2 for a,b in zip(lo,hi)]))
                transform.AddTransformOp().Set(scale*center*matrix)
            p=PhysxSchema.PhysxCollisionAPI.Apply(prim);p.CreateContactOffsetAttr(.00005);p.CreateRestOffsetAttr(0)
        elif role=='joint':
            # Articulation self-collision governs PhysX pairs; this per-joint
            # flag is ignored by the engine for articulations. A--C latch/rack
            # contact is checked directly; no explicit pair filters are authored.
            UsdPhysics.Joint(prim).CreateCollisionEnabledAttr(True)
            if name=='utility_knife.press':
                d=UsdPhysics.DriveAPI.Apply(prim,'linear');d.CreateTypeAttr('force')
                d.CreateStiffnessAttr(500);d.CreateDampingAttr(.05);d.CreateTargetPositionAttr(-.0004);d.CreateTargetVelocityAttr(0);d.CreateMaxForceAttr(2)
            else:
                prim.ApplyAPI('PhysxJointAxisAPI','linear')
                prim.GetAttribute('physxJointAxis:linear:staticFrictionEffort').Set(.05)
                prim.GetAttribute('physxJointAxis:linear:dynamicFrictionEffort').Set(.03)
                prim.GetAttribute('physxJointAxis:linear:viscousFrictionCoefficient').Set(0)
    layer.Save()
