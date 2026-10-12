"""Joint drives, solver settings and contact materials for X1Pro.

Gains are given per radian (revolute) or per metre (prismatic) and converted to
USD's per-degree angular units. Effort and velocity limits come from the URDF.
These are simulation values, not calibrated against the real controller.

Robot links ignore gravity: the joint drives stand for a gravity-compensated
controller, so position targets are tracked without sag. Payloads such as a
held object keep normal gravity.
"""

import math

# joint name prefix or exact name -> (stiffness, damping)
DRIVES = {
    'lift_joint': (20000., 1000.),
    'head_': (30., 3.),
    '_arm_joint': (1000., 60.),
    '_arm_gripper_': (1500., 20.),
    '_wheel_joint': (0., 1.),
}
SOLVER_ITERATIONS = (16, 4)
# Body pairs that never collide on the real robot but whose convex hulls
# overlap: each gripper's two fingers (guided racks that pass each other) and
# the head housing seated on the torso.
FILTERED_PAIRS = [('left_arm_gripper_left_link', 'left_arm_gripper_right_link'),
                  ('right_arm_gripper_left_link', 'right_arm_gripper_right_link'),
                  ('head_pitch_link', 'lift_link')]
# (static, dynamic) friction; gripper pads are rubber-like.
MATERIALS = {'body': (.4, .3), 'finger': (1., .8)}


def gains(name):
    matches = [value for key, value in DRIVES.items() if key in name]
    if len(matches) != 1:
        raise ValueError(f'expected one drive entry for {name}, found {len(matches)}')
    return matches[0]


def author(import_path, output_path):
    """Write ``output_path`` as a layer over the imported robot."""
    from pxr import PhysxSchema, Sdf, Usd, UsdPhysics, UsdShade

    layer = Sdf.Layer.CreateNew(str(output_path))
    layer.subLayerPaths = [str(import_path.relative_to(output_path.parent))]
    imported = Usd.Stage.Open(str(import_path))
    layer.defaultPrim = imported.GetDefaultPrim().GetName()
    stage = Usd.Stage.Open(layer)
    stage.SetEditTarget(layer)
    root = stage.GetDefaultPrim()

    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            articulation.CreateEnabledSelfCollisionsAttr(True)
            articulation.CreateSolverPositionIterationCountAttr(SOLVER_ITERATIONS[0])
            articulation.CreateSolverVelocityIterationCountAttr(SOLVER_ITERATIONS[1])
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        angular = prim.IsA(UsdPhysics.RevoluteJoint)
        stiffness, damping = gains(prim.GetName())
        scale = math.pi / 180 if angular else 1.
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular' if angular else 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(stiffness * scale)
        drive.CreateDampingAttr(damping * scale)

    for first, second in FILTERED_PAIRS:
        pair = UsdPhysics.FilteredPairsAPI.Apply(root.GetChild(first))
        pair.CreateFilteredPairsRel().AddTarget(root.GetChild(second).GetPath())

    materials = {}
    for name, (static, dynamic) in MATERIALS.items():
        material = UsdShade.Material.Define(stage, root.GetPath().AppendPath(f'PhysicsMaterials/{name}'))
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.)
        materials[name] = material
    for prim in root.GetChildren():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            kind = 'finger' if prim.GetName().endswith(('_gripper_left_link', '_gripper_right_link')) else 'body'
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(materials[kind], materialPurpose='physics')
    layer.Save()
