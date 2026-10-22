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
    # The left finger is driven; the right one follows it (MIMIC_FINGERS). The
    # drive reaches the URDF's 20 N force limit once the target is 4 mm past the
    # held object, so a 1 cm cube is squeezed as hard as a 4 cm one (at 1500 N/m
    # the drive gave a 1 cm cube 7.5 N). The pair splits it: each pad presses with 10 N.
    '_arm_gripper_left_joint': (5000., 60.),
    '_arm_gripper_right_joint': (0., 0.),
    '_wheel_joint': (0., 1.),
    # Virtual planar base joints (see table_1000.robots.build): stiff and close
    # to critically damped for the 73.5 kg robot.
    'base_x_joint': (1e5, 6000.),
    'base_y_joint': (1e5, 6000.),
    'base_yaw_joint': (8000., 500.),
}
# (position, velocity). At 16 position iterations a 1 cm steel cube held by the
# pads slips under a 5 N push (grasp_and_push); 32 holds it.
SOLVER_ITERATIONS = (32, 4)
# One motor moves both fingers of the real gripper through a rack. Two
# independently driven fingers, both at their force limit on a held object,
# have no restoring force along the closing axis: an acceleration of the hand
# shifts fingers and object sideways together (13 mm in a base turn). Each right
# finger therefore mimics its left one: right + left = 0.
MIMIC_FINGERS = [(f'{side}_arm_gripper_right_joint', f'{side}_arm_gripper_left_joint') for side in ('left', 'right')]
# Body pairs that never collide on the real robot but whose convex hulls
# overlap: each gripper's two fingers (guided racks that pass each other), the
# head housing seated on the torso, and each forearm (link4) resting on its
# upper arm (link2) in the folded zero pose, which the joint limits make the
# closest they come.
FILTERED_PAIRS = [('left_arm_gripper_left_link', 'left_arm_gripper_right_link'),
                  ('right_arm_gripper_left_link', 'right_arm_gripper_right_link'),
                  ('head_pitch_link', 'lift_link'),
                  ('left_arm_link2', 'left_arm_link4'),
                  ('right_arm_link2', 'right_arm_link4')]
# (static, dynamic) friction; gripper pads are rubber-like.
MATERIALS = {'body': (.4, .3), 'finger': (1., .8)}
# Pad contact patch radius (m) for torsional friction. PhysX point contacts
# otherwise let a held object pivot freely about the line between the pads;
# a soft pad resists that over its contact patch.
PAD_PATCH_RADIUS = .006


def gains(name, drives=DRIVES):
    matches = [value for key, value in drives.items() if key in name]
    if len(matches) != 1:
        raise ValueError(f'expected one drive entry for {name}, found {len(matches)}')
    return matches[0]


def author(import_path, output_path, drives=DRIVES, materials=MATERIALS, armature=None, iterations=SOLVER_ITERATIONS):
    """Write ``output_path`` as a layer over the imported robot, with the joint drives ``drives`` and
    contact ``materials`` (x1pro_rm001 passes its own finger drives, pads and solver iterations).
    ``armature`` maps joint name parts to added joint inertia (kg m^2 or kg)."""
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
            articulation.CreateSolverPositionIterationCountAttr(iterations[0])
            articulation.CreateSolverVelocityIterationCountAttr(iterations[1])
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        angular = prim.IsA(UsdPhysics.RevoluteJoint)
        stiffness, damping = gains(prim.GetName(), drives)
        scale = math.pi / 180 if angular else 1.
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular' if angular else 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(stiffness * scale)
        drive.CreateDampingAttr(damping * scale)
        for key, value in (armature or {}).items():
            if key in prim.GetName():
                PhysxSchema.PhysxJointAPI.Apply(prim).CreateArmatureAttr(value)

    joints = {prim.GetName(): prim for prim in Usd.PrimRange(root) if prim.IsA(UsdPhysics.Joint)}
    for follower, leader in MIMIC_FINGERS:
        # Single-degree-of-freedom joints ignore the axis token.
        mimic = PhysxSchema.PhysxMimicJointAPI.Apply(joints[follower], 'rotX')
        mimic.CreateGearingAttr(1.)
        mimic.CreateOffsetAttr(0.)
        mimic.CreateReferenceJointRel().SetTargets([joints[leader].GetPath()])

    for first, second in FILTERED_PAIRS:
        pair = UsdPhysics.FilteredPairsAPI.Apply(root.GetChild(first))
        pair.CreateFilteredPairsRel().AddTarget(root.GetChild(second).GetPath())

    bound = {}
    for name, (static, dynamic) in materials.items():
        material = UsdShade.Material.Define(stage, root.GetPath().AppendPath(f'PhysicsMaterials/{name}'))
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.)
        bound[name] = material
    for prim in root.GetChildren():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            kind = 'finger' if prim.GetName().endswith(('_gripper_left_link', '_gripper_right_link')) else 'body'
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(bound[kind], materialPurpose='physics')
            if kind == 'finger':
                # The importer instances colliders; a finger's own copy can be edited.
                for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                    if child.IsInstance():
                        child.SetInstanceable(False)
                for shape in Usd.PrimRange(prim):
                    if shape.HasAPI(UsdPhysics.CollisionAPI):
                        collision = PhysxSchema.PhysxCollisionAPI.Apply(shape)
                        pad = bound.get('pad')
                        # Mesh colliders are nested below the named URDF collision
                        # (the leaf itself is typically just "node_").
                        is_pad = 'pad' in str(shape.GetPath().MakeRelativePath(prim.GetPath())).lower()
                        if pad is not None and is_pad:
                            UsdShade.MaterialBindingAPI.Apply(shape).Bind(pad, materialPurpose='physics')
                        if pad is None or is_pad:
                            collision.CreateTorsionalPatchRadiusAttr(PAD_PATCH_RADIUS)
                            collision.CreateMinTorsionalPatchRadiusAttr(PAD_PATCH_RADIUS)
    layer.Save()
