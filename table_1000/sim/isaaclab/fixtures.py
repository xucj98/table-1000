"""Native six-axis fixture articulations controlled through PhysX tensors.

A collision-free serial XYZ translation/XYZ rotation chain provides the six
coordinates of a grasp. Each virtual link has 10 microgram mass and 1e-11 kg m²
inertia; gravity is disabled. The tested body is the terminal link, avoiding a mass-ratio-sensitive external joint. Releasing opens all limits and disables all drives, leaving
six free coordinates, without deleting joints or writing dynamic body poses.
"""

import numpy as np
from scipy.spatial.transform import Rotation

LINK_MASS = 1e-8
LINK_INERTIA = 1e-11


class RootClamp:
    """Permanent moving clamp of an existing articulation's root.

    The world joint is declared before initialization. Tensor root transforms
    move the fixed base without adding degrees of freedom to the asset tree.
    This fixture cannot be released during an experiment; use Grip for that.
    """

    def __init__(self, ctx, body, *, name):
        from pxr import Gf, UsdPhysics, PhysxSchema
        assert not ctx.initialized
        assert body.path == body.instance.articulation_root
        self.ctx, self.body = ctx, body
        self.locked, self.released = True, False
        self.position, self.quaternion = ctx.body_pose(body)
        self.path = '/World/Fixtures/' + name
        stage = ctx.session.stage
        source = stage.GetPrimAtPath(body.path)
        original = PhysxSchema.PhysxArticulationAPI(source)
        joint = UsdPhysics.FixedJoint.Define(stage, self.path)
        joint.CreateBody1Rel().SetTargets([body.path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*self.position))
        joint.CreateLocalRot0Attr(Gf.Quatf(float(self.quaternion[0]), Gf.Vec3f(*self.quaternion[1:])))
        UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
        api = PhysxSchema.PhysxArticulationAPI.Apply(joint.GetPrim())
        api.CreateEnabledSelfCollisionsAttr(original.GetEnabledSelfCollisionsAttr().Get())
        api.CreateSolverPositionIterationCountAttr(original.GetSolverPositionIterationCountAttr().Get())
        api.CreateSolverVelocityIterationCountAttr(original.GetSolverVelocityIterationCountAttr().Get())
        source.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        body.instance.fixture_root = self.path
        self.view = None
        ctx.fixtures.append(self)

    def initialize(self):
        from isaacsim.core.simulation_manager import SimulationManager
        from table_1000.sim.isaaclab.asset_views import TensorView
        self.view = TensorView(SimulationManager.get_physics_sim_view().create_articulation_view(self.path),
                               SimulationManager.get_physics_sim_device())
        self.indices = np.array([0], dtype=np.int32)
        self.move()

    def move(self, position=None, quaternion=None):
        if position is not None:
            self.position = np.asarray(position, dtype=float)
        if quaternion is not None:
            self.quaternion = np.asarray(quaternion, dtype=float)
        if self.view is not None:
            pose = np.array([[*self.position, *self.quaternion[[1, 2, 3, 0]]]], dtype=np.float32)
            self.view.set_root_transforms(pose, self.indices)


class Grip:
    def __init__(self, ctx, body, *, name, locked=False, linear_stiffness=2000,
                 linear_damping=40, angular_stiffness=20, angular_damping=.4,
                 linear_max_force=30, angular_max_force=1):
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        assert not ctx.initialized, 'Declare fixtures before the first simulation step'
        self.ctx, self.body, self.locked = ctx, body, locked
        self.position, self.quaternion = ctx.body_pose(body)
        self.origin, self.basis = self.position.copy(), Rotation.from_quat(self.quaternion[[1, 2, 3, 0]])
        self.target = np.zeros(6, dtype=np.float32)
        self.stiffness = np.array([linear_stiffness]*3+[angular_stiffness]*3, dtype=np.float32)
        self.damping = np.array([linear_damping]*3+[angular_damping]*3, dtype=np.float32)
        self.released = False
        self.path = '/World/Fixtures/' + name
        stage = ctx.session.stage
        # Changing the articulation root must retain the asset's solver and
        # collision settings, rather than silently taking PhysX defaults.
        source = stage.GetPrimAtPath(body.instance.articulation_root or body.path)
        source_api = (PhysxSchema.PhysxArticulationAPI(source) if body.instance.articulation_root
                      else PhysxSchema.PhysxRigidBodyAPI(source))
        position_iterations = source_api.GetSolverPositionIterationCountAttr().Get()
        velocity_iterations = source_api.GetSolverVelocityIterationCountAttr().Get()
        self_collisions = (source_api.GetEnabledSelfCollisionsAttr().Get()
                           if body.instance.articulation_root else False)
        UsdGeom.Xform.Define(stage, self.path)
        previous = None
        for i in range(6):
            path = f'{self.path}/link_{i}'
            link = UsdGeom.Xform.Define(stage, path)
            link.AddTranslateOp().Set(Gf.Vec3d(*self.origin))
            link.AddOrientOp().Set(Gf.Quatf(float(self.quaternion[0]), Gf.Vec3f(*map(float, self.quaternion[1:]))))
            UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
            mass = UsdPhysics.MassAPI.Apply(link.GetPrim())
            mass.CreateMassAttr(LINK_MASS)
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(LINK_INERTIA))
            PhysxSchema.PhysxRigidBodyAPI.Apply(link.GetPrim()).CreateDisableGravityAttr(True)
            if i == 0:
                joint = UsdPhysics.FixedJoint.Define(stage, self.path+'/root_joint')
                joint.CreateBody1Rel().SetTargets([path])
                joint.CreateLocalPos0Attr(Gf.Vec3f(*self.origin))
                joint.CreateLocalRot0Attr(Gf.Quatf(float(self.quaternion[0]), Gf.Vec3f(*map(float, self.quaternion[1:]))))
                UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
                api = PhysxSchema.PhysxArticulationAPI.Apply(joint.GetPrim())
                api.CreateEnabledSelfCollisionsAttr(self_collisions)
                api.CreateSolverPositionIterationCountAttr(position_iterations)
                api.CreateSolverVelocityIterationCountAttr(velocity_iterations)
            else:
                cls = UsdPhysics.PrismaticJoint if i <= 3 else UsdPhysics.RevoluteJoint
                joint = cls.Define(stage, f'{self.path}/axis_{i-1}')
                joint.CreateAxisAttr('XYZ'[(i-1) % 3])
                joint.CreateBody0Rel().SetTargets([previous])
                joint.CreateBody1Rel().SetTargets([path])
                # Explicit large limits select PhysX's unwrapped revolute type.
                joint.CreateLowerLimitAttr(-1e10)
                joint.CreateUpperLimitAttr(1e10)
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear' if i <= 3 else 'angular')
                factor = 1 if i <= 3 else np.pi/180
                drive.CreateTypeAttr('force')
                drive.CreateStiffnessAttr(float(self.stiffness[i-1]*factor))
                drive.CreateDampingAttr(float(self.damping[i-1]*factor))
                drive.CreateMaxForceAttr(linear_max_force if i <= 3 else angular_max_force)

            previous = path
        # Put the tested body in the reduced-coordinate tree: an external
        # fixed joint to a near-massless proxy would be ill-conditioned.
        joint = UsdPhysics.RevoluteJoint.Define(stage, self.path+'/axis_5')
        joint.CreateAxisAttr('Z')
        joint.CreateBody0Rel().SetTargets([previous])
        joint.CreateBody1Rel().SetTargets([body.path])
        joint.CreateLowerLimitAttr(-1e10)
        joint.CreateUpperLimitAttr(1e10)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(float(angular_stiffness*np.pi/180))
        drive.CreateDampingAttr(float(angular_damping*np.pi/180))
        drive.CreateMaxForceAttr(angular_max_force)
        # Both frames meet at the body's declared initial origin.
        body_pos, body_q = ctx.body_pose(body)
        native_pos, native_q = ctx.usd_body_pose(body)
        local = Rotation.from_quat(native_q[[1, 2, 3, 0]]).inv()*self.basis
        joint.CreateLocalRot1Attr(Gf.Quatf(float(local.as_quat()[3]), Gf.Vec3f(*map(float, local.as_quat()[:3]))))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*map(float, Rotation.from_quat(native_q[[1, 2, 3, 0]]).inv().apply(body_pos-native_pos))))
        instance = body.instance
        if not hasattr(instance, "fixture_bodies"):
            instance.fixture_bodies = set()
        instance.fixture_bodies.add(body.name)
        if instance.articulation_root:
            stage.GetPrimAtPath(instance.articulation_root).RemoveAPI(UsdPhysics.ArticulationRootAPI)
            instance.fixture_root = self.path+'/root_joint'
        else:
            # The pen's radial joints stay maximal, with their original drives.
            for name in instance.joint_names:
                UsdPhysics.Joint(instance.joint(name)).CreateExcludeFromArticulationAttr(True)
        self.view = None
        ctx.fixtures.append(self)

    def initialize(self):
        from isaacsim.core.simulation_manager import SimulationManager
        from table_1000.sim.isaaclab.asset_views import TensorView
        self.view = TensorView(SimulationManager.get_physics_sim_view().create_articulation_view(self.path+'/root_joint'),
                               SimulationManager.get_physics_sim_device())
        self.indices = np.array([0], dtype=np.int32)
        names = self.view.shared_metatype.dof_names
        self.order = [names.index("axis_"+str(i)) for i in range(6)]
        if not hasattr(self, 'free_limits'):
            self.free_limits = self.view.get_dof_limits().copy()
        self.apply()

    def apply(self):
        if self.view is None:
            return
        targets = self.view.get_dof_position_targets()
        stiffness = self.view.get_dof_stiffnesses()
        damping = self.view.get_dof_dampings()
        targets[0, self.order] = self.target
        stiffness[0, self.order] = 0 if self.released or self.locked else self.stiffness
        damping[0, self.order] = 0 if self.released or self.locked else self.damping
        self.view.set_dof_position_targets(targets, self.indices)
        self.view.set_dof_stiffnesses(stiffness, self.indices)
        self.view.set_dof_dampings(damping, self.indices)
        limits = self.free_limits.copy()
        if self.locked and not self.released:
            limits[0, self.order] = self.target[:, None]
        self.view.set_dof_limits(limits, self.indices)

    def move(self, position=None, quaternion=None):
        previous = self.target.copy()
        if position is not None:
            self.position = np.asarray(position, dtype=float)
        if quaternion is not None:
            self.quaternion = np.asarray(quaternion, dtype=float)
        self.target[:3] = self.basis.inv().apply(self.position-self.origin)
        relative = self.basis.inv()*Rotation.from_quat(self.quaternion[[1, 2, 3, 0]])
        angles = relative.as_euler('XYZ')
        alternate = np.array([angles[0]+np.pi, np.pi-angles[1], angles[2]+np.pi])
        candidates = [v+2*np.pi*np.round((self.target[3:]-v)/(2*np.pi)) for v in (angles, alternate)]
        self.target[3:] = min(candidates, key=lambda v: np.linalg.norm(v-self.target[3:]))
        self.apply()
        if self.view is not None and self.locked and not self.released:
            # A moving clamp prescribes fixture coordinates AND velocities.
            # Moving only the hard limits makes the solver infer motion from
            # constraint error and injects spurious acceleration into drawers.
            positions = self.view.get_dof_positions()
            velocities = self.view.get_dof_velocities()
            positions[0, self.order] = self.target
            velocities[0, self.order] = (self.target - previous) / self.ctx.dt
            self.view.set_dof_positions(positions, self.indices)
            self.view.set_dof_velocities(velocities, self.indices)

    def axial(self, stiffness=0, damping=0):
        self.stiffness[0], self.damping[0] = stiffness, damping
        self.apply()

    def release(self, released=True):
        self.released = released
        self.apply()
