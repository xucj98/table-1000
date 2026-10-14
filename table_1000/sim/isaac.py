"""Isaac Sim backend on CPU PhysX: one environment per process.

Run inside a started ``SimulationApp``. The stage is built once; :meth:`reset`
writes a new layout and robot state into it without rebuilding, so one process
can run many episodes. Every array has a leading environment axis (length
:attr:`num_envs`, here 1), the shape a GPU backend with many environments will
share.

All Isaac calls of the task environment live here; :mod:`table_1000.sim.env`
and everything above it see only this interface:

- ``robot_joints``, ``set_targets(joints, positions, velocities)``, ``step()``,
  ``joint_state(joints)``
- ``body_poses()``, ``center_of_mass(prefix)``, ``object_joints(name)``,
  ``link_pose(link)``
- ``reset(layout, robot_q)``, ``contacts``, ``capture(caption)``, ``close()``
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.scenes import stage as scene_stage
from table_1000.sim.contacts import Contacts

DT = .004
GROUND_PATH = '/World/defaultGroundPlane'


def matrix(position, quaternion_wxyz):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(np.asarray(quaternion_wxyz)[[1, 2, 3, 0]]).as_matrix()
    transform[:3, 3] = position
    return transform


def position_quaternion(transform):
    return transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]


class IsaacBackend:
    num_envs = 1
    physics_dt = DT

    def __init__(self, scene, layout, assets, *, cameras=None, contact_watch=('robot',), contact_objects=()):
        """``layout`` is the scene's placement used to build the stage.

        Contact reports come from the robot and the bodies of ``contact_objects``
        (object names); pairs are kept when a side starts with ``contact_watch``.
        Objects with internal contacts (a pen cap detent) should not report.
        """
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim, SingleArticulation
        self.timings = {'load_s': 0., 'step_s': 0., 'render_s': 0.}
        started = time.perf_counter()
        World.clear_instance()
        omni.usd.get_context().new_stage()
        self.scene = scene
        self.world = World(physics_dt=DT, rendering_dt=DT * 10, stage_units_in_meters=1,
                           backend='numpy', device='cpu')
        context = self.world.get_physics_context()
        context.enable_gpu_dynamics(False)
        context.set_solver_type('TGS')
        context.set_broadphase_type('MBP')
        context.enable_ccd(True)
        self.stage = omni.usd.get_context().get_stage()
        self.world.scene.add_default_ground_plane()
        scene_stage.populate(self.stage, scene, layout, assets, robot_transform=np.eye(4))
        self._filter_robot_ground()
        self.bodies = {}
        for name, path in scene_stage.object_paths(scene).items():
            for body, body_path in scene_stage.named_bodies(self.stage, path).items():
                self.bodies[f'{name}:{body}'] = body_path
        labels = {scene_stage.ROBOT_PATH: 'robot', scene_stage.TABLE_PATH: 'table', GROUND_PATH: 'ground'}
        labels.update({path: key for key, path in self.bodies.items()})
        reporting = [path for key, path in self.bodies.items() if key.split(':')[0] in contact_objects]
        self.contacts = Contacts(self.stage, [scene_stage.ROBOT_PATH, *reporting], labels, contact_watch)
        self.robot = self.world.scene.add(SingleArticulation(scene_stage.ROBOT_PATH, name='robot'))
        # Object articulations (a cabinet with drawers), by object name.
        self.articulations = {}
        for name, path in scene_stage.object_paths(scene).items():
            root = self._articulation_root(path)
            if root is not None:
                self.articulations[name] = self.world.scene.add(SingleArticulation(root, name=f'{name}_articulation'))
        self.keys = list(self.bodies)
        # Kinematic bodies (fixed test shapes) are posed but take no velocities.
        self.kinematic = {key for key, path in self.bodies.items()
                          if self.stage.GetPrimAtPath(path).GetAttribute('physics:kinematicEnabled').Get()}
        self.views = RigidPrim([self.bodies[key] for key in self.keys], name='bodies', reset_xform_properties=False)
        self.cameras = cameras(self.stage) if cameras else None
        self.world.reset()
        self.views.initialize()
        self.robot_joints = list(self.robot.dof_names)
        self.dof = {name: i for i, name in enumerate(self.robot_joints)}
        self.targets = np.zeros((1, len(self.robot_joints)))
        # Body poses at build time, relative to their object's placement, for resets.
        self.layout = layout
        poses = self.body_poses()
        self.rest = {key: np.linalg.inv(layout[key.split(':')[0]]) @ poses[key][0] for key in self.keys}
        self.object_rest = {name: art.get_joint_positions().copy() for name, art in self.articulations.items()}
        self.steps = 0
        self.timings['load_s'] = time.perf_counter() - started

    # -- construction helpers ---------------------------------------------------
    def _filter_robot_ground(self):
        """The base is carried by virtual joints; wheels must not drag on the floor."""
        from pxr import Usd, UsdPhysics
        bodies = [prim.GetPath() for prim in Usd.PrimRange(self.stage.GetPrimAtPath(scene_stage.ROBOT_PATH))
                  if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(GROUND_PATH)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                relation = UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
                for body in bodies:
                    relation.AddTarget(body)

    def _articulation_root(self, path):
        from pxr import Usd, UsdPhysics
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(path)):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(prim.GetPath())
        return None

    # -- state ---------------------------------------------------------------------
    @property
    def time(self):
        return self.steps * DT

    def reset(self, layout, robot_q):
        """Place the objects of ``layout`` and set the robot's joints (``{name: value}``, base included)."""
        from isaacsim.core.utils.types import ArticulationAction
        for name, articulation in self.articulations.items():
            position, orientation = position_quaternion(layout[name] @ self._root_rest(name))
            articulation.set_world_pose(position, orientation)
            articulation.set_joint_positions(self.object_rest[name])
            articulation.set_joint_velocities(np.zeros_like(self.object_rest[name]))
            articulation.set_linear_velocity(np.zeros(3))
            articulation.set_angular_velocity(np.zeros(3))
        free = [i for i, key in enumerate(self.keys) if key.split(':')[0] not in self.articulations]
        if free:
            poses = [layout[self.keys[i].split(':')[0]] @ self.rest[self.keys[i]] for i in free]
            positions, orientations = zip(*(position_quaternion(pose) for pose in poses))
            self.views.set_world_poses(np.array(positions), np.array(orientations), indices=np.array(free))
            moving = [i for i in free if self.keys[i] not in self.kinematic]
            if moving:
                self.views.set_velocities(np.zeros((len(moving), 6)), indices=np.array(moving))
        q = np.zeros(len(self.robot_joints))
        for name, value in robot_q.items():
            q[self.dof[name]] = value
        self.robot.set_joint_positions(q)
        self.robot.set_joint_velocities(np.zeros_like(q))
        self.targets = q[None].copy()
        self.robot.apply_action(ArticulationAction(joint_positions=q, joint_velocities=np.zeros_like(q)))
        self.layout = layout
        self.steps = 0
        # Contacts of the reset frame (bodies still at their old poses) are dropped.
        self.world.step(render=False)
        self.contacts.clear()
        self.contacts.time = 0.

    def _root_rest(self, name):
        root = self.articulations[name].prim_path
        key = next(key for key, path in self.bodies.items() if path == root)
        return self.rest[key]

    def set_targets(self, joints, positions, velocities=None):
        """Position (and velocity) targets ``[N, len(joints)]`` for the named robot joints."""
        from isaacsim.core.utils.types import ArticulationAction
        indices = [self.dof[name] for name in joints]
        self.targets[:, indices] = positions
        full = None
        if velocities is not None:
            full = np.zeros(len(self.robot_joints))
            full[indices] = np.asarray(velocities)[0]
        self.robot.apply_action(ArticulationAction(joint_positions=self.targets[0].copy(), joint_velocities=full))

    def step(self):
        started = time.perf_counter()
        self.contacts.time = self.time
        self.world.step(render=False)
        self.steps += 1
        self.timings['step_s'] += time.perf_counter() - started

    def joint_state(self, joints=None):
        joints = joints or self.robot_joints
        indices = [self.dof[name] for name in joints]
        return self.robot.get_joint_positions()[None, indices], self.robot.get_joint_velocities()[None, indices]

    def body_poses(self):
        """``{'cabinet:cabinet.drawer3': [N, 4, 4], ...}`` from the physics state."""
        positions, orientations = self.views.get_world_poses()
        return {key: matrix(p, q)[None] for key, p, q in zip(self.keys, positions, orientations)}

    def center_of_mass(self, prefix):
        """``[N, 3]`` world centre of mass of the bodies whose keys start with ``prefix``."""
        indices = [i for i, key in enumerate(self.keys) if key.startswith(prefix)]
        positions, orientations = self.views.get_world_poses()
        coms = np.asarray(self.views.get_coms()[0]).reshape(-1, 3)
        masses = np.asarray(self.views.get_masses()).reshape(-1)
        points = [matrix(positions[i], orientations[i])[:3, :3] @ coms[i] + positions[i] for i in indices]
        return np.average(points, axis=0, weights=masses[indices])[None]

    def body_velocities(self):
        """``{key: [N, 6]}`` linear and angular world velocities."""
        velocities = self.views.get_velocities()
        return {key: np.asarray(v)[None] for key, v in zip(self.keys, velocities)}

    def set_object_pose(self, name, transform):
        """Teleport a free (non-articulated) object to ``transform`` and stop it."""
        indices = [i for i, key in enumerate(self.keys) if key.split(':')[0] == name]
        poses = [transform @ self.rest[self.keys[i]] for i in indices]
        positions, orientations = zip(*(position_quaternion(pose) for pose in poses))
        self.views.set_world_poses(np.array(positions), np.array(orientations), indices=np.array(indices))
        moving = [i for i in indices if self.keys[i] not in self.kinematic]
        if moving:
            self.views.set_velocities(np.zeros((len(moving), 6)), indices=np.array(moving))

    def object_joints(self, name):
        """``{joint: [N]}`` of an object articulation."""
        articulation = self.articulations[name]
        return {joint: np.array([value]) for joint, value in zip(articulation.dof_names, articulation.get_joint_positions())}

    def link_pose(self, link):
        """``[N, 4, 4]`` world pose of a robot body as rendered (physics synced to USD)."""
        from omni.physx import get_physx_interface
        from pxr import UsdGeom
        get_physx_interface().update_transformations(True, True, True, False)
        prim = self.stage.GetPrimAtPath(f'{scene_stage.ROBOT_PATH}/{link}')
        return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T[None]

    def capture(self, caption='', save=True):
        if not self.cameras:
            return {}
        started = time.perf_counter()
        images = self.cameras.capture(caption, save)
        self.timings['render_s'] += time.perf_counter() - started
        return images

    def close(self):
        self.contacts.close()
        if self.cameras:
            self.cameras.close()
