"""Isaac Sim environment for a scene: one stage, a robot, live body poses.

Run inside a started ``SimulationApp``. The environment only steps physics,
sends joint targets to the robot drives and reads state; it never writes
object states after the initial placement.
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.scenes import stage as scene_stage

DT = .004
RENDER_EVERY = 10  # 25 frames per second of simulated time


def matrix(position, quaternion_wxyz):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(np.asarray(quaternion_wxyz)[[1, 2, 3, 0]]).as_matrix()
    transform[:3, 3] = position
    return transform


class Contacts:
    """PhysX contact reports for the bodies under ``paths``, kept as time intervals.

    Pairs are kept when either side's label starts with one of ``watched``.
    """

    TOUCH = .0005

    def __init__(self, stage, paths, labels, watched):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysxSchema, Usd, UsdPhysics
        self.labels, self.watched = labels, tuple(watched)
        for path in paths:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        self.time, self.phase = 0., 'start'
        self.events = []
        self.open = {}
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.on_report)

    def label(self, path):
        """Robot link (``robot.<link>``), object part (``<body key>.<part>``) or fixture."""
        for prefix, name in self.labels.items():
            if path == prefix or path.startswith(prefix + '/'):
                first = path[len(prefix):].strip('/').split('/')[0]
                if name == 'robot' or (first and name not in ('table', 'ground')):
                    return f'{name}.{first}' if first else name
                return name
        return path

    def on_report(self, headers, data):
        """Track intervals in which a watched body actually touches something.

        PhysX reports shape pairs within their contact offset; a pair counts as
        touching while its closest contact point has separation below TOUCH.
        """
        from pxr import PhysicsSchemaTools
        for header in headers:
            a = self.label(str(PhysicsSchemaTools.intToSdfPath(header.collider0)))
            b = self.label(str(PhysicsSchemaTools.intToSdfPath(header.collider1)))
            if not (a.startswith(self.watched) or b.startswith(self.watched)):
                continue
            pair = tuple(sorted((a, b)))
            points = data[header.contact_data_offset:header.contact_data_offset + header.num_contact_data]
            separation = min((point.separation for point in points), default=float('inf'))
            touching = 'LOST' not in str(header.type) and separation < self.TOUCH
            if touching:
                entry = self.open.setdefault(pair, {'pair': pair, 'start': self.time, 'phase': self.phase,
                                                    'max_penetration_m': 0.})
                entry['max_penetration_m'] = max(entry['max_penetration_m'], -float(separation))
            elif pair in self.open:
                entry = self.open.pop(pair)
                entry['end'] = self.time
                self.events.append(entry)

    def intervals(self):
        return self.events + [dict(entry, end=None) for entry in self.open.values()]

    def close(self):
        self.subscription = None


class Environment:
    def __init__(self, scene, layout, assets, *, cameras=None, gpu_render=False):
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim, SingleArticulation
        self.timings = {'load_s': 0., 'step_s': 0., 'render_s': 0., 'record_s': 0.}
        started = time.perf_counter()
        World.clear_instance()
        omni.usd.get_context().new_stage()
        self.scene, self.layout = scene, layout
        self.world = World(physics_dt=DT, rendering_dt=DT * RENDER_EVERY, stage_units_in_meters=1,
                           backend='numpy', device='cpu')
        context = self.world.get_physics_context()
        context.enable_gpu_dynamics(False)
        context.set_solver_type('TGS')
        context.set_broadphase_type('MBP')
        context.enable_ccd(True)
        self.stage = omni.usd.get_context().get_stage()
        self.world.scene.add_default_ground_plane()
        scene_stage.populate(self.stage, scene, layout, assets)
        self.bodies = {}
        for name, path in scene_stage.OBJECT_PATHS.items():
            for body, body_path in scene_stage.named_bodies(self.stage, path).items():
                self.bodies[f'{name}:{body}'] = body_path
        labels = {scene_stage.ROBOT_PATH: 'robot', scene_stage.TABLE_PATH: 'table',
                  '/World/defaultGroundPlane': 'ground'}
        labels.update({path: key for key, path in self.bodies.items()})
        # Reports come from the robot and the cabinet; reporting on the pen
        # itself would also deliver its cap detent's contacts on every step.
        cabinet = [path for key, path in self.bodies.items() if key.startswith('cabinet:')]
        self.contacts = Contacts(self.stage, [scene_stage.ROBOT_PATH, *cabinet], labels, ('robot', 'pen:'))
        self.robot = self.world.scene.add(SingleArticulation(scene_stage.ROBOT_PATH, name='robot'))
        cabinet_root = next(path for key, path in self.bodies.items() if key == 'cabinet:cabinet.housing')
        self.cabinet = self.world.scene.add(SingleArticulation(cabinet_root, name='cabinet'))
        self.keys = list(self.bodies)
        self.views = RigidPrim([self.bodies[key] for key in self.keys], name='bodies', reset_xform_properties=False)
        self.cameras = cameras(self.stage) if cameras else None
        self.world.reset()
        self.views.initialize()
        self.dof = {name: i for i, name in enumerate(self.robot.dof_names)}
        self.cabinet_dof = {name: i for i, name in enumerate(self.cabinet.dof_names)}
        initial = np.zeros(len(self.dof))
        for name, value in scene.ROBOT['joints'].items():
            initial[self.dof[name]] = value
        self.robot.set_joint_positions(initial)
        self.robot.set_joint_velocities(np.zeros(len(self.dof)))
        self.targets = initial.copy()
        self.send()
        # The reset frame still has the USD zero pose (arms folded); contacts
        # from it are dropped. Persisting contacts are reported again next step.
        self.world.step(render=False)
        self.contacts.events.clear()
        self.contacts.open.clear()
        self.steps, self.frames, self.trace = 0, [], []
        self.observers = []
        self.timings['load_s'] = time.perf_counter() - started

    @property
    def time(self):
        return self.steps * DT

    def send(self, velocities=None):
        from isaacsim.core.utils.types import ArticulationAction
        action = ArticulationAction(joint_positions=self.targets.copy(),
                                    joint_velocities=None if velocities is None else velocities)
        self.robot.apply_action(action)

    def set_targets(self, values, velocities=None):
        """Position (and optionally velocity) targets by joint name."""
        full_velocity = np.zeros(len(self.dof)) if velocities is not None else None
        for name, value in values.items():
            self.targets[self.dof[name]] = value
            if velocities is not None:
                full_velocity[self.dof[name]] = velocities.get(name, 0.)
        self.send(full_velocity)

    def step(self, count=1):
        for _ in range(count):
            started = time.perf_counter()
            self.contacts.time = self.time
            self.world.step(render=False)
            self.steps += 1
            self.timings['step_s'] += time.perf_counter() - started
            if self.steps % RENDER_EVERY == 0:
                started = time.perf_counter()
                for observer in self.observers:
                    observer(self)
                self.timings['record_s'] += time.perf_counter() - started
                if self.cameras:
                    started = time.perf_counter()
                    self.cameras.capture(self)
                    self.timings['render_s'] += time.perf_counter() - started

    def joint_positions(self, names=None):
        q = self.robot.get_joint_positions()
        names = names or list(self.dof)
        return np.array([q[self.dof[name]] for name in names])

    def joint_velocities(self, names):
        v = self.robot.get_joint_velocities()
        return np.array([v[self.dof[name]] for name in names])

    def center_of_mass(self, prefix):
        """World centre of mass of the bodies whose keys start with ``prefix``."""
        indices = [i for i, key in enumerate(self.keys) if key.startswith(prefix)]
        positions, orientations = self.views.get_world_poses()
        coms = np.asarray(self.views.get_coms()[0]).reshape(-1, 3)
        masses = np.asarray(self.views.get_masses()).reshape(-1)
        points = [matrix(positions[i], orientations[i])[:3, :3] @ coms[i] + positions[i] for i in indices]
        return np.average(points, axis=0, weights=masses[indices])

    def body_poses(self):
        """``{'cabinet:cabinet.drawer3': 4x4, ...}`` from the physics state."""
        positions, orientations = self.views.get_world_poses()
        return {key: matrix(p, q) for key, p, q in zip(self.keys, positions, orientations)}

    def link_pose(self, link):
        """World pose of a robot body as rendered (after syncing physics to USD)."""
        from omni.physx import get_physx_interface
        from pxr import UsdGeom
        get_physx_interface().update_transformations(True, True, True, False)
        prim = self.stage.GetPrimAtPath(f'{scene_stage.ROBOT_PATH}/{link}')
        return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T

    def drawer_opening(self, joint):
        return float(self.cabinet.get_joint_positions()[self.cabinet_dof[joint]])

    def close(self):
        self.contacts.close()
        if self.cameras:
            self.cameras.close()
