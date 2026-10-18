"""Python asset experiments with independent instances and native fixtures."""

from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from table_1000.assets.references import expand_mapping
from table_1000.physics.simulation import session_for


def keyframes(frames, dt):
    result, previous = [], {}
    for frame in frames:
        t = frame['time']
        if t < 0 or abs(t / dt - round(t / dt)) > 1e-7 or result and t <= result[-1]['time']:
            raise ValueError('Action times must increase and align with the physics step')
        previous.update(deepcopy(frame))
        result.append(deepcopy(previous))
    if len(result) < 2:
        raise ValueError('An action needs at least two keyframes')
    return result


def interpolate(frames, t):
    for a, b in zip(frames, frames[1:]):
        if a['time'] <= t < b['time']:
            u = (t-a['time']) / (b['time']-a['time'])
            values = {}
            for key in a.keys() - {'time'}:
                if key == 'rotation':
                    q = np.array([a[key], b[key]])[:, [1, 2, 3, 0]]
                    values[key] = Slerp([0, 1], Rotation.from_quat(q))([u]).as_quat()[0][[3, 0, 1, 2]]
                else:
                    values[key] = np.asarray(a[key])*(1-u) + np.asarray(b[key])*u
            return values
    return None


class TestContext:
    """One isolated experiment. Importing test definitions starts no simulator."""
    def __init__(self, asset_path, *, name, render=False, app=None, physics='cpu'):
        import omni.usd
        from isaaclab.sim import SimulationCfg, SimulationContext, PhysxCfg
        SimulationContext.clear_instance()
        omni.usd.get_context().new_stage()
        self.app, self.name, self.render = app, name, render
        self.physics = physics
        self.asset_path = Path(asset_path).resolve()
        if physics == 'cpu':
            import carb
            settings = carb.settings.get_settings()
            settings.set_bool('/physics/updateToUsd', True)
            settings.set_bool('/physics/updateVelocitiesToUsd', True)
        self.world = SimulationContext(SimulationCfg(
            device='cpu' if physics == 'cpu' else 'cuda:0', dt=.004, render_interval=10,
            use_fabric=physics == 'gpu',
            physx=PhysxCfg(solver_type=1, enable_ccd=physics == 'cpu',
                           enable_external_forces_every_iteration=physics == 'gpu',
                           min_position_iteration_count=64 if physics == 'gpu' else 1,
                           min_velocity_iteration_count=16 if physics == 'gpu' else 0)))
        pc = self.world.get_physics_context()
        if physics == 'cpu':
            pc.set_broadphase_type('MBP')
        self.session = session_for(omni.usd.get_context().get_stage())
        self.session.active = False
        self.asset = self.spawn(self.asset_path, name='asset')
        self.dt, self.gravity = .004, (0, 0, -9.81)
        self.initials, self.actions, self.action_values = {}, {}, {}
        self.observers, self.units, self.plot_columns = {}, {}, []
        self.rows, self.images = [], []
        self.camera_config = None
        self.camera_instance = None
        self.fixture_counter = 0
        self.fixtures = []
        self.session.fixtures = self.fixtures
        self.initialized = False
        self.integrating_seconds = self.preparation_seconds = 0.
        self.video_started = self.experiment_started = None
        self.session.actions = self.apply_actions

    def spawn(self, asset_path, *, name, position=(0, 0, 0), rotation=(1, 0, 0, 0)):
        return self.session.load(asset_path, name, position, rotation)

    def simulation(self, *, dt=.004, gravity=(0, 0, -9.81), gpu_solver=None):
        assert not self.initialized
        self.dt, self.gravity = dt, gravity
        self.world.set_simulation_dt(physics_dt=dt, rendering_dt=.04)
        from pxr import Gf, UsdPhysics
        pc = self.world.get_physics_context()
        scene = UsdPhysics.Scene.Get(self.session.stage, pc.prim_path)
        magnitude = np.linalg.norm(gravity)
        scene.CreateGravityMagnitudeAttr(float(magnitude))
        scene.CreateGravityDirectionAttr(Gf.Vec3f(*(np.asarray(gravity)/magnitude if magnitude else [0,0,-1])))
        if self.physics == 'gpu' and gpu_solver is not None:
            from pxr import PhysxSchema
            assert gpu_solver in ('PGS', 'TGS')
            api = PhysxSchema.PhysxSceneAPI(scene.GetPrim())
            api.CreateSolverTypeAttr(gpu_solver)
            api.CreateEnableExternalForcesEveryIterationAttr(gpu_solver == 'TGS')
            self.world.cfg.physx.solver_type = int(gpu_solver == 'TGS')
            self.world.cfg.physx.enable_external_forces_every_iteration = gpu_solver == 'TGS'

    def initial(self, instance=None, *, position=(0,0,0), rotation=(1,0,0,0), linear_velocity=(0,0,0),
                angular_velocity=(0,0,0), bodies=None, joints=None):
        assert not self.initialized
        from pxr import Gf, UsdGeom
        instance = instance or self.asset
        transform = UsdGeom.Xformable(self.session.stage.GetPrimAtPath(instance.path))
        ops = transform.GetOrderedXformOps()
        ops[0].Set(Gf.Vec3d(*position));ops[1].Set(Gf.Quatf(rotation[0], Gf.Vec3f(*rotation[1:])))
        self.initials[instance.name] = {'position': np.array(position), 'linear_velocity': np.array(linear_velocity),
            'angular_velocity': np.array(angular_velocity),
            'bodies': expand_mapping(bodies or {}, instance.body_names), 'joints': expand_mapping(joints or {}, instance.joint_names)}
        # Author body overrides before fixtures capture their initial frames.
        for name, values in self.initials[instance.name]['bodies'].items():
            prim = self.session.stage.GetPrimAtPath(instance.paths[name])
            cache = UsdGeom.XformCache()
            world = Gf.Transform(cache.GetLocalToWorldTransform(prim))
            if 'position' in values:
                world.SetTranslation(Gf.Vec3d(*values['position']))
            if 'rotation' in values:
                q = values['rotation']
                world.SetRotation(Gf.Rotation(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:]))))
            local = world.GetMatrix()*cache.GetLocalToWorldTransform(prim.GetParent()).GetInverse()
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(local)

    def ensure_initialized(self):
        if self.initialized:
            return
        started = time.perf_counter()
        if self.app:
            for _ in range(5): self.app.update()
        from pxr import Gf, UsdGeom
        cache = UsdGeom.XformCache()
        saved = {}
        for instance in self.session.instances.values():
            transforms = [Gf.Transform(cache.GetLocalToWorldTransform(self.session.stage.GetPrimAtPath(instance.paths[name])))
                          for name in instance.body_names]
            saved[instance.name] = (np.array([t.GetTranslation() for t in transforms], dtype=np.float32),
                np.array([[t.GetRotation().GetQuat().GetReal(), *t.GetRotation().GetQuat().GetImaginary()] for t in transforms], dtype=np.float32))
        self.session.active = False
        self.world.reset()
        self.session.initialize()
        from table_1000.physics.joints import set_initial
        for instance in self.session.instances.values():
            config = self.initials.get(instance.name, {})
            instance.restore_body_poses(*saved[instance.name])
            self.session.refresh()
            set_initial(instance, config.get('joints', {}))
            pos, quat = instance.view.get_world_poses()
            for name, values in config.get('bodies', {}).items():
                i = instance.indices[name]
                pos[i] = values.get('position', pos[i]);quat[i] = values.get('rotation', quat[i])
            instance.restore_body_poses(pos.astype(np.float32), quat.astype(np.float32))
            self.session.refresh()
            centers = np.array([instance.body(name).state['com'] for name in instance.body_names])
            omega, linear = config.get('angular_velocity', np.zeros(3)), config.get('linear_velocity', np.zeros(3))
            velocity = np.c_[linear + np.cross(omega, centers-config.get('position', np.zeros(3))), np.tile(omega,(len(pos),1))]
            for name, values in config.get('bodies', {}).items():
                i = instance.indices[name]
                velocity[i,:3] = values.get('linear_velocity', velocity[i,:3]);velocity[i,3:] = values.get('angular_velocity', velocity[i,3:])
            instance.restore_body_velocities(velocity.astype(np.float32))
            instance.initial_velocities = velocity
        for fixture in self.fixtures:
            fixture.initialize()
            if not fixture.locked:
                velocity = fixture.body.instance.initial_velocities[fixture.body.instance.indices[fixture.body.name]]
                state = fixture.body.state
                origin_velocity = velocity[:3] - np.cross(velocity[3:], state['com'] - state['position'])
                values = fixture.view.get_dof_velocities()
                values[0, fixture.order] = np.r_[fixture.basis.inv().apply(origin_velocity),
                                                fixture.basis.inv().apply(velocity[3:])]
                fixture.view.set_dof_velocities(values, fixture.indices)
        self.session.time = 0.;self.session.steps = 0
        self.session.callback_steps = 0
        self.world.forward()
        self.session.refresh()
        self.action_values.clear()
        self.initialized = True
        self.preparation_seconds += time.perf_counter()-started

    def usd_body_pose(self, body):
        """Read the declared initial pose without initializing physics."""
        from pxr import UsdGeom, Gf
        transform = Gf.Transform(UsdGeom.XformCache().GetLocalToWorldTransform(
            self.session.stage.GetPrimAtPath(body.path)))
        quat = transform.GetRotation().GetQuat()
        return np.asarray(transform.GetTranslation(), dtype=float), np.asarray([quat.GetReal(), *quat.GetImaginary()], dtype=float)

    def body_pose(self, body):
        position, quaternion = self.usd_body_pose(body)
        values = self.initials.get(body.instance.name, {}).get('bodies', {}).get(body.name, {})
        return np.asarray(values.get('position', position)), np.asarray(values.get('rotation', quaternion))

    @contextmanager
    def fixed(self, body, *, position=None, rotation=None):
        from table_1000.sim.isaaclab.fixtures import Grip
        grip = Grip(self, body, name='fixture_'+str(self.fixture_counter), locked=True)
        self.fixture_counter += 1
        grip.move(position, rotation)
        try:
            yield grip
        finally:
            grip.release()

    @contextmanager
    def spring(self, body, *, position, rotation=(1,0,0,0), linear_stiffness, linear_damping,
               angular_stiffness, angular_damping):
        from table_1000.sim.isaaclab.fixtures import Grip
        grip = Grip(self, body, name='fixture_'+str(self.fixture_counter),
                    linear_stiffness=linear_stiffness, linear_damping=linear_damping,
                    angular_stiffness=angular_stiffness, angular_damping=angular_damping,
                    linear_max_force=float('inf'), angular_max_force=float('inf'))
        self.fixture_counter += 1
        grip.move(position, rotation)
        try:
            yield grip
        finally:
            grip.release()

    def force(self, name, body, *, keyframes, frame='world', point_frame='world'):
        self._action(name,body,'force',keyframes,frame,point_frame)

    def torque(self, name, body, *, keyframes, frame='world'):
        self._action(name,body,'torque',keyframes,frame,'world')

    def _action(self,name,body,kind,frames,frame,point_frame):
        assert name not in self.actions
        self.actions[name] = {'body':body,'kind':kind,'frames':keyframes(frames,self.dt),'frame':frame,'point_frame':point_frame}
        for field,unit in [(kind,'N' if kind=='force' else 'N·m'),('point','m')]:
            if field == 'point' and kind != 'force': continue
            for i,axis in enumerate('xyz'):
                self.observe(f'{name}.{field}.{axis}', lambda n=name,f=field,i=i: self.action_values.get(n,{}).get(f,np.zeros(3))[i], unit=unit)

    def apply_actions(self,t,dt):
        self.action_values = {}
        for name, action in self.actions.items():
            body = action['body'];state = body.state
            value = interpolate(action['frames'],t)
            vector, point = np.zeros(3), state['com']
            if value is not None:
                vector = value[action['kind']]
                if action['frame'] == 'body': vector = state['rotation'] @ vector
                if 'point' in value:
                    point = value['point']
                    if action['point_frame'] == 'body': point = state['position'] + state['rotation'] @ point
                self.session.force(body, vector if action['kind']=='force' else np.zeros(3), point,
                                   vector if action['kind']=='torque' else None)
            self.action_values[name] = {action['kind']:vector,'point':point}

    def observe(self,name,getter,*,unit):
        self.observers[name] = getter
        self.units[name] = unit

    def observe_body(self,body,*,fields=('position','quaternion','linear_velocity','angular_velocity')):
        for field in fields:
            unit = {'position':'m','quaternion':'1','linear_velocity':'m/s','angular_velocity':'rad/s'}[field]
            for i,axis in enumerate('wxyz' if field=='quaternion' else 'xyz'):
                def get(field=field,i=i):
                    state = body.state
                    value = state['velocity'][:3] if field=='linear_velocity' else state['velocity'][3:] if field=='angular_velocity' else state[field]
                    return value[i]
                self.observe(f'{body.instance.name}.{body.name}.{field}.{axis}',get,unit=unit)

    def observe_joint(self,instance,name):
        from pxr import UsdPhysics
        from table_1000.physics.joints import frames
        prim = instance.joint(name)
        name = prim.GetCustomDataByKey('table1000:name')
        unit = 'm' if prim.IsA(UsdPhysics.PrismaticJoint) else 'rad'
        for field in ['position','velocity']:
            self.observe(f'{instance.name}.{name}.{field}',lambda field=field: frames(instance,name)[field],unit=unit+('/s' if field=='velocity' else ''))

    def plots(self,*columns):
        self.plot_columns = list(columns)

    def camera(self,*,position,target,up=(0,0,1),resolution=(960,480),fps=25,focal_length=28):
        self.camera_config = {'position':position,'target':target,'up':up,'resolution':resolution,'fps':fps,
                              'focal_length_mm':focal_length}

    def ground(self,*,z=0,static_friction=.35,dynamic_friction=.35,restitution=0,contact_offset=.02):
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
        stage = self.session.stage
        floor = UsdGeom.Cube.Define(stage,'/World/Ground');floor.CreateSizeAttr(1)
        transform = UsdGeom.Xformable(floor);transform.AddTranslateOp().Set(Gf.Vec3d(0,0,z-.05));transform.AddScaleOp().Set(Gf.Vec3f(4,4,.1))
        floor.CreateDisplayColorAttr([Gf.Vec3f(.46,.49,.53)]);UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(floor.GetPrim()).CreateContactOffsetAttr(contact_offset)
        material = UsdShade.Material.Define(stage,'/World/GroundMaterial')
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(static_friction);api.CreateDynamicFrictionAttr(dynamic_friction);api.CreateRestitutionAttr(restitution)
        UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material,materialPurpose='physics')
        self.ground_config = dict(z=z,static_friction=static_friction,dynamic_friction=dynamic_friction,restitution=restitution,contact_offset=contact_offset)

    def sample(self):
        self.rows.append({'time':self.session.time,**{name:float(getter()) for name,getter in self.observers.items()}})

    def run(self,duration):
        assert abs(duration/self.dt-round(duration/self.dt)) < 1e-7
        self.ensure_initialized()
        if not self.rows:
            # The first sample precedes any experiment step.
            self.action_values.clear()
            self.sample()
            if self.render and self.camera_config:
                from table_1000.physics.video import Camera
                self.camera_instance = Camera(self.session.stage,self.camera_config)
                start = time.perf_counter();self.camera_instance.capture(self);self.preparation_seconds += time.perf_counter()-start
                self.video_started = time.perf_counter()
                self.images.append(self.camera_instance.capture(self))
        if self.experiment_started is None:
            self.experiment_started = time.perf_counter()
        stride = round(1/(self.dt*(self.camera_config or {'fps':25})['fps']))
        assert abs(1/(self.dt*(self.camera_config or {'fps':25})['fps'])-stride) < 1e-7
        for _ in range(round(duration/self.dt)):
            start = time.perf_counter()
            self.session.step(self.world);self.sample()
            self.integrating_seconds += time.perf_counter()-start
            if self.camera_instance and self.session.steps % stride == 0:
                self.images.append(self.camera_instance.capture(self))

    def step(self):
        self.run(self.dt)

    def close(self):
        if self.camera_instance: self.camera_instance.close()
        self.world._disable_app_control_on_stop_handle = True
        self.world.stop()
        for instance in list(self.session.instances.values()): self.session.stage.RemovePrim(instance.path)
        if self.app:
            for _ in range(5): self.app.update()
        self.session.close()
        from isaaclab.sim import SimulationContext
        SimulationContext.clear_instance()
