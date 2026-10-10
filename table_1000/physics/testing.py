"""Python asset experiments with independent instances and native fixtures."""

from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from table_1000.assets.references import expand_mapping
from table_1000.physics.simulation import session_for, rotation as quaternion_matrix


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
    def __init__(self, asset_path, *, name, render=False, app=None, warmup=False):
        import omni.usd
        from isaacsim.core.api import World
        World.clear_instance()
        omni.usd.get_context().new_stage()
        self.app, self.name, self.render, self.warmup = app, name, render, warmup
        self.asset_path = Path(asset_path).resolve()
        self.world = World(physics_dt=.004, rendering_dt=.04, stage_units_in_meters=1, backend='numpy', device='cpu')
        pc = self.world.get_physics_context()
        pc.enable_gpu_dynamics(False);pc.set_solver_type('TGS');pc.set_broadphase_type('MBP');pc.enable_ccd(True)
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
        self.initialized = False
        self.integrating_seconds = self.preparation_seconds = self.warmup_seconds = 0.
        self.video_started = None
        self.session.actions = self.apply_actions

    def spawn(self, asset_path, *, name, position=(0, 0, 0), rotation=(1, 0, 0, 0)):
        return self.session.load(asset_path, name, position, rotation)

    def simulation(self, *, dt=.004, gravity=(0, 0, -9.81)):
        assert not self.initialized
        self.dt, self.gravity = dt, gravity
        self.world.set_simulation_dt(physics_dt=dt, rendering_dt=.04)
        from pxr import Gf, UsdPhysics
        pc = self.world.get_physics_context()
        scene = UsdPhysics.Scene.Get(self.session.stage, pc.prim_path)
        magnitude = np.linalg.norm(gravity)
        scene.CreateGravityMagnitudeAttr(float(magnitude))
        scene.CreateGravityDirectionAttr(Gf.Vec3f(*(np.asarray(gravity)/magnitude if magnitude else [0,0,-1])))

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
        self.session.time = 0.;self.session.steps = 0
        self.session.callback_steps = 0
        self.session.refresh()
        self.action_values.clear()
        self.initialized = True
        self.preparation_seconds += time.perf_counter()-started

    @contextmanager
    def fixed(self, body, *, position=None, rotation=None):
        self.ensure_initialized()
        from pxr import Gf, UsdPhysics
        state = body.state
        pos = state['position'] if position is None else np.asarray(position)
        q = state['quaternion'] if rotation is None else np.asarray(rotation)
        assert np.allclose(pos,state['position'],atol=1e-6) and np.allclose(quaternion_matrix(q),state['rotation'],atol=1e-6)
        path = '/World/Fixtures/fixture_' + str(self.fixture_counter)
        self.fixture_counter += 1
        joint = UsdPhysics.FixedJoint.Define(self.session.stage,path)
        if body.instance.articulation:
            joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateBody1Rel().SetTargets([body.path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*pos));joint.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(Gf.Quatf(1))
        try:
            yield joint
        finally:
            self.session.stage.RemovePrim(path)

    @contextmanager
    def spring(self, body, *, position, rotation=(1,0,0,0), linear_stiffness, linear_damping,
               angular_stiffness, angular_damping):
        self.ensure_initialized()
        from pxr import Gf, UsdPhysics
        path = '/World/Fixtures/fixture_' + str(self.fixture_counter)
        self.fixture_counter += 1
        joint = UsdPhysics.Joint.Define(self.session.stage,path)
        joint.CreateBody1Rel().SetTargets([body.path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*position));joint.CreateLocalRot0Attr(Gf.Quatf(rotation[0],Gf.Vec3f(*rotation[1:])))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(Gf.Quatf(1))
        for axis in ['transX','transY','transZ','rotX','rotY','rotZ']:
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis)
            limit.CreateLowAttr(-float('inf'));limit.CreateHighAttr(float('inf'))
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis)
            factor = 180/np.pi if axis.startswith('rot') else 1
            drive.CreateTypeAttr('force');drive.CreateTargetPositionAttr(0)
            drive.CreateStiffnessAttr((angular_stiffness if factor != 1 else linear_stiffness)/factor)
            drive.CreateDampingAttr((angular_damping if factor != 1 else linear_damping)/factor)
        try:
            yield joint
        finally:
            self.session.stage.RemovePrim(path)

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
            if self.warmup:
                saved = {name:a.snapshot() for name,a in self.session.instances.items()}
                start = time.perf_counter()
                for _ in range(round(1/self.dt)): self.session.step(self.world)
                self.warmup_seconds = time.perf_counter()-start
                start = time.perf_counter()
                self.session.active = False
                # Restore native fixture targets before rebuilding physics.
                # Otherwise a warmup kinematic clamp retains its moved pose
                # while the constrained asset is restored to its initial pose.
                if self.session.actions:
                    self.session.actions(0.,self.dt)
                self.world.reset();self.session.initialize()
                for name,state in saved.items():
                    self.session.instances[name].restore(state)
                self.session.steps = 0;self.session.time = 0.;self.session.refresh()
                self.session.callback_steps = 0
                self.preparation_seconds += time.perf_counter()-start
            # The first sample precedes any experiment step. Warmup actions
            # must not appear as forces applied at the restored t=0 state.
            self.action_values.clear()
            self.sample()
            if self.render and self.camera_config:
                from table_1000.physics.video import Camera
                self.camera_instance = Camera(self.session.stage,self.camera_config)
                start = time.perf_counter();self.camera_instance.capture(self);self.preparation_seconds += time.perf_counter()-start
                self.video_started = time.perf_counter()
                self.images.append(self.camera_instance.capture(self))
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
        self.world.stop()
        for instance in list(self.session.instances.values()): self.session.stage.RemovePrim(instance.path)
        if self.app:
            for _ in range(5): self.app.update()
        self.session.close()
