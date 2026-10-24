"""Engineering magnetic model for explicitly marked magnets and steel faces.

Official BehaviorScript/PhysX step callbacks, no joints, pose writes or velocity
resets. SI units. Each magnet pair is applied once, including intrinsic torques;
steel receives the opposite force at the projected surface point. This is a
low-speed force approximation, not a calibrated electromagnetic field solver.
"""
import numpy as np
from omni.kit.scripting import BehaviorScript

MOMENT_ATTR = 'table1000:magnet:moment'
STEEL_ATTR = 'table1000:magnet:steelHalfExtents'
SOFTENING_M = .002
TAPER_START_M = .025
CUTOFF_M = .035
CENTER_M = np.array([0., 0., .002])
PATCH_POINTS_M = np.array([[0., 0., 0.]] +
    [[.008*np.cos(i*np.pi/4), .008*np.sin(i*np.pi/4), 0.] for i in range(8)])
PATCH_WEIGHTS = np.array([.2] + [.1]*8)


def _value(prim, name):
    attr = prim.GetAttribute(name)
    return attr.Get() if attr else None


def dipole_pair(r, m1, m2):
    """Return force on 2 and intrinsic torques on 1/2 from one potential."""
    distance = float(np.linalg.norm(r))
    if distance >= CUTOFF_M:
        return np.zeros(3), np.zeros(3), np.zeros(3)
    # This domain is deep geometric overlap, not a useful softened near field.
    if distance < .003:
        raise RuntimeError('Magnetic centres closer than 3 mm: collision/model domain violation')
    s = np.sqrt(distance*distance + SOFTENING_M**2)
    a, b, d = np.dot(m1, r), np.dot(m2, r), np.dot(m1, m2)
    potential = 1e-7*(d/s**3-3*a*b/s**5)
    force = 1e-7*(3*(d*r+a*m2+b*m1)/s**5-15*a*b*r/s**7)
    torque1 = np.cross(m1, 1e-7*(3*r*b/s**5-m2/s**3))
    torque2 = np.cross(m2, 1e-7*(3*r*a/s**5-m1/s**3))
    u = max(0., (distance-TAPER_START_M)/(CUTOFF_M-TAPER_START_M))
    taper = 1-3*u*u+2*u*u*u
    derivative = (-6*u+6*u*u)/(CUTOFF_M-TAPER_START_M) if u else 0.
    return taper*force-potential*derivative*r/distance, taper*torque1, taper*torque2


class MagnetBehavior(BehaviorScript):
    def on_init(self):
        self.subscription = None
        self.session = None
        self.instance = None
        self.targets = []
        self.last_force = np.zeros(3)
        self.last_torque = np.zeros(3)
        self.last_pairs = []
        self.last_steel = None
        self.last_steel_diagnostic = None
        self.callback_count = 0
        self.error = None
        self.environment_diagnostics = []

    def on_play(self):
        import omni.usd
        from omni.physx import get_physx_interface
        from table_1000.physics.simulation import session_for
        self.subscription = None
        # One canonical component batches every physical environment separately.
        path = str(self.prim_path)
        if path.startswith('/World/envs/env_') and not path.startswith('/World/envs/env_0/'):
            return
        self.session = session_for(omni.usd.get_context().get_stage())
        self.session._magnet_steel_geometry = None
        self.session._magnet_environment_states = None
        self.instance = self.session.register(str(self.prim_path), name=str(self.prim_path))
        self.targets = []
        from pxr import UsdPhysics
        for prim in self.session.stage.Traverse():
            target_path = str(prim.GetPath())
            if target_path.startswith('/World/envs/env_') and not target_path.startswith('/World/envs/env_0/'):
                continue
            if _value(prim, STEEL_ATTR) is not None:
                target = {'path': str(prim.GetPath()), 'instance': None}
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    target['instance'] = self.session.register(str(prim.GetPath()), name=str(prim.GetPath()))
                self.targets.append(target)
        self.callback_count = 0
        self.error = None
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(self._step, True, 0)

    def on_stop(self):
        self.subscription = None
        self.targets = []
        if self.session is not None:
            self.session._magnet_steel_geometry = None
            self.session._magnet_environment_states = None
        self.environment_diagnostics = []
        self.last_force = np.zeros(3)
        self.last_torque = np.zeros(3)
        self.last_pairs = []
        self.last_steel = None
        self.last_steel_diagnostic = None
        self.callback_count = 0

    def on_destroy(self):
        self.on_stop()
        if self.session is not None:
            self.session.unregister(str(self.prim_path))
            # A dynamic ideal steel fixture is registered only for paired force
            # batching. Release its view when the last magnet leaves the stage.
            stage = self.session.stage
            magnets = [a for a in self.session.instances.values()
                       if stage.GetPrimAtPath(a.path) and _value(stage.GetPrimAtPath(a.path), MOMENT_ATTR) is not None]
            if not magnets:
                for a in list(self.session.instances.values()):
                    prim = stage.GetPrimAtPath(a.path)
                    if prim and _value(prim, STEEL_ATTR) is not None:
                        self.session.unregister(a.path)
            if not self.session.instances:
                self.session.close()
        self.instance = self.session = None

    def on_pause(self):
        pass

    def on_update(self, current_time, delta_time):
        pass

    def _step(self, dt):
        if self.session is None or not self.session.active or not self.session.ready:
            return
        try:
            self._forces()
            self.callback_count += 1
        except Exception as error:
            self.error = str(error)
            self.session.error = error

    def _forces(self):
        self.environment_diagnostics = []
        for environment in range(self.session.envs):
            self._environment_forces(environment)
            self.environment_diagnostics.append({
                'force_N': self.last_force.copy(), 'torque_N_m': self.last_torque.copy(),
                'pairs': self.last_pairs, 'steel': self.last_steel,
                'steel_diagnostic': self.last_steel_diagnostic})
        first = self.environment_diagnostics[0]
        self.last_force, self.last_torque = first['force_N'], first['torque_N_m']
        self.last_pairs, self.last_steel = first['pairs'], first['steel']
        self.last_steel_diagnostic = first['steel_diagnostic']

    def _state(self, instance, body, environment):
        session = self.session
        cached = getattr(session, '_magnet_environment_states', None)
        if cached is None or cached[0] != session.callback_steps:
            cached = (session.callback_steps, {})
            session._magnet_environment_states = cached
        if instance.path not in cached[1]:
            cached[1][instance.path] = instance.environment_states()
        values = cached[1][instance.path]
        position, rotation, com = (v[environment, instance.indices[body.name]] for v in values)
        return {'position': position, 'rotation': rotation, 'com': com}

    def _environment_forces(self, environment):
        session, own = self.session, self.instance
        def apply(body, force, point, torque=None):
            # Only this environment receives this calculation's force.
            forces, torques = np.zeros((session.envs, 3)), np.zeros((session.envs, 3))
            forces[environment] = force
            if torque is not None:
                torques[environment] = torque
            points = np.tile(point, (session.envs, 1))
            session.force(body, forces, points, torques, external=False)
        stage = session.stage
        body = own.body('magnet.body')
        state = self._state(own, body, environment)
        rotation = state['rotation']
        center = state['position']+rotation@CENTER_M
        moment = float(_value(stage.GetPrimAtPath(own.path), MOMENT_ATTR))*rotation[:, 2]
        self.last_force = np.zeros(3)
        self.last_torque = np.zeros(3)
        self.last_pairs = []
        self.last_steel = None
        self.last_steel_diagnostic = None
        for peer in list(session.instances.values()):
            prim = stage.GetPrimAtPath(peer.path)
            magnitude = _value(prim, MOMENT_ATTR) if prim else None
            if peer.path == own.path or magnitude is None:
                continue
            other = peer.body('magnet.body')
            other_state = self._state(peer, other, environment)
            other_center = other_state['position']+other_state['rotation']@CENTER_M
            other_moment = float(magnitude)*other_state['rotation'][:, 2]
            force2, torque1, torque2 = dipole_pair(other_center-center, moment, other_moment)
            force1 = -force2
            self.last_force += force1
            self.last_torque += torque1+np.cross(center-state['com'], force1)
            distance = float(np.linalg.norm(other_center-center))
            if distance < CUTOFF_M:
                self.last_pairs.append({'other': peer.path, 'distance_m': distance,
                    'force_N': force1.tolist(), 'intrinsic_torque_N_m': torque1.tolist(),
                    'applied_by_this_instance': own.path < peer.path})
            if own.path < peer.path:
                apply(body, force1, center, torque1)
                apply(other, force2, other_center, torque2)

        if not self.targets:
            return
        from pxr import Gf, UsdGeom
        # All magnets see one prepared physical state. Share only the USD
        # geometry reads within that step; refresh even moving static frames
        # on the next step. No force, pair or body-view cache is retained.
        cached = getattr(session, '_magnet_steel_geometry', None)
        if cached is None or cached[0] != session.callback_steps:
            cached = (session.callback_steps, {})
            session._magnet_steel_geometry = cached
        geometry = cached[1]
        points = state['position']+PATCH_POINTS_M@rotation.T
        candidates = []
        for target in self.targets:
            prim = stage.GetPrimAtPath(target['path'])
            if not prim:
                continue
            if target['path'] not in geometry:
                half = np.asarray(_value(prim, STEEL_ATTR), dtype=float)
                origin = basis = None
                if not target['instance']:
                    transform = Gf.Transform(UsdGeom.XformCache().GetLocalToWorldTransform(prim))
                    from table_1000.physics.simulation import rotation as qmatrix
                    q = transform.GetRotation().GetQuat()
                    origin = np.asarray(transform.GetTranslation())
                    basis = qmatrix([q.GetReal(), *q.GetImaginary()])
                geometry[target['path']] = (half, origin, basis)
            half, origin, basis = geometry[target['path']]
            target_body = None
            if target['instance']:
                instance = target['instance']
                target_body = instance.body(instance.body_names[0])
                target_state = self._state(instance, target_body, environment)
                origin, basis = target_state['position'], target_state['rotation']
            centre_local = (state['position']-origin)@basis
            distance_bound = float(np.hypot(centre_local[2], max(0., float(np.max(np.abs(centre_local[:2])-half)))))
            # Every patch point lies within 8 mm of the base centre. A face
            # >=16 mm away cannot exert force. Also retain any face that
            # could beat the current exact diagnostic distance.
            if (distance_bound >= .016 and self.last_steel_diagnostic is not None
                    and distance_bound-.008 >= self.last_steel_diagnostic['distance_m']):
                continue
            normal = basis[:, 2]
            facing = float(np.dot(rotation[:, 2], normal))
            angular = np.clip((facing-.5)/.5, 0, 1)**2
            local = (points-origin)@basis
            gaps = local[:, 2]
            margins = np.min(half-np.abs(local[:, :2]), axis=1)
            edge_u = np.clip(margins/.001, 0, 1)
            edge = edge_u*edge_u*(3-2*edge_u)
            cutoff_u = np.clip((gaps-.006)/.002, 0, 1)
            cutoff = 1-3*cutoff_u**2+2*cutoff_u**3
            magnitudes = PATCH_WEIGHTS*angular*edge*cutoff/(1+np.maximum(gaps, 0)/.002)**2
            magnitudes[(gaps < -.0005) | (gaps >= .008)] = 0
            # Read-only geometry/force diagnostics remain available after the
            # force cuts off, including back faces and out-of-range poses.
            distance = float(np.min(np.hypot(gaps, np.maximum(-margins, 0))))
            diagnostic = {'target': target['path'], 'distance_m': distance,
                'gaps_m': gaps.tolist(), 'forces_N': magnitudes.tolist(),
                'angular_factor': float(angular), 'edge_factors': edge.tolist(),
                'cutoff_factors': cutoff.tolist(), 'facing_cosine': facing}
            if self.last_steel_diagnostic is None or distance < self.last_steel_diagnostic['distance_m']:
                self.last_steel_diagnostic = diagnostic
            if np.any(magnitudes > 0):
                candidates.append((float(np.min(gaps[magnitudes > 0])), target['path'],
                    target_body, normal, points, gaps, magnitudes, facing))
        if not candidates:
            return
        _, path, target_body, normal, points, gaps, magnitudes, facing = min(candidates, key=lambda x: (x[0], x[1]))
        for point, gap, magnitude in zip(points, gaps, magnitudes):
            force = -normal*magnitude
            apply(body, force, point)
            if target_body:
                apply(target_body, -force, point-gap*normal)
            self.last_force += force
            self.last_torque += np.cross(point-state['com'], force)
        self.last_steel = {'target': path, 'gaps_m': gaps.tolist(), 'forces_N': magnitudes.tolist(),
            'facing_cosine': facing, 'dynamic_reaction': target_body is not None}
