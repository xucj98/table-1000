"""Official component for the marker's equivalent cap retention forces.

This is not measured plastic deformation. Parameters and force law are retained
from the previous portable pen checkpoint; no pose is written during motion.
"""
import numpy as np
from omni.kit.scripting import BehaviorScript
from omni.physx import get_physx_interface
from table_1000.physics.simulation import session_for


PARAMETERS = {'static_retention_N': 2, 'dynamic_retention_N': 1.4, 'engagement_length_m': 0.026, 'radial_tolerance_m': 0.002, 'axis_cosine_min': 0.98, 'seating_barrier_N': 3, 'seating_width_m': 0.002, 'radial_stiffness': 3000, 'radial_damping': 2, 'angular_stiffness': 0.015, 'angular_damping': 0.0002}
_PEERS = {}


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


class CapFit(BehaviorScript):
    def on_init(self):
        self.session = session_for(self.stage)
        self.instance = self.session.register(str(self.prim_path))
        self.subscription = None
        self.male = None
        self.last_male = self
        self.parameters = {name: self.prim.GetAttribute("table1000:capFit:"+name).Get()
                           if self.prim.GetAttribute("table1000:capFit:"+name).HasAuthoredValueOpinion() else value
                           for name,value in PARAMETERS.items()}
        self.compatible = self.prim.GetAttribute("table1000:capFit:compatible").Get() or "marker_18mm"
        self.metrics = {'engaged': False, 'opening': 0., 'retention_force': 0., 'radial_error': 0.}
        self.callback_steps = 0

    def on_play(self):
        self.callback_steps = 0
        self.male = None
        self.last_male = self
        self.metrics.update(engaged=False, opening=0., radial_error=0., retention_force=0.)
        _PEERS[(self.session.key, self.instance.path)] = self
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(self.before_step, True, 0)

    def on_stop(self):
        self.subscription = None
        _PEERS.pop((self.session.key, self.instance.path), None)
        self.male = None
        self.metrics.update(engaged=False, retention_force=0.)
        for peer in _PEERS.values():
            if peer.male is self:
                peer.male = None
            if peer.last_male is self:
                peer.last_male = peer
        self.session.ready = False

    def on_destroy(self):
        self.on_stop()
        self.session.unregister(self.instance.path)
        if not self.session.instances:
            self.session.close()

    def interface(self, male):
        body = self.instance.body('body' if male else 'cap')
        state = body.state
        offset = np.array([-.032, 0, 0]) if male else np.zeros(3)
        return {'body': body, 'position': state['position'] + state['rotation'] @ offset,
                'rotation': state['rotation'], 'state': state}

    def match(self):
        female = self.interface(False)
        peers = [peer for (stage,path),peer in _PEERS.items() if stage == self.session.key]
        candidates = ([self.male] if self.male in peers else []) + [peer for peer in peers if peer is not self.male]
        for peer in candidates:
            if peer.compatible != self.compatible or any(other is not self and other.male is peer for other in peers):
                continue
            male = peer.interface(True)
            relative = male['rotation'].T @ (female['position'] - male['position'])
            p = self.parameters
            if (-.001 < -relative[0] < p['engagement_length_m'] and np.linalg.norm(relative[1:]) < p['radial_tolerance_m']
                    and male['rotation'][:,0] @ female['rotation'][:,0] > p['axis_cosine_min']):
                self.male = peer
                self.last_male = peer
                return female, male
        self.male = None
        return None

    def before_step(self, dt):
        if not self.session.active or not self.session.ready:
            return
        try:
            self.apply_retention(dt)
            self.callback_steps += 1
        except Exception as error:
            self.session.error = error

    def apply_retention(self, dt):
        pair = self.match()
        self.metrics['engaged'] = pair is not None
        self.metrics['retention_force'] = 0.
        if pair is None:
            return
        female, male = pair
        a, b = male['state'], female['state']
        axis = male['rotation'][:, 0]
        relative = male['rotation'].T @ (female['position'] - male['position'])
        opening = -float(relative[0])
        point = female['position']
        arms = [point - state['com'] for state in (a, b)]
        angular = [state['velocity'][3:] for state in (a, b)]
        point_v = [state['velocity'][:3] + np.cross(omega, arm)
                   for state, omega, arm in zip((a, b), angular, arms)]
        inverse = [state['inverse_inertia'] for state in (a, b)]
        c0, c1 = [skew(arm) for arm in arms]
        mobility = (a['inverse_mass'] + b['inverse_mass']) * np.eye(3) - c0 @ inverse[0] @ c0 - c1 @ inverse[1] @ c1
        cross = inverse[0] @ c0 + inverse[1] @ c1
        full = np.block([[mobility, cross.T], [cross, inverse[0] + inverse[1]]])
        acceleration = []
        angular_acceleration = []
        for state, arm in zip((a, b), arms):
            alpha = state['inverse_inertia'] @ state['external_torque']
            angular_acceleration.append(alpha)
            acceleration.append(state['inverse_mass'] * state['external_force'] + np.cross(alpha, arm))
        relative_v = point_v[1] - point_v[0]
        predicted_v = relative_v + dt * (acceleration[1] - acceleration[0])
        predicted_omega = angular[1] - angular[0] + dt * (angular_acceleration[1] - angular_acceleration[0])
        speed = -float(relative_v @ axis)
        predicted_speed = -float(predicted_v @ axis)
        p = self.parameters
        width = p['seating_width_m']
        barrier = p['seating_barrier_N'] * max(0, 1 - abs(opening - width / 2) / (width / 2))
        limit = p['static_retention_N'] if abs(speed) < .0005 else p['dynamic_retention_N']
        effective_mass = 1 / float(axis @ mobility @ axis)
        retention = float(np.clip(effective_mass * predicted_speed / dt + barrier, -limit, limit))
        axial = (retention - barrier) * axis
        project = np.eye(3) - np.outer(axis, axis)
        radial = male['rotation'] @ np.r_[0, relative[1:]]
        gain = np.zeros((6, 6))
        gain[:3, :3] = (p['radial_damping'] + p['radial_stiffness'] * dt) * project
        gain[3:, 3:] = p['angular_damping'] * np.eye(3) + dt * p['angular_stiffness'] * project
        drive = np.r_[-p['radial_stiffness'] * radial,
                      p['angular_stiffness'] * np.cross(female['rotation'][:, 0], axis)]
        wrench = np.linalg.solve(np.eye(6) + dt * gain @ full,
                                drive - gain @ (np.r_[predicted_v, predicted_omega] + dt * full @ np.r_[axial, np.zeros(3)]))
        force, torque = wrench[:3] + axial, wrench[3:]
        # Equal opposite forces at the SAME point conserve total force/moment.
        self.session.force(male['body'], -force, point, -torque, external=False)
        self.session.force(female['body'], force, point, torque, external=False)
        self.metrics.update(opening=opening, radial_error=float(np.linalg.norm(relative[1:])),
                            retention_force=retention - barrier)


    def observations(self):
        values = self.metrics.copy()
        female, male = self.interface(False), self.last_male.interface(True)
        relative = male['rotation'].T @ (female['position'] - male['position'])
        values.update(opening=-float(relative[0]), radial_error=float(np.linalg.norm(relative[1:])))
        return values
