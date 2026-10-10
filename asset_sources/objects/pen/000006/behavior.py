"""Equivalent interface retention, not measured plastic deformation or a joint."""

import numpy as np


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


class CapFit:
    def __init__(self, context, bindings, parameters):
        self.context = context
        self.interface = bindings['interface']
        self.parameters = parameters
        self.reset()

    def reset(self):
        self.context.release_interface(self.interface)
        self.male = None
        self.metrics = {'engaged': False, 'opening': 0, 'retention_force': 0, 'radial_error': 0}

    def before_step(self, dt):
        pair = self.context.match_interface(self.interface, self.parameters)
        self.metrics['engaged'] = pair is not None
        self.metrics['retention_force'] = 0
        if pair is None:
            return
        female, male = pair
        self.male = male['key']
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
        self.context.apply_force(male['body'], -force, point, -torque)
        self.context.apply_force(female['body'], force, point, torque)
        self.metrics.update(opening=opening, radial_error=float(np.linalg.norm(relative[1:])),
                            retention_force=retention - barrier)

    def after_step(self, dt):
        if self.male is not None and self.context.interface_exists(self.male):
            female = self.context.get_interface(self.interface)
            male = self.context.get_interface(self.male)
            relative = male['rotation'].T @ (female['position'] - male['position'])
            self.metrics['opening'] = -float(relative[0])
            self.metrics['radial_error'] = float(np.linalg.norm(relative[1:]))

    def close(self):
        self.context.release_interface(self.interface)
        self.male = None
