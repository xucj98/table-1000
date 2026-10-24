"""Approved PC-01: per-instance rigid-lip interlock using native joint limits.

Half retraction cannot create a local opening in an indivisible rigid lip.
No dynamic pose, velocity, drive target, or test-dependent sequence is written.
"""
import math
from omni.kit.scripting import BehaviorScript

SLIDER = 'pencil_case.unzip'
FRONT = 'pencil_case.front_lip_open'
BACK = 'pencil_case.back_lip_open'
TRAVEL = .216
UNLOCK = .2158
RELOCK = .2154
PARK = .2156
CLOSED = math.radians(.5)
OPEN = math.radians(1.)
# Closed-state numerical play avoids snapping a closing lip to exactly zero.
BASE_LIMITS = {SLIDER: (0., TRAVEL), FRONT: (0., 45.), BACK: (-45., 0.)}


class PencilCaseInterlock(BehaviorScript):
    def on_init(self):
        import omni.usd
        from table_1000.physics.simulation import session_for
        self.subscription = None
        self.session = session_for(omni.usd.get_context().get_stage())
        self.instance = self.session.register(str(self.prim_path), name=str(self.prim_path))
        self.mode = None
        self.callback_count = self.transition_count = 0
        self.error = None

    def _limits(self, name, values):
        from pxr import UsdPhysics
        prim = self.instance.joint(name)
        joint = UsdPhysics.PrismaticJoint(prim) if name == SLIDER else UsdPhysics.RevoluteJoint(prim)
        joint.GetLowerLimitAttr().Set(values[0])
        joint.GetUpperLimitAttr().Set(values[1])

    def _mode(self, mode):
        if mode == self.mode:
            return
        self._limits(SLIDER, (PARK, TRAVEL) if mode == 'slider_locked' else BASE_LIMITS[SLIDER])
        self._limits(FRONT, (0., .5) if mode == 'lips_locked' else BASE_LIMITS[FRONT])
        self._limits(BACK, (-.5, 0.) if mode == 'lips_locked' else BASE_LIMITS[BACK])
        self.mode = mode
        self.transition_count += 1

    def on_play(self):
        if self.subscription is not None:
            return
        from omni.physx import get_physx_interface
        self.mode = None
        self.error = None
        self.callback_count = self.transition_count = 0
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(self._step, True, 0)

    def _step(self, dt):
        if not self.session.active or not self.session.ready:
            return
        try:
            from table_1000.physics.joints import frames
            s = frames(self.instance, SLIDER)['position']
            f = frames(self.instance, FRONT)['position']
            b = frames(self.instance, BACK)['position']
            angle = max(abs(f), abs(b))
            if self.mode is None:
                if s < UNLOCK and angle > CLOSED:
                    raise ValueError('PC-01 invalid initial state: open lips with incompletely retracted slider')
                self._mode('slider_locked' if angle > CLOSED else
                           'opening_allowed' if s >= UNLOCK else 'lips_locked')
            elif self.mode == 'lips_locked':
                if s >= UNLOCK:
                    self._mode('opening_allowed')
            elif self.mode == 'opening_allowed':
                if angle > OPEN:
                    if s < PARK:
                        raise ValueError('PC-01 simultaneous opening/closing left the parking interval')
                    self._mode('slider_locked')
                elif s <= RELOCK:
                    if angle > CLOSED:
                        raise ValueError('PC-01 close command before both lips entered closure tolerance')
                    self._mode('lips_locked')
            elif self.mode == 'slider_locked' and angle <= CLOSED:
                self._mode('opening_allowed')
            self.callback_count += 1
        except Exception as error:
            self.error = str(error)
            self.session.error = error

    def on_stop(self):
        self.subscription = None
        self.mode = None
        self.callback_count = self.transition_count = 0
        self.error = None
        if self.instance is not None and self.session.stage.GetPrimAtPath(self.prim_path):
            for name, limits in BASE_LIMITS.items():
                self._limits(name, limits)

    def on_destroy(self):
        self.on_stop()
        if self.session is not None:
            self.session.unregister(str(self.prim_path))
            if not self.session.instances:
                self.session.close()
        self.instance = self.session = None

    def on_pause(self):
        pass

    def on_update(self, current_time, delta_time):
        pass
