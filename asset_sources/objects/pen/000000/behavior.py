"""Position-triggered two-state pen approximation; no pose or velocity writes.

A full button compression arms one toggle, completed only on button return.
Native finite-force drives carry reaction to the free shell. Not a ratchet model.
"""
from omni.kit.scripting import BehaviorScript

class ClickBehavior(BehaviorScript):
    def on_init(self):
        self.subscription = None
        self.session = self.instance = None
        self.armed = self.extended = False
        self.toggle_count = self.callback_count = 0
        self.error = None
        # Build the Isaac views while stopped, before articulation startup.
        import omni.usd
        from table_1000.physics.simulation import session_for
        self.session = session_for(omni.usd.get_context().get_stage())
        self.instance = self.session.register(str(self.prim_path), name=str(self.prim_path))

    def on_play(self):
        # The official component also calls on_play when resuming a pause.
        if self.subscription is not None:
            return
        from omni.physx import get_physx_interface
        self.extended = bool(self.session.stage.GetPrimAtPath(self.prim_path).GetAttribute('table1000:click:initialExtended').Get())
        self.armed = False
        self.toggle_count = self.callback_count = 0
        self.error = None
        self._target()
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(self._step, True, 0)

    def _target(self):
        from pxr import UsdPhysics
        UsdPhysics.DriveAPI(self.instance.joint('refill_slide'), 'linear').GetTargetPositionAttr().Set(-.003 if self.extended else 0.)

    def _step(self, dt):
        if not self.session.active or not self.session.ready:
            return
        try:
            from table_1000.physics.joints import frames
            q = frames(self.instance, 'button_press')['position']
            if q <= -.0027:
                self.armed = True
            elif self.armed and q >= -.0003:
                self.extended = not self.extended
                self.armed = False
                self.toggle_count += 1
                self._target()
            self.callback_count += 1
        except Exception as error:
            self.error = str(error)
            self.session.error = error

    def on_stop(self):
        self.subscription = None
        self.armed = False
        self.toggle_count = self.callback_count = 0
        if self.instance is not None and self.session.stage.GetPrimAtPath(self.prim_path):
            self.extended = bool(self.session.stage.GetPrimAtPath(self.prim_path).GetAttribute('table1000:click:initialExtended').Get())
            self._target()

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
