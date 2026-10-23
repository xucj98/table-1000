"""Position-triggered two-state pen approximation; no pose or velocity writes.

A full button compression arms one toggle, completed only on button return.
Native finite-force drives carry reaction to the free shell. Not a ratchet model.
"""
from omni.kit.scripting import BehaviorScript
import numpy as np

class ClickBehavior(BehaviorScript):
    def on_init(self):
        self.subscription = None
        self.session = self.instance = None
        self.armed = self.extended = False
        self.toggle_count = self.callback_count = 0
        self.error = None
        path = str(self.prim_path)
        self.replica = '/env_' in path and '/env_0/' not in path
        if self.replica:
            return
        # Build the Isaac views while stopped, before articulation startup.
        import omni.usd
        from table_1000.physics.simulation import session_for
        self.session = session_for(omni.usd.get_context().get_stage())
        self.instance = self.session.register(str(self.prim_path), name=str(self.prim_path))

    def on_play(self):
        # The official component also calls on_play when resuming a pause.
        if self.replica or self.subscription is not None:
            return
        from omni.physx import get_physx_interface
        self.extended = bool(self.session.stage.GetPrimAtPath(self.prim_path).GetAttribute('table1000:click:initialExtended').Get())
        self.extended_by_environment = np.full(self.session.envs, self.extended, dtype=bool)
        self.armed_by_environment = np.zeros(self.session.envs, dtype=bool)
        self.toggles_by_environment = np.zeros(self.session.envs, dtype=int)
        self.armed = False
        self.toggle_count = self.callback_count = 0
        self.error = None
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(self._step, True, 0)

    def _target(self):
        from table_1000.sim.isaaclab.asset_views import TensorView
        articulation = self.instance.articulation
        view = TensorView(articulation.view, articulation.device)
        names = view.shared_metatype.dof_names
        target = view.get_dof_position_targets()
        target[:, names.index(self.instance.joint('refill_slide').GetName())] = np.where(self.extended_by_environment, -.003, 0.)
        view.set_dof_position_targets(target, np.arange(self.session.envs, dtype=np.int32))

    def _step(self, dt):
        if self.replica or not self.session.active or not self.session.ready:
            return
        try:
            from table_1000.sim.isaaclab.asset_views import TensorView
            articulation = self.instance.articulation
            view = TensorView(articulation.view, articulation.device)
            index = view.shared_metatype.dof_names.index(self.instance.joint('button_press').GetName())
            q = view.get_dof_positions()[:, index]
            self.armed_by_environment |= q <= -.0027
            toggle = self.armed_by_environment & (q >= -.0003)
            self.extended_by_environment ^= toggle
            self.armed_by_environment[toggle] = False
            self.toggles_by_environment += toggle
            self.extended = bool(self.extended_by_environment[0])
            self.armed = bool(self.armed_by_environment[0])
            self.toggle_count = int(self.toggles_by_environment[0])
            self._target()
            self.callback_count += 1
        except Exception as error:
            self.error = str(error)
            self.session.error = error

    def on_stop(self):
        if self.replica:
            return
        self.subscription = None
        self.armed = False
        self.toggle_count = self.callback_count = 0
        if self.instance is not None and self.session.stage.GetPrimAtPath(self.prim_path):
            self.extended = bool(self.session.stage.GetPrimAtPath(self.prim_path).GetAttribute('table1000:click:initialExtended').Get())
            self.extended_by_environment = np.full(self.session.envs,self.extended,dtype=bool)
            self.armed_by_environment = np.zeros(self.session.envs,dtype=bool)
            self.toggles_by_environment = np.zeros(self.session.envs,dtype=int)

    def on_destroy(self):
        if self.replica:
            return
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
