"""PhysX contact reports kept as time intervals per labelled body pair (environment 0).

Isaac Lab's contact sensor reports forces, not how deep two bodies overlap,
which the robot checks need; this reads PhysX's contact reports directly. They
come from the bodies whose spawners activated contact reporting (the robot and
the scene's shapes, see :mod:`table_1000.sim.isaaclab.scene`).
"""


class Contacts:
    """Contact intervals of labelled bodies, timed by physics steps of ``dt``.

    ``labels`` maps prim path prefixes to names; pairs are kept when either side's
    label starts with one of ``watched``.
    """

    TOUCH = .0005

    def __init__(self, labels, watched, dt):
        import carb
        from omni.physx import get_physx_interface, get_physx_simulation_interface
        # Isaac Lab turns PhysX contact processing off unless one of its contact sensors needs it.
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
        self.labels, self.watched, self.dt = labels, tuple(watched), dt
        self.time, self.phase = 0., 'start'
        self.events = []
        self.open = {}
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.on_report)
        self.clock = get_physx_interface().subscribe_physics_step_events(self.on_step)

    def on_step(self, dt):
        self.time += dt

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
        self.subscription = self.clock = None

    def clear(self):
        """Forget every interval and restart the clock (after a reset)."""
        self.events.clear()
        self.open.clear()
        self.time = 0.
