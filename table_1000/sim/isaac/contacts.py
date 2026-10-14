"""PhysX contact reports kept as time intervals per labelled body pair."""


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

    def clear(self):
        self.events.clear()
        self.open.clear()
