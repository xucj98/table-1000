"""What a scene's ``initialize(layout)`` builds: object poses and joint values, sampled from ``layout.rng``.

Three layers, used as the scene needs them:

- **state**: :meth:`Layout.set_pose`, :meth:`Layout.set_joint` write the initial
  state directly, with no constraint;
- **geometry queries**: :meth:`Layout.surface_height`, :meth:`Layout.resting_pose`
  (how an object rests on a support: the stable orientation nearest the asked
  one and the height, from its collision geometry), :meth:`Layout.penetration`
  (against the table and the objects placed so far), plus every object's
  :class:`~table_1000.scenes.geometry.AssetGeometry`;
- **convenience**: :meth:`Layout.place` samples a resting pose in a region and
  rejects poses that penetrate what is already placed; :meth:`Layout.drop` lowers
  an object until it touches the table or a placed object (into a container,
  onto a stack), and :meth:`Layout.scatter` samples where to drop it.

An object's pose is that of its asset frame; joints keep their rest values
unless set, and the geometry follows the set joints (an open drawer). Dropped
objects are *loose*: they touch what they landed on but may not rest there yet
(a pen leaning into a cup), so they may still move when the scene first settles
(:mod:`table_1000.scenes.initial`): within what holds them, or anywhere on the
table with ``free_landing``. Whatever the layout misses, the checks after the
reset catch.
"""

import numpy as np

from table_1000.scenes.geometry import AssetGeometry, box_hull, colliding, penetration, transform_points

PENETRATION_LIMIT = .001    # m; resting contact is allowed
DROP_STEP, DROP_TOLERANCE = .003, .0002     # m


def yaw_pose(x, y, z, yaw=0.):
    transform = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    transform[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    transform[:3, 3] = [x, y, z]
    return transform


class SamplingFailed(RuntimeError):
    pass


class Layout:
    def __init__(self, scene, assets, rng):
        self.scene, self.rng = scene, rng
        self.geometry = {name: AssetGeometry.of(assets, spec['asset']) for name, spec in scene.OBJECTS.items()}
        self.poses, self.joints, self.supports, self.loose = {}, {}, {}, set()
        # Whether loose objects may land anywhere on the table (a messy desk), not only within what holds them.
        self.free_landing = False
        table = scene.TABLE
        top = np.eye(4)
        top[:3, 3] = [*table['center'], table['height'] - table['top_thickness'] / 2]
        self.table = box_hull((*table['size'], table['top_thickness']), top)

    # -- state -------------------------------------------------------------------
    def set_pose(self, name, transform, on='table', loose=False):
        """Put object ``name`` at world ``transform``, resting on ``on`` (the table, the ``'floor'``
        beside it or an object; the stability check measures it relative to its support);
        ``loose`` as if dropped there."""
        self.poses[name] = np.asarray(transform, dtype=float)
        self.supports[name] = on
        (self.loose.add if loose else self.loose.discard)(name)

    def set_joint(self, name, joint, value):
        """Joint ``joint`` (simulator name, e.g. ``cabinet_drawer3_slide``) of object ``name`` at ``value``."""
        self.joints.setdefault(name, {})[joint] = float(value)

    # -- geometry queries ----------------------------------------------------------
    def surface_height(self, on='table'):
        """Height of the support's top: the table top, the floor (0), or the top of a placed object's collision box."""
        if on == 'table':
            return float(self.scene.TABLE['height'])
        if on == 'floor':
            return 0.
        points = self.geometry[on].points()
        transform = self.poses[on]
        return float((points @ transform[:3, :3].T + transform[:3, 3])[:, 2].max())

    def resting_pose(self, name, xy, yaw=0., on='table', lift=0.):
        """Pose of ``name`` with its origin over ``xy``, resting on ``on``: the stable orientation
        nearest to the asset frame turned by ``yaw``, the lowest collision point ``lift`` above
        the support's top."""
        geometry = self.geometry[name]
        transform = yaw_pose(*xy, 0., yaw)
        transform[:3, :3] = geometry.stable_rotation(transform[:3, :3])
        transform[2, 3] = self.surface_height(on) + lift - geometry.lowest(transform[:3, :3])
        return transform

    def hulls(self, name, transform=None):
        """Collision hulls of ``name`` at ``transform`` (default: where it is placed), its joints as set."""
        geometry = self.geometry[name]
        return geometry.hulls(geometry.body_poses(self.poses[name] if transform is None else transform,
                                                  self.joints.get(name)))

    def world_points(self, name, transform=None):
        """Collision hull vertices of ``name`` in the world (default: where it is placed), its joints as set."""
        return np.vstack([hull.vertices for hull in self.hulls(name, transform)])

    def over_table(self, name, margin=.01):
        """Whether placed object ``name`` lies wholly over the table top, ``margin`` inside its edges."""
        (cx, cy), (width, depth) = self.scene.TABLE['center'], self.scene.TABLE['size']
        points = self.world_points(name)[:, :2]
        return bool((np.abs(points - (cx, cy)) <= (width / 2 - margin, depth / 2 - margin)).all())

    def frame(self, of=None):
        """World pose of a frame regions are given in: the world (``None``), or a placed object's
        asset frame turned about the vertical only."""
        if of is None:
            return np.eye(4)
        pose = self.poses[of]
        return yaw_pose(*pose[:3, 3], np.arctan2(pose[1, 0], pose[0, 0]))

    def penetration(self, name, transform):
        """Deepest penetration of ``name`` at ``transform`` into the table and the placed objects."""
        objects = self.hulls(name, transform)
        depth = penetration(objects, [self.table])
        for other in self.poses:
            if other != name:
                depth = max(depth, penetration(objects, self.hulls(other)))
        return depth

    # -- convenience -----------------------------------------------------------------
    def place(self, name, region, yaw=0., on='table', tries=50):
        """Rest ``name`` on ``on`` at a uniform position of its origin in ``region`` ``((x0, y0), (x1, y1))``
        and a yaw (a value, or a ``(low, high)`` range sampled uniformly), retrying positions that
        penetrate what is placed. Returns the pose."""
        (x0, y0), (x1, y1) = region
        for _ in range(tries):
            angle = self.rng.uniform(*yaw) if np.ndim(yaw) else float(yaw)
            transform = self.resting_pose(name, (self.rng.uniform(x0, x1), self.rng.uniform(y0, y1)), angle, on)
            if self.penetration(name, transform) <= PENETRATION_LIMIT:
                self.set_pose(name, transform, on)
                return transform
        raise SamplingFailed(f'no free pose for {name} in {region} after {tries} tries')

    def drop(self, name, xy, yaw=0., rotation=None, frame=None):
        """Lower ``name`` with its origin over ``xy`` until it touches the table or a placed object, and
        put it there as a loose object; ``xy`` and ``yaw`` are in ``frame`` (see :meth:`frame`).
        The orientation is the resting one nearest the asset frame turned by ``yaw``, or
        ``rotation`` (3x3, in ``frame``) as given, e.g. a pen tilted into a cup. Returns
        ``(pose, support)``: the support is what it touches, ``'table'`` or an object."""
        geometry = self.geometry[name]
        base = self.frame(frame)
        transform = base @ yaw_pose(*xy, 0., yaw)
        if rotation is None:
            transform[:3, :3] = geometry.stable_rotation(transform[:3, :3])
        else:
            transform[:3, :3] = base[:3, :3] @ np.asarray(rotation)
        points = transform_points(transform, geometry.points())
        low, high = points[:, :2].min(0), points[:, :2].max(0)
        below = [('table', [self.table], float(self.scene.TABLE['height']))]
        for other in self.poses:
            if other == name:
                continue
            others = self.world_points(other)
            if (others[:, :2].min(0) < high).all() and (others[:, :2].max(0) > low).all():
                below.append((other, self.hulls(other), float(others[:, 2].max())))

        def touching(z):
            """What ``name`` overlaps at height ``z``: an object rather than the table it stands on."""
            transform[2, 3] = z
            mine = self.hulls(name, transform)
            hits = [other for other, objects, _ in below if colliding(mine, objects)]
            return next((other for other in hits if other != 'table'), hits[0] if hits else None)

        offset = transform[2, 3] - points[:, 2].min()      # origin height above the lowest point
        free = max(top for *_, top in below) + offset + DROP_STEP
        if touching(free):
            raise SamplingFailed(f'{name} touches something above everything at {xy}')
        while not touching(free - DROP_STEP):
            free -= DROP_STEP
            if free < offset:       # down to the floor: nothing under it
                raise SamplingFailed(f'{name} has nothing under it at {xy}')
        hit = free - DROP_STEP
        while free - hit > DROP_TOLERANCE:
            middle = (free + hit) / 2
            if touching(middle):
                hit = middle
            else:
                free = middle
        support = touching(hit)     # what it meets first, not what a coarse step reached
        transform[2, 3] = free
        self.set_pose(name, transform, support, loose=True)
        return transform, support

    def scatter(self, name, region, yaw=0., rotation=None, frame=None, onto=None, tries=50):
        """:meth:`drop` ``name`` at a uniform position in ``region`` ``((x0, y0), (x1, y1))`` of
        ``frame`` with a yaw (a value or a ``(low, high)`` range), retrying until it lands on
        ``onto`` (an object or ``'table'``; a list of them; ``None``: anything), wholly over the
        table when it lands on the table. Returns the pose."""
        (x0, y0), (x1, y1) = region
        allowed = None if onto is None else {onto} if isinstance(onto, str) else set(onto)
        for _ in range(tries):
            angle = self.rng.uniform(*yaw) if np.ndim(yaw) else float(yaw)
            xy = (self.rng.uniform(x0, x1), self.rng.uniform(y0, y1))
            try:
                transform, support = self.drop(name, xy, angle, rotation, frame)
            except SamplingFailed:
                continue
            if (allowed is None or support in allowed) and (support != 'table' or self.over_table(name)):
                return transform
            self.remove(name)
        raise SamplingFailed(f'{name} did not land on {onto} in {region} after {tries} tries')

    def row(self, names, start, direction=0., yaw=0., gap=.01, frame=None):
        """Line ``names`` up on the table from ``start`` (x, y in ``frame``) along ``direction``
        (an angle in ``frame``), each turned by ``yaw`` from ``direction`` (a value or one per
        object) and resting on the table, ``gap`` apart between their collision outlines.
        Returns where the row ends (x, y in ``frame``), so rows can continue."""
        base = self.frame(frame)
        along = np.array([np.cos(direction), np.sin(direction)])
        position = np.asarray(start, dtype=float)
        yaws = yaw if np.ndim(yaw) else [yaw] * len(names)
        for name, turn in zip(names, yaws):
            rotation = self.geometry[name].stable_rotation(yaw_pose(0., 0., 0., direction + turn)[:3, :3])
            extent = self.geometry[name].points() @ rotation.T
            low, high = (extent[:, :2] @ along).min(), (extent[:, :2] @ along).max()
            origin = position - low * along
            transform = base @ yaw_pose(*origin, 0.)
            transform[:3, :3] = base[:3, :3] @ rotation
            transform[2, 3] = self.surface_height() - (extent[:, 2]).min()
            if self.penetration(name, transform) > PENETRATION_LIMIT:
                raise SamplingFailed(f'{name} does not fit in its row at {np.round(origin, 3).tolist()}')
            self.set_pose(name, transform)
            position = origin + (high + gap) * along
        return position

    def remove(self, name):
        """Take ``name`` out of the layout again (its joint values stay)."""
        for record in (self.poses, self.supports):
            record.pop(name, None)
        self.loose.discard(name)

    def penetrations(self):
        """``{(a, b): depth}`` of every penetrating pair among the placed objects and the table."""
        found, names = {}, list(self.poses)
        for i, name in enumerate(names):
            objects = self.hulls(name)
            pairs = [('table', [self.table])] + [(other, self.hulls(other)) for other in names[i + 1:]]
            for other, other_objects in pairs:
                depth = penetration(objects, other_objects)
                if depth > PENETRATION_LIMIT:
                    found[(name, other)] = depth
        return found
