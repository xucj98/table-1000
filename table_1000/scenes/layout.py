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
  rejects poses that penetrate what is already placed.

An object's pose is that of its asset frame; joints keep their rest values
unless set. Geometry here is at rest (joints at zero); whatever the layout
misses, the checks after the reset catch (:mod:`table_1000.scenes.initial`).
"""

import numpy as np

from table_1000.scenes.geometry import AssetGeometry, box_object, penetration

PENETRATION_LIMIT = .001    # m; resting contact is allowed


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
        self.poses, self.joints, self.supports = {}, {}, {}
        table = scene.TABLE
        top = np.eye(4)
        top[:3, 3] = [*table['center'], table['height'] - table['top_thickness'] / 2]
        self.table = box_object((*table['size'], table['top_thickness']), top)

    # -- state -------------------------------------------------------------------
    def set_pose(self, name, transform, on='table'):
        """Put object ``name`` at world ``transform``, resting on ``on`` (the table or an object;
        the stability check measures it relative to its support)."""
        self.poses[name] = np.asarray(transform, dtype=float)
        self.supports[name] = on

    def set_joint(self, name, joint, value):
        """Joint ``joint`` (simulator name, e.g. ``cabinet_drawer3_slide``) of object ``name`` at ``value``."""
        self.joints.setdefault(name, {})[joint] = float(value)

    # -- geometry queries ----------------------------------------------------------
    def surface_height(self, on='table'):
        """Height of the support's top: the table top, or the top of a placed object's collision box."""
        if on == 'table':
            return float(self.scene.TABLE['height'])
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

    def collision_objects(self, name, transform=None):
        geometry = self.geometry[name]
        return geometry.collision_objects(geometry.body_poses(self.poses[name] if transform is None else transform))

    def penetration(self, name, transform):
        """Deepest penetration of ``name`` at ``transform`` into the table and the placed objects."""
        objects = self.collision_objects(name, transform)
        depth = penetration(objects, [self.table])
        for other in self.poses:
            if other != name:
                depth = max(depth, penetration(objects, self.collision_objects(other)))
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

    def penetrations(self):
        """``{(a, b): depth}`` of every penetrating pair among the placed objects and the table."""
        found, names = {}, list(self.poses)
        for i, name in enumerate(names):
            objects = self.collision_objects(name)
            pairs = [('table', [self.table])] + [(other, self.collision_objects(other)) for other in names[i + 1:]]
            for other, other_objects in pairs:
                depth = penetration(objects, other_objects)
                if depth > PENETRATION_LIMIT:
                    found[(name, other)] = depth
        return found
