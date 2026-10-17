"""Geometry of a built asset from its ``model.json``: rigid bodies, parts, collision shapes and joints.

The physics layer of every asset is authored over the same export, so these are
the shapes the simulator collides. Bodies and parts carry their
``table1000:name`` (``cabinet.drawer3``, ``cabinet.drawer3.handle``); poses are
in the asset frame at the built (rest) configuration unless a body frame is
named. Used by scene initialization (resting heights, penetration) and by
experts (part boxes, joint axes).
"""

import json
from pathlib import Path

import numpy as np

from table_1000.planning.world import geometry, model_shapes, pose


def transform_points(transform, points):
    return np.asarray(points) @ np.asarray(transform)[:3, :3].T + np.asarray(transform)[:3, 3]


class AssetGeometry:
    def __init__(self, model):
        model = json.loads(Path(model).read_text()) if not isinstance(model, dict) else model
        self.shapes, self.rest = model_shapes(model)
        self.joints = {joint['name']: joint for joint in model.get('joints', [])}

    @classmethod
    def of(cls, assets, asset):
        """Geometry of the built asset ``asset`` (path under ``assets``)."""
        return cls(Path(assets) / asset / 'model.json')

    @property
    def bodies(self):
        return list(self.rest)

    def _selected(self, part):
        return [shape for shape in self.shapes
                if part is None or shape[1] == part or shape[1].startswith(part + '.') or shape[0] == part]

    def points(self, part=None, frame=None):
        """Collision vertices of ``part`` (a body, a part and its sub-parts, or everything) in the
        frame of body ``frame`` (default: the asset frame), at rest. Spheres contribute the six
        points of their axis-aligned extent."""
        to_frame = np.eye(4) if frame is None else np.linalg.inv(self.rest[frame])
        points = []
        for body, _, transform, kind, data in self._selected(part):
            local = data if kind == 'convex' else np.vstack([np.eye(3) * data, -np.eye(3) * data])
            points.append(transform_points(to_frame @ self.rest[body] @ transform, local))
        if not points:
            raise KeyError(f'no collision geometry for {part!r}')
        return np.vstack(points)

    def bounds(self, part=None, frame=None):
        """``(low, high)`` corners of ``part``'s collision box, axis-aligned in ``frame``."""
        points = self.points(part, frame)
        return points.min(0), points.max(0)

    def stable_rotation(self, rotation):
        """The resting orientation of the asset on a flat support nearest to ``rotation`` (3x3), up
        to a turn about the vertical: one of the stable poses of its collision hull (uniform
        density), turned to best match ``rotation``. A pen lying across a vertex of its polygonal
        barrel rolls onto the nearest face."""
        import trimesh
        if not hasattr(self, '_stable'):
            hull = trimesh.convex.convex_hull(self.points())
            self._stable = [transform[:3, :3] for transform in trimesh.poses.compute_stable_poses(hull)[0]]
        best = None
        for stable in self._stable:
            difference = np.asarray(rotation) @ stable.T
            yaw = np.arctan2(difference[1, 0] - difference[0, 1], difference[0, 0] + difference[1, 1])
            c, s = np.cos(yaw), np.sin(yaw)
            turned = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ stable
            error = np.arccos(np.clip((np.trace(np.asarray(rotation).T @ turned) - 1) / 2, -1., 1.))
            if best is None or error < best[0]:
                best = (error, turned)
        return best[1]

    def lowest(self, rotation):
        """Lowest collision point (z, asset frame origin at 0) of the asset turned by ``rotation`` (3x3)."""
        return float((self.points() @ np.asarray(rotation).T)[:, 2].min())

    def joint_axis(self, joint):
        """``(body1, origin, axis)`` of ``joint`` (its ``model.json`` name): the frame body and, in it,
        the joint origin and the unit direction of positive motion (a slider's travel)."""
        entry = self.joints[joint]
        matrix = np.array(entry['matrix'], dtype=float)
        return entry['body1'], matrix[:3, 3], matrix[:3, 0] / np.linalg.norm(matrix[:3, 0])

    def body_poses(self, transform):
        """``{body: world pose}`` of every body with the asset frame at ``transform``, at rest."""
        return {body: np.asarray(transform) @ rest for body, rest in self.rest.items()}

    def collision_objects(self, body_poses, padding=0.):
        """FCL collision objects of every shape posed by ``body_poses`` (``{body: world pose}``)."""
        from mplib.collision_detection import fcl
        objects = []
        for body, _, transform, kind, data in self.shapes:
            obj = fcl.CollisionObject(geometry(kind, data, padding))
            obj.set_pose(pose(body_poses[body] @ transform))
            objects.append(obj)
        return objects


def penetration(objects_a, objects_b):
    """Deepest penetration (m, 0 when apart or only touching) between two lists of FCL objects."""
    from mplib.collision_detection import fcl
    request = fcl.CollisionRequest(num_max_contacts=4, enable_contact=True)
    depth = 0.
    for a in objects_a:
        for b in objects_b:
            result = fcl.collide(a, b, request)
            if result.is_collision():
                depth = max(depth, max(contact.penetration_depth for contact in result.get_contacts()))
    return depth


def box_object(size, transform):
    """FCL box of ``size`` centred on ``transform``."""
    from mplib.collision_detection import fcl
    obj = fcl.CollisionObject(fcl.Box(*map(float, size)))
    obj.set_pose(pose(transform))
    return obj
