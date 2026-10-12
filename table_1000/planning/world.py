"""Planning-world collision objects for built assets and scene primitives.

Asset collision geometry comes from the built ``model.json``, the same export
the USD physics layer is authored over: every collision node of a rigid body
becomes a convex hull (or sphere) placed relative to that body. Shapes are
grouped into named planning objects so contact permissions can be granted per
part (for example only a drawer handle), and each object is posed from the live
pose of its body.
"""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh


def pose(transform):
    """``mplib.Pose`` of a 4x4 transform."""
    from mplib import Pose
    transform = np.asarray(transform, dtype=float)
    return Pose(transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]])


def model_shapes(model):
    """``[(body, part, T_body_shape, kind, data)]`` for every collision node.

    ``part`` is the collision node's parent (the named part); ``kind`` is
    ``'sphere'`` with a radius or ``'convex'`` with hull vertices in the shape
    frame. ``T_rest`` of each body in the asset frame is returned as well.
    """
    model = json.loads(Path(model).read_text()) if not isinstance(model, dict) else model
    worlds, owners, rest, shapes = {}, {}, {}, []
    for node in model['nodes']:
        name, parent = node['name'], node['parent']
        worlds[name] = worlds.get(parent, np.eye(4)) @ np.array(node['matrix'], dtype=float)
        owners[name] = name if node['rigid'] else owners.get(parent)
        if node['rigid']:
            rest[name] = worlds[name]
        if node.get('role') != 'collision':
            continue
        body = owners[name]
        transform = np.linalg.inv(worlds[body]) @ worlds[name]
        if node.get('shape') == 'SPHERE':
            shapes.append((body, parent, transform, 'sphere', float(node['radius'])))
        else:
            shapes.append((body, parent, transform, 'convex', np.asarray(node['vertices'], dtype=float)))
    return shapes, rest


def grow(vertices, padding):
    """Convex-hull vertices grown by ``padding`` per axis about their box centre."""
    vertices = np.asarray(vertices, dtype=float)
    low, high = vertices.min(0), vertices.max(0)
    centre, half = (low + high) / 2, np.maximum((high - low) / 2, 1e-9)
    return centre + (vertices - centre) * (half + padding) / half


def geometry(kind, data, padding=0.):
    """FCL shape grown by ``padding`` on every side.

    Convex hulls grow per axis of their own frame about their box centre, which
    is exact for the box-like parts of these assets.
    """
    from mplib.collision_detection import fcl
    if kind == 'sphere':
        return fcl.Sphere(data + padding)
    if kind == 'box':
        return fcl.Box(*(np.asarray(data) + 2 * padding))
    hull = trimesh.Trimesh(data).convex_hull
    vertices = grow(hull.vertices, padding) if padding else np.asarray(hull.vertices, dtype=float)
    return fcl.Convex(vertices, np.asarray(hull.faces, dtype=np.int32))


class PlanningObject:
    """Named group of shapes that moves with one body.

    ``body`` is the key of that body's live pose (``None`` for static objects).
    ``padded`` says whether the object is grown by the scene's padding.
    """

    def __init__(self, name, body, shapes, padded=True):
        self.name, self.body, self.raw, self.padded = name, body, shapes, padded
        self.cache = {}

    def fcl(self, world_pose, padding=0.):
        from mplib.collision_detection import fcl
        padding = padding if self.padded else 0.
        if padding not in self.cache:
            self.cache[padding] = [(transform, geometry(kind, data, padding)) for transform, kind, data in self.raw]
        shapes = self.cache[padding]
        objects = [fcl.CollisionObject(geom) for _, geom in shapes]
        return fcl.FCLObject(self.name, pose(world_pose), objects, [pose(t) for t, _ in shapes])


def asset_objects(model, prefix, group, padded=lambda name: True, merged=lambda name: False):
    """Planning objects of one asset instance.

    ``group(body, part)`` returns ``(object_name, frame_body)``; a shape on a
    body other than ``frame_body`` is placed by the bodies' rest transforms, so
    several bodies of a jointed but static assembly can form one object.
    Object names are prefixed with ``prefix``; ``padded(name)`` says whether
    the scene's padding applies to the object, ``merged(name)`` whether its
    shapes are replaced by their common convex hull (for a compact, convex
    assembly with many small parts).
    """
    shapes, rest = model_shapes(model)
    grouped = {}
    for body, part, transform, kind, data in shapes:
        name, frame = group(body, part)
        relative = np.linalg.inv(rest[frame]) @ rest[body] @ transform
        grouped.setdefault((prefix + name, frame), []).append((relative, kind, data))
    for key, entries in grouped.items():
        if merged(key[0][len(prefix):]):
            points = []
            for transform, kind, data in entries:
                local = data if kind == 'convex' else np.vstack([np.zeros(3), np.eye(3) * data, -np.eye(3) * data])
                points.append(local @ transform[:3, :3].T + transform[:3, 3])
            grouped[key] = [(np.eye(4), 'convex', np.vstack(points))]
    return [PlanningObject(name, prefix + frame, entries, padded(name)) for (name, frame), entries in grouped.items()]


def box_object(name, size, transform):
    """A static box centred on ``transform``: ``(object, transform)``."""
    return PlanningObject(name, None, [(np.eye(4), 'box', tuple(size))]), np.asarray(transform, dtype=float)


class PlanningScene:
    """Keeps a planner's world objects in sync with body poses.

    Poses are given in the world frame and stored in the planner's frame,
    ``frame`` being the planner-from-world transform. Padded objects are grown
    by ``padding``, which :meth:`set_padding` changes for later plans.
    """

    def __init__(self, planner, frame=np.eye(4)):
        self.planner, self.objects, self.frame = planner, {}, np.asarray(frame, dtype=float)
        self.padding, self.poses = 0., {}
        # Re-adding an object drops its matrix entries, so permissions are kept here.
        self.permissions = set()

    def add(self, obj, world_pose):
        self.objects[obj.name] = obj
        self.set_pose(obj.name, world_pose)

    def set_padding(self, padding):
        if padding != self.padding:
            self.padding = padding
            for name, world_pose in self.poses.items():
                if not self.planner.planning_world.is_object_attached(name):
                    self.set_pose(name, world_pose)

    def set_pose(self, name, world_pose):
        world = self.planner.planning_world
        if world.has_object(name):
            world.remove_object(name)
        self.poses[name] = np.asarray(world_pose, dtype=float)
        world.add_object(self.objects[name].fcl(self.frame @ world_pose, self.padding))
        others = sorted(other for pair in self.permissions if name in pair for other in pair if other != name)
        if others:
            world.get_allowed_collision_matrix().set_entry(name, others, True)

    def update(self, body_poses):
        """``body_poses`` maps a body key (``prefix + body``) to its world 4x4."""
        for name, obj in self.objects.items():
            if obj.body in body_poses and not self.planner.planning_world.is_object_attached(name):
                self.set_pose(name, body_poses[obj.body])

    def allow(self, names, others, allowed=True):
        """Grant or revoke contact between planning objects and robot links or other objects."""
        pairs = {frozenset((name, other)) for name in names for other in others}
        if allowed:
            self.permissions |= pairs
        else:
            self.permissions -= pairs
        acm = self.planner.planning_world.get_allowed_collision_matrix()
        acm.set_entry(list(names), list(others), allowed)
