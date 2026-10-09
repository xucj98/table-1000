"""Stage-one geometry checks. Convex SAT, sampled poses; no dynamics solver.

Supports BOX and explicit CONVEX_HULL colliders. Other shapes fail explicitly
until their analytic detection and matching preview geometry are implemented.
"""
import itertools
import numpy as np
import bpy
import bmesh
from mathutils import Vector


def require(ok, message):
    if not ok:
        raise ValueError(message)


def directions(vectors):
    result = []
    keys = set()
    for v in vectors:
        norm = np.linalg.norm(v)
        if norm < 1e-8:
            continue
        v = v / norm
        if next((x for x in v if abs(x) > 1e-7), 1) < 0:
            v = -v
        key = tuple(np.round(v, 6))
        if key not in keys:
            keys.add(key)
            result.append(v)
    return np.array(result).reshape((-1, 3))


def convex_data(obj):
    mesh = obj.data
    require(not obj.modifiers, f'Collision modifiers must be applied: {obj.name}')
    bm = bmesh.new()
    bm.from_mesh(mesh)
    try:
        require(len(bm.verts) >= 4 and all(e.is_manifold for e in bm.edges),
                f'Collider must be a closed manifold: {obj.name}')
        bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
        bm.normal_update()
        require(abs(bm.calc_volume()) > 1e-12, f'Zero volume collider: {obj.name}')
        points = np.array([tuple(v.co) for v in bm.verts])
        normals = []
        for face in bm.faces:
            n = np.array(face.normal)
            require(np.max((points - np.array(face.verts[0].co)) @ n) < 1e-6,
                    f'Nonconvex collider: {obj.name}')
            normals.append(n)
        edges = [np.array(e.verts[1].co) - np.array(e.verts[0].co) for e in bm.edges]
        if obj.rigid_body.collision_shape == 'BOX':
            lo, hi = points.min(axis=0), points.max(axis=0)
            corners = np.array(list(itertools.product(*zip(lo, hi))))
            require(len(points) == 8 and all(np.min(np.linalg.norm(corners-p, axis=1)) < 1e-6 for p in points),
                    f'BOX display mesh must match its collision box: {obj.name}')
        return points, directions(normals), directions(edges)
    finally:
        bm.free()


def sat_depth(a, b):
    """Separating-axis penetration depth, including full containment; metres."""
    pa, na, ea = a
    pb, nb, eb = b
    axes = np.concatenate((na, nb, directions(np.cross(ea[:, None, :], eb[None, :, :]).reshape(-1, 3))))
    axes = directions(axes)
    aa, bb = pa @ axes.T, pb @ axes.T
    depth = np.minimum(aa.max(axis=0)-bb.min(axis=0), bb.max(axis=0)-aa.min(axis=0))
    return float(depth.min())


class GeometryChecks:
    def __init__(self):
        scene = bpy.context.scene
        require(scene.unit_settings.system == 'METRIC' and abs(scene.unit_settings.scale_length-1) < 1e-8,
                'Use metre scene units and unit scale 1')
        self.bodies = [o for o in scene.objects if o.get('rigid_body_root')]
        require(self.bodies, 'No identified rigid bodies')
        self.colliders = []
        self.visuals = []
        self.owner = {}
        for body in self.bodies:
            require(body.rigid_body and body.rigid_body.collision_shape == 'COMPOUND', f'Expected Compound: {body.name}')
            require(body.type == 'MESH' and len(body.data.vertices) == 0, f'Compound frame must have no geometry: {body.name}')
            ancestor = body.parent
            while ancestor:
                require(ancestor not in self.bodies, f'Nested independent rigid bodies: {body.name}')
                ancestor = ancestor.parent
        for obj in scene.objects:
            require(all(abs(v-1) < 1e-6 for v in obj.scale), f'Unapplied scale: {obj.name}')
            if obj.type != 'MESH' or obj in self.bodies:
                continue
            root = obj.parent
            while root and root not in self.bodies:
                root = root.parent
            require(root is not None, f'Geometry without body owner: {obj.name}')
            self.owner[obj.name] = root.name
            role = obj.get('geometry_role')
            require(role in {'visual', 'collision'}, f'Missing geometry role: {obj.name}')
            if role == 'collision':
                require(obj.parent == root, f'Compound colliders must be direct children: {obj.name}')
                require(obj.rigid_body and obj.rigid_body.collision_shape in {'BOX', 'CONVEX_HULL'},
                        f'Unsupported collision shape: {obj.name}')
                require(obj.hide_render, f'Collision mesh must be hidden in asset render: {obj.name}')
                require(obj.rigid_body.use_margin and obj.rigid_body.collision_margin == 0,
                        f'Coarse geometric check requires zero margin: {obj.name}')
                self.colliders.append(obj)
            else:
                require(not obj.hide_render and not obj.rigid_body, f'Visual role mismatch: {obj.name}')
                require(len(obj.data.polygons) > 0 and obj.data.materials, f'Missing visual surface: {obj.name}')
                self.visuals.append(obj)
        for body in self.bodies:
            require(any(self.owner[o.name] == body.name for o in self.colliders), f'No collision geometry: {body.name}')
            require(any(self.owner[o.name] == body.name for o in self.visuals), f'No visual geometry: {body.name}')
        self.local = {obj.name: convex_data(obj) for obj in self.colliders}
        self.ignored = set()
        targets = set()
        joint_rows = []
        for obj in scene.objects:
            c = obj.rigid_body_constraint
            if not c:
                continue
            require(c.enabled and obj.get('preview_joint') and c.type in {'SLIDER','HINGE'}, f'Unsupported joint: {obj.name}')
            require(c.object1 in self.bodies and c.object2 in self.bodies and c.object1 != c.object2,
                    f'Invalid joint bodies: {obj.name}')
            require(c.object2.name not in targets, 'Preview requires a joint tree: duplicate child body')
            targets.add(c.object2.name)
            limited = c.use_limit_lin_x if c.type == 'SLIDER' else c.use_limit_ang_z
            limits = [c.limit_lin_x_lower,c.limit_lin_x_upper] if c.type == 'SLIDER' else [c.limit_ang_z_lower,c.limit_ang_z_upper]
            require(not limited or limits[0] <= 0 <= limits[1], f'Zero pose outside limits: {obj.name}')
            if c.disable_collisions:
                self.ignored.add(tuple(sorted((c.object1.name,c.object2.name))))
            joint_rows.append({'name':obj.name,'type':c.type,'body1':c.object1.name,'body2':c.object2.name,
                               'limits':limits if limited else None,'disable_collisions':c.disable_collisions})
        triangles = 0
        deps = bpy.context.evaluated_depsgraph_get()
        for obj in self.visuals:
            evaluated = obj.evaluated_get(deps)
            mesh = evaluated.to_mesh()
            mesh.calc_loop_triangles()
            triangles += len(mesh.loop_triangles)
            evaluated.to_mesh_clear()
        convex = [o for o in self.colliders if o.rigid_body.collision_shape == 'CONVEX_HULL']
        totals = {'visual_triangles':triangles, 'colliders':len(self.colliders),
                  'primitive_colliders':len(self.colliders)-len(convex), 'convex_colliders':len(convex),
                  'convex_vertices':sum(len(o.data.vertices) for o in convex),
                  'convex_faces':sum(len(o.data.polygons) for o in convex)}
        self.tolerance = float(scene.get('penetration_tolerance_m',0.0002))
        require(np.isfinite(self.tolerance) and self.tolerance >= 0, 'Invalid penetration tolerance')
        self.report = {'status':'running','scope':'geometry coarse check at video FPS; no dynamics or inter-frame guarantee',
                       'complexity':totals,'bodies':len(self.bodies),'joints':joint_rows,
                       'ignored_body_pairs':[list(p) for p in sorted(self.ignored)],
                       'penetration_tolerance_m':self.tolerance,'samples':[],'failures':[]}

    def sample(self, output, frame, values, transforms=None):
        world = {}
        for obj in self.colliders:
            p,n,e = self.local[obj.name]
            m = np.array(obj.matrix_world)
            rot = m[:3,:3]
            points = p @ rot.T + m[:3,3]
            world[obj.name] = (points,n @ rot.T,e @ rot.T)
        failures = []
        for a,b in itertools.combinations(self.colliders,2):
            owners = tuple(sorted((self.owner[a.name],self.owner[b.name])))
            if owners[0] == owners[1] or owners in self.ignored:
                continue
            aa,bb = world[a.name][0],world[b.name][0]
            if np.any(np.minimum(aa.max(axis=0),bb.max(axis=0))-np.maximum(aa.min(axis=0),bb.min(axis=0)) <= self.tolerance):
                continue
            depth = sat_depth(world[a.name],world[b.name])
            if depth > self.tolerance:
                failures.append({'output':output,'frame':frame,'bodies':list(owners),'colliders':[a.name,b.name],
                                 'penetration_m':depth})
        self.report['samples'].append({'output':output,'frame':frame,'joints':values.copy(),'penetrations':len(failures)})
        if transforms:
            self.report['samples'][-1]['transforms'] = transforms.copy()
        self.report['failures'].extend(failures)
        return failures
