"""Choices for turning the vendor URDF package into this robot source.

Read by ``scripts/robots/prepare_robot.py x1pro_rm001``. The raw package is the
one of x1pro (``ex001_render``, see ../x1pro/README.md); this robot takes its
rm001_g1_dvt1 variant and otherwise makes the same choices. Meshes that x1pro
uses too are referenced from ../x1pro/meshes/; only the gripper's are written
here.
"""

from table_1000.robots import source_dir, source_module

_X1PRO = source_module(source_dir('x1pro'), 'prepare')

BASE = 'x1pro'
# dvt1 chassis, hy_1.0 arms and the rm001_g1_dvt1 gripper with pivoting fingers.
RAW = _X1PRO.RAW
URDF = 'urdf/ex001_robot_rm001_g1_dvt1.urdf'
PIECES = _X1PRO.PIECES
# Each finger's inner face is a concave arc along its length (x): one hull or CoACD's pieces bridge
# it, up to 5 mm off the face, and a held object rests on the bridge; 8 slabs follow it to about 0.75 mm.
SLABS = {f'{side}_arm_gripper_{finger}_link': (0, 8) for side in ('left', 'right') for finger in ('left', 'right')}
NO_COLLISION = _X1PRO.NO_COLLISION


def add_pads(root):
    """Split a 2 mm lining from the inner collision surface, without thickening it.

    The vendor package has no separate pad mesh. This simulation approximation
    preserves its collision envelope: pad and metal partition each original
    convex slab, so only the pad carries high friction at the inner face.
    """
    from pathlib import Path
    import xml.etree.ElementTree as ET

    import numpy as np
    import trimesh
    from table_1000.robots.prepare import slabs

    def convex(points):
        # As in slabs(): avoid nearly coincident float32 vertices in exported meshes.
        return trimesh.convex.convex_hull(np.unique(np.round(points, 6), axis=0))

    source = Path(__file__).parent
    for finger, sign in (('left', 1), ('right', -1)):
        stem = f'gripper_{finger}_link'
        hulls = slabs(trimesh.load(source / f'meshes/visual/{stem}.ply'), 0, 8)
        # The first three slabs cover the pivot/shaft; lining starts at x=38 mm.
        for i, hull in enumerate(hulls):
            if i >= 3:
                candidates = np.where(sign * hull.face_normals[:, 1] < -.5)[0]
                face = candidates[np.argmax(hull.area_faces[candidates])]
                normal = hull.face_normals[face]
                distance = hull.vertices @ normal
                plane = distance.max() - .002
                edges = hull.vertices[hull.edges_unique]
                first, second = edges[:, 0] @ normal, edges[:, 1] @ normal
                cross = (first - plane) * (second - plane) < 0
                t = (plane - first[cross]) / (second[cross] - first[cross])
                intersections = edges[cross, 0] + t[:, None] * (edges[cross, 1] - edges[cross, 0])
                pad = convex(np.concatenate((hull.vertices[distance >= plane], intersections)))
                metal = convex(np.concatenate((hull.vertices[distance <= plane], intersections)))
                # Same stem as the original visual: source coverage checks both
                # parts together. Named URDF collisions identify the pad in USD.
                pad.export(source / f'meshes/collision/{stem}_pad{i}.ply')
                hull = metal
            hull.export(source / f'meshes/collision/{stem}_{i}.ply')
        for side in ('left', 'right'):
            link = root.find(f"link[@name='{side}_arm_gripper_{finger}_link']")
            for item in list(link):
                if 'pad' in item.get('name', ''):
                    link.remove(item)
            for i in range(3, 8):
                collision = ET.SubElement(link, 'collision', name=f'{finger}_pad_{i}')
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=f'meshes/collision/{stem}_pad{i}.ply')


def edit(root):
    _X1PRO.edit(root)
    add_pads(root)
