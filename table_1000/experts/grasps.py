"""Grasp frames, after ManiSkill3's motion-planning utilities (``build_grasp_pose``,
``compute_grasp_info_by_obb``).

TCP frames follow the robots' convention (see ``asset_sources/robots/*/robot.py``):
+X is the approach direction, +Y the direction in which the fingers open (the
closing axis), +Z completes the frame. Boxes are ``(T_world_box, size)``, for
example from :meth:`~table_1000.experts.context.Context.part_box`.
"""

import numpy as np


def normalized(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def build_grasp_pose(approaching, closing, center):
    """World TCP pose approaching along ``approaching`` with the fingers closing along ``closing``
    (made orthogonal to it), the TCP at ``center``."""
    approaching = normalized(approaching)
    closing = normalized(closing - (closing @ approaching) * approaching)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack([approaching, closing, np.cross(approaching, closing)])
    transform[:3, 3] = center
    return transform


def compute_grasp_info_by_obb(box, approaching=(0., 0., -1.), target_closing=None, depth=0., ortho=True):
    """ManiSkill's grasp of an oriented box: approach along the box axis closest to ``approaching``,
    close across the shorter of the other two (of nearly equal ones, the one closest to
    ``target_closing``), the TCP ``depth`` into the box from the face it approaches (at most to its
    centre). Returns ``{'approaching', 'closing', 'center', 'extents'}``."""
    transform, extents = np.asarray(box[0], dtype=float), np.asarray(box[1], dtype=float)
    approaching = normalized(approaching)
    angles = approaching @ transform[:3, :3]
    order = np.argsort(np.abs(angles))
    ind0 = order[-1]
    shorter = np.argsort(extents[order[:-1]])
    ind1, ind2 = order[:-1][shorter[0]], order[:-1][shorter[1]]
    if target_closing is not None and extents[ind1] / extents[ind2] > .99:
        if abs(np.dot(target_closing, transform[:3, ind1])) < abs(np.dot(target_closing, transform[:3, ind2])):
            ind1, ind2 = ind2, ind1
    closing = transform[:3, ind1]
    if target_closing is not None and np.dot(target_closing, closing) < 0:
        closing = -closing
    extents = extents[[ind0, ind1, ind2]]
    half = extents[0] / 2
    center = transform[:3, 3] + approaching * (-half + min(depth, half))
    if ortho:
        closing = normalized(closing - (approaching @ closing) * approaching)
    return {'approaching': approaching, 'closing': closing, 'center': center, 'extents': extents}


def standoff(tcp, distance):
    """``tcp`` moved back along its approach axis by ``distance``."""
    result = np.array(tcp, dtype=float)
    result[:3, 3] -= result[:3, 0] * distance
    return result


def translated(transform, offset):
    result = np.array(transform, dtype=float)
    result[:3, 3] += offset
    return result
