"""Task frames for putting the pen into the cabinet drawer.

All grasp and placement poses are built from live body poses and the assets'
own collision geometry, as ``T_world_tcp = T_world_frame @ T_frame_tcp``.
TCP axes follow ``table_1000.robots.x1pro``: +X approach, +Y finger opening,
+Z wrist camera.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from table_1000.robots.x1pro import TIP_BEYOND_TCP

DRAWER = 'cabinet.drawer3'
HANDLE = 'cabinet.drawer3.handle'
# Drawer travel used for placing the pen (stroke 0.173 m). With 0.12 m the
# exposed opening is about 0.11 m deep, leaving the fingers clear of the
# housing face while the pen is released.
OPENING = .12
# Handle grasp: fingers close on the handle's two ends; the approach points
# into the cabinet, tilted down by HANDLE_PITCH, and stops with the fingertips
# HANDLE_TIP_CLEARANCE in front of the drawer face.
HANDLE_PITCH = np.radians(30)
HANDLE_TIP_CLEARANCE = .006
# Pen grasp: top-down, fingers across the barrel at the pen's centre of mass
# (so it does not pivot between the pads), kept within PEN_BARREL along the
# axis (body frame); fingertips PEN_TIP_CLEARANCE above the table.
PEN_BARREL = (-.024, .078)
PEN_TIP_CLEARANCE = .004
PEN_RADIUS = .009
PLACE_ABOVE = .06
PEN_APPROACH = .05
# Placement: pen axis over the exposed part of the drawer, released with the
# pen's lowest point PEN_DROP above the drawer walls.
PEN_DROP = .012


def rigid(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def handle_frame(drawer_world, handle_center):
    """Frame at the handle's collision centre with the drawer body's axes.

    ``handle_center`` is the centre in the drawer body frame. Its -Y axis is
    the pull direction, +X runs along the handle, +Z is up.
    """
    drawer_world = np.asarray(drawer_world, dtype=float)
    return rigid(drawer_world[:3, :3], drawer_world[:3, :3] @ handle_center + drawer_world[:3, 3])


def handle_tcp(face_distance):
    """``T_handle_tcp`` of the handle grasp.

    ``face_distance`` is the distance along handle +Y from the handle centre to
    the drawer face behind it.
    """
    c, s = np.cos(HANDLE_PITCH), np.sin(HANDLE_PITCH)
    approach = np.array([0., c, -s])
    opening = np.array([-1., 0., 0.])
    rotation = np.column_stack([approach, opening, np.cross(approach, opening)])
    # Fingertip on the handle's mid-plane, HANDLE_TIP_CLEARANCE before the face.
    tip_depth = face_distance - HANDLE_TIP_CLEARANCE
    tip = np.array([0., tip_depth, -s * tip_depth / c])
    return rigid(rotation, tip - approach * TIP_BEYOND_TCP)


def standoff(tcp_world, distance):
    """``tcp_world`` moved back along its approach axis."""
    result = np.array(tcp_world, dtype=float)
    result[:3, 3] -= result[:3, 0] * distance
    return result


def translated(transform, offset):
    result = np.array(transform, dtype=float)
    result[:3, 3] += offset
    return result


def pen_tcp(pen_world, table_height, grasp_x):
    """Top-down grasp across the barrel at ``grasp_x`` (pen frame), from the live pen pose.

    The finger-opening axis is horizontal and perpendicular to the pen, with
    the wrist camera facing the pen's -X (cap) end. The TCP sits on the pen
    axis, raised if needed to keep the fingertips PEN_TIP_CLEARANCE above the table.
    """
    pen_world = np.asarray(pen_world, dtype=float)
    axis = pen_world[:3, 0].copy()
    axis[2] = 0.
    axis /= np.linalg.norm(axis)
    down = np.array([0., 0., -1.])
    opening = np.cross(down, axis)
    rotation = np.column_stack([down, opening, np.cross(down, opening)])
    grasp_x = float(np.clip(grasp_x, *PEN_BARREL))
    centre = pen_world[:3, :3] @ [grasp_x, 0., 0.] + pen_world[:3, 3]
    height = max(centre[2], table_height + PEN_TIP_CLEARANCE + TIP_BEYOND_TCP)
    return rigid(rotation, [centre[0], centre[1], height])


def drawer_release_point(drawer_world, interior, housing_face_y, wall_top):
    """World point above the exposed drawer interior where the pen is released.

    ``interior`` is ``((x0, y0), (x1, y1))`` of the drawer's inner floor in its
    body frame; ``housing_face_y`` is the housing front face, in the drawer
    body frame at the current opening. The point is centred in X and between
    the drawer's inner front wall and the housing face in Y.
    """
    (x0, y0), _ = interior
    y = (y0 + housing_face_y) / 2
    local = np.array([(interior[0][0] + interior[1][0]) / 2, y, wall_top + PEN_DROP + PEN_RADIUS])
    drawer_world = np.asarray(drawer_world, dtype=float)
    return drawer_world[:3, :3] @ local + drawer_world[:3, 3]


def place_tcp(release_point, tcp_pen, pen_world_axis_target, tilt=0., toward=(0., 1., 0.)):
    """TCP pose putting the held pen's origin at ``release_point``.

    ``tcp_pen`` is the measured ``T_tcp_pen`` after the grasp. The TCP points
    down, turned about vertical so the pen axis lies along
    ``pen_world_axis_target``, then tilted by ``tilt`` about the pen axis so the
    approach leans toward the horizontal direction ``toward``.
    """
    tcp_pen = np.asarray(tcp_pen, dtype=float)
    axis_in_tcp = tcp_pen[:3, 0]
    target = np.asarray(pen_world_axis_target, dtype=float)
    down = np.array([0., 0., -1.])
    # Rotation about the TCP X axis (vertical) aligning the pen axis.
    best = None
    for sign in (1., -1.):
        opening = np.cross(down, sign * target)
        rotation = np.column_stack([down, opening, np.cross(down, opening)])
        error = np.linalg.norm(rotation @ axis_in_tcp - target)
        if best is None or error < best[0]:
            best = (error, rotation)
    rotation = best[1]
    axis = target / np.linalg.norm(target)
    lean = Rotation.from_rotvec(axis * tilt)
    if np.dot(lean.apply(down), toward) < 0:
        lean = Rotation.from_rotvec(-axis * tilt)
    rotation = lean.as_matrix() @ rotation
    position = np.asarray(release_point) - rotation @ tcp_pen[:3, 3]
    return rigid(rotation, position)
