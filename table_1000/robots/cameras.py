"""Robot cameras: where they are mounted and how points project into them.

Projection matches ``cv2.projectPoints`` (pinhole with radial and tangential
distortion) and ``cv2.fisheye.projectPoints`` (equidistant fisheye,
Kannala-Brandt k1-k4) for points in the camera's optical frame (see
:class:`~table_1000.robots.spec.Sensor`).
"""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    return transform


def mount_on_body(urdf, mount):
    """``(link, T_link_optical)`` of a camera ``mount`` (a URDF frame name or ``(link, xyz, rpy)``),
    with ``link`` the moving body carrying it: fixed joints above the frame are folded into the
    transform, since the simulator merges them into their body."""
    if isinstance(mount, str):
        link, transform = mount, np.eye(4)
    else:
        link, xyz, rpy = mount
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        transform[:3, 3] = xyz
    joints = {joint.find('child').get('link'): joint for joint in ET.parse(urdf).getroot().findall('joint')}
    while link in joints and joints[link].get('type') == 'fixed':
        joint = joints[link]
        transform = origin(joint.find('origin')) @ transform
        link = joint.find('parent').get('link')
    return link, transform


def optical_pose(model, mount, urdf):
    """World pose of a camera's optical frame from a :class:`~table_1000.robots.kinematics.RobotModel` state."""
    link, transform = mount_on_body(urdf, mount)
    return model.fk(link, world=True) @ transform


def project(sensor, points):
    """``[M, 2]`` pixel coordinates (x right, y down) of ``[M, 3]`` points in the optical frame."""
    points = np.asarray(points, dtype=float)
    a, b = points[:, 0] / points[:, 2], points[:, 1] / points[:, 2]
    coefficients = np.zeros(5)
    coefficients[:len(sensor.distortion)] = sensor.distortion
    if sensor.model == 'fisheye':
        r = np.hypot(a, b)
        theta = np.arctan(r)
        k1, k2, k3, k4 = coefficients[:4]
        theta_d = theta * (1 + k1 * theta**2 + k2 * theta**4 + k3 * theta**6 + k4 * theta**8)
        scale = np.where(r > 1e-12, theta_d / np.maximum(r, 1e-12), 1.)
        x, y = a * scale, b * scale
    elif sensor.model == 'pinhole':
        k1, k2, p1, p2, k3 = coefficients
        r2 = a**2 + b**2
        radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
        x = a * radial + 2 * p1 * a * b + p2 * (r2 + 2 * a**2)
        y = b * radial + p1 * (r2 + 2 * b**2) + 2 * p2 * a * b
    else:
        raise ValueError(f'unknown camera model {sensor.model!r}')
    fx, fy, cx, cy = sensor.intrinsics
    return np.stack([fx * x + cx, fy * y + cy], axis=1)


def direction(sensor, pixel):
    """Unit direction in the optical frame of the ray through ``pixel``, ignoring distortion
    (equidistant for a fisheye); good enough to aim at a region of the image."""
    fx, fy, cx, cy = sensor.intrinsics
    a, b = (pixel[0] - cx) / fx, (pixel[1] - cy) / fy
    if sensor.model == 'fisheye':
        theta = np.hypot(a, b)
        ray = np.array([a, b, 0.]) * (np.sin(theta) / theta if theta > 1e-12 else 1.)
        ray[2] = np.cos(theta)
        return ray
    ray = np.array([a, b, 1.])
    return ray / np.linalg.norm(ray)
