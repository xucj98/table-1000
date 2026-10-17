"""Isaac Lab cameras: robot cameras with their OpenCV lens, and fixed views for videos.

Isaac Lab's pinhole camera ignores the principal point (aperture offsets are not
supported by the renderer) and has no OpenCV distortion. A robot camera is
therefore spawned as a pinhole and given the RTX lens-distortion schema of its
:class:`~table_1000.robots.spec.Sensor` model (OpenCV pinhole or fisheye), whose
``fx, fy, cx, cy`` and coefficients the renderer applies exactly (verified to
1.1 px against OpenCV). The calibrated intrinsics are the sensor's, not Isaac
Lab's ``intrinsic_matrices`` (computed from the pinhole aperture).
"""

from dataclasses import MISSING

import numpy as np
from scipy.spatial.transform import Rotation

# RTX lens distortion schemas by Sensor model.
LENSES = {'pinhole': ('OmniLensDistortionOpenCvPinholeAPI', 'opencvPinhole', ('k1', 'k2', 'p1', 'p2', 'k3')),
          'fisheye': ('OmniLensDistortionOpenCvFisheyeAPI', 'opencvFisheye', ('k1', 'k2', 'k3', 'k4'))}
DATA_TYPES = {'rgb': 'rgb', 'depth': 'distance_to_image_plane'}
FAR = 100.
APERTURE = 20.955   # mm, USD's default horizontal aperture


def apply_lens(prim, model, intrinsics, distortion, resolution):
    from pxr import Gf
    schema, attribute, names = LENSES[model]
    prim.ApplyAPI(schema)
    prim.GetAttribute('omni:lensdistortion:model').Set(attribute)
    fx, fy, cx, cy = intrinsics
    for key, value in dict(fx=fx, fy=fy, cx=cx, cy=cy, **dict(zip(names, distortion))).items():
        prim.GetAttribute(f'omni:lensdistortion:{attribute}:{key}').Set(float(value))
    prim.GetAttribute(f'omni:lensdistortion:{attribute}:imageSize').Set(Gf.Vec2i(*resolution))


def spawn_lens_camera(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Spawn a pinhole camera at every prim matching ``prim_path``, then give it ``cfg``'s lens."""
    import isaaclab.sim as sim_utils
    pinhole = sim_utils.PinholeCameraCfg(**{name: getattr(cfg, name) for name in sim_utils.PinholeCameraCfg.__dataclass_fields__
                                            if name != 'func'})
    prim = sim_utils.spawn_camera(prim_path, pinhole, translation, orientation, **kwargs)
    for path in sim_utils.find_matching_prim_paths(prim_path):
        apply_lens(sim_utils.get_current_stage().GetPrimAtPath(path), cfg.lens_model, cfg.intrinsics,
                   cfg.distortion, cfg.resolution)
    return prim


def lens_camera_cfg(sensor):
    """Spawner configuration of a :class:`~table_1000.robots.spec.Sensor`: a pinhole whose aperture
    matches its focal lengths (for Isaac Lab's own bookkeeping) plus the lens (see :func:`spawn_lens_camera`)."""
    import isaaclab.sim as sim_utils
    from isaaclab.utils import configclass

    @configclass
    class LensCameraCfg(sim_utils.PinholeCameraCfg):
        func: object = spawn_lens_camera
        lens_model: str = 'pinhole'
        intrinsics: tuple = MISSING
        distortion: tuple = ()
        resolution: tuple = MISSING

    fx, fy, cx, cy = sensor.intrinsics
    width, height = sensor.resolution
    pinhole = sim_utils.PinholeCameraCfg.from_intrinsic_matrix([fx, 0., cx, 0., fy, cy, 0., 0., 1.], width, height,
                                                                clipping_range=(sensor.near, FAR))
    return LensCameraCfg(**{name: getattr(pinhole, name) for name in sim_utils.PinholeCameraCfg.__dataclass_fields__
                            if name != 'func'},
                         lens_model=sensor.model, intrinsics=tuple(sensor.intrinsics),
                         distortion=tuple(sensor.distortion), resolution=(width, height))


def robot_camera_cfg(camera, urdf, robot_path, depth=False):
    """``CameraCfg`` of a robot :class:`~table_1000.robots.spec.CameraSpec` under the robot prim
    ``robot_path`` (a scene path such as ``{ENV_REGEX_NS}/Robot``); ``depth`` adds ground-truth
    depth to an rgb stream (for checks)."""
    from isaaclab.sensors import CameraCfg
    from table_1000.robots.cameras import mount_on_body
    sensor = camera.sensor
    link, transform = mount_on_body(urdf, camera.mount)
    kinds = ['depth'] if sensor.kind == 'depth' else ['rgb', 'depth'] if depth else ['rgb']
    width, height = sensor.resolution
    spawn = lens_camera_cfg(sensor)
    rotation = Rotation.from_matrix(transform[:3, :3]).as_quat()[[3, 0, 1, 2]]
    return CameraCfg(prim_path=f'{robot_path}/{link}/{{name}}', width=width, height=height,
                     data_types=[DATA_TYPES[kind] for kind in kinds], spawn=spawn, depth_clipping_behavior='zero',
                     offset=CameraCfg.OffsetCfg(pos=tuple(transform[:3, 3]), rot=tuple(rotation), convention='ros'))


def view_camera_cfg(view):
    """``CameraCfg`` of a fixed view ``{'position', 'target', 'up'?, 'focal_length_mm', 'resolution'}``
    (USD's default 20.955 mm horizontal aperture), for videos."""
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg
    forward = np.subtract(view['target'], view['position'])
    forward = forward / np.linalg.norm(forward)
    up = np.asarray(view.get('up', (0., 0., 1.)), dtype=float)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    # Optical frame (ROS): +X right, +Y down, +Z forward.
    rotation = Rotation.from_matrix(np.stack([right, down, forward], axis=1)).as_quat()[[3, 0, 1, 2]]
    width, height = view['resolution']
    spawn = sim_utils.PinholeCameraCfg(focal_length=float(view['focal_length_mm']), horizontal_aperture=APERTURE,
                                       vertical_aperture=APERTURE * height / width, clipping_range=(.01, FAR))
    return CameraCfg(prim_path='/World/Views/{name}', width=width, height=height, data_types=['rgb'], spawn=spawn,
                     offset=CameraCfg.OffsetCfg(pos=tuple(map(float, view['position'])), rot=tuple(rotation),
                                                convention='ros'))


def depth_image(depth, far=2.):
    """Grey HxWx3 uint8 picture of a depth image: white near, dark at ``far`` metres, black where nothing is hit."""
    grey = np.where(depth > 0, 255 - np.clip(depth / far, 0, 1) * 200, 0).astype(np.uint8)
    return np.repeat(grey[:, :, None], 3, axis=2)
