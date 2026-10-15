"""Synchronous multi-camera capture: video frames, and robot camera images.

Capturing renders without advancing physics; the asset physics tests use one
fixed view (``table_1000.physics.video``). A view is either fixed
(``position``, ``target`` and optionally ``up``) or follows a prim (``parent``
and a ``local`` 4x4 in the USD camera convention: -Z forward, +Y up). A view
uses ``focal_length_mm`` (35 mm film aperture) or a ``lens`` from a robot
camera's :class:`~table_1000.robots.spec.CameraSpec` (see :func:`robot_view`).
"""

from pathlib import Path
import subprocess

import numpy as np

# RTX lens distortion schemas by CameraSpec model.
LENSES = {'pinhole': ('OmniLensDistortionOpenCvPinholeAPI', 'opencvPinhole', ('k1', 'k2', 'p1', 'p2', 'k3')),
          'fisheye': ('OmniLensDistortionOpenCvFisheyeAPI', 'opencvFisheye', ('k1', 'k2', 'k3', 'k4'))}
ANNOTATORS = {'rgb': 'rgb', 'depth': 'distance_to_image_plane'}


def camera_mount(urdf, frame):
    """``(parent_link, T_parent_camera)`` of an optical frame fixed to a robot body in ``urdf``, with the
    optical convention (+Z forward, +Y down) turned into USD's (-Z forward, +Y up)."""
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation
    joint = next(joint for joint in ET.parse(urdf).getroot().findall('joint')
                 if joint.find('child').get('link') == frame)
    origin = joint.find('origin')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    return joint.find('parent').get('link'), transform @ np.diag([1., -1., -1., 1.])


def robot_view(urdf, camera, robot_path, ground_truth=False):
    """View of a robot camera stream (a ``CameraSpec``) on the robot prim at ``robot_path``;
    ``ground_truth`` adds depth to an rgb stream (for checks)."""
    link, local = camera_mount(urdf, camera.frame)
    outputs = ('depth',) if camera.kind == 'depth' else ('rgb', 'depth') if ground_truth else ('rgb',)
    return {'parent': f'{robot_path}/{link}', 'local': local, 'resolution': tuple(camera.resolution),
            'near': camera.near, 'outputs': outputs,
            'lens': {'model': camera.model, 'intrinsics': tuple(camera.intrinsics), 'distortion': tuple(camera.distortion)}}


def depth_image(depth, far=2.):
    """Grey HxWx3 uint8 picture of a depth image: white near, dark at ``far`` metres, black where nothing is hit."""
    grey = np.where(depth > 0, 255 - np.clip(depth / far, 0, 1) * 200, 0).astype(np.uint8)
    return np.repeat(grey[:, :, None], 3, axis=2)


class Cameras:
    """One camera, render product and annotators per view, rendered together.

    Each view keeps its own camera so the renderer's temporal accumulation
    carries over between frames. Moving one camera between views cleared it on
    every frame and the videos flickered (about ten times the frame-to-frame
    change of a fixed camera on a static scene). Fixed cameras are placed once;
    cameras that follow a prim are moved before each capture. A render updates
    only the views it is asked for.

    ``subframes`` renders per capture, all of the same physical state. With
    one (the default, and the asset physics tests' standard), a fast-moving
    object (a released pen) leaves a short trail from the renderer's temporal
    denoising; more subframes reduce it at a proportional render cost.
    """

    SUBFRAMES = 1

    def __init__(self, stage, views, output, fps, subframes=SUBFRAMES, settle=True, root='/World/Cameras'):
        """``output``: directory for the saved frames (None: frames are only returned);
        the camera prims are ``<root>/<view>``.
        ``settle`` runs three app updates so Replicator's orchestrator exists before
        the first capture; with a playing timeline each update also steps physics."""
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom
        from table_1000.sim.isaac import render
        render.configure()
        self.rep, self.stage, self.fps = rep, stage, fps
        self.output = None if output is None else Path(output)
        self.subframes = int(subframes)
        self.views = views
        self.counts = {name: 0 for name in views}
        self.transforms, self.products, self.annotators = {}, {}, {}
        self.paused = set()
        for name, view in views.items():
            outputs = view.get('outputs', ('rgb',))
            if self.output:
                (self.output / name).mkdir(parents=True, exist_ok=True)
            camera = UsdGeom.Camera.Define(stage, f'{root}/{name}')
            camera.CreateClippingRangeAttr(Gf.Vec2f(view.get('near', .001), 100))
            if 'lens' in view:
                self.apply_lens(camera.GetPrim(), view['lens'], view['resolution'])
            else:
                camera.CreateFocalLengthAttr(float(view['focal_length_mm']))
            self.transforms[name] = UsdGeom.Xformable(camera).AddTransformOp()
            self.transforms[name].Set(self.world_transform(view))
            self.products[name] = rep.create.render_product(str(camera.GetPath()), tuple(view['resolution']))
            self.annotators[name] = {}
            for kind in outputs:
                annotator = rep.AnnotatorRegistry.get_annotator(ANNOTATORS[kind])
                annotator.attach([self.products[name]])
                self.annotators[name][kind] = annotator
        # Replicator's orchestrator is created on an app update; without one a
        # first capture can find it missing.
        import omni.kit.app
        for _ in range(3 if settle else 0):
            omni.kit.app.get_app().update()

    @staticmethod
    def apply_lens(prim, lens, resolution):
        from pxr import Gf
        schema, model, names = LENSES[lens['model']]
        prim.ApplyAPI(schema)
        prim.GetAttribute('omni:lensdistortion:model').Set(model)
        fx, fy, cx, cy = lens['intrinsics']
        coefficients = dict(zip(names, lens['distortion']))
        for key, value in dict(fx=fx, fy=fy, cx=cx, cy=cy, **coefficients).items():
            prim.GetAttribute(f'omni:lensdistortion:{model}:{key}').Set(float(value))
        prim.GetAttribute(f'omni:lensdistortion:{model}:imageSize').Set(Gf.Vec2i(*resolution))

    def world_transform(self, view):
        from pxr import Gf, UsdGeom
        if 'parent' in view:
            parent = UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(view['parent']))
            return Gf.Matrix4d(np.asarray(view['local'], dtype=float).T.tolist()) * parent
        return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view['position']), Gf.Vec3d(*view['target']),
                                       Gf.Vec3d(*view.get('up', (0, 0, 1)))).GetInverse()

    def pause(self, names=None):
        """Stop updating views (default: all) in later renders, of this or other ``Cameras``."""
        for name in self.views if names is None else names:
            self.products[name].hydra_texture.set_updates_enabled(False)
            self.paused.add(name)

    def render(self, names=None):
        """Render the current physical state into ``names`` (default: every view); the
        others are paused."""
        from omni.physx import get_physx_interface
        names = list(self.views) if names is None else list(names)
        get_physx_interface().update_transformations(True, True, True, False)
        self.pause([name for name in self.views if name not in names])
        resumed = self.paused & set(names)
        for name in names:
            self.products[name].hydra_texture.set_updates_enabled(True)
            if 'parent' in self.views[name]:
                self.transforms[name].Set(self.world_transform(self.views[name]))
        self.paused -= resumed
        # A view resumed after a pause returns its last frame from before the pause
        # on its first render.
        for _ in range(2 if resumed else 1):
            self.rep.orchestrator.step(rt_subframes=self.subframes, delta_time=0., pause_timeline=False)
        return names

    def read(self, name):
        """Last rendered outputs of view ``name``: ``rgb`` HxWx3 uint8 and ``depth`` HxW
        metres (0 where nothing is hit)."""
        out = {}
        for kind, annotator in self.annotators[name].items():
            data = annotator.get_data()
            if kind == 'rgb':
                out[kind] = np.asarray(data[:, :, :3])
            else:
                depth = np.asarray(data, dtype=np.float32)
                out[kind] = np.where(np.isfinite(depth), depth, 0.).astype(np.float32)
        return out

    def capture(self, caption='', save=True, names=None):
        """Render ``names`` (default: every view); returns ``{view: HxWx3 uint8}``, a depth-only
        view as :func:`depth_image`. Frames are saved (with ``caption`` drawn on top) unless
        ``save`` is False."""
        from PIL import Image, ImageDraw
        images = {}
        for name in self.render(names):
            outputs = self.read(name)
            images[name] = outputs['rgb'] if 'rgb' in outputs else depth_image(outputs['depth'])
            if not save or not self.output:
                continue
            image = Image.fromarray(images[name])
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, image.width, 18), fill='white')
            draw.text((6, 3), caption, fill='black')
            image.save(self.output / name / f'{self.counts[name]:05d}.jpg', quality=90)
            self.counts[name] += 1
        return images

    def encode(self, destination):
        """Write ``<destination>/<view>.mp4`` for every view with frames."""
        written = {}
        for name in self.views:
            if not self.counts[name]:
                continue
            target = Path(destination) / f'{name}.mp4'
            subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(self.fps),
                            '-i', str(self.output / name / '%05d.jpg'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                            '-crf', '23', str(target)], check=True)
            written[name] = str(target)
        return written

    def close(self):
        # Destroying a render product releases its annotators. Detaching them one by
        # one first fails once several products share depth nodes, and the
        # half-released graph then crashes Kit at exit.
        for product in self.products.values():
            product.destroy()
