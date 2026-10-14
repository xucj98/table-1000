"""Synchronous multi-camera capture to JPEG frames and MP4 videos.

Capturing renders without advancing physics; the asset physics tests use one
fixed view (``table_1000.physics.video``).
A view is either fixed (``position``, ``target`` and optionally ``up``) or follows a prim
(``parent`` and a ``local`` 4x4 in the USD camera convention: -Z forward, +Y up).
"""

from pathlib import Path
import subprocess

import numpy as np


class Cameras:
    """One camera, render product and RGB annotator per view, rendered together.

    Each view keeps its own camera so the renderer's temporal accumulation
    carries over between frames. Moving one camera between views cleared it on
    every frame and the videos flickered (about ten times the frame-to-frame
    change of a fixed camera on a static scene). Fixed cameras are placed once;
    cameras that follow a prim are moved before each capture.

    ``subframes`` renders per capture, all of the same physical state. With
    one (the default, and the asset physics tests' standard), a fast-moving
    object (a released pen) leaves a short trail from the renderer's temporal
    denoising; more subframes reduce it at a proportional render cost.
    """

    SUBFRAMES = 1

    def __init__(self, stage, views, output, fps, subframes=SUBFRAMES, settle=True):
        """``output``: directory for the saved frames (None: frames are only returned).
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
        for name, view in views.items():
            if self.output:
                (self.output / name).mkdir(parents=True, exist_ok=True)
            camera = UsdGeom.Camera.Define(stage, f'/World/Cameras/{name}')
            camera.CreateClippingRangeAttr(Gf.Vec2f(.001, 100))
            camera.CreateFocalLengthAttr(float(view['focal_length_mm']))
            self.transforms[name] = UsdGeom.Xformable(camera).AddTransformOp()
            self.transforms[name].Set(self.world_transform(view))
            self.products[name] = rep.create.render_product(str(camera.GetPath()), tuple(view['resolution']))
            self.annotators[name] = rep.AnnotatorRegistry.get_annotator('rgb')
            self.annotators[name].attach([self.products[name]])
        # Replicator's orchestrator is created on an app update; without one a
        # first capture can find it missing.
        import omni.kit.app
        for _ in range(3 if settle else 0):
            omni.kit.app.get_app().update()

    def world_transform(self, view):
        from pxr import Gf, UsdGeom
        if 'parent' in view:
            parent = UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(view['parent']))
            return Gf.Matrix4d(np.asarray(view['local'], dtype=float).T.tolist()) * parent
        return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view['position']), Gf.Vec3d(*view['target']),
                                       Gf.Vec3d(*view.get('up', (0, 0, 1)))).GetInverse()

    def capture(self, caption='', save=True):
        """Render every view; returns ``{view: HxWx3 uint8}``. Frames are saved
        (with ``caption`` drawn on top) unless ``save`` is False."""
        from PIL import Image, ImageDraw
        from omni.physx import get_physx_interface
        get_physx_interface().update_transformations(True, True, True, False)
        for name, view in self.views.items():
            if 'parent' in view:
                self.transforms[name].Set(self.world_transform(view))
        self.rep.orchestrator.step(rt_subframes=self.subframes, delta_time=0., pause_timeline=False)
        images = {}
        for name in self.views:
            images[name] = np.asarray(self.annotators[name].get_data()[:, :, :3])
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
        for name in self.views:
            self.annotators[name].detach()
            self.products[name].destroy()


def camera_mount(urdf, frame):
    """``(parent_link, T_parent_camera)`` of an optical frame fixed to a robot body in ``urdf``, with the
    optical convention (+Z forward, +Y down) turned into USD's (-Z forward, +Y up)."""
    import xml.etree.ElementTree as ET
    import numpy as np
    from scipy.spatial.transform import Rotation
    joint = next(joint for joint in ET.parse(urdf).getroot().findall('joint')
                 if joint.find('child').get('link') == frame)
    origin = joint.find('origin')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    return joint.find('parent').get('link'), transform @ np.diag([1., -1., -1., 1.])
