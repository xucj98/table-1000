"""Synchronous multi-camera capture to JPEG frames and MP4 videos.

Capturing renders without advancing physics (as in ``table_1000.physics.video``).
A view is either fixed (``position`` and ``target``) or follows a prim
(``parent`` and a ``local`` 4x4 in the USD camera convention: -Z forward, +Y up).
"""

from pathlib import Path
import subprocess

import numpy as np


class Cameras:
    """Views rendered in turn through one camera and one render product.

    With a render product per view, the annotators were seen returning images
    of the wrong view; one product whose camera is moved between views, with a
    settling render after each move, gives each view its own image.
    """

    SETTLE_RENDERS = 1

    def __init__(self, stage, views, output, fps):
        import carb
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom
        settings = carb.settings.get_settings()
        for setting in ['/omni/replicator/captureOnPlay', '/omni/replicator/asyncRendering', '/app/asyncRendering']:
            settings.set(setting, False)
        settings.set('/rtx/post/motionblur/enabled', False)
        self.rep, self.stage, self.output, self.fps = rep, stage, Path(output), fps
        resolutions = {tuple(view['resolution']) for view in views.values()}
        if len(resolutions) != 1:
            raise ValueError('all views share one render product and need one resolution')
        self.camera = UsdGeom.Camera.Define(stage, '/World/Cameras/view')
        self.camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100))
        self.focal = self.camera.CreateFocalLengthAttr(24.)
        self.transform = UsdGeom.Xformable(self.camera).AddTransformOp()
        self.views = views
        self.counts = {name: 0 for name in views}
        for name in views:
            (self.output / name).mkdir(parents=True, exist_ok=True)
        self.product = rep.create.render_product(str(self.camera.GetPath()), resolutions.pop())
        self.rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        self.rgb.attach([self.product])

    def world_transform(self, view):
        from pxr import Gf, UsdGeom
        if 'parent' in view:
            parent = UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(view['parent']))
            return Gf.Matrix4d(np.asarray(view['local'], dtype=float).T.tolist()) * parent
        return Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view['position']), Gf.Vec3d(*view['target']),
                                       Gf.Vec3d(0, 0, 1)).GetInverse()

    def capture(self, env):
        from PIL import Image, ImageDraw
        from omni.physx import get_physx_interface
        get_physx_interface().update_transformations(True, True, True, False)
        for name, view in self.views.items():
            self.focal.Set(float(view['focal_length_mm']))
            self.transform.Set(self.world_transform(view))
            for _ in range(1 + self.SETTLE_RENDERS):
                self.rep.orchestrator.step(rt_subframes=1, delta_time=0., pause_timeline=False)
            image = Image.fromarray(self.rgb.get_data()[:, :, :3])
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, image.width, 18), fill='white')
            draw.text((6, 3), f't={env.time:5.2f}s  {env.contacts.phase}', fill='black')
            image.save(self.output / name / f'{self.counts[name]:05d}.jpg', quality=90)
            self.counts[name] += 1

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
        self.rgb.detach()
        self.product.destroy()
