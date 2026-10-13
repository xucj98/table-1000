"""Synchronous multi-camera capture to JPEG frames and MP4 videos.

Capturing renders without advancing physics (as in ``table_1000.physics.video``).
A view is either fixed (``position`` and ``target``) or follows a prim
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
    """

    RT_SUBFRAMES = 1

    def __init__(self, stage, views, output, fps):
        import carb
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom
        settings = carb.settings.get_settings()
        for setting in ['/omni/replicator/captureOnPlay', '/omni/replicator/asyncRendering', '/app/asyncRendering']:
            settings.set(setting, False)
        settings.set('/rtx/post/motionblur/enabled', False)
        # As in the asset physics tests: closed translucent walls need enough
        # transmission rays in RTX Real-Time 2.0.
        for setting in ['/rtx/rtpt/maxBounces', '/rtx/rtpt/maxSpecularAndTransmissionBounces']:
            settings.set(setting, 12)
        self.rep, self.stage, self.output, self.fps = rep, stage, Path(output), fps
        self.views = views
        self.counts = {name: 0 for name in views}
        self.transforms, self.products, self.annotators = {}, {}, {}
        for name, view in views.items():
            (self.output / name).mkdir(parents=True, exist_ok=True)
            camera = UsdGeom.Camera.Define(stage, f'/World/Cameras/{name}')
            camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 100))
            camera.CreateFocalLengthAttr(float(view['focal_length_mm']))
            self.transforms[name] = UsdGeom.Xformable(camera).AddTransformOp()
            self.transforms[name].Set(self.world_transform(view))
            self.products[name] = rep.create.render_product(str(camera.GetPath()), tuple(view['resolution']))
            self.annotators[name] = rep.AnnotatorRegistry.get_annotator('rgb')
            self.annotators[name].attach([self.products[name]])
        # Replicator's orchestrator is created on an app update; without one a
        # first capture can find it missing.
        import omni.kit.app
        for _ in range(3):
            omni.kit.app.get_app().update()

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
            if 'parent' in view:
                self.transforms[name].Set(self.world_transform(view))
        self.rep.orchestrator.step(rt_subframes=self.RT_SUBFRAMES, delta_time=0., pause_timeline=False)
        for name in self.views:
            image = Image.fromarray(self.annotators[name].get_data()[:, :, :3])
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
        for name in self.views:
            self.annotators[name].detach()
            self.products[name].destroy()
