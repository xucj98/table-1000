"""Synchronous camera capture without advancing physical time."""
import numpy as np

class Camera:
    def __init__(self, stage, config):
        import carb
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom, UsdLux
        for setting in ['/omni/replicator/captureOnPlay', '/omni/replicator/asyncRendering', '/app/asyncRendering']:
            carb.settings.get_settings().set(setting, False)
        self.rep, self.size = rep, config['resolution']
        camera = UsdGeom.Camera.Define(stage, '/World/Camera')
        camera.CreateFocalLengthAttr(config['focal_length_mm'])
        camera.CreateClippingRangeAttr(Gf.Vec2f(.001, 100))
        transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*config['position']), Gf.Vec3d(*config['target']), Gf.Vec3d(*config['up'])).GetInverse()
        UsdGeom.Xformable(camera).AddTransformOp().Set(transform)
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000)
        self.product = rep.create.render_product(camera.GetPath(), tuple(self.size))
        self.rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        self.rgb.attach([self.product])

    def capture(self, ctx):
        runtime = ctx.session
        from PIL import Image, ImageDraw
        from omni.physx import get_physx_interface
        get_physx_interface().update_transformations(True, True, True, False)
        before = {key: value['position'].copy() for key, value in runtime.states.items()}
        self.rep.orchestrator.step(rt_subframes=1, delta_time=0., pause_timeline=False)
        runtime.refresh()
        if any(not np.array_equal(value, runtime.states[key]['position']) for key, value in before.items()):
            raise RuntimeError('Rendering advanced physical state')
        image = Image.fromarray(self.rgb.get_data()[:, :, :3])
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, self.size[0], 35), fill='white')
        active = ', '.join(key for key, value in ctx.action_values.items()
                           if any(abs(float(component)) > 0 for field in ('force','torque') for component in value.get(field,[])))
        draw.text((10, 5), f'Isaac 5.1 | {ctx.name} | t={runtime.time:.2f}s | 1x', fill='black')
        draw.text((10, 20), f'Actions: {active or "none"}', fill='black')
        if getattr(ctx, 'video_annotation', None):
            draw.rectangle((0, 35, self.size[0], 54), fill='white')
            draw.text((10, 38), ctx.video_annotation(), fill='black')
        return image

    def close(self):
        self.rgb.detach()
        self.product.destroy()
