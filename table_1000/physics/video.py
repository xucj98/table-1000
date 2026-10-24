"""Synchronous camera capture without advancing physical time."""
import numpy as np

MAX_CAPTURE_RENDERS = 10

class Camera:
    def __init__(self, stage, config):
        import carb
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom, UsdLux
        for setting in ['/omni/replicator/captureOnPlay', '/omni/replicator/asyncRendering', '/app/asyncRendering']:
            carb.settings.get_settings().set(setting, False)
        # Closed translucent walls need enough transmission rays in RTX Real-Time 2.0.
        for setting in ['/rtx/rtpt/maxBounces', '/rtx/rtpt/maxSpecularAndTransmissionBounces']:
            carb.settings.get_settings().set(setting, 12)
        self.rep, self.size = rep, config['resolution']
        camera = UsdGeom.Camera.Define(stage, '/World/Camera')
        camera.CreateFocalLengthAttr(config['focal_length_mm'])
        camera.CreateClippingRangeAttr(Gf.Vec2f(.001, 100))
        transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*config['position']), Gf.Vec3d(*config['target']), Gf.Vec3d(*config['up'])).GetInverse()
        UsdGeom.Xformable(camera).AddTransformOp().Set(transform)
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(900)
        sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
        sun.CreateIntensityAttr(2500)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))
        self.product = rep.create.render_product(camera.GetPath(), tuple(self.size), force_new=True)
        self.capture_render_counts = []
        self.rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        self.rgb.attach([self.product])

    def capture(self, ctx):
        runtime = ctx.session
        from PIL import Image, ImageDraw
        before = {key: {field: value[field].copy() for field in ('position', 'quaternion', 'velocity')}
                  for key, value in runtime.states.items()}
        if ctx.world.is_fabric_enabled():
            ctx.world.forward()
        else:
            from omni.physx import get_physx_interface
            get_physx_interface().update_transformations(True, True, True, False)
        # A newly attached annotator can return an empty 1-D buffer while
        # Hydra compiles/activates the render product. Render only (no step)
        # until this product has a real frame, with a bounded startup cost.
        ctx.world.render()
        for renders in range(2, MAX_CAPTURE_RENDERS + 1):
            ctx.world.render()
            data = self.rgb.get_data()
            if np.asarray(data).size:
                break
        self.capture_render_counts.append(renders)
        runtime.refresh()
        if any(not np.array_equal(values[field], runtime.states[key][field])
               for key, values in before.items() for field in values):
            raise RuntimeError('Rendering advanced physical state')
        data = np.asarray(data)
        expected = (self.size[1], self.size[0])
        if data.ndim != 3 or data.shape[:2] != expected or data.shape[2] < 3:
            raise RuntimeError(f'RGB render product not ready after {renders} render passes: '
                               f'expected {expected} RGB image, got shape {data.shape}')
        image = Image.fromarray(data[:, :, :3])
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
        self.rgb.detach([self.product.path])
        self.product.destroy()
