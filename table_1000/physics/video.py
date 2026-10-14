"""Synchronous camera capture without advancing physical time."""
import numpy as np

class Camera:
    """One fixed view of :class:`~table_1000.sim.isaac.cameras.Cameras` with the test's caption."""
    def __init__(self, stage, config):
        from table_1000.sim.isaac import render
        from table_1000.sim.isaac.cameras import Cameras
        render.add_lights(stage)
        self.size = config['resolution']
        # The timeline is playing: settling app updates would step physics.
        self.cameras = Cameras(stage, {'main': config}, None, config['fps'], settle=False)

    def capture(self, ctx):
        runtime = ctx.session
        from PIL import Image, ImageDraw
        before = {key: value['position'].copy() for key, value in runtime.states.items()}
        rgb = self.cameras.capture(save=False)['main']
        runtime.refresh()
        if any(not np.array_equal(value, runtime.states[key]['position']) for key, value in before.items()):
            raise RuntimeError('Rendering advanced physical state')
        image = Image.fromarray(rgb)
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
        self.cameras.close()
