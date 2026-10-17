"""Videos of a :class:`~table_1000.sim.isaaclab.env.RobotEnv`: one frame per capture for each view."""

from pathlib import Path
import subprocess

import numpy as np


class Video:
    """Frames of the env's cameras ``names`` (fixed views or robot cameras; depth as grey, white
    near) saved under ``output/<name>/`` with a caption (none for ``caption=None``), and encoded at ``fps``."""

    def __init__(self, env, names, output, fps):
        self.env, self.names, self.output, self.fps = env, list(names), Path(output), fps
        self.counts = {name: 0 for name in self.names}
        for name in self.names:
            (self.output / name).mkdir(parents=True, exist_ok=True)

    def capture(self, caption='', rendered=None):
        """Save one frame per camera; ``rendered`` (``RobotEnv.render`` of these cameras) reuses images
        rendered already."""
        from PIL import Image, ImageDraw
        from table_1000.sim.isaaclab.cameras import depth_image
        rendered = self.env.render(self.names) if rendered is None else rendered
        for name in self.names:
            outputs = rendered[name]
            frame = outputs['rgb'][0] if 'rgb' in outputs else depth_image(outputs['depth'][0])
            image = Image.fromarray(np.ascontiguousarray(frame))
            if caption is not None:
                draw = ImageDraw.Draw(image)
                draw.rectangle((0, 0, image.width, 18), fill='white')
                draw.text((6, 3), caption, fill='black')
            image.save(self.output / name / f'{self.counts[name]:05d}.jpg', quality=90)
            self.counts[name] += 1

    def encode(self, destination):
        """Write ``<destination>/<name>.mp4`` for every view with frames."""
        written = {}
        for name in self.names:
            if not self.counts[name]:
                continue
            target = Path(destination) / f'{name}.mp4'
            subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(self.fps),
                            '-i', str(self.output / name / '%05d.jpg'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                            '-crf', '23', str(target)], check=True)
            written[name] = str(target)
        return written
