"""Videos of a :class:`~table_1000.sim.isaaclab.env.RobotEnv`: :class:`Video` saves one captioned frame per capture
for each view; :func:`tile`, :func:`caption`, :func:`save_frame` and :func:`encode` compose frames and encode them."""

from pathlib import Path
import subprocess

import math

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


def tile(images, cell_height):
    """PIL ``images`` in a grid of ceil(sqrt(n)) columns, row by row: each scaled to ``cell_height``
    and centred in cells as wide as the widest."""
    from PIL import Image
    scaled = [image.resize((round(image.width * cell_height / image.height), cell_height), Image.LANCZOS)
              for image in images]
    columns = math.ceil(math.sqrt(len(scaled)))
    width = max(image.width for image in scaled)
    canvas = Image.new('RGB', (columns * width, math.ceil(len(scaled) / columns) * cell_height), 'black')
    for index, image in enumerate(scaled):
        canvas.paste(image, ((index % columns) * width + (width - image.width) // 2, (index // columns) * cell_height))
    return canvas


def caption(image, left, right=''):
    """PIL ``image`` below a white band holding ``left`` and ``right`` aligned to its sides, the font
    scaled to the width."""
    from PIL import Image, ImageDraw, ImageFont
    size = max(20, image.width // 60)
    band = round(size * 1.6)
    canvas = Image.new('RGB', (image.width, image.height + band), 'white')
    canvas.paste(image, (0, band))
    draw, font = ImageDraw.Draw(canvas), ImageFont.load_default(size=size)
    top = (band - size) // 2
    draw.text((size // 2, top), left, fill='black', font=font)
    draw.text((image.width - size // 2 - draw.textlength(right, font=font), top), right, fill='black', font=font)
    return canvas


def save_frame(image, path):
    """Save a PIL frame for :func:`encode`, padded to even dimensions (H.264 with yuv420p)."""
    from PIL import Image
    if image.width % 2 or image.height % 2:
        padded = Image.new('RGB', (image.width + image.width % 2, image.height + image.height % 2), 'black')
        padded.paste(image, (0, 0))
        image = padded
    image.save(path, quality=95)


def encode(directory, fps, target):
    """Encode ``directory/%05d.jpg`` at ``fps`` into ``target``; returns it."""
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(fps),
                    '-i', str(Path(directory) / '%05d.jpg'), '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                    '-crf', '23', str(target)], check=True)
    return Path(target)
