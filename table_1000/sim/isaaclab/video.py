"""Video frames: :func:`tile`, :func:`caption` and :func:`save_frame` compose them, :func:`encode` encodes them."""

import math
from pathlib import Path
import subprocess


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
