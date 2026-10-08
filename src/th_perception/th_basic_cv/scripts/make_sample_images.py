"""Regenerate the synthetic images in sample_images/ (needs Pillow and numpy)."""

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

OUT = Path(__file__).resolve().parent.parent / 'sample_images'
SIZE = (640, 480)


def shapes():
    """Three solid shapes on a plain background: the pipeline should find 3."""
    img = Image.new('RGB', SIZE, (235, 235, 230))
    d = ImageDraw.Draw(img)
    d.rectangle([60, 80, 220, 220], fill=(200, 40, 40))
    d.ellipse([280, 60, 440, 220], fill=(40, 80, 200))
    d.polygon([(500, 380), (600, 220), (620, 400)], fill=(40, 160, 70))
    return img


def blocks():
    """Five blocks on a gradient, slightly noisy table: the pipeline should find 5."""
    rng = np.random.default_rng(0)
    ramp = np.linspace(150, 190, SIZE[0], dtype=np.float32)
    base = np.stack([ramp * 1.0, ramp * 0.85, ramp * 0.65], axis=-1)
    table = np.broadcast_to(base, (SIZE[1], SIZE[0], 3)).copy()
    table += rng.normal(0, 4, table.shape)
    img = Image.fromarray(np.clip(table, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    colors = [(220, 50, 50), (50, 120, 220), (240, 200, 40), (60, 170, 90), (130, 60, 170)]
    boxes = [(40, 60, 150, 170), (220, 40, 360, 140), (420, 70, 580, 190),
             (90, 280, 250, 420), (330, 250, 470, 400)]
    for color, box in zip(colors, boxes):
        d.rectangle(box, fill=color, outline=tuple(c // 2 for c in color), width=3)
    return img


def low_light():
    """Two dim shapes in a dark scene, for trying lower Canny thresholds."""
    img = Image.new('RGB', SIZE, (22, 22, 26))
    d = ImageDraw.Draw(img)
    d.rectangle([100, 140, 260, 320], fill=(55, 50, 45))
    d.ellipse([360, 160, 520, 320], fill=(40, 48, 62))
    return img


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    # The noisy one is a JPEG to stay small; the flat ones compress well as PNG.
    for name, make in [('01_shapes.png', shapes), ('02_blocks.jpg', blocks),
                       ('03_low_light.png', low_light)]:
        make().save(OUT / name, optimize=True, quality=85)
        print(f'wrote {name}')
