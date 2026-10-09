"""Black retractable ballpoint: hollow shell and one moving button/refill body.

Local +X points to the button; tip is -X; barrel center is Z=0.006 m.
One native slider describes travel, without spring or click-lock dynamics.
"""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, box, cylinder, generate, joint, material, tube


def build():
    black = material("Black pen lacquer", (0.007, 0.010, 0.012), 0.19)
    metal = material("Brushed nickel hardware", (0.50, 0.54, 0.57), 0.24, metallic=0.92)
    ink = material("Refill tube", (0.24, 0.20, 0.12), 0.35)
    root = asset("13_Black_Ballpoint", "below barrel midpoint; barrel Z=0.006 m",
                 "+X length toward button; +Y width; +Z up", "ballpoint tip at -X")
    shell = body("Black pen shell", root)
    tube("Black lacquer barrel", 0.0056, 0.0034, 0.113, (0, 0, 0.006), black, shell)
    tube("Hollow metal nose", 0.0011, 0.0008, 0.024, (-0.0685, 0, 0.006), metal, shell,
         outer_end=0.0056, inner_end=0.0034)
    for x in (-0.010, -0.005, 0, 0.005):
        tube("Silver grip band", 0.0058, 0.00555, 0.0017, (x, 0, 0.006), metal, shell,
             collision=False)
    box("Pocket clip", (0.045, 0.0024, 0.0025), (0.031, 0.005, 0.009), black, shell,
        bevel=0.0008)
    box("Pocket clip bridge", (0.004, 0.0055, 0.003), (0.050, 0.0025, 0.011), black, shell,
        bevel=0.0008)
    moving = body("Black pen button and refill", root)
    cylinder("Silver push button", 0.003, 0.012, (0.062, 0, 0.006), metal, moving)
    cylinder("Refill shaft", 0.0009, 0.127, (-0.0075, 0, 0.006), ink, moving)
    cylinder("Fine ballpoint", 0.00045, 0.010, (-0.0753, 0, 0.006), metal, moving)
    joint("button_press", "SLIDER", shell, moving, (0.056, 0, 0.006), (-0.003, 0))


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
