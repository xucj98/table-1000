"""White/black retractable pen: shell and one moving button/refill body.

Local +X points to the button; tip is -X; barrel center is Z=0.005 m.
The black rear housing is fixed; only its small end button translates.
"""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, cylinder, generate, joint, material, tube


def build():
    white = material("White pen plastic", (0.78, 0.80, 0.78), 0.29)
    black = material("Black pen plastic", (0.009, 0.012, 0.014), 0.30)
    metal = material("Ballpoint steel", (0.5, 0.54, 0.57), 0.24, metallic=0.92)
    ink = material("Refill tube", (0.24, 0.20, 0.12), 0.35)
    root = asset("16_White_Black_Pen", "below barrel reference midpoint; barrel Z=0.005 m",
                 "+X length toward button; +Y width; +Z up", "ballpoint tip at -X")
    shell = body("White black pen shell", root)
    tube("White barrel", 0.0048, 0.0028, 0.102, (0.006, 0, 0.005), white, shell)
    tube("Black rear housing", 0.0051, 0.0032, 0.011, (0.0625, 0, 0.005), black, shell)
    tube("Black tapered nose", 0.0008, 0.0005, 0.042, (-0.066, 0, 0.005), black, shell,
         outer_end=0.0049, inner_end=0.0028)
    tube("Black shoulder", 0.0055, 0.0028, 0.003, (0.055, 0, 0.005), black, shell,
         collision=False)
    moving = body("White black pen button and refill", root)
    cylinder("Black push button", 0.003, 0.007, (0.0715, 0, 0.005), black, moving)
    cylinder("Refill shaft", 0.0008, 0.144, (-0.004, 0, 0.005), ink, moving)
    cylinder("Fine ballpoint", 0.0003, 0.010, (-0.0813, 0, 0.005), metal, moving)
    joint("button_press", "SLIDER", shell, moving, (0.068, 0, 0.005), (-0.003, 0))


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
