"""Yellow cylindrical glue stick and white cap, represented by one rigid body."""
import math
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, cylinder, generate, label_patch, material


def build():
    yellow = material("Yellow glue stick plastic", (0.70, 0.79, 0.012), 0.32)
    white = material("White cap plastic", (0.78, 0.80, 0.78), 0.29)
    paper = material("Pale paper label", (0.87, 0.88, 0.77), 0.65)
    ink = material("Printed label", (0.009, 0.012, 0.014), 0.75)
    root = asset("17_Yellow_Solid_Glue", "bottom center of the upright tube",
                 "+X width; +Y back; +Z upright toward white cap", "label faces -Y")
    stick = body("Glue stick", root)
    cylinder("Yellow glue stick body", 0.0125, 0.070, (0, 0, 0.035), yellow, stick, axis="Z")
    cylinder("Yellow shoulder", 0.0128, 0.004, (0, 0, 0.069), yellow, stick, axis="Z")
    cylinder("White cap", 0.0136, 0.036, (0, 0, 0.089), white, stick, axis="Z")
    cylinder("White cap rim", 0.014, 0.0035, (0, 0, 0.0705), white, stick, axis="Z")
    label_patch("Paper label", 0.0127, 0.042, (0, 0, 0.035), paper, stick,
                axis="Z", angle=-math.pi / 2)
    for i, width in enumerate((0.0005, 0.0009, 0.0004, 0.0007, 0.0004, 0.0008)):
        label_patch("Barcode line", 0.01278, width, (0, 0, 0.045 + i * 0.0015),
                    ink, stick, angle=-1.82, span=0.5, axis="Z")
    for z in (0.022, 0.026, 0.030):
        label_patch("Label print", 0.01278, 0.0018, (0, 0, z), ink, stick, span=0.75,
                    axis="Z", angle=-math.pi / 2)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
