"""White superglue bottle with a yellow tapered nozzle cap; one rigid body."""
import math
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, cylinder, generate, label_patch, material


def build():
    white = material("White bottle plastic", (0.78, 0.80, 0.78), 0.29)
    yellow = material("Yellow nozzle and label", (0.70, 0.79, 0.012), 0.32)
    black = material("Label ink", (0.009, 0.012, 0.014), 0.70)
    root = asset("12_Superglue_Bottle", "bottom center of the upright bottle",
                 "+X width; +Y back; +Z upright toward nozzle", "label faces -Y")
    bottle = body("Superglue bottle", root)
    cylinder("White glue bottle", 0.012, 0.039, (0, 0, 0.025), white, bottle, axis="Z")
    cylinder("Yellow label band", 0.0121, 0.026, (0, 0, 0.027), yellow, bottle,
             axis="Z", collision=False)
    cylinder("White base rim", 0.0124, 0.006, (0, 0, 0.003), white, bottle, axis="Z")
    cylinder("White bottle neck", 0.0075, 0.004, (0, 0, 0.0455), white, bottle, axis="Z")
    cylinder("Yellow screw collar", 0.009, 0.010, (0, 0, 0.051), yellow, bottle, axis="Z")
    cylinder("Tapered nozzle cap", 0.0078, 0.039, (0, 0, 0.075), yellow, bottle,
             axis="Z", radius_end=0.0021)
    label_patch("Black label stripe", 0.01236, 0.0025, (0, 0, 0.013), black, bottle,
                span=1.5, segments=6, axis="Z", angle=-math.pi / 2)
    label_patch("Black label block", 0.01236, 0.011, (0, 0, 0.027), black, bottle,
                axis="Z", angle=-math.pi / 2)
    for z in (0.024, 0.028, 0.032):
        label_patch("Label print", 0.01245, 0.0015, (0, 0, z), yellow, bottle,
                    span=0.7, axis="Z", angle=-math.pi / 2)


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
