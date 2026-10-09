"""Hollow milky tape roll, one compound body with an open cardboard core."""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "asset_builders.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from asset_builders import asset, body, generate, material, tube


def build():
    tape = material("Milky adhesive tape", (0.78, 0.80, 0.72), 0.30, transmission=0.35)
    core = material("Cardboard core", (0.48, 0.40, 0.28), 0.70)
    root = asset("09_Tape_Roll", "bottom center of the roll", "+X and +Y radial; +Z roll axis",
                 "radially symmetric")
    roll = body("Tape roll", root)
    tube("Wound tape", 0.0305, 0.0173, 0.020, (0, 0, 0.0102), tape, roll, axis="Z")
    tube("Cardboard inner core", 0.0173, 0.0158, 0.0204, (0, 0, 0.0102), core, roll, axis="Z")


def main(argv=None):
    generate(build, __file__, argv)


if __name__ == "__main__":
    main()
