"""Build the green disk magnet as one complete rigid body."""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "magnet_assets.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from magnet_assets import generate_magnet


def main(argv=None):
    generate_magnet("green", __file__, argv)


if __name__ == "__main__":
    main()
