"""Build the closed black pocket notebook."""
from pathlib import Path
import sys

source = Path(__file__).resolve()
sys.path.insert(0, str(source.parent))
if not (source.parent / "notebook_assets.py").exists():
    sys.path.insert(0, str(source.parents[4] / "table_1000/modeling"))
from notebook_assets import generate_notebook


def main(argv=None):
    generate_notebook("black", __file__, argv)


if __name__ == "__main__":
    main()
