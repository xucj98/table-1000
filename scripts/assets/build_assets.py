"""Build object assets with Blender."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from table_1000.modeling.build import main


if __name__ == "__main__":
    main()
