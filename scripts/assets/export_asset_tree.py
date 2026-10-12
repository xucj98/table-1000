"""Export an object asset's hierarchy with Blender."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from table_1000.modeling.asset_tree import main


if __name__ == "__main__":
    main()
