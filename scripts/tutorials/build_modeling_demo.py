"""Run the two single-asset builders with their shared main(argv) interface."""

from __future__ import annotations

import argparse
from pathlib import Path
import runpy
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parser.parse_args(argv)
    output = args.output.resolve()
    source = Path(__file__).resolve().parents[2] / "asset_sources/objects"
    for kind in ("cabinet", "pen"):
        builder = runpy.run_path(str(source / kind / "000000/object.py"))
        builder["main"](["--output", str(output / "objects" / kind / "000000")])


if __name__ == "__main__":
    main()
