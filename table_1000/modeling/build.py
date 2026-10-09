"""Build all or selected object assets from a source directory."""

from __future__ import annotations

import argparse
from pathlib import Path
import runpy
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("asset_sources/objects"))
    parser.add_argument("--output", type=Path, default=Path("assets/objects"))
    parser.add_argument("--assets", nargs="+", help="Asset directories relative to --source")
    if argv is None:
        argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parser.parse_args(argv)
    source, output = args.source.resolve(), args.output.resolve()
    references = ([Path(reference) for reference in args.assets] if args.assets is not None
                  else [path.parent.relative_to(source) for path in sorted(source.rglob("object.py"))])
    for reference in references:
        builder = runpy.run_path(str(source / reference / "object.py"))
        builder["main"](["--output", str(output / reference)])
