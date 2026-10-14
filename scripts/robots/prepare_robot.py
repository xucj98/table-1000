"""Prepare or check a robot source from its raw package (see table_1000/robots/prepare.py)."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from table_1000.robots.prepare import main
if __name__ == '__main__':
    raise SystemExit(main())
