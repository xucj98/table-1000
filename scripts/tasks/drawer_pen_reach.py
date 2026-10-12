"""Offline reachability of the drawer-and-pen key poses (MPlib only, no Isaac Sim).

``"$ISAAC_PYTHON" scripts/tasks/drawer_pen_reach.py --output outputs/drawer-pen/reach``
writes ``reach.json`` with every cabinet/pen corner combination.
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

if __name__ == '__main__':
    from table_1000.tasks.reach import main
    raise SystemExit(main())
