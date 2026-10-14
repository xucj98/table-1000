"""Run the drawer-and-pen task over a set of initial placements.

``"$ISAAC_PYTHON" scripts/tasks/drawer_pen_batch.py standard --output outputs/drawer-pen/standard --gpus 1 4``
runs the acceptance set: each object over its region's centre, edge midpoints
and corners with the other at its centre, plus the two opposite-corner pairs
(19 cases). ``corners`` runs every combination (9 x 9 = 81) and ``seeds --first
0 --count 50`` random seeds, for deeper checks. ``--per-process`` cases share one supervised process, which builds the
stage once and resets it between cases (see drawer_pen.py); ``--jobs``
processes run at once, assigned round-robin to ``--gpus``. Every case is kept in
``summary.json`` and ``summary.csv``, failures included.
"""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('set', choices=['standard', 'corners', 'seeds'])
    parser.add_argument('--first', type=int, default=0)
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpus', type=int, nargs='+', default=[0])
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--per-process', type=int, default=10)
    args = parser.parse_args(argv)
    from table_1000.tasks import batch
    return batch.main(args, Path(__file__).resolve().parent / 'drawer_pen.py')


if __name__ == '__main__':
    raise SystemExit(main())
