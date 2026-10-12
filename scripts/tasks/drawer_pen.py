"""Run the scene-000000 drawer-and-pen task once.

``"$ISAAC_PYTHON" scripts/tasks/drawer_pen.py --seed 3 --output outputs/drawer-pen/seed-3 --gpu 1 [--video]``
or ``--offsets CX CY PX PY`` for explicit cabinet/pen XY offsets. The run is a
child process under a supervising parent; it writes ``result.json``,
``trace.csv``, ``contacts.json`` and, with ``--video``, ``videos/*.mp4`` to the
output directory.
"""

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--seed', type=int)
    group.add_argument('--offsets', type=float, nargs=4, metavar=('CX', 'CY', 'PX', 'PY'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--child', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    from table_1000.tasks import run
    if args.child:
        return run.child(args)
    return run.parent(args, Path(__file__).resolve())


if __name__ == '__main__':
    raise SystemExit(main())
