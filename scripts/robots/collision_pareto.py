"""Accuracy against cost of one link's collision piece count (see table_1000/robots/pareto.py)."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot')
    parser.add_argument('link')
    parser.add_argument('--counts', type=int, nargs='+', default=[1, 2, 3, 4, 8, 16, 32])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--rounds', type=int, default=2)
    args = parser.parse_args(argv)
    from table_1000.robots.pareto import run
    run(args.robot, args.link, args.counts, args.output.resolve(), args.gpu, args.rounds)


if __name__ == '__main__':
    main()
