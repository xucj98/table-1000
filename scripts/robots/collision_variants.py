"""Report one collision variant of a robot, or compare reports (see table_1000/robots/variants.py).

``"$ISAAC_PYTHON" scripts/robots/collision_variants.py x1pro baseline --source asset_sources/robots/x1pro --output outputs/robots/variants --gpus 7 6``
``"$ISAAC_PYTHON" scripts/robots/collision_variants.py --compare outputs/robots/variants``
"""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot', nargs='?')
    parser.add_argument('name', nargs='?')
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--gpus', type=int, nargs='+', default=[0])
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--per-process', type=int, default=5)
    parser.add_argument('--skip-task', action='store_true', help='accuracy, size and renders only')
    parser.add_argument('--compare', type=Path, help='print a table of the reports under this folder')
    args = parser.parse_args(argv)
    from table_1000.robots import variants
    if args.compare:
        reports = [json.loads(path.read_text()) for path in sorted(args.compare.glob('*/report.json'))]
        print(variants.table(reports))
        return
    report = variants.run(args.robot, args.name, args.source, args.output.resolve(), args.gpus, args.jobs,
                          args.per_process, args.skip_task)
    print(variants.table([report]))


if __name__ == '__main__':
    main()
