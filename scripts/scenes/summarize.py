"""Summary of one or more generation outputs (``generate.py`` run in parts, e.g. per seed range).

``python scripts/scenes/summarize.py outputs/scenes/scene-000000/gen/part* --output outputs/scenes/scene-000000/gen/summary.json``
reads every ``episodes/<seed>.json`` and writes the same summary as ``generate.py``
(success rate, failures by subtask and reason, ...).
"""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from table_1000.experts.generate import summarize  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('outputs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    reports = sorted((json.loads(path.read_text()) for directory in args.outputs
                      for path in (directory / 'episodes').glob('*.json')), key=lambda report: report['seed'])
    summary = summarize(reports)
    args.output.write_text(json.dumps(summary, indent=1, default=float))
    print(json.dumps({key: value for key, value in summary.items() if key != 'rows'}, indent=1, default=float))


if __name__ == '__main__':
    main()
