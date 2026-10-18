"""Expert trajectories of a scene in Isaac Lab.

``.venv/bin/python scripts/scenes/generate.py 000000 --seeds 0:100 --output outputs/scenes/000000/gen --gpu 1``
writes ``trajectories.hdf5`` (succeeded episodes, Isaac Lab's format, ``demo_<seed>``),
``trajectories_failed.hdf5``, ``episodes/<seed>.json`` and ``summary.json`` /
``summary.csv``. ``--seeds`` takes ``a:b`` ranges and single seeds. The exit
status is 1 only when the run itself fails; failed episodes are part of the
result. See ``table_1000/experts/generate.py``.
"""

import argparse
import csv
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def seeds(values):
    result = []
    for value in values:
        if ':' in value:
            start, stop = map(int, value.split(':'))
            result += list(range(start, stop))
        else:
            result.append(int(value))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene')
    parser.add_argument('--seeds', nargs='+', default=['0:10'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', type=int, default=0)
    parser.add_argument('--root', type=Path, default=ROOT, help='repository root holding the scene and robot sources')
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from table_1000.sim.isaaclab.app import start_app
    started = time.perf_counter()
    app = start_app(gpu=args.gpu, workers=4, cameras=False)
    startup_s = time.perf_counter() - started
    failed = False
    generator = None
    try:
        from table_1000.experts.generate import SUMMARY_FIELDS, Generator, summarize
        generator = Generator(args.scene, output, args.assets.resolve(), args.root.resolve())
        reports = []
        for seed in seeds(args.seeds):
            reports.append(generator.run(seed))
            print(seed, reports[-1]['status'], reports[-1].get('failure', {}).get('reason', ''),
                  f'{reports[-1]["wall_s"]:.0f} s', flush=True)
        summary = summarize(reports)
        summary.update(scene=args.scene, startup_s=startup_s, load_s=generator.load_s,
                       total_s=time.perf_counter() - started)
        (output / 'summary.json').write_text(json.dumps(summary, indent=1, default=float))
        with open(output / 'summary.csv', 'w', newline='') as file:
            writer = csv.DictWriter(file, SUMMARY_FIELDS)
            writer.writeheader()
            writer.writerows(summary['rows'])
        print(json.dumps({key: value for key, value in summary.items() if key != 'rows'}, indent=1, default=float))
    except Exception:
        traceback.print_exc()
        failed = True
    if generator is not None:
        generator.close()
    app.close(exit_code=int(failed))


if __name__ == '__main__':
    main()
