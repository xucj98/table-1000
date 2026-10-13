"""Batch of supervised drawer-and-pen runs (see scripts/tasks/drawer_pen_batch.py).

A run that ends without ``result.json`` failed before or outside the task --
with eight runs at once, Isaac sometimes failed to build its ray-tracing
pipelines at startup and hung until the supervisor stopped it. Such a run is
moved aside to ``<case>.no-result-<n>`` and started again, up to ATTEMPTS times;
a run with a result, PASS or FAIL, is never repeated. Starts on one GPU are
spaced by START_GAP_S.
"""

from concurrent.futures import ThreadPoolExecutor
import csv
import itertools
import json
import subprocess
import sys
import threading
import time

from table_1000.runtime.process import write_json

ATTEMPTS = 2
START_GAP_S = 20.


def cases(kind, first=0, count=0, half_range=.05):
    """``[(name, arguments)]`` for the corner grid or a range of seeds."""
    if kind == 'seeds':
        return [(f'seed-{seed:04d}', ['--seed', str(seed)]) for seed in range(first, first + count)]
    values = (-half_range, 0., half_range)
    grid = list(itertools.product(values, values))
    result = []
    for (cx, cy), (px, py) in itertools.product(grid, grid):
        name = 'cab{:+.0f}{:+.0f}_pen{:+.0f}{:+.0f}'.format(*(100 * v for v in (cx, cy, px, py)))
        result.append((name, ['--offsets', *map(str, (cx, cy, px, py))]))
    return result


def summarize(directory):
    path = directory / 'result.json'
    lifecycle = directory / 'parent-lifecycle.json'
    result = json.loads(path.read_text()) if path.exists() else {}
    record = json.loads(lifecycle.read_text()) if lifecycle.exists() else {}
    failure = result.get('failure') or {}
    evaluation = result.get('evaluation') or {}
    offsets = (result.get('layout') or {}).get('offsets') or {}
    return {'case': directory.name, 'status': result.get('status', 'NO_RESULT'), 'seed': result.get('seed'),
            'cabinet_offset': offsets.get('cabinet'), 'pen_offset': offsets.get('pen'),
            'failure_stage': failure.get('stage'), 'failure_reason': failure.get('reason'),
            'error': (result.get('error') or '').strip().splitlines()[-1:] or None,
            'regrasps': result.get('regrasps'), 'unexpected_contacts': evaluation.get('unexpected_contacts'),
            'drawer_closed': evaluation.get('drawer_closed'), 'pen_in_drawer': evaluation.get('pen_in_drawer'),
            'simulated_s': result.get('simulated_s'),
            'wall_s': (result.get('timings') or {}).get('total_before_close_s'),
            'returncode': record.get('returncode'), 'signal': record.get('signal')}


def main(args, script):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    todo = cases(args.set, args.first, args.count)
    started = time.time()

    locks = {gpu: threading.Lock() for gpu in args.gpus}
    last_start = {gpu: 0. for gpu in args.gpus}

    def start_slot(gpu):
        with locks[gpu]:
            wait = last_start[gpu] + START_GAP_S - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            last_start[gpu] = time.monotonic()

    def run(index_case):
        index, (name, arguments) = index_case
        directory = output / name
        gpu = args.gpus[index % len(args.gpus)]
        attempts = 0
        while not (directory / 'result.json').exists() and attempts < ATTEMPTS:
            if directory.exists():
                directory.rename(output / f'{name}.no-result-{len(list(output.glob(name + ".no-result-*"))) + 1}')
            start_slot(gpu)
            attempts += 1
            subprocess.run([sys.executable, str(script), *arguments, '--output', str(directory), '--gpu', str(gpu)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return {**summarize(directory), 'attempts': attempts,
                'no_result_attempts': len(list(output.glob(name + '.no-result-*')))}

    with ThreadPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(run, enumerate(todo)))
    passed = sum(row['status'] == 'PASS' for row in rows)
    summary = {'set': args.set, 'cases': len(rows), 'passed': passed, 'failed': len(rows) - passed,
               'wall_s': time.time() - started, 'rows': rows}
    write_json(output / 'summary.json', summary)
    with (output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({key: value for key, value in summary.items() if key != 'rows'}, indent=2))
    return 0 if passed == len(rows) else 1
