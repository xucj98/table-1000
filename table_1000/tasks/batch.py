"""Batch of supervised drawer-and-pen runs (see scripts/tasks/drawer_pen_batch.py).

Cases are split into chunks of ``--per-process``; each chunk runs in one
supervised process that builds the stage once and resets it between cases
(``<output>/_workers/<chunk>/`` keeps the process logs). A case that ends
without ``result.json`` failed before or outside the task -- with eight
processes at once, Isaac sometimes failed to build its ray-tracing pipelines at
startup and hung until the supervisor stopped it. Such a case is moved aside to
``<case>.no-result-<n>`` and run again in a new process, up to ATTEMPTS times in
all; a case with a result, PASS or FAIL, is never repeated. Starts on one GPU
are spaced by START_GAP_S.
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
    """``[{'name', 'seed', 'offsets'}]`` of a case set.

    ``standard`` (19): each object over its 3 x 3 region points with the other
    at its centre, plus the two opposite-corner pairs (objects closest together
    and farthest apart). ``corners`` (81): every pair of region points.
    ``seeds``: a range of seeds.
    """
    if kind == 'seeds':
        return [{'name': f'seed-{seed:04d}', 'seed': seed, 'offsets': None} for seed in range(first, first + count)]
    values = (-half_range, 0., half_range)
    grid = list(itertools.product(values, values))
    if kind == 'corners':
        pairs = list(itertools.product(grid, grid))
    else:
        h = half_range
        pairs = [(point, (0., 0.)) for point in grid] + [((0., 0.), point) for point in grid if point != (0., 0.)]
        # The cabinet sits at +x +y of the pen: (-,-) and (+,+) bring them closest together.
        pairs += [((-h, -h), (h, h)), ((h, h), (-h, -h))]
    result = []
    for (cx, cy), (px, py) in pairs:
        name = 'cab{:+.0f}{:+.0f}_pen{:+.0f}{:+.0f}'.format(*(100 * v for v in (cx, cy, px, py)))
        result.append({'name': name, 'seed': None, 'offsets': [cx, cy, px, py]})
    return result


def summarize(directory):
    path = directory / 'result.json'
    result = json.loads(path.read_text()) if path.exists() else {}
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
            'case_s': (result.get('timings') or {}).get('case_s'),
            'planning_s': (result.get('timings') or {}).get('planning_s'),
            'step_s': (result.get('timings') or {}).get('step_s')}


def main(args, script):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    todo = cases(args.set, args.first, args.count)
    started = time.time()

    locks = {gpu: threading.Lock() for gpu in args.gpus}
    last_start = {gpu: 0. for gpu in args.gpus}
    workers = []

    def start_slot(gpu):
        with locks[gpu]:
            wait = last_start[gpu] + START_GAP_S - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            last_start[gpu] = time.monotonic()

    def run(index_chunk):
        index, chunk = index_chunk
        gpu = args.gpus[index % len(args.gpus)]
        attempts = {case['name']: 0 for case in chunk}
        for attempt in range(ATTEMPTS):
            pending = [case for case in chunk if not (output / case['name'] / 'result.json').exists()]
            if not pending:
                break
            for case in pending:
                directory = output / case['name']
                if directory.exists():
                    directory.rename(output / f'{case["name"]}.no-result-{len(list(output.glob(case["name"] + ".no-result-*"))) + 1}')
                attempts[case['name']] += 1
            # A new directory per process, also when a batch is run again over the same output.
            runs = len(list((output / '_workers').glob(f'{index:03d}-*'))) if (output / '_workers').exists() else 0
            worker = output / '_workers' / f'{index:03d}-{runs}'
            worker.mkdir(parents=True)
            (worker / 'cases.json').write_text(json.dumps([{**case, 'directory': str(output / case['name'])}
                                                           for case in pending]))
            start_slot(gpu)
            started_worker = time.time()
            completed = subprocess.run([sys.executable, str(script), '--cases', str(worker / 'cases.json'),
                                        '--output', str(worker / 'process'), '--gpu', str(gpu)],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            process = worker / 'process' / 'process.json'
            process = json.loads(process.read_text()) if process.exists() else {}
            workers.append({'worker': worker.name, 'gpu': gpu, 'cases': len(pending), 'returncode': completed.returncode,
                            'wall_s': time.time() - started_worker, 'startup_s': process.get('startup_s'),
                            'load_s': process.get('load_s')})
        return [{**summarize(output / case['name']), 'attempts': attempts[case['name']],
                 'no_result_attempts': len(list(output.glob(case['name'] + '.no-result-*')))} for case in chunk]

    size = max(1, args.per_process)
    chunks = [todo[i:i + size] for i in range(0, len(todo), size)]
    with ThreadPoolExecutor(args.jobs) as pool:
        rows = [row for rows in pool.map(run, enumerate(chunks)) for row in rows]
    passed = sum(row['status'] == 'PASS' for row in rows)
    summary = {'set': args.set, 'cases': len(rows), 'passed': passed, 'failed': len(rows) - passed,
               'per_process': size, 'jobs': args.jobs, 'wall_s': time.time() - started,
               'processes': sorted(workers, key=lambda w: w['worker']),
               'processes_ok': all(w['returncode'] == 0 for w in workers), 'rows': rows}
    write_json(output / 'summary.json', summary)
    with (output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({key: value for key, value in summary.items() if key not in ('rows', 'processes')}, indent=2))
    return 0 if passed == len(rows) and summary['processes_ok'] else 1
