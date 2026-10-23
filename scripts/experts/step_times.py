"""Where expert episodes spend simulated and wall time, by step and subtask, from their reports.

``uv run python scripts/experts/step_times.py data/scenes/000002/isaac-cpu [more batches ...]``

Per batch: the success rate, the mean wall time per episode, and how many successes stayed within
``--sim-limit`` s of simulated time; then per step of the expert (``steps`` of the reports) the
episodes that completed it and their mean simulated and wall seconds in it, and per subtask the
same over every episode that ran it, with the seconds its motion plans took. Wall seconds are those of the expert itself (without the
environment's reset); reports written before they were recorded show ``-``.
"""

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np

from table_1000.sim import CONTROL_HZ


def step_times(report):
    """``[(step, name, sim_s, wall_s or None)]`` of the steps ``report`` completed."""
    rows, at, wall = [], 0, 0.
    for entry in report['steps'] if isinstance(report['steps'], list) else []:     # a count before steps were kept
        rows.append((entry['step'], entry['name'], (entry['at'] - at) / CONTROL_HZ,
                     entry['wall_s'] - wall if 'wall_s' in entry else None))
        at, wall = entry['at'], entry.get('wall_s', 0.)
    return rows


def subtask_times(report):
    """``[(instruction, sim_s, wall_s or None, planning_s)]`` of the subtasks ``report`` ran."""
    walls = report.get('subtask_wall_s') or [None] * len(report['subtasks'])
    # Each plan to the run of its subtask whose control steps it falls in (a subtask may run again).
    planning = [0.] * len(report['subtasks'])
    for plan in report['plans']:
        planning[next(i for i, entry in enumerate(report['subtasks']) if entry['instruction'] == plan['subtask']
                      and entry['start'] <= plan['step'] <= entry['end'])] += plan['planning_s']
    return [(entry['instruction'], (entry['end'] - entry['start']) / CONTROL_HZ, wall, planned)
            for entry, wall, planned in zip(report['subtasks'], walls, planning)]


def mean(values):
    values = [value for value in values if value is not None]
    return f'{np.mean(values):8.1f}' if values else f'{"-":>8}'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('batches', nargs='+', type=Path, help='directories holding episodes/*.json')
    parser.add_argument('--sim-limit', type=float, default=60., help='s of simulated time a success should stay within')
    args = parser.parse_args(argv)
    for batch in args.batches:
        reports = [json.loads(path.read_text()) for path in sorted((batch / 'episodes').glob('*.json'))]
        succeeded = [report for report in reports if report['status'] == 'success']
        within = sum(report['duration_s'] < args.sim_limit for report in succeeded)
        print(f'== {batch}: {len(succeeded)}/{len(reports)} succeeded, mean wall {mean(r["wall_s"] for r in reports)} s, '
              f'successes within {args.sim_limit:g} s sim {within}/{len(succeeded)}, '
              f'mean sim of successes {mean(r["duration_s"] for r in succeeded)} s')
        steps = defaultdict(list)
        for report in reports:
            for step, name, sim, wall in step_times(report):
                steps[step, name].append((sim, wall))
        print(f'  {"step":<60}{"done":>6}{"sim s":>8}{"wall s":>8}')
        for (step, name), times in sorted(steps.items()):
            print(f'  {f"{step} {name}"[:58]:<60}{len(times):>6}{mean(t[0] for t in times)}{mean(t[1] for t in times)}')
        subtasks = defaultdict(list)
        for report in reports:
            for instruction, *times in subtask_times(report):
                subtasks[instruction].append(times)
        print(f'  {"subtask":<60}{"ran":>6}{"sim s":>8}{"wall s":>8}{"plan s":>8}')
        for instruction, times in subtasks.items():
            print(f'  {instruction[:58]:<60}{len(times):>6}' + ''.join(mean(t[k] for t in times) for k in range(3)))


if __name__ == '__main__':
    main()
