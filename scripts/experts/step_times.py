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


def profile_times(reports):
    """Mean measured categories over profiled episodes, including absent categories as zero."""
    reports = [report for report in reports if 'wall_profile' in report]
    names = sorted({name for report in reports for name in report['wall_profile']})
    return {name: {key: sum(report['wall_profile'].get(name, {}).get(key, 0.) for report in reports) / len(reports)
                   for key in ('calls', 'wall_s', 'self_s')} for name in names}


def step_profiles(report):
    """Completed steps' measured deltas, from their cumulative snapshots."""
    before = {}
    for step in report['steps'] if isinstance(report['steps'], list) else []:
        if 'wall_profile' not in step:
            continue
        after = step['wall_profile']
        yield (step['step'], step['name']), {
            name: {key: value - before.get(name, {}).get(key, 0.) for key, value in row.items()}
            for name, row in after.items()}
        before = after


def profile_detail(profiles):
    rows = profile_times([{'wall_profile': profile} for profile in profiles])
    return ', '.join(f'{name}={row["self_s"]:.2f}' for name, row in
                     sorted(rows.items(), key=lambda item: -item[1]['self_s']) if row['self_s'] >= .01)


def print_profile(reports):
    rows = profile_times(reports)
    if not rows:
        return
    print('  wall profile: seconds per profiled episode; self excludes measured children')
    print(f'  {"category":<30}{"calls":>12}{"self s":>12}{"nested s":>12}')
    for name, row in sorted(rows.items(), key=lambda item: -item[1]['self_s']):
        print(f'  {name:<30}{row["calls"]:12.1f}{row["self_s"]:12.2f}{row["wall_s"]:12.2f}')
    profiled = [report for report in reports if 'wall_profile' in report]
    other = np.mean([report['expert_wall_s'] for report in profiled]) - sum(row['self_s'] for row in rows.values())
    print(f'  {"other expert work":<30}{"":12}{other:12.2f}')
    physics = [report['timings']['physics_step_s'] for report in profiled]
    print(f'  physics/action/recording (inside control_step): {np.mean(physics):.2f} s')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('batches', nargs='+', type=Path, help='directories holding episodes/*.json')
    parser.add_argument('--sim-limit', type=float, default=60., help='s of simulated time a success should stay within')
    args = parser.parse_args(argv)
    for batch in args.batches:
        # Each seed's first run (not the other runs of --repeats).
        reports = [json.loads(path.read_text()) for path in sorted((batch / 'episodes').glob('*.json'))
                   if '-' not in path.stem]
        succeeded = [report for report in reports if report['status'] == 'success']
        within = sum(report['duration_s'] < args.sim_limit for report in succeeded)
        print(f'== {batch}: {len(succeeded)}/{len(reports)} succeeded, mean wall {mean(r["wall_s"] for r in reports)} s, '
              f'successes within {args.sim_limit:g} s sim {within}/{len(succeeded)}, '
              f'mean sim of successes {mean(r["duration_s"] for r in succeeded)} s')
        print_profile(reports)
        steps = defaultdict(list)
        for report in reports:
            for step, name, sim, wall in step_times(report):
                steps[step, name].append((sim, wall))
        profiles = defaultdict(list)
        for report in reports:
            for key, profile in step_profiles(report):
                profiles[key].append(profile)
        print(f'  {"step":<60}{"done":>6}{"sim s":>8}{"wall s":>8}')
        for (step, name), times in sorted(steps.items()):
            print(f'  {f"{step} {name}"[:58]:<60}{len(times):>6}{mean(t[0] for t in times)}{mean(t[1] for t in times)}')
            if profiles[step, name]:
                print('    self s: ' + profile_detail(profiles[step, name]))
        subtasks = defaultdict(list)
        for report in reports:
            for instruction, *times in subtask_times(report):
                subtasks[instruction].append(times)
        print(f'  {"subtask":<60}{"ran":>6}{"sim s":>8}{"wall s":>8}{"plan s":>8}')
        subprofiles = defaultdict(list)
        for report in reports:
            for entry, profile in zip(report['subtasks'], report.get('subtask_profiles', [])):
                subprofiles[entry['instruction']].append(profile)
        for instruction, times in subtasks.items():
            print(f'  {instruction[:58]:<60}{len(times):>6}' + ''.join(mean(t[k] for t in times) for k in range(3)))
            if subprofiles[instruction]:
                print('    self s: ' + profile_detail(subprofiles[instruction]))


if __name__ == '__main__':
    main()
