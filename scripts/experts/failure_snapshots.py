"""Failure snapshots of a batch: the scene state at the moment each failed episode failed, as checkpoints.

``uv run python scripts/experts/failure_snapshots.py <batch> [--seeds 4 72] [--before 0] [--name LABEL]``

For each failed seed with a recorded trajectory (``trajectories_failed.hdf5``), the state
``--before`` s before its end (the moment it failed; not before the failed step began) is written as
the checkpoint the failed step ``k`` starts from: ``checkpoints/step<k-1>.hdf5`` of a new batch
``runs/expert/<scene>/<date>-<time>-<commit>-snapshots[-<label>]/``, with the robot's start of the
episode (from the batch's checkpoint of step ``k - 1``, else the trajectory's first observation).
Each seed's group also has ``snapshot``: the batch, the trajectory sample and the seconds before the
failure. Run the failed steps from there:

``run_expert.py <scene> --physics <backend> --dev --from <snapshots> --steps k --seeds ... --repeats K``

The snapshot is the whole scene (every body's pose and velocity, every joint); the expert's step runs
again from its first subtask in that scene.
"""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def take(group, index):
    import h5py
    return {key: take(value, index) if isinstance(value, h5py.Group) else value[index:index + 1]
            for key, value in group.items()}


def main(argv=None):
    import h5py
    from table_1000.common.batch import commit, parse_seeds
    from table_1000.experts.generate import checkpoint_path, read_checkpoints
    from table_1000.experts.teleop import failed_step
    from table_1000.scenes.initial import write_tree
    from table_1000.sim import CONTROL_HZ
    from scripts.scenes.run_expert import run_name
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('batch', type=Path)
    parser.add_argument('--seeds', nargs='+', help='a:b (a to b - 1) and single seeds (default: every failed one)')
    parser.add_argument('--before', type=float, default=0., help='s before the failure')
    parser.add_argument('--name', help='a label for the new batch')
    parser.add_argument('--output', type=Path, default=ROOT / 'runs' / 'expert')
    args = parser.parse_args(argv)
    batch = args.batch.resolve()
    summary = json.loads((batch / 'summary.json').read_text())
    path = batch / 'trajectories_failed.hdf5'
    if not path.is_file():
        parser.error(f'{batch} has no failed trajectories')
    wanted = set(parse_seeds(args.seeds)) if args.seeds else None
    snapshots = defaultdict(dict)       # step k -> {seed: (attempt, state, start, record)}
    with h5py.File(path, 'r') as file:
        for name, group in file['data'].items():
            seed = int(name.split('_')[1])
            report_path = batch / 'episodes' / f'{seed}.json'
            if wanted is not None and seed not in wanted or not report_path.is_file():
                continue
            report = json.loads(report_path.read_text())
            step = failed_step(report)
            if step is None:
                continue
            length = int(group.attrs['num_samples'])
            began = next((entry['at'] for entry in report.get('steps') or [] if entry['step'] == step - 1), 0)
            index = max(began, length - 1 - round(args.before * CONTROL_HZ))
            own = checkpoint_path(batch, step - 1)
            found = read_checkpoints(own)[1] if own.is_file() else {}
            if seed in found:
                start = found[seed][2]
            else:
                obs = group['obs']
                start = {'qpos': obs['qpos'][0:1], 'base_pose': obs['base_pose'][0:1]}
            record = {'batch': str(batch), 'sample': index, 'before_failure_s': (length - 1 - index) / CONTROL_HZ,
                      'failure': report.get('failure', {}).get('reason'), 'subtask': report.get('failure', {}).get('subtask')}
            snapshots[step][seed] = (int(report.get('attempt', 0)), take(group['states'], index), start, record)
    if not snapshots:
        parser.error('no failed seed with a recorded trajectory')
    checkout = commit(ROOT)
    run = run_name(checkout, '-'.join(['snapshots', *([args.name] if args.name else [])]))
    directory = args.output / summary['scene'] / run
    directory.mkdir(parents=True)
    for step, seeds in sorted(snapshots.items()):
        out = checkpoint_path(directory, step - 1)
        out.parent.mkdir(exist_ok=True)
        with h5py.File(out, 'w') as file:
            file.attrs.update(scene=summary['scene'], robot=summary.get('robot', ''), physics=summary['physics'],
                              commit=str(summary.get('commit')), step=step - 1, name=f'failure snapshots of step {step}')
            for seed, (attempt, state, start, record) in sorted(seeds.items()):
                group = file.create_group(str(seed))
                group.attrs.update(seed=seed, attempt=attempt, at=0, snapshot=json.dumps(record))
                write_tree(group.create_group('state'), state)
                write_tree(group.create_group('start'), start)
    out = {'run': run, 'command': ' '.join(['failure_snapshots.py', *(argv if argv is not None else sys.argv[1:])]),
           'scene': summary['scene'], 'robot': summary.get('robot'), 'physics': summary['physics'], 'commit': checkout,
           'snapshots_of': str(batch), 'before_s': args.before,
           'snapshots': {str(step): sorted(seeds) for step, seeds in sorted(snapshots.items())}}
    (directory / 'summary.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    print('SNAPSHOTS', directory, json.dumps(out['snapshots']))


if __name__ == '__main__':
    main()
