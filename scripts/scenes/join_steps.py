"""Join the episodes of batches run step by step into whole episodes, one per seed.

``uv run python scripts/scenes/join_steps.py BATCH ... --output DIR``

The batches (directories ``run_expert.py`` wrote, in order) each run some steps of
the expert (``--steps k[-m]``): the first from the initial states, every other from
the checkpoints of the one before (``--from``). A seed's episode in a batch is cut
where the next batch's start state was kept (the checkpoint of its step ``k - 1``)
and continued by its episode there, also when the episode failed after that
checkpoint (the next batch ran the step again); the chain ends where the seed has no
such continuation. The joined episode keeps the first's initial state and attributes,
the subtasks and steps shifted to their place, ``status`` and ``failure`` of the last
(``failed`` with reason ``not run further`` when it succeeded before the last batch),
and ``parts``: each part's batch, commit, steps, first step in the joined episode and
length.

``DIR`` gets ``trajectories.hdf5`` and ``trajectories_failed.hdf5`` (by the last
part's status) and a ``summary.json``, so ``render.py --input <parent of scene>
--physics <name>`` renders them when ``DIR`` is ``<input>/<scene>/<name>``.
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def episodes(batch):
    """``{seed: (file, name, status)}`` of a batch's trajectory files (open h5py files)."""
    import h5py
    found = {}
    for kind in ('trajectories', 'trajectories_failed'):
        path = batch / f'{kind}.hdf5'
        if path.is_file():
            file = h5py.File(path, 'r')
            for name in file['data']:
                found[int(name.split('_')[1])] = (file, name, file['data'][name].attrs['status'])
    return found


def main(argv=None):
    import h5py
    from table_1000.common.batch import seeds_record
    from table_1000.scenes.recordings import concatenate, decoded_attrs
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('batches', type=Path, nargs='+', help='batch directories, in step order')
    parser.add_argument('--output', type=Path, required=True, help='the joined batch directory (replaced)')
    args = parser.parse_args(argv)
    summaries = []
    for index, batch in enumerate(args.batches):
        if not (batch / 'summary.json').is_file():
            parser.error(f'{batch} has no summary.json')
        summary = json.loads((batch / 'summary.json').read_text())
        if index and Path(summary.get('from') or '').resolve() != args.batches[index - 1].resolve():
            parser.error(f'{batch} did not start from {args.batches[index - 1]}')
        summaries.append(summary)
    first = summaries[0]
    if (first.get('steps_run') or [1])[0] != 1 or first.get('from'):
        parser.error(f'{args.batches[0]} does not start from the initial states (step 1)')
    found = [episodes(batch) for batch in args.batches]
    if not found[0]:
        parser.error(f'{args.batches[0]} has no episodes')
    # Where each batch started: the step before its first, cut out of the previous part there.
    starts = [(summary.get('steps_run') or [1])[0] for summary in summaries]
    # Each seed's chain of (batch, episode group, status, length): a part goes on in the next batch
    # when that batch ran the seed from the part's checkpoint (also after a failure later in the part).
    chains = {}
    for seed in sorted(found[0]):
        chain, index = [], 0
        while True:
            file, name, status = found[index][seed]
            group = file['data'][name]
            done = decoded_attrs(group).get('steps', [])
            cut = None
            if index + 1 < len(found) and seed in found[index + 1]:
                cut = next((entry['at'] for entry in done if entry['step'] == starts[index + 1] - 1), None)
            if cut is None:
                chain.append((index, group, status, int(group.attrs['num_samples']), done))
                break
            chain.append((index, group, status, cut, done))
            index += 1
        chains[seed] = chain
    shutil.rmtree(args.output, ignore_errors=True)
    args.output.mkdir(parents=True)
    files = {kind: h5py.File(args.output / f'{kind}.hdf5', 'w') for kind in ('trajectories', 'trajectories_failed')}
    counts = {}
    try:
        env_file = next(entry[0] for batch_episodes in found for entry in batch_episodes.values())
        for out_file in files.values():
            out_file.create_group('data').attrs.update(dict(env_file['data'].attrs))
        for seed, chain in chains.items():
            sources, subtasks, steps, records, offset = [], [], [], [], 0
            for index, group, _, length, done in chain:
                sources.append((group, length))
                subtasks += [{**entry, 'start': entry['start'] + offset,
                              'end': None if entry['end'] is None else min(entry['end'], length) + offset}
                             for entry in decoded_attrs(group).get('subtasks', []) if entry['start'] < length]
                steps += [{**entry, 'at': entry['at'] + offset} for entry in done if entry['at'] <= length]
                records.append({'batch': str(args.batches[index].resolve()), 'commit': summaries[index].get('commit'),
                                'steps_run': summaries[index].get('steps_run'), 'start': offset, 'length': length})
                offset += length
            index, last, status = chain[-1][:3]
            if status == 'success' and index + 1 < len(found):
                status = 'not run further'
            out = concatenate(sources, files['trajectories' if status == 'success' else 'trajectories_failed'],
                              f'demo_{seed}')
            for key in ('status', 'failure'):
                if key in last.attrs:
                    out.attrs[key] = last.attrs[key]
                elif key in out.attrs:
                    del out.attrs[key]
            if status == 'not run further':
                status = 'failed'
                out.attrs['failure'] = json.dumps({'subtask': None, 'reason': 'not run further'})
            out.attrs['status'] = status       # 'error' (the expert's code) stays apart from 'failed'
            out.attrs['success'] = status == 'success'
            out.attrs['subtasks'] = json.dumps(subtasks)
            out.attrs['steps'] = json.dumps(steps)
            out.attrs['parts'] = json.dumps(records)
            counts[status] = counts.get(status, 0) + 1
        for out_file in files.values():
            data = out_file['data']
            data.attrs['total'] = sum(int(data[name].attrs['num_samples']) for name in data)
    finally:
        for file in files.values():
            file.close()
        for batch_episodes in found:
            for file in {entry[0] for entry in batch_episodes.values()}:
                file.close()
    summary = {'scene': summaries[0].get('scene'), 'robot': summaries[0].get('robot'),
               'physics': summaries[0].get('physics'), 'joined': [str(batch.resolve()) for batch in args.batches],
               'seeds': seeds_record(sorted(chains)), 'succeeded': counts.get('success', 0),
               'failed': counts.get('failed', 0), 'error': counts.get('error', 0),
               'steps': summaries[-1].get('steps'), 'videos': {}, 'fails': {}}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False) + '\n')
    print('JOINED', args.output, json.dumps(counts))


if __name__ == '__main__':
    main()
