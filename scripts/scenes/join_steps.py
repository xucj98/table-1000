"""Join the episodes of batches run step by step into whole episodes, one per seed.

``uv run python scripts/scenes/join_steps.py BATCH ... --output DIR``

The batches (directories ``run_expert.py`` wrote, in order) each run some steps of
the expert (``--steps k[-m]``), every one after the first from the checkpoints of the
one before (``--from``). A seed's episode in a batch is cut where the next batch's
start state was kept (the checkpoint of its step ``k - 1``) and continued by its
episode there; the chain ends where the seed has no episode in the next batch. The
joined episode keeps the first's initial state and attributes, the subtasks and
steps shifted to their place, ``status`` and ``failure`` of the last, and ``parts``:
each part's batch, commit, steps, first step in the joined episode and length.

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


def decoded(attrs, key, default):
    value = attrs.get(key, default)
    return json.loads(value) if isinstance(value, str) and value[:1] in '[{' else value


def main(argv=None):
    import h5py
    from table_1000.scenes.recordings import concatenate
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
    found = [episodes(batch) for batch in args.batches]
    # Where each batch started: the step before its first, cut out of the previous part there.
    starts = [(summary.get('steps_run') or [1])[0] for summary in summaries]
    shutil.rmtree(args.output, ignore_errors=True)
    args.output.mkdir(parents=True)
    files = {status: h5py.File(args.output / name, 'w') for status, name in
             (('success', 'trajectories.hdf5'), ('failed', 'trajectories_failed.hdf5'))}
    counts = {}
    try:
        for status_file in files.values():
            data = status_file.create_group('data')
            for key, value in found[0][next(iter(found[0]))][0]['data'].attrs.items():
                data.attrs[key] = value
        for seed in sorted(found[0]):
            parts = []
            for index, batch_episodes in enumerate(found):
                if seed not in batch_episodes:
                    break
                file, name, status = batch_episodes[seed]
                group = file['data'][name]
                parts.append((index, group, status))
                if status != 'success':
                    break
            sources, subtasks, steps, records, offset = [], [], [], [], 0
            for position, (index, group, status) in enumerate(parts):
                length = int(group.attrs['num_samples'])
                done = decoded(group.attrs, 'steps', [])
                if position + 1 < len(parts):
                    # Cut at the checkpoint the next part started from.
                    cut = [entry['at'] for entry in done if entry['step'] == starts[parts[position + 1][0]] - 1]
                    if not cut:
                        raise SystemExit(f'seed {seed}: {args.batches[index]} has no checkpoint of step '
                                         f'{starts[parts[position + 1][0]] - 1}')
                    length = cut[0]
                sources.append((group, length))
                subtasks += [{**entry, 'start': entry['start'] + offset,
                              'end': None if entry['end'] is None else min(entry['end'], length) + offset}
                             for entry in decoded(group.attrs, 'subtasks', []) if entry['start'] < length]
                steps += [{**entry, 'at': entry['at'] + offset} for entry in done if entry['at'] <= length]
                records.append({'batch': str(args.batches[index].resolve()), 'commit': summaries[index].get('commit'),
                                'steps_run': summaries[index].get('steps_run'), 'start': offset, 'length': length})
                offset += length
            last = parts[-1][1]
            status = 'success' if parts[-1][2] == 'success' and len(parts) == len(found) else 'failed'
            out = concatenate(sources, files[status], f'demo_{seed}')
            for key in ('status', 'failure'):
                if key in last.attrs:
                    out.attrs[key] = last.attrs[key]
                elif key in out.attrs:
                    del out.attrs[key]
            if status == 'failed' and 'failure' not in last.attrs:
                out.attrs['failure'] = json.dumps({'subtask': None, 'reason': 'not run further'})
            out.attrs['status'] = status
            out.attrs['success'] = status == 'success'
            out.attrs['subtasks'] = json.dumps(subtasks)
            out.attrs['steps'] = json.dumps(steps)
            out.attrs['parts'] = json.dumps(records)
            counts[status] = counts.get(status, 0) + 1
        for status_file in files.values():
            data = status_file['data']
            data.attrs['total'] = sum(int(data[name].attrs['num_samples']) for name in data)
    finally:
        for file in files.values():
            file.close()
        for batch_episodes in found:
            for file in {entry[0] for entry in batch_episodes.values()}:
                file.close()
    summary = {'scene': summaries[0].get('scene'), 'robot': summaries[0].get('robot'),
               'physics': summaries[0].get('physics'), 'joined': [str(batch.resolve()) for batch in args.batches],
               'succeeded': counts.get('success', 0), 'failed': counts.get('failed', 0),
               'steps': summaries[-1].get('steps'), 'videos': {}, 'fails': {}}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False) + '\n')
    print('JOINED', args.output, json.dumps(counts))


if __name__ == '__main__':
    main()
