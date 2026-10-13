import argparse
import json
import textwrap

from table_1000.tasks import batch

FAKE = textwrap.dedent('''
    import json, sys
    from pathlib import Path
    args = sys.argv[1:]
    output = Path(args[args.index('--output') + 1])
    seed = int(args[args.index('--seed') + 1])
    marker = output.parent / f'calls-{seed}'
    calls = int(marker.read_text()) + 1 if marker.exists() else 1
    marker.write_text(str(calls))
    output.mkdir(parents=True)
    if seed == 1 and calls == 1:
        raise SystemExit(-15)          # died before writing a result
    status = 'FAIL' if seed == 2 else 'PASS'
    (output / 'result.json').write_text(json.dumps({'status': status, 'seed': seed}))
''')


def test_no_result_runs_are_retried_once_and_results_are_kept(tmp_path, monkeypatch):
    monkeypatch.setattr(batch, 'START_GAP_S', 0.)
    script = tmp_path / 'fake.py'
    script.write_text(FAKE)
    out = tmp_path / 'out'
    args = argparse.Namespace(set='seeds', first=0, count=3, output=out, gpus=[0], jobs=2)
    batch.main(args, script)
    rows = {row['case']: row for row in json.loads((out / 'summary.json').read_text())['rows']}
    assert rows['seed-0000']['status'] == 'PASS' and rows['seed-0000']['attempts'] == 1
    assert rows['seed-0001']['status'] == 'PASS' and rows['seed-0001']['attempts'] == 2
    assert rows['seed-0001']['no_result_attempts'] == 1 and (out / 'seed-0001.no-result-1').is_dir()
    assert rows['seed-0002']['status'] == 'FAIL' and rows['seed-0002']['attempts'] == 1   # FAIL is not repeated
    batch.main(args, script)                                                             # existing results are kept
    assert (out / 'calls-0').read_text() == '1'
