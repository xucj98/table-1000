import argparse
import json
import textwrap

from table_1000.tasks import batch

FAKE = textwrap.dedent('''
    import json, sys
    from pathlib import Path
    args = sys.argv[1:]
    output = Path(args[args.index('--output') + 1])
    output.mkdir(parents=True)
    for case in json.loads(Path(args[args.index('--cases') + 1]).read_text()):
        seed, directory = case['seed'], Path(case['directory'])
        marker = directory.parent / f'calls-{seed}'
        calls = int(marker.read_text()) + 1 if marker.exists() else 1
        marker.write_text(str(calls))
        directory.mkdir(parents=True)
        if seed == 1 and calls == 1:
            raise SystemExit(-15)          # the process died; seed 1 and the rest of its chunk have no result
        status = 'FAIL' if seed == 2 else 'PASS'
        (directory / 'result.json').write_text(json.dumps({'status': status, 'seed': seed}))
    (output / 'process.json').write_text(json.dumps({'startup_s': 1.}))
''')


def test_cases_without_result_are_retried_once_and_results_are_kept(tmp_path, monkeypatch):
    monkeypatch.setattr(batch, 'START_GAP_S', 0.)
    script = tmp_path / 'fake.py'
    script.write_text(FAKE)
    out = tmp_path / 'out'
    args = argparse.Namespace(set='seeds', first=0, count=5, output=out, gpus=[0], jobs=2, per_process=3)
    batch.main(args, script)
    summary = json.loads((out / 'summary.json').read_text())
    rows = {row['case']: row for row in summary['rows']}
    assert rows['seed-0000']['status'] == 'PASS' and rows['seed-0000']['attempts'] == 1
    assert rows['seed-0001']['status'] == 'PASS' and rows['seed-0001']['attempts'] == 2
    assert rows['seed-0001']['no_result_attempts'] == 1 and (out / 'seed-0001.no-result-1').is_dir()
    assert rows['seed-0002']['status'] == 'FAIL' and rows['seed-0002']['attempts'] == 2   # not run before the crash
    assert (out / 'calls-2').read_text() == '1'                                           # FAIL is not repeated
    assert [w['cases'] for w in summary['processes']] == [3, 2, 2]
    assert not summary['processes_ok']
    batch.main(args, script)                                                             # existing results are kept
    assert (out / 'calls-0').read_text() == '1'


def test_a_second_batch_run_retries_in_new_worker_directories(tmp_path, monkeypatch):
    monkeypatch.setattr(batch, 'START_GAP_S', 0.)
    monkeypatch.setattr(batch, 'ATTEMPTS', 1)
    script = tmp_path / 'fake.py'
    script.write_text(FAKE)
    out = tmp_path / 'out'
    args = argparse.Namespace(set='seeds', first=1, count=1, output=out, gpus=[0], jobs=1, per_process=1)
    batch.main(args, script)                       # seed 1 dies on its first call
    batch.main(args, script)
    rows = json.loads((out / 'summary.json').read_text())['rows']
    assert rows[0]['status'] == 'PASS'
    assert sorted(p.name for p in (out / '_workers').iterdir()) == ['000-0', '000-1']


def test_standard_set():
    cases = batch.cases('standard')
    assert len(cases) == 19 == len({case['name'] for case in cases})
    assert {'name': 'cab-5-5_pen+5+5', 'seed': None, 'offsets': [-.05, -.05, .05, .05]} in cases
    assert len(batch.cases('corners')) == 81
