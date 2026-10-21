"""Physics test runs: per-case results, batch scheduling and the per-asset summary."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from table_1000.physics.test import main, run_case, run_jobs, summary


class CaseResultsTests(unittest.TestCase):
    def test_definition_import_failure_closes_kit_with_nonzero_status(self):
        # Kit's close() normally terminates the process before Python can
        # propagate an exception. It must receive the failure explicitly.
        with tempfile.TemporaryDirectory() as temporary:
            app = Mock()
            with patch('table_1000.sim.isaaclab.app.start_app', return_value=app), \
                 patch('table_1000.physics.test.subprocess.run', return_value=Mock(stdout='GPU')), \
                 patch('table_1000.physics.test.load_tests', side_effect=ImportError('broken definition')):
                with self.assertRaisesRegex(ImportError, 'broken definition'):
                    main(['--worker', temporary, '--physics', 'isaac-cpu', '--gpu', '0',
                          '--script', str(Path(temporary) / 'physics_test.py')])
            app.close.assert_called_once_with(exit_code=1)

    def test_benchmark_failure_removes_old_case_results(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / 'physics_test'
            case = output / 'pull'
            (case / 'frames').mkdir(parents=True)
            (case / 'frames' / 'old.jpg').write_bytes(b'old frame')
            (case / 'result.json').write_text('{"execution_status":"completed"}')
            video = output / 'pull.mp4'
            video.write_bytes(b'old video')
            other = output / 'close.mp4'
            other.write_bytes(b'other result')
            source = root / 'physics_test.py'
            source.write_text('TESTS = {}\n')
            with patch('table_1000.physics.test.benchmark', side_effect=AssertionError('benchmark failed')), \
                 patch('table_1000.physics.test.run_video') as render:
                with self.assertRaisesRegex(AssertionError, 'benchmark failed'):
                    run_case(root / 'object.usda', 'pull', None, output, None, 0, {}, standard=True)
                render.assert_not_called()
            self.assertFalse(video.exists())
            self.assertFalse((case / 'frames').exists())
            result = json.loads((case / 'result.json').read_text())
            self.assertEqual(result['execution_status'], 'failed')
            self.assertIn('benchmark failed', result['error'])
            self.assertEqual(other.read_bytes(), b'other result')
            self.assertEqual(source.read_text(), 'TESTS = {}\n')

    def test_video_failure_keeps_completed_independent_benchmark(self):
        measured = {'median_RTF': 2.5, 'rounds': [{'RTF': 2.5}]}
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / 'physics_test'
            with patch('table_1000.physics.test.benchmark', return_value=measured), \
                 patch('table_1000.physics.test.run_video', side_effect=RuntimeError('empty RGB')):
                with self.assertRaisesRegex(RuntimeError, 'empty RGB'):
                    run_case(root / 'object.usda', 'pull', None, output, None, 0, 0, standard=True)
            result = json.loads((output / 'pull' / 'result.json').read_text())
        self.assertEqual(result['execution_status'], 'failed')
        self.assertEqual(result['failed_phase'], 'video')
        self.assertEqual(result['benchmark_status'], 'completed')
        self.assertEqual(result['performance'], {'no_render': measured})
        self.assertNotIn('video_end_to_end', result['performance'])

    def test_default_run_skips_the_standard_benchmark(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'physics_test'
            with patch('table_1000.physics.test.benchmark') as measure, \
                 patch('table_1000.physics.test.run_video', return_value={}) as render:
                run_case(Path(temporary) / 'object.usda', 'pull', None, output, None, 0, {})
            measure.assert_not_called()
            self.assertIsNone(render.call_args.args[6])

    def test_jobs_share_the_listed_gpus(self):
        def fake(command, **kwargs):
            return Mock(returncode=int(command[0] == 'bad'))
        with tempfile.TemporaryDirectory() as temporary, \
             patch('table_1000.physics.test.subprocess.run', side_effect=fake) as run:
            jobs = [([name], Path(temporary) / name / 'run.log', name) for name in ('a', 'bad', 'c')]
            self.assertEqual(run_jobs(jobs, [4, 5]), ['bad'])
        self.assertEqual({call.args[0][-1] for call in run.call_args_list} <= {'4', '5'}, True)
        self.assertEqual(len(run.call_args_list), 3)

    def test_batch_runs_every_asset_backend_and_writes_summaries(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for reference in ('pen/000002', 'cabinet/000000'):
                (root / reference).mkdir(parents=True)
                (root / reference / 'physics_test.py').write_text("TESTS = {'drop': None}\n")
            with patch('table_1000.physics.test.run_jobs', return_value=[]) as jobs:
                main(['--source', temporary, '--physics', 'isaac-cpu', 'isaac-gpu', '--gpus', '1', '2'])
            commands = jobs.call_args.args[0]
            self.assertEqual(jobs.call_args.args[1], [1, 2])
            self.assertEqual(sorted(label for _, _, label in commands),
                             ['cabinet/000000 isaac-cpu', 'cabinet/000000 isaac-gpu', 'pen/000002 isaac-cpu', 'pen/000002 isaac-gpu'])
            for reference in ('pen/000002', 'cabinet/000000'):
                self.assertTrue((root / reference / 'physics_test/summary.json').exists())

    def test_summary_lines_up_backends_per_metric(self):
        with tempfile.TemporaryDirectory() as temporary:
            asset = Path(temporary) / 'pen/000002'
            def write(backend, name, result):
                path = asset / 'physics_test' / backend / name / 'result.json'
                path.parent.mkdir(parents=True)
                path.write_text(json.dumps(result))
            def completed(push, envs):
                return {'execution_status': 'completed', 'conditions': {'isaac_version': ['5.1.0']},
                        'environments': {'count': envs},
                        'metrics': {'seating_push': {'label': '合帽力', 'unit': 'N', 'value': push}},
                        'performance': {'no_render': {'RTF': .57123}, 'video_end_to_end': {'RTF': .34}}}
            write('isaac-cpu', 'close', completed(3.7412, 1))
            write('isaac-gpu', 'close', completed(2.14, 4))
            write('isaac-gpu', 'drop', {'execution_status': 'failed', 'error': 'Traceback\nAssertionError: pen did not settle\n'})
            write('isaac-cpu', 'removed', completed(1, 1))
            result = summary(asset, ['close', 'drop'])
        self.assertEqual(result['asset'], 'pen/000002')
        self.assertEqual(result['backends']['isaac-cpu'], {'completed': 1, 'failed': [], 'engine': 'Isaac Sim 5.1.0 CPU PhysX', 'environments': 1})
        self.assertEqual(result['backends']['isaac-gpu']['failed'], ['drop'])
        self.assertEqual(result['tests']['close']['metrics']['seating_push'],
                         {'label': '合帽力', 'unit': 'N', 'isaac-cpu': 3.741, 'isaac-gpu': 2.14})
        self.assertEqual(result['tests']['close']['rtf']['isaac-cpu'], {'physics': .571, 'video': .34})
        self.assertEqual(result['tests']['drop']['error'], {'isaac-gpu': 'AssertionError: pen did not settle'})
        self.assertEqual(list(result['tests']), ['close', 'drop'])

    def test_failed_backend_never_reports_old_results(self):
        with tempfile.TemporaryDirectory() as temporary:
            asset = Path(temporary) / 'pen/000002'
            asset.mkdir(parents=True)
            (asset / 'physics_test.py').write_text("TESTS = {'close': None, 'drop': None}\n")
            for name in ('close', 'drop'):
                path = asset / 'physics_test/isaac-gpu' / name / 'result.json'
                path.parent.mkdir(parents=True)
                path.write_text(json.dumps({'execution_status': 'completed', 'conditions': {'isaac_version': ['5.1.0']},
                                            'environments': {'count': 4}, 'metrics': {},
                                            'performance': {'no_render': {'RTF': 1}, 'video_end_to_end': {'RTF': 1}}}))
            # The worker dies at startup and writes nothing.
            with patch('table_1000.physics.test.run_jobs', return_value=['pen/000002 isaac-gpu']):
                with self.assertRaises(SystemExit):
                    main(['--source', temporary, '--physics', 'isaac-gpu', '--tests', 'close'])
            result = json.loads((asset / 'physics_test/summary.json').read_text())
        self.assertEqual(result['tests']['close']['status'], {'isaac-gpu': 'failed'})
        self.assertIn('not run', result['tests']['close']['error']['isaac-gpu'])
        self.assertEqual(result['tests']['drop']['status'], {'isaac-gpu': 'completed'})
        self.assertEqual(result['backends']['isaac-gpu']['failed'], ['close'])

    def test_broken_definition_fails_only_its_asset(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for reference, text in (('pen/000002', "TESTS = {'drop': None}\n"), ('cabinet/000000', "raise ImportError('broken')\n")):
                (root / reference).mkdir(parents=True)
                (root / reference / 'physics_test.py').write_text(text)
            with patch('table_1000.physics.test.run_jobs', return_value=[]) as jobs, \
                 patch('traceback.print_exc'):
                with self.assertRaisesRegex(SystemExit, 'cabinet/000000'):
                    main(['--source', temporary, '--physics', 'isaac-cpu'])
            self.assertEqual([label for _, _, label in jobs.call_args.args[0]], ['pen/000002 isaac-cpu'])
            broken = json.loads((root / 'cabinet/000000/physics_test/summary.json').read_text())
            self.assertIn('broken', broken['error'])
            self.assertTrue((root / 'pen/000002/physics_test/summary.json').exists())

    def test_plots_only_fails_on_a_broken_definition(self):
        with tempfile.TemporaryDirectory() as temporary:
            asset = Path(temporary) / 'bad/000000'
            asset.mkdir(parents=True)
            (asset / 'physics_test.py').write_text("raise ImportError('broken')\n")
            with patch('traceback.print_exc'):
                with self.assertRaisesRegex(SystemExit, 'bad/000000'):
                    main(['--source', temporary, '--assets', 'bad/000000', '--physics', 'isaac-cpu', '--plots-only'])
