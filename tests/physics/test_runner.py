"""A failed benchmark must replace results from the previous execution."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from table_1000.physics.test import main, run_case


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
                    main([temporary, '--output', temporary])
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

    def test_default_run_skips_the_standard_benchmark(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'physics_test'
            with patch('table_1000.physics.test.benchmark') as measure, \
                 patch('table_1000.physics.test.run_video', return_value={}) as render:
                run_case(Path(temporary) / 'object.usda', 'pull', None, output, None, 0, {})
            measure.assert_not_called()
            self.assertIsNone(render.call_args.args[6])
