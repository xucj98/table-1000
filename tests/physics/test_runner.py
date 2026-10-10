"""A failed benchmark must replace results from the previous execution."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from table_1000.physics.test import run_case


class CaseResultsTests(unittest.TestCase):
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
                    run_case(root / 'object.usda', 'pull', None, output, None, 0, {})
                render.assert_not_called()
            self.assertFalse(video.exists())
            self.assertFalse((case / 'frames').exists())
            result = json.loads((case / 'result.json').read_text())
            self.assertEqual(result['execution_status'], 'failed')
            self.assertIn('benchmark failed', result['error'])
            self.assertEqual(other.read_bytes(), b'other result')
            self.assertEqual(source.read_text(), 'TESTS = {}\n')
