"""The parent records real child exits and bounds only the post-save shutdown."""

import json
from pathlib import Path
import signal
import sys
import tempfile
import time
import unittest

from table_1000.runtime.process import LIFECYCLE, start_detached, supervise

MARK = ('from table_1000.runtime.process import mark_shutdown;'
        'mark_shutdown(".", "FAIL");')


class SuperviseTest(unittest.TestCase):
    def run_child(self, source, **options):
        with tempfile.TemporaryDirectory() as folder:
            code = supervise([sys.executable, '-c', source], cwd=folder, output=folder,
                             env={'PYTHONPATH': str(Path(__file__).resolve().parents[2])},
                             interval_s=.01, **options)
            record = json.loads((Path(folder) / LIFECYCLE).read_text())
        self.assertEqual(record['status'], 'exited')
        self.assertEqual(record['returncode'], code)
        return record

    def test_exit_codes_and_signals_are_recorded(self):
        for source, code in (('pass', 0), ('raise SystemExit(7)', 7),
                             ('import os,signal;os.kill(os.getpid(),signal.SIGTERM)', -signal.SIGTERM)):
            with self.subTest(source=source):
                record = self.run_child(source)
                self.assertEqual(record['returncode'], code)
                self.assertEqual(record['signal'], -code if code < 0 else None)
                self.assertEqual(record['interventions'], [])

    def test_no_deadline_before_shutdown_marker(self):
        record = self.run_child('import time;time.sleep(.2)', shutdown_timeout_s=.02, term_grace_s=.02)
        self.assertEqual(record['returncode'], 0)
        self.assertEqual(record['interventions'], [])

    def test_hung_close_after_saved_fail_is_terminated(self):
        record = self.run_child(MARK + 'import time;time.sleep(10)', shutdown_timeout_s=.05, term_grace_s=.05)
        self.assertEqual(record['returncode'], -signal.SIGTERM)
        self.assertEqual(record['shutdown_marker']['task_status'], 'FAIL')
        self.assertEqual([row['signal'] for row in record['interventions']], [signal.SIGTERM])

    def test_ignored_term_escalates_to_kill_of_own_group(self):
        source = MARK + 'import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(10)'
        record = self.run_child(source, shutdown_timeout_s=.05, term_grace_s=.05)
        self.assertEqual(record['returncode'], -signal.SIGKILL)
        self.assertEqual([row['signal'] for row in record['interventions']], [signal.SIGTERM, signal.SIGKILL])
        self.assertTrue(all(row['target_process_group'] == record['child_pid'] for row in record['interventions']))

    def test_normal_close_after_marker_is_left_alone(self):
        record = self.run_child(MARK + 'import time;time.sleep(.1)', shutdown_timeout_s=5.)
        self.assertEqual(record['returncode'], 0)
        self.assertEqual(record['interventions'], [])


class DetachedTest(unittest.TestCase):
    def test_parent_outlives_caller_and_records_exit(self):
        with tempfile.TemporaryDirectory() as folder:
            start_detached([sys.executable, '-c', "print('detached')"], cwd=folder, output=folder)
            record = {}
            deadline = time.monotonic() + 10
            while record.get('status') != 'exited' and time.monotonic() < deadline:
                time.sleep(.05)
                try:
                    record = json.loads((Path(folder) / LIFECYCLE).read_text())
                except (OSError, ValueError):
                    pass
            self.assertEqual(record.get('returncode'), 0)
            self.assertEqual((Path(folder) / 'stdout.log').read_text().strip(), 'detached')


if __name__ == '__main__':
    unittest.main()
