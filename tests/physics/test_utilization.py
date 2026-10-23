"""GPU sampler errors are disclosed and its child is always reaped."""
import subprocess
import threading
import unittest
from unittest.mock import Mock, patch

from table_1000.physics.utilization import Utilization


class UtilizationTests(unittest.TestCase):
    def make_sampler(self):
        sampler=Utilization.__new__(Utilization)
        sampler.gpu=2
        sampler.active=True
        sampler.samples=[(1.0,12.0,44.0,9000.0)]
        sampler.lock=threading.Lock()
        sampler.errors=[]
        return sampler

    def test_process_query_timeout_is_reported_without_fabricated_zero(self):
        sampler=self.make_sampler()
        timeout=subprocess.TimeoutExpired('nvidia-smi',5)
        with patch('table_1000.physics.utilization.subprocess.run',side_effect=timeout):
            result=sampler.summary()
        self.assertEqual(result['status'],'partial')
        self.assertIsNone(result['compute_processes_on_gpu'])
        self.assertEqual(result['gpu_utilization_percent'],{'mean':44.0,'max':44.0})
        self.assertIn('TimeoutExpired',result['errors'][0])

    def test_close_waits_for_and_reaps_monitor_child(self):
        sampler=self.make_sampler()
        sampler.stopped=threading.Event()
        sampler.smi=Mock(pid=123,stdout=Mock())
        sampler.reader=Mock()
        sampler.reader.is_alive.return_value=False
        sampler.sampler=Mock()
        sampler.sampler.is_alive.return_value=False
        status=sampler.close()
        sampler.smi.terminate.assert_called_once()
        sampler.smi.wait.assert_called_once_with(timeout=2)
        self.assertTrue(status['child_reaped'])
        self.assertTrue(status['reader_stopped'])
        self.assertTrue(status['sampler_stopped'])

    def test_unresponsive_monitor_is_killed_and_reaped(self):
        sampler=self.make_sampler()
        sampler.stopped=threading.Event()
        sampler.smi=Mock(pid=124,stdout=Mock())
        sampler.smi.wait.side_effect=[subprocess.TimeoutExpired('nvidia-smi',2),None]
        sampler.reader=Mock()
        sampler.reader.is_alive.return_value=False
        sampler.sampler=Mock()
        sampler.sampler.is_alive.return_value=False
        status=sampler.close()
        sampler.smi.kill.assert_called_once()
        self.assertEqual(sampler.smi.wait.call_count,2)
        self.assertEqual(status['status'],'killed_after_timeout')
        self.assertTrue(status['child_reaped'])


if __name__=='__main__':
    unittest.main()
