"""Failed runs retain diagnostics without becoming accepted runs or masking errors."""
import csv
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from PIL import Image

from table_1000.physics.test import run_case, summary


class FailureEvidenceTests(unittest.TestCase):
    def context(self, empty=False):
        return SimpleNamespace(
            rows=[] if empty else [{'time':0., **{f'env{i}.q':0. for i in range(4)}},
                                  {'time':.004, **{f'env{i}.q':.01*i for i in range(4)}, 'late_sample':2.}],
            images=[] if empty else [Image.new('RGB',(8,8),'red')],
            units={'env0.q':'rad'}, plot_columns=['env0.q'], metrics={'speed':{'value':.07,'unit':'rad/s','label':'speed'}},
            experiment_details={'environment_failures':[{'env':i} for i in range(4)]},
            session=SimpleNamespace(steps=1,time=.004), dt=.004, close=Mock())

    def fail(self, ctx):
        raise AssertionError('original physical criterion failed')

    def run_failure(self, root, ctx, *, standard=False):
        with patch('table_1000.physics.testing.TestContext',return_value=ctx), \
             patch('table_1000.physics.test.conditions',return_value={'dt':.004,'engine':'synthetic test fixture'}):
            with self.assertRaisesRegex(AssertionError,'original physical criterion failed'):
                run_case(root/'object.usdz','probe',self.fail,root/'physics_test/isaac-gpu',None,0,{},
                         physics='gpu',num_envs=4,standard=standard)
        return root/'physics_test/isaac-gpu/probe'

    def test_assertion_saves_all_environment_samples_and_conditions_before_close(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);ctx=self.context()
            ctx.close.side_effect=lambda: self.assertTrue((root/'physics_test/isaac-gpu/probe/diagnostics/trace.csv').exists())
            folder=self.run_failure(root,ctx)
            result=json.loads((folder/'result.json').read_text())
            with (folder/'diagnostics/trace.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),2)
            self.assertEqual(rows[1]['env3.q'],'0.03')
            self.assertEqual(rows[0]['late_sample'],'')
            self.assertTrue((folder/'diagnostics/frames/00000.jpg').exists())
            self.assertEqual(result['conditions']['dt'],.004)
            self.assertEqual(result['environments']['count'],4)
            self.assertEqual(result['experiment'],ctx.experiment_details)
            self.assertEqual(result['metrics'],ctx.metrics)
            self.assertFalse(result['diagnostics']['acceptance_eligible'])
            self.assertEqual(result['execution_status'],'failed')
            self.assertNotIn('performance',result)
            self.assertFalse((folder.parent/'probe.mp4').exists())
            digest=summary(root,['probe'])
            self.assertEqual(digest['backends']['isaac-gpu']['completed'],0)
            self.assertEqual(digest['tests']['probe']['metrics'],{})
            self.assertEqual(digest['tests']['probe']['rtf'],{})
            ctx.close.assert_called_once()

    def test_empty_initialization_failure_does_not_fabricate_trace_or_engine_readback(self):
        ctx=self.context(empty=True)
        with tempfile.TemporaryDirectory() as tmp, \
             patch('table_1000.physics.testing.TestContext',return_value=ctx), \
             patch('table_1000.physics.test.conditions',side_effect=RuntimeError('no engine yet')):
            root=Path(tmp)
            with self.assertRaisesRegex(AssertionError,'original physical criterion failed'):
                run_case(root/'object.usdz','probe',self.fail,root/'physics_test',None,0,{},physics='gpu',num_envs=4)
            folder=root/'physics_test/probe'
            result=json.loads((folder/'result.json').read_text())
            self.assertFalse((folder/'diagnostics/trace.csv').exists())
            self.assertEqual(result['diagnostics']['sample_count'],0)
            self.assertFalse(result['conditions']['complete'])
            self.assertIn('no engine yet',result['diagnostics']['capture_errors']['conditions'])

    def test_bad_frame_and_unserializable_metric_do_not_discard_trace_or_experiment(self):
        ctx=self.context();ctx.images=[object()];ctx.metrics={'bad':object()}
        with tempfile.TemporaryDirectory() as tmp:
            folder=self.run_failure(Path(tmp),ctx)
            result=json.loads((folder/'result.json').read_text())
            self.assertTrue((folder/'diagnostics/trace.csv').exists())
            self.assertIn('metrics',result['diagnostics']['capture_errors'])
            self.assertIn('frame_0',result['diagnostics']['capture_errors'])
            self.assertEqual(result['experiment'],ctx.experiment_details)

    def test_diagnostic_write_and_cleanup_errors_preserve_original_exception(self):
        ctx=self.context();ctx.close.side_effect=RuntimeError('cleanup failed')
        with tempfile.TemporaryDirectory() as tmp, \
             patch('table_1000.physics.test.retain_failure',side_effect=OSError('disk unavailable')), \
             patch('sys.stderr'):
            folder=self.run_failure(Path(tmp),ctx)
            result=json.loads((folder/'result.json').read_text())
            self.assertIn('original physical criterion failed',result['error'])
            self.assertNotIn('cleanup failed',result['error'])
            self.assertEqual(result['execution_status'],'failed')

    def test_failed_warmup_retains_samples_without_claiming_completed_benchmark(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=self.run_failure(Path(tmp),self.context(),standard=True)
            result=json.loads((folder/'result.json').read_text())
            self.assertEqual(result['failed_phase'],'benchmark_warmup')
            self.assertTrue((folder/'diagnostics/trace.csv').exists())
            self.assertNotIn('benchmark_status',result)
            self.assertNotIn('performance',result)

    def test_completed_benchmark_survives_video_assertion_with_diagnostics(self):
        measured={'rounds':[{'simulated_seconds':10,'RTF':2.}]*3,'median_RTF':2.}
        with tempfile.TemporaryDirectory() as tmp, \
             patch('table_1000.physics.test.benchmark',return_value=measured):
            folder=self.run_failure(Path(tmp),self.context(),standard=True)
            result=json.loads((folder/'result.json').read_text())
            self.assertEqual(result['performance'],{'no_render':measured})
            self.assertEqual(result['benchmark_status'],'completed')
            self.assertTrue((folder/'diagnostics/trace.csv').exists())
            self.assertEqual(result['execution_status'],'failed')

    def test_failed_timed_round_preserves_its_own_samples_after_warmup(self):
        warm=self.context();warm.session.time=1.
        measured=self.context()
        def experiment(ctx):
            if ctx is measured:
                self.fail(ctx)
        with tempfile.TemporaryDirectory() as tmp, \
             patch('table_1000.physics.testing.TestContext',side_effect=[warm,measured]), \
             patch('table_1000.physics.test.conditions',return_value={'dt':.004}):
            root=Path(tmp)
            with self.assertRaisesRegex(AssertionError,'original physical criterion failed'):
                run_case(root/'object.usdz','probe',experiment,root/'physics_test',None,0,{},standard=True)
            result=json.loads((root/'physics_test/probe/result.json').read_text())
            self.assertEqual(result['failed_phase'],'benchmark_round')
            self.assertEqual(result['diagnostics']['sample_count'],len(measured.rows))
            self.assertNotIn('performance',result)
            warm.close.assert_called_once()
            measured.close.assert_called_once()

    def test_success_keeps_normal_trace_and_has_no_failure_diagnostics(self):
        ctx=self.context();ctx.rows=[{'time':0.,'q':0.},{'time':.004,'q':.01}]
        ctx.session.instances={'asset':SimpleNamespace(view=SimpleNamespace(replica_deviation=lambda:0.))}
        ctx.camera_config={'fps':25};ctx.video_started=0.;ctx.integrating_seconds=.1
        ctx.preparation_seconds=0.;ctx.grips=[]
        with tempfile.TemporaryDirectory() as tmp, \
             patch('table_1000.physics.testing.TestContext',return_value=ctx), \
             patch('table_1000.physics.test.conditions',return_value={'isaac_version':['test']}), \
             patch('table_1000.physics.test.subprocess.run'), \
             patch('table_1000.physics.plots.plots'):
            root=Path(tmp)
            result=run_case(root/'object.usdz','probe',lambda c:None,root/'physics_test',None,0,{})
            self.assertEqual(result['execution_status'],'completed')
            self.assertTrue((root/'physics_test/probe/trace.csv').exists())
            self.assertFalse((root/'physics_test/probe/diagnostics').exists())
            ctx.close.assert_called_once()

    def test_gpu_grip_metadata_does_not_claim_zero_means_no_reaction(self):
        from table_1000.physics.testing import TestContext
        context=SimpleNamespace(physics='gpu',grips=[SimpleNamespace(name='guide',gains={'held_mass_kg':.014},
            body=SimpleNamespace(name='body',instance=SimpleNamespace(name='asset')))])
        result=TestContext.grip_details(context)['guide']
        self.assertTrue(result['wrench_readback']['zero_does_not_prove_no_support_wrench'])
        self.assertEqual(result['held_mass_kg'],.014)
        self.assertEqual(result['body'],'asset.body')
