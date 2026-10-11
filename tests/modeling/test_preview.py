"""Batch selection and failed reruns use each asset's own preview directory."""

import json
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest
from unittest.mock import patch

from table_1000.modeling.preview import main


class PreviewTests(unittest.TestCase):
    def test_direct_worker_script_imports_package(self):
        script = Path(__file__).resolve().parents[2] / "table_1000/modeling/preview.py"
        subprocess.run([sys.executable, "-I", str(script), "--help"],
                       check=True, capture_output=True)

    def test_discovery_and_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            references = ['cabinet/000000', 'pen/000000']
            for reference in references:
                asset = root / reference
                asset.mkdir(parents=True)
                (asset / 'object.blend').write_bytes(b'model')
                (asset / 'preview.json').write_text(json.dumps({'front.jpg': {'camera': [0,-1,0,0,0,1]}}))
            for selected in (None, references[::-1]):
                with self.subTest(selected=selected), patch('table_1000.modeling.preview.run_worker') as worker:
                    main(['--source', directory, '--worker', '--device', 'cpu',
                          *(['--assets', *selected] if selected is not None else [])])
                    self.assertEqual([c.args[0] for c in worker.call_args_list],
                                     [root / ref / 'object.blend' for ref in (selected or references)])
                    for call in worker.call_args_list:
                        self.assertEqual(call.args[2], call.args[0].parent / 'preview')
                        self.assertEqual(call.kwargs, {'device': 'cpu'})

    def test_missing_or_invalid_config_clears_selected_preview(self):
        for config, error in [(None, FileNotFoundError),
                              ('{"front.jpg":{"camera":[0,-1,0,0,-1,0]}}', ValueError)]:
            with self.subTest(config=config), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                asset = root / 'pen/000000'
                output = asset / 'preview'
                (output / 'nested').mkdir(parents=True)
                (asset / 'object.blend').write_bytes(b'model')
                if config is not None:
                    (asset / 'preview.json').write_text(config)
                for name in ['front.jpg', 'motion.mp4', 'acceptance.json', 'nested/frame.jpg']:
                    (output / name).write_bytes(b'old result')
                other = root / 'other/preview/keep.jpg'
                other.parent.mkdir(parents=True)
                other.write_bytes(b'other result')
                with self.assertRaises(error):
                    main(['--source', directory, '--assets', 'pen/000000', '--worker'])
                self.assertEqual(list(output.iterdir()), [])
                self.assertEqual((asset / 'object.blend').read_bytes(), b'model')
                self.assertEqual(other.read_bytes(), b'other result')


if __name__ == '__main__':
    unittest.main()
