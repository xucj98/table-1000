"""Host-side failure reporting tests; no Blender installation required."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from table_1000.modeling.preview import main


class FailureReportTests(unittest.TestCase):
    def test_invalid_input_replaces_previous_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            blend = root / "object.blend"
            blend.touch()
            views = root / "views.json"
            report = root / "preview/acceptance.json"
            report.parent.mkdir()
            for config in ('{', '{"front.jpg":{"camera":[0,-1,0,0,-1,0]}}'):
                with self.subTest(config=config):
                    report.write_text(json.dumps({"status": "passed", "preview": {"status": "completed"}}))
                    views.write_text(config)
                    with self.assertRaises(ValueError):
                        main([str(blend), "--views", str(views)])
                    result = json.loads(report.read_text())
                    self.assertEqual(result["status"], "failed")
                    self.assertIn("error", result)
                    self.assertNotIn("preview", result)

    def test_missing_blender_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            blend = Path(directory) / "object.blend"
            blend.touch()
            with patch("table_1000.modeling.preview.shutil.which", return_value=None):
                with self.assertRaisesRegex(ValueError, "blender not found"):
                    main([str(blend)])
            self.assertEqual(json.loads((blend.parent / "preview/acceptance.json").read_text())["status"], "failed")


if __name__ == "__main__":
    unittest.main()
