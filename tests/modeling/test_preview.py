"""A failed rerun must invalidate the previous acceptance report."""

import json
from pathlib import Path
import tempfile
import unittest

from table_1000.modeling.preview import main


class FailureReportTests(unittest.TestCase):
    def test_invalid_config_removes_previous_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            blend = root / "object.blend"
            blend.touch()
            views = root / "views.json"
            report = root / "preview/acceptance.json"
            report.parent.mkdir()
            report.write_text(json.dumps({"status": "passed", "preview": {"status": "completed"}}))
            views.write_text('{"front.jpg":{"camera":[0,-1,0,0,-1,0]}}')
            with self.assertRaises(ValueError):
                main([str(blend), "--views", str(views)])
            self.assertFalse(report.exists())


if __name__ == "__main__":
    unittest.main()
