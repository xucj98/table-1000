"""A failed rerun must recreate the dedicated preview directory."""

import json
from pathlib import Path
import tempfile
import unittest

from table_1000.modeling.preview import main


class FailureReportTests(unittest.TestCase):
    def test_invalid_config_removes_previous_success(self):
        for flags in ([], ["--check-only"]):
            with self.subTest(flags=flags), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                inputs = {
                    "object.blend": b"saved model",
                    "views.json": b'{"front.jpg":{"camera":[0,-1,0,0,-1,0]}}',
                }
                for name, content in inputs.items():
                    (root / name).write_bytes(content)
                output = root / "preview"
                (output / "nested").mkdir(parents=True)
                report = output / "acceptance.json"
                report.write_text(json.dumps({"status": "passed", "preview": {"status": "completed"}}))
                old_media = [output / "front.jpg", output / "motion.mp4", output / "nested/frame.jpg"]
                for path in old_media:
                    path.write_bytes(b"previous preview")
                with self.assertRaises(ValueError):
                    main([str(root / "object.blend"), "--views", str(root / "views.json"),
                          "--output", str(output), *flags])
                self.assertTrue(output.is_dir())
                self.assertEqual(list(output.iterdir()), [])
                for name, content in inputs.items():
                    self.assertEqual((root / name).read_bytes(), content)


if __name__ == "__main__":
    unittest.main()
