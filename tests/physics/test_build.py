"""The build entry uses the author hook and preserves its source snapshot."""

from pathlib import Path
import tempfile
import unittest

from table_1000.physics.build import author_physics


class PhysicsAuthorTest(unittest.TestCase):
    def test_source_generates_layer_and_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory) / 'source', Path(directory) / 'asset'
            source.mkdir()
            output.mkdir()
            script = "def author(geometry_path, output_path):\n    output_path.write_text(geometry_path.read_text() + ':physics')\n"
            (source / 'physics.py').write_text(script)
            (source / 'physics.usda').write_text('unused old layer')
            (output / 'geometry.usdc').write_text('geometry')
            author_physics(source, output)
            self.assertEqual((output / 'physics.usda').read_text(), 'geometry:physics')
            self.assertEqual((output / 'physics.py').read_bytes(), (source / 'physics.py').read_bytes())


if __name__ == '__main__':
    unittest.main()
