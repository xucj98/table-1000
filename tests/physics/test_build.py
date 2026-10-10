"""The build entry uses the author hook and preserves its source snapshot."""

from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from table_1000.physics.build import author_physics, main


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


class PhysicsBatchTest(unittest.TestCase):
    def test_discovery_and_selection_share_one_application(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory) / 'source', Path(directory) / 'assets'
            for reference in ['cabinet/000002', 'pen/000006']:
                path = source / reference
                path.mkdir(parents=True)
                (path / 'physics.py').write_text('')
            for selected in [None, ['pen/000006', 'cabinet/000002']]:
                arguments = ['--source', str(source), '--output', str(output), '--gpu', '3']
                references = ['cabinet/000002', 'pen/000006']
                if selected:
                    arguments += ['--assets', *selected]
                    references = selected
                app = Mock()
                simulation_app = Mock(return_value=app)
                with self.subTest(selected=selected), patch.dict(sys.modules, {'isaacsim': SimpleNamespace(SimulationApp=simulation_app)}), \
                        patch('table_1000.physics.build.build') as build:
                    main(arguments)
                    self.assertEqual(build.call_args_list, [call(output/ref, source/ref, output/ref) for ref in references])
                    simulation_app.assert_called_once()
                    self.assertEqual(simulation_app.call_args.args[0]['active_gpu'], 3)
                    app.close.assert_called_once_with(wait_for_replicator=False)


if __name__ == '__main__':
    unittest.main()
