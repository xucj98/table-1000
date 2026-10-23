"""Regressions for mixed rigid/articulated assets and startup render buffers."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest
import numpy as np
from table_1000.physics.simulation import Asset
from table_1000.physics.video import Camera, MAX_CAPTURE_RENDERS
from table_1000.physics.testing import TestContext


class RenderInitializationTests(unittest.TestCase):
    def test_render_product_is_attached_before_physics_reset(self):
        # Stop at reset: the regression concerns scene initialization order,
        # before any simulator state restoration or experiment steps.
        ctx = TestContext.__new__(TestContext)
        ctx.initialized = False
        ctx.render = True
        ctx.camera_config = {'resolution': (80, 60)}
        ctx.app = Mock()
        ctx.session = SimpleNamespace(stage=object(), instances={}, replicate=Mock())
        events = []
        ctx.world = Mock()
        def reset():
            events.append('reset')
            raise RuntimeError('reset boundary')
        ctx.world.reset.side_effect = reset
        camera = object()
        def attach(stage, config):
            events.append('camera')
            self.assertIs(stage, ctx.session.stage)
            self.assertIs(config, ctx.camera_config)
            return camera
        with patch.dict('sys.modules', {'pxr': SimpleNamespace(Gf=Mock(), UsdGeom=Mock())}), \
             patch('table_1000.physics.video.Camera', side_effect=attach):
            with self.assertRaisesRegex(RuntimeError, 'reset boundary'):
                ctx.ensure_initialized()
        self.assertEqual(events, ['camera', 'reset'])
        self.assertIs(ctx.camera_instance, camera)
        ctx.world.step.assert_not_called()


class MixedRestoreTests(unittest.TestCase):
    def asset(self, held_tree=False):
        asset = Asset.__new__(Asset)
        asset.body_names = ['housing', 'drawer', 'hook', 'tray']
        asset.root_index = 0
        asset.articulation_bodies = {'housing', 'drawer'}
        asset.fixture_bodies = {'hook'}
        asset.articulation = Mock()
        asset.view = Mock()
        if held_tree:
            asset.fixture_root = '/fixture/root'
        return asset

    def test_free_accessory_is_restored_with_or_without_held_housing(self):
        positions = np.arange(12).reshape(4, 3)
        quaternions = np.tile([1, 0, 0, 0], (4, 1))
        velocities = np.arange(24).reshape(4, 6)
        for held in (False, True):
            with self.subTest(held_housing=held):
                asset = self.asset(held)
                asset.restore_body_poses(positions, quaternions)
                asset.restore_body_velocities(velocities)
                args, kwargs = asset.view.set_world_poses.call_args
                np.testing.assert_array_equal(kwargs['indices'], [3])
                np.testing.assert_array_equal(args[0], positions[[3]])
                args, kwargs = asset.view.set_velocities.call_args
                np.testing.assert_array_equal(kwargs['indices'], [3])
                np.testing.assert_array_equal(args[0], velocities)
                self.assertEqual(asset.articulation.set_root_transforms.call_count, int(not held))
                self.assertEqual(asset.articulation.set_root_velocities.call_count, int(not held))


class CameraReadinessTests(unittest.TestCase):
    def make(self, buffers):
        camera = Camera.__new__(Camera)
        camera.size = (80, 60)
        camera.capture_render_counts = []
        camera.rgb = Mock()
        camera.rgb.get_data.side_effect = buffers
        state = {'position': np.zeros(3), 'quaternion': np.array([1., 0, 0, 0]), 'velocity': np.zeros(6)}
        runtime = SimpleNamespace(states={'body':state}, refresh=Mock(), time=0.)
        ctx = SimpleNamespace(session=runtime, physics='gpu', world=Mock(), name='readiness', action_values={})
        return camera, ctx

    def test_empty_buffers_wait_for_real_rgb_without_physics_steps(self):
        frame = np.full((60, 80, 4), 123, dtype=np.uint8)
        camera, ctx = self.make([np.array([]), np.array([]), frame])
        result = camera.capture(ctx)
        self.assertEqual(result.size, (80, 60))
        self.assertEqual(result.getpixel((40, 59)), (123, 123, 123))
        self.assertEqual(camera.capture_render_counts, [4])
        self.assertEqual(ctx.world.render.call_count, 4)
        ctx.world.step.assert_not_called()

    def test_cpu_fabric_capture_flushes_kinematic_transforms_without_stepping(self):
        camera, ctx = self.make([np.zeros((60, 80, 4), dtype=np.uint8)])
        ctx.physics = 'cpu'
        ctx.world.is_fabric_enabled.return_value = True
        camera.capture(ctx)
        ctx.world.forward.assert_called_once_with()
        ctx.world.step.assert_not_called()
        self.assertEqual(ctx.session.time, 0.)

    def test_native_sync_must_not_change_physical_velocity(self):
        camera, ctx = self.make([np.zeros((60, 80, 4), dtype=np.uint8)])
        ctx.physics = 'cpu'
        ctx.world.is_fabric_enabled.return_value = True
        ctx.world.forward.side_effect = lambda: ctx.session.states['body']['velocity'].__setitem__(0, 1.)
        with self.assertRaisesRegex(RuntimeError, 'Rendering advanced physical state'):
            camera.capture(ctx)
        ctx.world.step.assert_not_called()

    def test_cpu_without_fabric_keeps_native_usd_sync(self):
        camera, ctx = self.make([np.zeros((60, 80, 4), dtype=np.uint8)])
        ctx.physics = 'cpu'
        ctx.world.is_fabric_enabled.return_value = False
        interface = Mock()
        with patch.dict('sys.modules', {'omni.physx': SimpleNamespace(get_physx_interface=lambda: interface)}):
            camera.capture(ctx)
        interface.update_transformations.assert_called_once_with(True, True, True, False)
        ctx.world.forward.assert_not_called()
        ctx.world.step.assert_not_called()

    def test_permanently_empty_capture_fails_instead_of_fabricating_frame(self):
        camera, ctx = self.make([np.array([])] * MAX_CAPTURE_RENDERS)
        with self.assertRaisesRegex(RuntimeError, 'RGB render product not ready'):
            camera.capture(ctx)
        self.assertEqual(ctx.world.render.call_count, MAX_CAPTURE_RENDERS)

    def test_close_detaches_render_product_by_path(self):
        camera = Camera.__new__(Camera)
        camera.rgb = Mock()
        camera.product = Mock(path='/Render/HydraTextures/Regression')
        camera.close()
        camera.rgb.detach.assert_called_once_with(['/Render/HydraTextures/Regression'])
        camera.product.destroy.assert_called_once_with()

    def test_render_must_preserve_velocity_as_well_as_pose(self):
        camera, ctx = self.make([np.zeros((60, 80, 4), dtype=np.uint8)])
        ctx.session.refresh.side_effect = lambda: ctx.session.states['body']['velocity'].__setitem__(0, 1.)
        with self.assertRaisesRegex(RuntimeError, 'Rendering advanced physical state'):
            camera.capture(ctx)
