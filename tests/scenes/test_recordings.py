"""Rendering/export must not combine a regenerated seed with old camera frames."""

import json
import shutil
from types import SimpleNamespace
import sys

import h5py
import numpy as np
from PIL import Image
import pytest

from scripts.scenes import generate, render, to_lerobot
from table_1000.scenes import summary
from table_1000.scenes.recordings import begin_render, demo_dir, fingerprint, read_render, video_path, write_render


@pytest.fixture
def recording(tmp_path):
    with h5py.File(tmp_path / 'trajectories.hdf5', 'w') as file:
        data = file.create_group('data')
        data.attrs['env_args'] = json.dumps({'scene': '000000', 'robot': 'x1pro',
                                            'action_layout': {'arm': [0, 1]}, 'joints': ['joint'],
                                            'control_hz': 25, 'instruction': 'tidy up the table'})
        group = data.create_group('demo_0')
        group.attrs.update(num_samples=1, subtasks='[]')
        for path in ['actions', 'obs/qpos', 'initial_state/pose', 'states/pose']:
            group.create_dataset(path, data=np.array([[1.]], dtype=np.float32))
        yield file, group


def camera_render(output, identity, name='demo_0'):
    begin_render(output, name, identity, sheets=False)
    directory = demo_dir(output, name)
    (directory / 'frames' / 'face').mkdir(parents=True)
    Image.new('RGB', (40, 20), 'red').save(directory / 'frames' / 'face' / '00000.jpg')
    video_path(output, name).write_bytes(b'old video')
    (directory / 'sheet.jpg').write_bytes(b'old sheet')
    write_render(directory, {'trajectory': identity, 'sheet': True, 'views': ['wide'], 'cameras': ['face']})


@pytest.mark.parametrize('changed', ['actions', 'obs/qpos', 'initial_state/pose', 'states/pose', 'subtasks', 'env_args'])
def test_export_rejects_changed_recording(recording, tmp_path, changed):
    file, group = recording
    camera_render(tmp_path, fingerprint(group))
    assert len(to_lerobot.rendered_episodes([file], tmp_path)) == 1
    if changed == 'subtasks':
        group.attrs[changed] = '[{"instruction": "new", "start": 0, "end": 1}]'
    elif changed == 'env_args':
        meta = json.loads(group.parent.attrs[changed])
        meta['instruction'] = 'new instruction'
        group.parent.attrs[changed] = json.dumps(meta)
    else:
        group[changed][0] = [123.]
    with pytest.raises(ValueError, match='stale or incomplete'):
        to_lerobot.rendered_episodes([file], tmp_path)


def test_fingerprint_survives_file_copy(recording, tmp_path):
    file, group = recording
    file.flush()
    shutil.copyfile(file.filename, tmp_path / 'copy.hdf5')
    with h5py.File(tmp_path / 'copy.hdf5') as copy:
        assert fingerprint(group) == fingerprint(copy['data/demo_0'])


@pytest.mark.parametrize('change', ['missing_manifest', 'missing_frame', 'partial_render'])
def test_export_rejects_unverified_frames(recording, tmp_path, change):
    file, group = recording
    directory = demo_dir(tmp_path, 'demo_0')
    camera_render(tmp_path, fingerprint(group))
    if change == 'missing_manifest':
        (directory / 'render.json').unlink()
    elif change == 'missing_frame':
        (directory / 'frames/face/00000.jpg').unlink()
    else:
        begin_render(tmp_path, 'demo_0', fingerprint(group), sheets=False)
        (directory / 'frames/face').mkdir(parents=True)
    with pytest.raises(ValueError, match='incomplete'):
        to_lerobot.rendered_episodes([file], tmp_path)


@pytest.mark.parametrize('same_episode', [False, True])
def test_sheets_worker_preserves_only_matching_camera_frames(recording, tmp_path, monkeypatch, same_episode):
    file, group = recording
    import table_1000.sim.isaaclab as lab
    from table_1000.sim.isaaclab import replay
    output = tmp_path / 'scene'
    directory = demo_dir(output, 'demo_0')
    camera_render(output, fingerprint(group) if same_episode else 'old trajectory')
    original_image = (directory / 'frames/face/00000.jpg').read_bytes()

    class Env:
        def __init__(self, *args, **kwargs):
            pass

        def render(self, names):
            return {name: {'rgb': np.zeros((1, 20, 40, 3), np.uint8)} for name in names}

        def close(self):
            pass

    def close(exit_code=0):
        assert exit_code == 0

    modules = {
        'app': SimpleNamespace(start_app=lambda **kw: SimpleNamespace(close=close)),
        'env': SimpleNamespace(RobotEnv=Env),
        'render': SimpleNamespace(RT2_ARGS=(), configure=lambda: None),
        'cameras': SimpleNamespace(depth_image=lambda x: x),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, 'table_1000.sim.isaaclab.' + name, module)
        monkeypatch.setattr(lab, name, module, raising=False)
    monkeypatch.setattr(replay, 'episodes', lambda path: (file, {'demo_0': replay.Episode(group)}))
    monkeypatch.setattr(replay, 'set_state', lambda *a: None)
    render.main([file.filename, '--worker', '--sheets', '--views', 'wide', '--output', str(output)])
    assert (directory / 'sheet.jpg').read_bytes() != b'old sheet'
    assert read_render(directory)['sheet'] is True
    if same_episode:
        assert video_path(output, 'demo_0').read_bytes() == b'old video'
        assert (directory / 'frames/face/00000.jpg').read_bytes() == original_image
    else:
        assert not video_path(output, 'demo_0').exists() and not (directory / 'frames').exists()


def test_render_summary_uses_custom_output(recording, tmp_path, monkeypatch):
    file, group = recording
    from table_1000.physics import test as physics_test
    output, assets = tmp_path / 'custom', tmp_path / 'assets'

    def jobs(commands, gpus):
        for command, _, _ in commands:
            destination = command[command.index('--output') + 1]
            assert destination == str(output)
        camera_render(output, fingerprint(group))
        return []

    monkeypatch.setattr(physics_test, 'run_jobs', jobs)
    file.flush()
    render.main([file.filename, '--output', str(output), '--assets', str(assets)])
    preview = json.loads(summary.path(assets, '000000').read_text())['preview']
    assert preview['sheets'] == 1 and preview['output'] == str(output)


def test_export_checks_provenance_before_replacing_dataset(recording, tmp_path, monkeypatch):
    file, group = recording
    camera_render(tmp_path, fingerprint(group))
    group['actions'][0] = [456.]
    file.flush()
    destination = tmp_path / 'dataset'
    destination.mkdir()
    (destination / 'keep').touch()
    monkeypatch.setitem(sys.modules, 'lerobot.datasets.lerobot_dataset',
                        SimpleNamespace(LeRobotDataset=None))
    with pytest.raises(ValueError, match='stale or incomplete'):
        to_lerobot.main([file.filename, '--frames', str(tmp_path), '--output', str(destination)])
    assert (destination / 'keep').exists()


def test_new_generation_invalidates_preview_counts(tmp_path, monkeypatch):
    from table_1000.physics import test as physics_test
    assets = tmp_path / 'assets'
    summary.update(assets, '000000', 'preview', {'sheets': 98, 'videos': ['demo_0']})
    camera_render(assets / 'scenes' / '000000', 'old trajectory')

    def jobs(commands, gpus):
        assert json.loads(summary.path(assets, '000000').read_text())['preview'] == {'status': 'needs_render'}
        for command, _, _ in commands:
            directory = tmp_path / 'runs/000000/isaac-cpu/part0/episodes'
            directory.mkdir(parents=True)
            (directory / '0.json').write_text(json.dumps({
                'seed': 0, 'status': 'init_failed', 'wall_s': 1.}))
        return []

    monkeypatch.setattr(physics_test, 'run_jobs', jobs)
    generate.main(['000000', '--seeds', '0', '--physics', 'isaac-cpu',
                   '--assets', str(assets), '--output', str(tmp_path / 'runs')])
    data = json.loads(summary.path(assets, '000000').read_text())
    assert data['preview'] == {'status': 'needs_render'}
    assert data['generation']['isaac-cpu']['init_failed'] == 1
    assert not (assets / 'scenes/000000/demos').exists() and not (assets / 'scenes/000000/demo_0.mp4').exists()


def test_unselected_stale_demo_cannot_enter_new_export(recording, tmp_path):
    file, group = recording
    file.copy(group, 'data/demo_1')
    second = file['data/demo_1']
    camera_render(tmp_path, fingerprint(group))
    camera_render(tmp_path, fingerprint(second), 'demo_1')
    group['actions'][0] = [2.]
    second['actions'][0] = [3.]
    camera_render(tmp_path, fingerprint(group))
    preview = summary.preview_section(tmp_path, {'demo_0': fingerprint(group), 'demo_1': fingerprint(second)})
    assert preview['camera_frames'] == ['demo_0'] and preview['stale'] == ['demo_1']
    with pytest.raises(ValueError, match='demo_1.*stale'):
        to_lerobot.rendered_episodes([file], tmp_path)
