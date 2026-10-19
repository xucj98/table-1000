"""The three scene steps without a simulator: seeds, initial-state files, merged batches, the choice of renders."""

import json

import h5py
import numpy as np
from PIL import Image
import pytest

from scripts.scenes import build_scene, render, run_expert
from table_1000.experts.generate import PROCESS_EXITED, summarize
from table_1000.physics import test as physics_test
from table_1000.scenes.batch import parse_seeds, seed_range, split
from table_1000.scenes.initial import read_states, write_states
from table_1000.scenes.recordings import merge, video_path, videos

STATE = {'articulation': {'robot': {'joint_position': np.zeros((1, 3), np.float32)}},
         'rigid_object': {'pen': {'root_pose': np.ones((1, 7), np.float32)}}, 'deformable_object': {}}


def test_seeds_are_ranges_and_single_seeds():
    assert parse_seeds(['100:103', '7', '101']) == [7, 100, 101, 102]
    assert seed_range([0, 1, 2, 5, 7, 8]) == '0:3 5 7:9'
    assert parse_seeds(seed_range([3, 4, 9]).split()) == [3, 4, 9]
    assert split(list(range(10)), 3) == [[0, 1, 2, 3], [4, 5, 6], [7, 8, 9]]
    assert split([5, 6], 8) == [[5], [6]]


def test_initial_states_round_trip(tmp_path):
    write_states(tmp_path / 'states.hdf5', {12: (2, STATE), 3: (0, STATE)}, {'robot': 'x1pro'})
    attrs, states = read_states(tmp_path / 'states.hdf5')
    assert attrs['robot'] == 'x1pro' and list(states) == [3, 12]
    attempt, state = states[12]
    assert attempt == 2 and state['deformable_object'] == {}
    assert np.array_equal(state['rigid_object']['pen']['root_pose'], STATE['rigid_object']['pen']['root_pose'])


def trajectory_file(path, seeds):
    with h5py.File(path, 'w') as file:
        data = file.create_group('data')
        data.attrs.update(total=0, env_args=json.dumps({'scene': '000000', 'robot': 'x1pro'}))
        for seed in seeds:
            group = data.create_group(f'demo_{seed}')
            group.attrs['num_samples'] = 2
            group.create_dataset('actions', data=np.zeros((2, 1)))


def test_merge_orders_episodes_by_seed(tmp_path):
    trajectory_file(tmp_path / 'a.hdf5', [10, 2])
    trajectory_file(tmp_path / 'b.hdf5', [5])
    assert merge([tmp_path / 'a.hdf5', tmp_path / 'b.hdf5', tmp_path / 'missing.hdf5'], tmp_path / 'all.hdf5') == 3
    with h5py.File(tmp_path / 'all.hdf5', 'r') as file:
        assert list(file['data']) == ['demo_10', 'demo_2', 'demo_5']   # HDF5 lists names alphabetically
        assert file['data'].attrs['total'] == 6 and 'env_args' in file['data'].attrs


def test_summary_counts_every_seed():
    reports = [{'seed': 0, 'status': 'success', 'duration_s': 20., 'wall_s': 60.},
               {'seed': 1, 'status': 'failed', 'failure': {'subtask': 'close the drawer', 'reason': 'handle not held'},
                'wall_s': 40.},
               {'seed': 3, 'status': 'error', 'failure': {'subtask': 'pick up the pen', 'reason': 'error'}, 'wall_s': 1.}]
    summary = summarize(reports, [0, 1, 2, 3])
    assert summary['seeds'] == {'range': '0:4', 'count': 4} and summary['success_rate'] == .25
    assert summary['failures'] == {PROCESS_EXITED: [2], 'close the drawer: handle not held': [1],
                                   'pick up the pen: error': [3]}


def test_render_chooses_succeeded_seeds():
    assert render.choose([9, 2, 5]) == [2, 5, 9]
    assert render.choose([9, 2, 5], count=2) == [2, 5]
    assert render.choose([9, 2, 5], seeds=[9, 5], count=1) == [5]
    with pytest.raises(ValueError) as error:
        render.choose([9, 2, 5], seeds=[2, 3])
    assert error.value.args[0] == [3]


@pytest.fixture
def batch(tmp_path):
    directory = tmp_path / 'data' / '000000' / 'isaac-cpu'
    directory.mkdir(parents=True)
    trajectory_file(directory / 'trajectories.hdf5', [0, 1, 4])
    (directory / 'summary.json').write_text(json.dumps({'seeds': {'range': '0:5', 'count': 5}, 'videos': '',
                                                        'failures': {'close the drawer: x': [2, 3]}}))
    return directory


def test_render_rejects_seeds_without_episode(batch, tmp_path, capsys):
    with pytest.raises(SystemExit):
        render.main(['000000', '--physics', 'isaac-cpu', '--seeds', '1:4', '--input', str(tmp_path / 'data')])
    assert 'seeds 2:4 have no succeeded episode; the batch covers 0:5, failed: 2:4' in capsys.readouterr().err


def test_render_replaces_earlier_videos(batch, tmp_path, monkeypatch):
    video_path(batch, 4).write_bytes(b'old')
    chosen = []

    def jobs(commands, gpus):
        assert gpus == [0, 1, 0, 1]
        for command, _, _ in commands:
            seeds = parse_seeds(command[command.index('--seeds') + 1:command.index('--batch')])
            chosen.extend(seeds)
            for seed in seeds:
                video_path(batch, seed).write_bytes(b'new')
        return []

    monkeypatch.setattr(physics_test, 'run_jobs', jobs)
    render.main(['000000', '--physics', 'isaac-cpu', '--count', '2', '--gpus', '0', '1',
                 '--jobs-per-gpu', '2', '--input', str(tmp_path / 'data'), '--root', str(tmp_path)])
    assert chosen == [0, 1] and videos(batch) == [0, 1]
    assert json.loads((batch / 'summary.json').read_text())['videos'] == '0:2'
    chosen.clear()
    trajectory_file(batch / 'trajectories_failed.hdf5', [3, 2])
    render.main(['000000', '--physics', 'isaac-cpu', '--count', '1', '--failed', '--gpus', '0', '1',
                 '--jobs-per-gpu', '2', '--input', str(tmp_path / 'data'), '--root', str(tmp_path)])
    assert chosen == [0, 2, 3] and videos(batch) == [0, 2, 3]


def test_run_expert_merges_parts(tmp_path, monkeypatch):
    initial = tmp_path / 'assets' / 'scenes' / '000000'
    initial.mkdir(parents=True)
    write_states(initial / 'initial_states.hdf5', {seed: (0, STATE) for seed in range(4)},
                 {'scene': '000000', 'robot': 'x1pro'})

    def jobs(commands, gpus):
        for command, _, _ in commands:
            work = tmp_path / command[command.index('--work') + 1]
            seeds = parse_seeds(command[command.index('--seeds') + 1:command.index('--states')])
            if seeds == [2, 3]:
                continue       # the process stopped before writing anything
            (work / 'episodes').mkdir(parents=True)
            trajectory_file(work / 'trajectories.hdf5', seeds)
            for seed in seeds:
                (work / 'episodes' / f'{seed}.json').write_text(json.dumps(
                    {'seed': seed, 'status': 'success', 'duration_s': 1., 'wall_s': 2.}))
        return ['part1']

    monkeypatch.setattr(physics_test, 'run_jobs', jobs)
    monkeypatch.setattr(run_expert, 'scene_robot', lambda scene, root: 'x1pro')
    output = tmp_path / 'data'
    with pytest.raises(SystemExit, match='part1'):
        run_expert.main(['000000', '--physics', 'isaac-cpu', '--gpus', '0', '1', '--input', str(tmp_path / 'assets/scenes'),
                         '--output', str(output), '--root', str(tmp_path / 'checkout')])
    directory = output / '000000' / 'isaac-cpu'
    summary = json.loads((directory / 'summary.json').read_text())
    assert summary['failures'] == {PROCESS_EXITED: [2, 3]} and summary['succeeded'] == 2
    assert sorted(path.name for path in (directory / 'episodes').iterdir()) == ['0.json', '1.json']
    with h5py.File(directory / 'trajectories.hdf5', 'r') as file:
        assert sorted(file['data']) == ['demo_0', 'demo_1']


def test_run_expert_rejects_another_robot(tmp_path, monkeypatch):
    monkeypatch.setattr(run_expert, 'scene_robot', lambda scene, root: 'x1pro')
    initial = tmp_path / '000000'
    initial.mkdir()
    write_states(initial / 'initial_states.hdf5', {0: (0, STATE)}, {'scene': '000000', 'robot': 'other'})
    with pytest.raises(SystemExit):
        run_expert.main(['000000', '--physics', 'isaac-cpu', '--input', str(tmp_path), '--output', str(tmp_path / 'out')])


def test_build_scene_combines_parts(tmp_path, monkeypatch):
    def jobs(commands, gpus):
        for command, _, _ in commands:
            work = tmp_path / command[command.index('--work') + 1]
            seeds = parse_seeds(command[command.index('--seeds') + 1:command.index('--work')])
            (work / 'images').mkdir(parents=True)
            records = {seed: {'seed': seed, 'attempt': None if seed == 1 else 0,
                              'attempts': [{'attempt': 0, 'settle': {'objects': {}, 'robot_contacts': []}}]}
                       for seed in seeds}
            write_states(work / 'states.hdf5', {seed: (0, STATE) for seed in seeds if seed != 1},
                         {'scene': '000000', 'robot': 'x1pro'})
            for seed in seeds:
                if seed != 1:
                    Image.new('RGB', (64, 36), 'red').save(work / 'images' / f'{seed}.jpg')
            (work / 'report.json').write_text(json.dumps({'robot': 'x1pro', 'camera': 'face_view', 'records': records}))
        return []

    monkeypatch.setattr(physics_test, 'run_jobs', jobs)
    root = tmp_path / 'checkout'
    output = tmp_path / 'custom'
    (output / '000000' / 'initial').mkdir(parents=True)
    with pytest.raises(SystemExit, match='without a valid state: 1'):
        build_scene.main(['000000', '--seeds', '0:5', '--gpus', '0', '1', '--grid', '2x1', '--output', str(output),
                          '--root', str(root)])
    directory = output / '000000'
    report = json.loads((directory / 'initial.json').read_text())
    assert report['seeds'] == {'range': '0:5', 'count': 5} and report['without_state'] == [1]
    assert list(read_states(directory / 'initial_states.hdf5')[1]) == [0, 2, 3, 4]
    assert Image.open(directory / 'initial_states.jpg').size == (2 * build_scene.TILE_WIDTH, 216 + build_scene.CAPTION)
    assert not (directory / 'initial').exists()            # replaced, and no --images this time
    assert not (root / 'asset_sources').exists()           # a custom output keeps the thumbnail
