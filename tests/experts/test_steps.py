"""Steps of an expert without a simulator: which steps run, checkpoints by step, per-step rates, joined episodes."""

import json
import types

import h5py
import numpy as np
import pytest

from table_1000.experts.context import ExpertFailed, Step, run_steps
from table_1000.experts.generate import INITIAL_STATE, merge_checkpoints, read_checkpoints, step_rates, write_checkpoints


class FakeContext:
    def __init__(self, only=None):
        self.only, self.current, self.step_names, self.ran, self.kept = only, None, None, [], []
        self.env = types.SimpleNamespace(steps=0)

    def checkpoint(self, index, name):
        self.kept.append(index)

    def fail(self, reason):
        raise ExpertFailed(self.current, reason)


def steps(failing=None):
    def check(name):
        def run(ctx):
            if name == failing:
                ctx.fail('not done')
        return run
    def run(ctx, name):
        ctx.ran.append(name)
        ctx.env.steps += 10
    return [Step(name, lambda ctx, name=name: run(ctx, name), check(name)) for name in 'abc']


def test_run_steps_runs_the_chosen_steps_and_keeps_a_checkpoint_after_each():
    ctx = FakeContext(only=(2, 3))
    run_steps(ctx, steps())
    assert ctx.ran == ['b', 'c'] and ctx.kept == [2, 3] and ctx.step_names == ['a', 'b', 'c']


def test_a_failed_check_is_the_steps_and_keeps_no_checkpoint():
    ctx = FakeContext()
    with pytest.raises(ExpertFailed) as error:
        run_steps(ctx, steps(failing='b'))
    assert error.value.subtask == 'b' and ctx.kept == [1]


def test_run_steps_rejects_steps_the_expert_does_not_have():
    with pytest.raises(ValueError):
        run_steps(FakeContext(only=(2, 4)), steps())


def state(value):
    return {'articulation': {'robot': {'joint_position': np.full((1, 3), value)}}}


def test_checkpoints_are_merged_by_step(tmp_path):
    start = {'qpos': np.zeros(3), 'base_pose': np.array([0., 0., 1.])}
    for seed, done in ((0, 2), (1, 1)):
        write_checkpoints(tmp_path / f'{seed}.hdf5', seed, 3, start,
                          [{'step': k, 'name': f'step {k}', 'at': 10 * k, 'state': state(seed + k)}
                           for k in range(1, done + 1)])
    assert merge_checkpoints(tmp_path.glob('*.hdf5'), tmp_path / 'batch', {'scene': 's'}) == [1, 2]
    attrs, states = read_checkpoints(tmp_path / 'batch' / 'checkpoints' / 'step2.hdf5')
    assert attrs['scene'] == 's' and attrs['step'] == 2 and attrs['name'] == 'step 2'
    assert list(states) == [0]
    attempt, scene_state, robot = states[0]
    assert attempt == 3 and scene_state['articulation']['robot']['joint_position'][0, 0] == 2.
    assert robot['base_pose'][2] == 1.
    assert sorted(read_checkpoints(tmp_path / 'batch' / 'checkpoints' / 'step1.hdf5')[1]) == [0, 1]



def test_the_initial_state_is_step_0(tmp_path):
    start = {'qpos': np.zeros(3), 'base_pose': np.zeros(3)}
    write_checkpoints(tmp_path / '0.hdf5', 0, 0, start, [{'step': 0, 'name': INITIAL_STATE, 'at': 0, 'state': state(5)},
                                                        {'step': 1, 'name': 'step 1', 'at': 10, 'state': state(6)}])
    assert merge_checkpoints([tmp_path / '0.hdf5'], tmp_path / 'batch', {'scene': 's'}) == [0, 1]
    attrs, states = read_checkpoints(tmp_path / 'batch' / 'checkpoints' / 'step0.hdf5')
    assert attrs['step'] == 0 and attrs['name'] == INITIAL_STATE
    assert states[0][1]['articulation']['robot']['joint_position'][0, 0] == 5.


def report(seed, done, names='abc', run=None):
    return {'seed': seed, 'step_names': list(names), 'steps_run': run,
            'steps': [{'step': k, 'name': names[k - 1], 'at': k} for k in done]}


def test_step_rates_count_from_the_seeds_the_first_step_started_in():
    reports = [report(0, [1, 2, 3]), report(1, [1, 2]), report(2, [1]), report(3, [])]
    rates = step_rates(reports, [0, 1, 2, 3])
    assert rates['seeds'] == 4
    assert [(row['started'], row['completed']) for row in rates['rows']] == [(4, 3), (3, 2), (2, 1)]
    assert [row['progressive_rate'] for row in rates['rows']] == [.75, .5, .25]
    # Steps 2-3 again from the step 1 checkpoints (seeds 0-2) of that batch.
    resumed = step_rates([report(0, [2, 3], run=[2, 3]), report(1, [2, 3], run=[2, 3]), report(2, [], run=[2, 3])],
                         [0, 1, 2], parent={'steps': rates})
    assert [(row['step'], row['started'], row['completed']) for row in resumed['rows']] == [(1, 4, 3), (2, 3, 2),
                                                                                             (3, 2, 2)]
    assert resumed['rows'][-1]['progressive_rate'] == .5
    assert step_rates([{'seed': 0, 'step_names': None}], [0]) is None


def write_batch(directory, episodes, summary):
    """A batch directory: ``episodes`` ``{seed: (status, actions, attrs)}``."""
    directory.mkdir(parents=True)
    for kind, wanted in (('trajectories', 'success'), ('trajectories_failed', 'failed')):
        with h5py.File(directory / f'{kind}.hdf5', 'w') as file:
            data = file.create_group('data')
            data.attrs['env_args'] = json.dumps({'robot': 'r'})
            for seed, (status, actions, attrs) in episodes.items():
                if (status == 'success') != (wanted == 'success'):
                    continue
                group = data.create_group(f'demo_{seed}')
                group.create_dataset('actions', data=np.asarray(actions, dtype=float)[:, None])
                group.create_group('initial_state').create_dataset('x', data=[[actions[0] - 1.]])
                group.create_group('states').create_dataset('x', data=np.asarray(actions, dtype=float)[:, None])
                group.attrs.update(num_samples=len(actions), status=status,
                                   **{key: json.dumps(value) for key, value in attrs.items()})
    (directory / 'summary.json').write_text(json.dumps(summary))


def test_join_cuts_each_part_at_the_checkpoint_the_next_started_from(tmp_path):
    from scripts.scenes.join_steps import main
    first, second = tmp_path / 'scene' / 'isaac-cpu' / 'steps1-2', tmp_path / 'scene' / 'isaac-cpu' / 'steps2-3'
    # Steps 1-2 ran in the first batch; the second ran step 2-3 again from the step 1 checkpoint (at 2).
    write_batch(first, {
        0: ('success', [1, 2, 3, 4], {'steps': [{'step': 1, 'at': 2}, {'step': 2, 'at': 4}],
                                      'subtasks': [{'instruction': 'a', 'start': 0, 'end': 2},
                                                   {'instruction': 'b', 'start': 2, 'end': 4}]}),
        1: ('failed', [1, 2], {'steps': [], 'subtasks': [], 'failure': {'subtask': 'a', 'reason': 'r'}}),
        # Failed in step 2 after its step-1 checkpoint: the second batch ran step 2 again.
        2: ('failed', [1, 2, 3], {'steps': [{'step': 1, 'at': 2}], 'subtasks': [],
                                  'failure': {'subtask': 'b', 'reason': 'slipped'}}),
        3: ('error', [1, 2], {'steps': [], 'subtasks': [], 'failure': {'subtask': 'a', 'reason': 'error'}})},
        {'steps_run': [1, 2], 'from': None, 'commit': 'c1'})
    write_batch(second, {
        0: ('success', [7, 8, 9], {'steps': [{'step': 2, 'at': 1}, {'step': 3, 'at': 3}],
                                   'subtasks': [{'instruction': 'b', 'start': 0, 'end': 3}]}),
        2: ('success', [5, 6], {'steps': [{'step': 2, 'at': 1}, {'step': 3, 'at': 2}], 'subtasks': []})},
        {'steps_run': [2, 3], 'from': str(first), 'commit': 'c2'})
    joined = tmp_path / 'data' / 'scene' / 'isaac-cpu'
    (joined / 'episodes').mkdir(parents=True)     # an earlier batch's, replaced
    main([str(first), str(second), '--output', str(joined)])
    assert not (joined / 'episodes').exists() and (first / 'summary.json').is_file()
    with h5py.File(joined / 'trajectories.hdf5', 'r') as file:
        group = file['data/demo_0']
        assert group['actions'][:, 0].tolist() == [1, 2, 7, 8, 9]
        assert group['states/x'][:, 0].tolist() == [1, 2, 7, 8, 9] and group.attrs['num_samples'] == 5
        assert [entry['at'] for entry in json.loads(group.attrs['steps'])] == [2, 3, 5]
        assert [(e['instruction'], e['start'], e['end']) for e in json.loads(group.attrs['subtasks'])] == [
            ('a', 0, 2), ('b', 2, 5)]
        assert [part['commit'] for part in json.loads(group.attrs['parts'])] == ['c1', 'c2']
        assert file['data/demo_2']['actions'][:, 0].tolist() == [1, 2, 5, 6]
    with h5py.File(joined / 'trajectories_failed.hdf5', 'r') as file:
        assert json.loads(file['data/demo_1'].attrs['failure'])['reason'] == 'r'
        assert file['data/demo_3'].attrs['status'] == 'error'
    summary = json.loads((joined / 'summary.json').read_text())
    assert (summary['succeeded'], summary['failed'], summary['error']) == (2, 1, 1)
    assert summary['seeds']['count'] == 4
    with pytest.raises(SystemExit):     # the first batch must start from the initial states
        main([str(second), '--output', str(tmp_path / 'scene' / 'other')])
    with pytest.raises(SystemExit):     # nor replace a batch it joins, or one inside it
        main([str(first), str(second), '--output', str(second)])
    with pytest.raises(SystemExit):
        main([str(first), str(second), '--output', str(tmp_path / 'scene' / 'isaac-cpu')])
    assert (first / 'summary.json').is_file()


def test_a_failure_case_is_the_last_run_of_its_subtask():
    from scripts.experts.failure_cases import case, failing_start
    report = {'seed': 3, 'duration_s': 1., 'wall_s': 2., 'control_steps': 50,
              'failure': {'subtask': 'pick', 'reason': 'no reachable grasp'},
              'subtasks': [{'instruction': 'pick', 'start': 0, 'end': 10}, {'instruction': 'place', 'start': 10, 'end': 20},
                           {'instruction': 'pick', 'start': 20, 'end': None}],
              'events': [{'step': 5, 'subtask': 'pick', 'note': 'next grasp', 'reason': 'old'},
                         {'step': 25, 'subtask': 'pick', 'note': 'next grasp', 'reason': 'goal in collision',
                          'goal_collisions': [['finger', 'drawer', 'drawer']]}]}
    assert failing_start(report) == 20
    numbers = case(report)
    assert numbers['rejections'] == {'goal in collision': 1} and numbers['collisions'] == {'finger - drawer': 1}
    assert failing_start({**report, 'failure': {'subtask': None, 'reason': 'r'}}) == 50


def test_the_steps_of_a_resumed_run_keep_the_seed_s_start():
    from table_1000.experts.context import Context
    env = types.SimpleNamespace(spec=types.SimpleNamespace(base=None), steps=0,
                                obs={'agent': {'qpos': np.ones((1, 2)), 'base_pose': np.zeros((1, 3))}}, joints=['a', 'b'])
    scene = types.SimpleNamespace(OBJECTS={}, ROBOT={'position': [0., 0., .5]})
    model = types.SimpleNamespace(set_state=lambda *args: None)
    start = {'qpos': np.zeros(2), 'base_pose': np.zeros(3)}
    assert Context(env, model, scene, '.', None, start=start).start is start
    assert Context(env, model, scene, '.', None).start['qpos'].tolist() == [1., 1.]
