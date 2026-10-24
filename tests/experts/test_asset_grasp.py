"""Asset inventory, random placement, and the stability metric (no simulator)."""
import types

import numpy as np

from table_1000.experts.asset_grasp import displacement, inventory, summarize


def test_repeated_instances_run_once_and_exclusions_are_explicit():
    source = types.SimpleNamespace(OBJECTS={
        'a': {'asset': 'objects/a/000000'}, 'b': {'asset': 'objects/a/000000'},
        'fixture': {'asset': 'objects/f/000000'}})
    result = inventory(source, ('a', 'fixture'))
    assert result['objects/a/000000'] == {'instances': ['a', 'b'], 'excluded': False}
    assert result['objects/f/000000']['excluded']


def test_slip_includes_rotation_and_cannot_cancel_at_final_pose():
    reference = np.eye(4)
    turned = np.eye(4)
    turned[:2, :2] = [[0, -1], [1, 0]]
    points = np.array([[.1, 0, 0], [-.1, 0, 0]])
    assert np.isclose(displacement(reference, turned, points), np.sqrt(.02))
    assert displacement(reference, reference, points) == 0
    assert max(displacement(reference, pose, points) for pose in (reference, turned, reference)) > .005


def test_missing_and_failed_trials_stay_in_denominator():
    report = summarize([{'status': 'success'}, {'status': 'failed', 'failure': {'reason': 'no reachable grasp'}}], 3)
    assert report['rate'] == 1 / 3
    assert report['status'] == 'FAIL'
    assert report['failures'] == {'no reachable grasp': 1, 'missing report': 1}


def test_failed_closure_before_losing_reach_is_a_physical_failure():
    report = {'status': 'failed', 'failure': {'reason': 'no reachable grasp'},
              'events': [{'note': 'missed grasp'}]}
    assert summarize([report], 1)['failures'] == {'object not held': 1}
    assert report['failure']['reason'] == 'no reachable grasp'


def test_hollow_assembly_tries_real_parts_before_filled_bounding_box():
    from table_1000.experts.asset_grasp import targets
    from table_1000.experts.grasps import Box
    names = ('ring', 'small', 'large')
    geometry = types.SimpleNamespace(
        shapes=[],
        points=lambda part, body: Box((.01 if part == 'small' else .03,) * 3).points(),
        parts=lambda: [('body', name) for name in names],
        solid=lambda name, narrow: name in ('small', 'large'))
    def target(obj, part=None):
        return types.SimpleNamespace(body='object:body', names=[part] if part else list(names),
                                     shape=Box((.01 if part == 'small' else .03,) * 3))
    skill = types.SimpleNamespace(target=target, ctx=types.SimpleNamespace(geometry={'object': geometry}))
    assert [t.names for t in targets(skill)] == [['large'], ['small'], list(names)]


def test_diagonal_thin_part_keeps_its_actual_width():
    from table_1000.experts.asset_grasp import oriented_shape
    from table_1000.experts.grasps import Box
    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_euler('z', 30, degrees=True).as_matrix()
    points = Box((.04, .004, .002)).points() @ rotation.T
    shape, offset = oriented_shape(points)
    assert np.allclose(sorted(shape.size), [.002, .004, .04])
    local = (points - offset[:3, 3]) @ offset[:3, :3]
    assert np.all(np.abs(local) <= np.asarray(shape.size) / 2 + 1e-10)


def test_placement_uses_ten_cm_square_and_full_yaw(monkeypatch):
    from table_1000.experts import asset_grasp
    class Layout:
        def __init__(self, *args):
            pass
        def resting_pose(self, name, xy, yaw):
            out = np.eye(4)
            out[:2, 3] = xy
            return out
    monkeypatch.setattr(asset_grasp, 'Layout', Layout)
    samples = [asset_grasp.placement(None, None, seed, (.45, -.15))[1] for seed in range(100)]
    xy = np.array([v['xy_m'] for v in samples])
    assert (np.abs(xy - [.45, -.15]) <= .05).all()
    yaw = [v['yaw_rad'] for v in samples]
    assert min(yaw) < -3 and max(yaw) > 3
    assert samples[5] == asset_grasp.placement(None, None, 5, (.45, -.15))[1]


def test_grasp_family_does_not_change_with_world_yaw():
    from table_1000.experts.asset_grasp import grasp_family
    body = np.eye(4)
    pose = np.eye(4)
    report = {'pick': {'arm': 'right', 'grasp': {'pose': pose.tolist()}}, 'settled_pose': body.tolist()}
    family = grasp_family(report)
    body[:2, :2] = [[0, -1], [1, 0]]
    report['settled_pose'] = body.tolist()
    report['pick']['grasp']['pose'] = (body @ pose).tolist()
    assert grasp_family(report) == family
    summary = summarize([dict(report, status='success'), dict(report, status='failed')], 2)
    assert summary['by_grasp'][family] == {'trials': 2, 'success': 1, 'rate': .5}


def test_shake_samples_transient_slip_and_executes_full_wrist_motion():
    from table_1000.experts.asset_grasp import Evaluator
    model = types.SimpleNamespace(q=np.zeros(1), limits=np.array([[-2., 2.]]), arm_indices=lambda arm: [0])
    evaluator = Evaluator.__new__(Evaluator)
    evaluator.model = model
    ctx = types.SimpleNamespace(observers=[], steps=0,
                                env=types.SimpleNamespace(contacts=types.SimpleNamespace(phase='start')),
                                geometry={'object': types.SimpleNamespace(points=lambda **kw: np.array([[.1, 0, 0]]))})
    ctx.tcp = lambda arm: np.eye(4)
    def poses():
        pose = np.eye(4)
        if ctx.steps == 3:
            pose[0, 3] = .006  # transient slip, recovered before every segment ends
        return {'object:body': pose}
    ctx.poses = poses
    def step(action=None):
        ctx.steps += 1
        if action:
            model.q[:] = action['right'][0]
        for observer in ctx.observers:
            observer(ctx)
    ctx.step = step
    ctx.hold = lambda seconds: [step() for _ in range(round(seconds * 25))]
    result = evaluator.shake(ctx, types.SimpleNamespace(body='object:body'), 'right', {})
    assert result['completed'] and not result['held']
    assert np.isclose(result['max_slip_m'], .006)
    assert np.isclose(result['wrist_excursion_rad'], .8)
    assert not ctx.observers
