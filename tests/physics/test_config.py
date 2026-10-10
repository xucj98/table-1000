import unittest
from table_1000.physics.config import test_configs as resolve, keyframes


class PhysicsConfigTests(unittest.TestCase):
  def test_semantic_ranges_and_keyframe_inheritance(self):
    model = {'nodes': [{'name': f'drawer{i}', 'rigid': True} for i in (1, 2)], 'joints': []}
    config = {'defaults': {'duration': 1, 'simulation': {'backend': 'isaac', 'dt': .004},
                          'camera': {'fps': 25, 'resolution': [960, 480]},
                          'initial': {'rigid_bodies': {'drawer1..2': {'position': [0, 0, 1]}}}},
              'tests': {'ordinary1..2.mp4': {
                  'actions': {'pull1..2': {'type': 'force', 'body': 'drawer1..2', 'frame': 'world',
                                         'keyframes': [{'time': 0, 'value': [1, 0, 0]}, {'time': 1}]}},
                  'observe': {'actions': ['pull1..2']}}}}
    test = resolve(config, model)['ordinary1..2.mp4']
    self.assertEqual(test['initial']['rigid_bodies'].keys(), {'drawer1', 'drawer2'})
    self.assertEqual(test['observe']['actions'], ['pull1..2[drawer1]', 'pull1..2[drawer2]'])
    self.assertEqual(test['actions']['pull1..2[drawer2]']['keyframes'][1]['value'], [1, 0, 0])


  def test_action_times_are_actual_physics_steps(self):
    with self.assertRaisesRegex(ValueError, 'align'):
        keyframes([{'time': 0}, {'time': .003}], 1, .004)
