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
                                         'keyframes': [{'time': 0, 'force': [1, 0, 0]}, {'time': 1}]}},
                  'observe': {'plots': ['actions.pull1..2.force.x', 'rigid_bodies.drawer1..2.position.x']}}}}
    test = resolve(config, model)['ordinary1..2.mp4']
    self.assertEqual(test['initial']['rigid_bodies'].keys(), {'drawer1', 'drawer2'})
    self.assertEqual(test['observe']['actions'], ['pull1..2[drawer1]', 'pull1..2[drawer2]'])
    self.assertEqual(test['actions']['pull1..2[drawer2]']['keyframes'][1]['force'], [1, 0, 0])
    self.assertEqual(test['observe']['rigid_bodies'], ['drawer1', 'drawer2'])
    self.assertEqual(test['observe']['plots'], ['actions.pull1..2[drawer1].force.x',
                                              'actions.pull1..2[drawer2].force.x',
                                              'rigid_bodies.drawer1.position.x', 'rigid_bodies.drawer2.position.x'])


  def test_action_times_are_actual_physics_steps(self):
    with self.assertRaisesRegex(ValueError, 'align'):
        keyframes([{'time': 0}, {'time': .003}], 1, .004)

  def test_plot_only_observation_of_named_joint(self):
    model = {'nodes': [], 'joints': [{'name': 'cabinet.drawer1.slide', 'type': 'SLIDER'}]}
    config = {'tests': {'open.mp4': {'duration': 1, 'simulation': {'backend': 'isaac', 'dt': .004},
                                   'camera': {'fps': 25},
                                   'observe': {'plots': ['joints.cabinet.drawer1.slide.velocity']}}}}
    test = resolve(config, model)['open.mp4']
    self.assertEqual(test['observe']['joints'], ['cabinet.drawer1.slide'])
    self.assertEqual(test['observe']['rigid_bodies'], [])
