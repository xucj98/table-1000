import unittest
from table_1000.physics.plots import plot_unit


class PlotUnitTests(unittest.TestCase):
    def test_physical_units_and_length_display_scale(self):
        model = {'joints': [{'name': 'drawer.slide', 'type': 'SLIDER'},
                            {'name': 'lid.hinge', 'type': 'HINGE'}]}
        actions = {'pull': {'type': 'force'}, 'twist': {'type': 'torque'},
                   'drive': {'type': 'joint_drive', 'joint': 'lid.hinge'}}
        cases = [('actions.pull.force.x', [.5, 2.5], ('N', 1)),
                 ('actions.twist.torque.y', [.02], ('N·m', 1)),
                 ('rigid_bodies.cap.position.x', [-.05, -.032], ('mm', 1000)),
                 ('rigid_bodies.cap.position.z', [1, .01], ('m', 1)),
                 ('rigid_bodies.cap.linear_velocity.z', [-4], ('m/s', 1)),
                 ('rigid_bodies.cap.angular_velocity.y', [3], ('rad/s', 1)),
                 ('joints.drawer.slide.position', [.003], ('mm', 1000)),
                 ('joints.drawer.slide.velocity', [.1], ('m/s', 1)),
                 ('joints.lid.hinge.position', [.05], ('rad', 1)),
                 ('joints.lid.hinge.velocity', [.05], ('rad/s', 1)),
                 ('actions.drive.target_position', [.05], ('rad', 1))]
        for source, values, expected in cases:
            with self.subTest(source=source):
                self.assertEqual(plot_unit(source, values, model, actions), expected)
