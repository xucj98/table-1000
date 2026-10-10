import unittest
from table_1000.assets.references import expand_names, expand_mapping


class ReferenceTests(unittest.TestCase):
    def test_suffix_and_padded_range(self):
        self.assertEqual(expand_names('cabinet.drawer1..3.slide'),
                         [f'cabinet.drawer{i}.slide' for i in (1, 2, 3)])
        self.assertEqual(expand_names('cap.shell.collision00..02'),
                         [f'cap.shell.collision{i:02}' for i in (0, 1, 2)])

    def test_missing_and_duplicate_targets(self):
        with self.assertRaisesRegex(ValueError, 'Unknown'):
            expand_names('drawer1..3', {'drawer1', 'drawer2'})
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            expand_mapping({'drawer1..3': 1, 'drawer2': 2})
        with self.assertRaisesRegex(ValueError, 'Invalid'):
            expand_names('drawer01..3')

    def test_configuration_values_are_not_name_references(self):
        expanded = expand_mapping({'drawer1..2': {'output': 'trial1..3.mp4', 'mass': .12}})
        self.assertEqual(expanded['drawer1']['output'], 'trial1..3.mp4')
        expanded['drawer1']['mass'] = .2
        self.assertEqual(expanded['drawer2']['mass'], .12)


if __name__ == '__main__':
    unittest.main()
