import unittest
from table_1000.physics.plots import plot_unit


class PlotUnitTests(unittest.TestCase):
    def test_explicit_units_and_length_display_scale(self):
        for unit,values,expected in [('N',[.5,2.5],('N',1)),('N·m',[.02],('N·m',1)),
               ('m',[-.05,-.032],('mm',1000)),('m',[1,.01],('m',1)),
               ('m/s',[-4],('m/s',1)),('rad/s',[3],('rad/s',1)),('rad',[.05],('rad',1))]:
            with self.subTest(unit=unit):
                self.assertEqual(plot_unit(unit,values),expected)
