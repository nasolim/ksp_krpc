import unittest
import AirResistanceModule

class TestAirResistanceModule(unittest.TestCase):
    def test_calculate_force(self):
        testClass = AirResistanceModule.AirResistance()
        testClass.set_area(7.2573)
        testClass.set_coefficient_of_drag(0.85)
        testClass.set_density_of_fluid(0.5)
        testClass.set_velocity(100)
        result = testClass.calculate_force(inplace=True)
        self.assertEqual(result, testClass.force_of_drag)
        self.assertEqual(result, 15421.7625)

    def test_calculate_acceleration(self):
        testClass = AirResistanceModule.AirResistance()
        result = testClass.calculate_acceleration(1300, 1000)
        self.assertEqual(result, 1.3)

    def test_calculate_coefficient_of_drag(self):
        testClass = AirResistanceModule.AirResistance()
        result = testClass.calculate_coefficient_of_drag(15421.7625, 0.5, 7.2573, 100, inplace=True)
        self.assertEqual(result, testClass.coefficient_of_drag)
        self.assertEqual(result, 0.85)
