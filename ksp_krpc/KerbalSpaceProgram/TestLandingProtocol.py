import unittest
import Landing as LP
import CraftModule

class TestLandingProtocol(unittest.TestCase):
    def test_inital_altitude(self):
        testClass = LP.Landing()
        self.assertEqual(testClass.alt, 0)

    def test_free_fall_decent_model_with_no_inital_velocity(self):
        testClass = LP.Landing()
        result = testClass.free_fall_decent_model(45, 10000, 0)
        self.assertEqual(len(result.keys()),45)

    def test_acceleration_with_no_air_drag(self):
        testClass = LP.Landing()
        self.assertEqual(testClass.calculate_acceleration(100,1000,-9.81,0),9.91)

    def test_solve_quadratic(self):
        testClass = LP.Landing()
        self.assertEqual(testClass.solve_quadratic(5,6,1),(-0.2,-1))

    def test_stopping_distance(self):
        testClass = LP.Landing()
        self.assertEqual(testClass.stopping_distance(500,22),5681.818)

    def test_is_radar_below_stopping_distance_return_true_when_altitude_below_stopping(self):
        testClass = LP.Landing()
        self.assertTrue(testClass.is_radar_below_stopping_distance(100,200))


    def test_is_radar_below_stopping_distance_return_false_when_altitude_above_stopping(self):
        testClass = LP.Landing()
        self.assertFalse(testClass.is_radar_below_stopping_distance(200, 100))

if __name__ == '__main__':
    unittest.main()