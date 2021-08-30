import unittest
import ModelDescentModule


class MyTestCase(unittest.TestCase):
    def test_free_fall_model_returns_expected_number_of_values(self):
        testClass = ModelDescentModule.DescentModel()
        testClass.set_initial_velocity(180.1428287)
        testClass.set_initial_altitude(2187.362845)
        testClass.set_initial_mass(7532.206055)
        testClass.set_time_step(1.000008106)
        testClass.free_fall()
        self.assertEqual(46,len(testClass.estimated_free_fall_profile))

    def test_free_fall_model_returns_expected_velocity(self):
        testClass = ModelDescentModule.DescentModel()
        testClass.set_initial_velocity(180.1428287)
        testClass.set_initial_altitude(2187.362845)
        testClass.set_initial_mass(7532.206055)
        testClass.set_time_step(1.000008106)
        testClass.free_fall()
        self.assertEqual(-205.3775,testClass.estimated_free_fall_profile[41][2])


    def test_determine_points_of_interest(self):
        testClass = ModelDescentModule.DescentModel()
        testClass.set_initial_velocity(180.1428287)
        testClass.set_initial_altitude(2187.362845)
        testClass.set_initial_mass(7532.206055)
        testClass.set_time_step(1.000008106)
        testClass.free_fall()
        testClass.determine_points_of_interest(329985.94726955)
        self.assertEqual(2,len(testClass.points_of_interest))

    def test_determine_burn_height(self):
        testClass = ModelDescentModule.DescentModel()
        testClass.set_initial_velocity(180.1428287)
        testClass.set_initial_altitude(2187.362845)
        testClass.set_initial_mass(7532.206055)
        testClass.set_time_step(1.000008106)
        testClass.free_fall()
        testClass.determine_points_of_interest(329985.94726955)
        testClass.determine_burn_height(329985.94726955)
        self.assertEqual(545.787, testClass.recommended_burn_altitude)


if __name__ == '__main__':
    unittest.main()
