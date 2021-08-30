import CraftModule
import time
import math
import pickle
import AirResistanceModule
import PIDController as pid
import datetime as dt
import ModelDescentModule


class LandingComplete(Exception): pass


class Protocol(CraftModule.Craft):
    def __init__(self):
        CraftModule.Craft.__init__(self)
        self.descent_model = ModelDescentModule.DescentModel()
        self.alt = 0
        self.height_of_center_of_mass = 0
        self.decent_profile = None
        self.burn_height = 0
        self.telemetry = []  # time,mass, vertical_velocity, altitude(asl), altitude, throttle
        self.start_time = None
        self.last_time = None
        self.time_step = 0.5
        self.controller_timestep = 0.001
        self.kp = 2000 * 0.6
        self.ki = 2042.5531914894
        self.kd = 750 #* 0.55
        self.autopilot = self.vessel.auto_pilot
        self.pitch_max = 90
        self.pitch_min = 10
        self.pitch_step = 5
        self.pitch = 90
        self.heading = 0
        self.MinController = pid.PIDController(self.kp, self.ki, self.kd, -5, self.controller_timestep,
                                               self.max_thrust(),
                                               type='Minimum')
        self.controller = pid.PIDController(self.kp, self.ki, self.kd, -5, self.controller_timestep, self.max_thrust())

    def reset(self):
        self.alt = 0
        self.height_of_center_of_mass = 0
        self.decent_profile = None
        self.burn_height = 0
        self.telemetry = []  # time,mass, vertical_velocity, altitude(asl), altitude, throttle
        self.start_time = None
        self.last_time = None

    def set_height_of_center_of_mass(self, value):
        self.height_of_center_of_mass = value

    def add_readings(self, mass, vertical_velocity, altitude_asl, altitude, throttle, status):
        if time.time() - self.last_time > self.time_step:
            self.telemetry.append(
                (time.time() - self.start_time
                 , mass
                 , vertical_velocity
                 , altitude_asl
                 , altitude
                 , throttle
                 , status
                 , self.drag()
                 , self.CoM()
                 , self.acceleration()))
            self.last_time = time.time()

    def save(self):
        with open('flight_telemetry_' + dt.datetime.now().strftime("%m%d%Y_%H%M%S") + '.pkl', 'wb') as f:
            pickle.dump(self.telemetry, f)

        with open('flight_est_profile_' + dt.datetime.now().strftime("%m%d%Y_%H%M%S") + '.pkl', 'wb') as f:
            pickle.dump(self.decent_profile, f)

    def free_fall_decent_model(self, initial_altitude, initial_velocity, mass):
        time_step = 1  # s
        current_altitude = initial_altitude  # meters
        current_velocity = initial_velocity  # meters
        density = 4.336714412  # this is an estimated air density for kerbin
        time_stamp = time_step
        data = []
        while current_altitude > 0:
            # for next drag, area of the cross-section is using 1.25m part
            # Cd is set for cylinder => 0.82
            next_drag = ((math.pi * (pow(1.25, 2))) * 0.82 * 0.05 * (pow(current_velocity, 2)) * density) / mass
            if current_velocity < 0:
                next_drag *= -1
            next_altitude = current_altitude + (current_velocity * time_step) - (
                    0.5 * (9.81 + next_drag) * pow(time_step, 2))
            next_velocity = math.sqrt(
                pow(current_velocity, 2) - (2 * (9.81 + next_drag) * (next_altitude - current_altitude)))
            sign_aid = (next_altitude - current_altitude)
            next_velocity = next_velocity * (sign_aid / abs(sign_aid))
            current_altitude = next_altitude
            current_velocity = next_velocity
            data.append((round(time_stamp, 4), round(next_altitude, 4), round(next_velocity, 4), round(next_drag, 4)))
            time_stamp += time_step
        return data

    def build_decent_profile(self):
        if self.decent_profile is None:
            self.descent_model.set_initial_velocity(self.vertical_speed())
            self.descent_model.set_initial_altitude(self.srf_altitude())
            self.descent_model.set_initial_mass(self.mass())
            self.descent_model.set_time_step(self.time_step)
            self.descent_model.free_fall()
            self.descent_model.determine_points_of_interest(self.max_thrust() * 0.85)
            self.descent_model.determine_burn_height(self.max_thrust() * 0.85)
            # initial_velocity = self.vertical_speed()
            # initial_height = self.srf_altitude()
            # mass = self.mass()
            # impact_time = self.solve_quadratic((-9.81 * 0.5), initial_velocity, initial_height)
            # self.decent_profile = self.free_fall_decent_model(initial_height, initial_velocity, mass)
            self.decent_profile = self.descent_model.estimated_free_fall_profile
        pass

    def print_landing_status(self, data):
        headers = list(data.keys())
        display_text = ''
        for header in list(headers):  # merge the header with its value into a string
            display_text += ' {heading}: {value} |'.format(heading=header, value=data[header])
        count = 100 - len(display_text)
        filler = '-' * count
        display_text += filler
        print(display_text, end="\r")

    def shutdown_engine_if_landed(self):
        if self.is_landed() or not self.is_flying():
            self.shutdown_engine()
            return LandingComplete
        pass

    def solve_quadratic(self, a, b, c):
        '''
        a => gravity * .5
        b => inital velocity
        c => distance to ground
        results are in seconds '''
        pos_x = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        neg_x = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        return (pos_x, neg_x)

    def is_radar_below_stopping_distance(self, radar, stopping_distance):
        ''' data should be in meters'''
        return radar < stopping_distance

    def calculate_time_to_burn(self, vertical_velocity, acceleration):
        return vertical_velocity / acceleration

    def calculate_stopping_distance(self, velocity, max_deceleration):
        ''' result is in meters '''
        # return round((velocity ** 2) / (2 * max_deceleration), 3)
        time_to_burn = self.calculate_time_to_burn(velocity, max_deceleration)
        return ((velocity * time_to_burn) + (.5 * max_deceleration * time_to_burn ** 2)) * 0.8  # 10% safety factor
        # return (velocity / 2) * time_to_burn

    def we_are_falling(self):
        return self.vertical_speed() < 0

    def calculate_radar_height(self, height):
        return height + self.height_of_center_of_mass

    def calculate_acceleration_due_to_thrust(self, thrust, mass, gravity, air_resistance):
        ''' results are in meters / s^2 '''
        return (thrust / mass) - (gravity + air_resistance)

    def calculate_landing_thrust(self, desired_acceleration, gravity, mass):
        return (desired_acceleration + gravity) * mass

    def determine_thrust_pct(self, thrust_in_newtons):
        return (thrust_in_newtons / self.max_thrust()) * 100.000

    def while_flying_upward(self):
        return self.vertical_speed() > 0

    def waiting_to_fall_sequence(self):
        flight_data = {
            'Craft Ascending': round(self.srf_altitude(), 2),
            'A': round(self.acceleration(), 2),
            'V': round(self.vertical_speed(), 2)
        }
        self.print_landing_status(flight_data)
        self.add_readings(self.mass()
                          , self.vertical_speed()
                          , self.srf_altitude()
                          , self.mean_altitude()
                          , self.throttle
                          , 'Coasting to Ap')

    def set_controller(self):
        self.controller = pid.PIDController(self.kp, self.ki, self.kd, self.hover_alt, self.controller_timestep,
                                            self.max_thrust())

    def land(self):
        try:
            self.start_time = time.time()
            self.last_time = self.start_time
            # TODO: can CoM be automated?
            self.set_height_of_center_of_mass(7)
            self.reposition(90, 90, stepped=False)
            while self.while_flying_upward():
                self.waiting_to_fall_sequence()
            while self.we_are_falling():  # i am past my apoapsis and am falling
                self.reposition(90, 90, stepped=False)
                self.build_decent_profile()
                gravity_calculation = -9.81
                # calculate air resistance
                air_resistance_package = AirResistanceModule.AirResistance()
                air_resistance_package.set_density_of_fluid(58484.090)
                air_resistance_package.set_velocity(self.vertical_speed() * 2)
                # TODO: Can area of craft be automated
                air_resistance_package.set_area(math.pi * pow(1.25 / 2, 2))
                air_resistance_package.calculate_force(inplace=True)
                air_drag = air_resistance_package.calculate_acceleration(air_resistance_package.force_of_drag
                                                                         , self.mass())

                max_deceleration = self.calculate_acceleration_due_to_thrust(self.max_thrust()
                                                                             , self.mass()
                                                                             , gravity_calculation
                                                                             , air_drag)
                # gravity_calculation = gravitational_acceleration(5.2915158e22, 600000, mean_altitude())
                # self.burn_height = self.calculate_stopping_distance(self.vertical_speed(), max_deceleration)
                self.burn_height = self.descent_model.recommended_burn_altitude * 1.3
                self.add_readings(self.mass()
                                  , self.vertical_speed()
                                  , self.srf_altitude()
                                  , self.mean_altitude()
                                  , self.throttle
                                  , 'Free Fall')
                while not self.is_radar_below_stopping_distance(self.calculate_radar_height(self.srf_altitude()),
                                                                self.burn_height):
                    self.reposition(90, 90, stepped=False)
                    self.burn_height = self.descent_model.recommended_burn_altitude * 1.3
                    # self.calculate_stopping_distance(self.vertical_speed(), max_deceleration)
                    flight_data = {
                        'Burn Height(m)': round(self.burn_height)
                        , 'Radar(m)': round(self.calculate_radar_height(self.srf_altitude()))
                        , 'Acceleration(m/s**2)': round(self.acceleration())
                        # this isnt actually showing me acceleration
                        , 'Velocity(m/s)': round(self.vertical_speed())
                    }
                    self.print_landing_status(flight_data)
                    self.add_readings(self.mass()
                                      , self.vertical_speed()
                                      , self.srf_altitude()
                                      , self.mean_altitude()
                                      , self.throttle
                                      , 'Free Fall')
                    pass

                # might be worth maintaining the "max decel" and using PID to stay at that level
                self.set_throttle(1)

                while not self.is_landed() and self.srf_altitude() > self.burn_height * 0.10:
                    self.reposition(90, 90, stepped=False)
                    self.landing_thrust_sequence(-15, -20, 0.0)

                while not self.is_landed() and 25 < self.srf_altitude() <= self.burn_height * 0.10:  # pre-final landing sequence
                    self.reposition(90, 90, stepped=False)
                    self.landing_thrust_sequence(-7, -10, 0.1)

                while not self.is_landed() and self.srf_altitude() <= 25:  # final landing sequence
                    self.reposition(90, 90, stepped=False)
                    self.landing_thrust_sequence(-4, -1, 0.15)

                self.shutdown_engine_if_landed()
                print('Engine Shutdown' + '-' * 85)
                break

        except LandingComplete:
            pass

        except ZeroDivisionError:
            print(self.decent_profile)

    def landing_thrust_sequence(self, vertical_speed_limit, vertical_speed_target, default_thrust):
        throttle_value = default_thrust
        if vertical_speed_limit > self.vertical_speed() < 0:
            thrust = self.MinController.compute(vertical_speed_target,
                                                self.vertical_speed())  # 5 is the abs of velocity
            throttle_value = self.determine_thrust_pct(thrust)
        self.set_throttle(throttle_value)
        flight_data = {
            'Burn Height(m)': round(self.burn_height)
            , 'Radar(m)': round(self.calculate_radar_height(self.srf_altitude()))
            , 'Velocity(m/s)': round(self.vertical_speed())
            , 'Limit': vertical_speed_limit
        }
        self.print_landing_status(flight_data)

    def set_pitch(self, value):
        self.pitch = value
        return True

    def set_heading(self, value):
        '''
        value between 0 and 360
        N - 0
        E - 90
        S - 180
        W - 270
        '''
        self.heading = value
        return True

    def reposition(self, desired_pitch, desired_heading, stepped=True):
        if stepped:
            for step in range(int(((self.pitch - desired_pitch) / self.pitch_step))):
                self.set_heading(desired_heading)
                self.set_pitch(self.pitch - self.pitch_step)
                self.autopilot.target_pitch_and_heading(self.pitch, self.heading)
                time.sleep(1 / 3)
        if not stepped:
            self.autopilot.target_pitch_and_heading(self.pitch, self.heading)
        return True


    def test(self):
        self.set_throttle(1)
        self.vessel.control.activate_next_stage()  # release clamps and start engine
        while self.srf_altitude() < 8000:
            display_text = 'Craft Ascending: {} | A: {}'.format(self.srf_altitude(), self.acceleration())
            count = 100 - len(display_text)
            filler = '-' * count
            display_text += filler
            print(display_text, end="\r")
            pass
        self.set_throttle(0)
        self.land()
        self.save()
#  items to keep in mind
#  impact time to open landing gear

# plan your expected landing sequence
# check if your current situation is following expected

# time_to_impact =
# a = -9.81 * .5 #gravity
# b = -0 # inital velocity
# c = 10000 # i want to cover 10000 meters in the downward(-) direction
# pos_x = (-b + math.sqrt(b**2 - 4*a*c))/ (2 * a)
# neg_x = (-b - math.sqrt(b**2 - 4*a*c))/ (2 * a)

# expected_velocity = vi + gravity * (second + 1)
# expected_height = (vi * (1 + second)) + (.5 * gravity * (second + 1)**2)


# def air_resistance(self, altitude, mass, velocity_z):
#     if altitude < 0:
#         rho = 1.0
#     else:
#         rho = atmosphere_model(altitude)
#
#     S = 0.01
#     Cd = 0.1
#
#     qinf = -np.pi * rho * S * Cd / mass
#     return qinf * abs(velocity_z) * velocity_z
