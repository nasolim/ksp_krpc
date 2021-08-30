import math
import AirResistanceModule


class DescentModel(AirResistanceModule.AirResistance):
    def __init__(self):
        self.drag_model = AirResistanceModule.AirResistance()
        self.initial_velocity = 0  # m/s
        self.initial_altitude = 0  # m
        self.initial_mass = 0  # kg
        self.time_step = 1  # second
        self.coefficient_of_drag = 0.82
        self.craft_radius = 1.25
        self.gravity = 9.81
        self.craft_cross_sectional_area = (math.pi * (pow(self.craft_radius, 2)))
        self.estimated_free_fall_profile = []
        self.estimated_deceleration_burn_profile = []
        self.points_of_interest = []
        self.recommended_burn_altitude = None
        self.estimated_stopping_point = None

    def set_initial_velocity(self, value):
        ''' m/s'''
        self.initial_velocity = value  # m/s

    def set_gravity(self, value):
        ''' m/s'''
        self.gravity = value  # m/s^2

    def set_initial_altitude(self, value):
        ''' m'''
        self.initial_altitude = value  # m

    def set_initial_mass(self, value):
        ''' kg'''
        self.initial_mass = value  # kg

    def set_time_step(self, value):
        ''' second'''
        self.time_step = value  # second

    def set_coefficient_of_drag(self, value):
        '''unitless'''
        self.coefficient_of_drag = value

    def set_craft_radius(self, value):
        '''m'''
        self.craft_radius = value

    def set_estimated_free_fall_profile(self, value):
        '''m'''
        self.estimated_free_fall_profile = value

    def calculate_acceleration_due_to_thrust(self, thrust, mass, gravity, air_resistance):
        ''' results are in meters / s^2 '''
        return (thrust / mass) - (gravity + air_resistance)

    def calculate_next_velocity(self, current_altitude, current_velocity, next_altitude, next_acceleration_from_drag):
        ''' returns the absolute value of my velocity at the next timestamp in m/s'''
        change_in_altitude = next_altitude - current_altitude
        change_due_to_acceleration = 2 * -self.calculate_acceleration_due_to_thrust(0
                                                                                    , self.initial_mass
                                                                                    , self.gravity
                                                                                    , next_acceleration_from_drag)
        velocity_squared = pow(current_velocity, 2)

        return math.sqrt(velocity_squared - change_due_to_acceleration * change_in_altitude)

    def calculate_next_altitude_position(self, current_altitude, current_velocity, next_acceleration_from_drag):
        return current_altitude + (current_velocity * self.time_step) - (
                0.5 * (self.gravity + next_acceleration_from_drag) * pow(self.time_step, 2))

    def free_fall(self):
        current_altitude = self.initial_altitude
        current_velocity = self.initial_velocity
        density = 4.336714412  # air density ratio
        time_stamp = self.time_step
        data = []
        self.drag_model.set_area((self.craft_cross_sectional_area))
        self.drag_model.set_density_of_fluid(density)
        # while we are above ground, calculate flight information in regular intervals
        while current_altitude > 0:
            self.set_velocity(current_velocity)
            # next_acceleration_from_drag is in m/s^2
            next_acceleration_from_drag = (self.craft_cross_sectional_area * self.coefficient_of_drag * 0.05 * (
                pow(current_velocity, 2)) * density) / self.initial_mass
            if current_velocity < 0:
                next_acceleration_from_drag *= -1
            next_altitude = self.calculate_next_altitude_position(current_altitude, current_velocity, next_acceleration_from_drag)
            next_velocity = self.calculate_next_velocity(current_altitude, current_velocity, next_altitude, next_acceleration_from_drag)
            sign_aid = (next_altitude - current_altitude)
            next_velocity = next_velocity * (sign_aid / abs(sign_aid))  # change velocity to proper direction
            # update altitude and velocity for next iteration
            acceleration = (next_velocity - current_velocity) / self.time_step
            current_altitude = next_altitude
            current_velocity = next_velocity
            data.append((round(time_stamp, 4), round(next_altitude, 4), round(next_velocity, 4),round(acceleration,4), round(next_acceleration_from_drag, 4)))
            # update the time stamp
            time_stamp += self.time_step
        #     print((round(time_stamp + 1,4),round(next_altitude,4), round(next_velocity,4), round(next_acceleration_from_drag,4)))

        self.set_estimated_free_fall_profile(data)

    def calc_difference_sign(self, distance):
        return int(distance / abs(distance))

    def calc_difference(self, stopping_distance, alt):
        return alt - stopping_distance

    def calc_distance(self, velocity, time_to_burn, acceleration):
        return ((abs(velocity) * time_to_burn) + (.5 * acceleration * time_to_burn ** 2))

    def submit_information(self, datum, acceleration, data_position):
        alt, velocity = datum[1], datum[2]
        time_to_burn = abs(velocity) / acceleration
        stopping_distance = self.calc_distance(velocity, time_to_burn, acceleration)
        difference = self.calc_difference(stopping_distance, alt)
        self.points_of_interest.append({'altitude': alt, 'time_to_burn': time_to_burn,
                                        'stopping_distance': stopping_distance, 'difference': difference,
                                        'velocity': velocity, 'data_index': data_position})

    def determine_points_of_interest(self, thrust):
        # determine the close data points to a successful landing opportunity
        data_position = 0
        # TODO: determine acceleration from ship
        acceleration = self.calculate_acceleration_due_to_thrust(thrust, self.initial_mass, self.gravity, 0)
        difference_sign = -1
        self.estimated_free_fall_profile.reverse()
        #  I am attempting to limit the number of data points in distances
        for datum in self.estimated_free_fall_profile:
            alt, velocity = datum[1], datum[2]
            if velocity < 0 and alt > 0:
                time_to_burn = abs(velocity) / acceleration
                stopping_distance = self.calc_distance(velocity, time_to_burn, acceleration)
                difference = self.calc_difference(stopping_distance, alt)
                if self.calc_difference_sign(difference) != difference_sign:
                    self.submit_information(self.estimated_free_fall_profile[data_position - 1]
                                            , acceleration
                                            , data_position)
                    self.submit_information(self.estimated_free_fall_profile[data_position]
                                            , acceleration
                                            , data_position)
                    break
            data_position += 1

        self.estimated_free_fall_profile.reverse()

    def calculate_acceleration_due_to_thrust(self, thrust, mass, gravity, air_resistance):
        ''' results are in meters / s^2 '''
        return (thrust / mass) - (gravity + air_resistance)

    def determine_burn_height(self, thrust):
        self.estimated_free_fall_profile.reverse()
        # best option is put at randomly high position
        best_option = 1000000
        best = []
        for height_record in self.estimated_free_fall_profile:
            # reduce the time step to get better accuracy
            adjusted_time_step = self.time_step / 2
            current_landing_altitude = height_record[1]
            current_landing_velocity = height_record[2]
            # density is a random ratio that is specific to kerbin
            density = 3.336714412
            time_stamp = adjusted_time_step
            current_landing_mass = self.initial_mass
            landing_data = []  # for the data point being examined, save all progression data to landing_data
            while current_landing_velocity < 0 and current_landing_altitude > 0:
                #  250 is the Isp of the engine
                current_landing_mass = current_landing_mass - (thrust / (250 * self.gravity)) * adjusted_time_step
                # 1.25 is radius of craft
                # .82 is Cd of cylinder
                next_acceleration_from_drag = (self.craft_cross_sectional_area * self.coefficient_of_drag * 0.05 * (
                    pow(current_landing_velocity, 2)) * density) / current_landing_mass
                if current_landing_velocity < 0:
                    next_acceleration_from_drag *= -1
                # 9.81 is kerbin gravity
                acceleration = self.calculate_acceleration_due_to_thrust(thrust
                                                                         , current_landing_mass
                                                                         , self.gravity
                                                                         , next_acceleration_from_drag)
                # print('Acceleration: {} Mass: {} Drag: {}'.format(acceleration, current_landing_mass, next_acceleration_from_drag))
                next_landing_altitude = current_landing_altitude + (current_landing_velocity * adjusted_time_step) - (
                        0.5 * (-acceleration) * pow(adjusted_time_step, 2))
                next_landing_velocity = math.sqrt(pow(current_landing_velocity, 2) - (
                        2 * (-acceleration) * (next_landing_altitude - current_landing_altitude)))
                sign_aid = (next_landing_altitude - current_landing_altitude)
                next_landing_velocity = next_landing_velocity * (sign_aid / abs(sign_aid))
                # if velocity is positive, this means we are moving back up, therefore we are done
                if next_landing_velocity > 0:
                    break
                current_landing_altitude = next_landing_altitude
                current_landing_velocity = next_landing_velocity
                landing_data.append((round(time_stamp, 4)
                                     , round(next_landing_altitude, 4)
                                     , round(next_landing_velocity, 4)
                                     , round(acceleration, 4)
                                     , round(next_acceleration_from_drag, 4)))
                time_stamp += adjusted_time_step
            if len(landing_data) > 0 and 0 < landing_data[-1][1] < best_option:
                best = landing_data
                best_option = landing_data[-1][1]
        self.recommended_burn_altitude = best[0][1]
        self.estimated_stopping_point = best[-1][1]
        self.estimated_deceleration_burn_profile = best
        self.estimated_free_fall_profile.reverse()

