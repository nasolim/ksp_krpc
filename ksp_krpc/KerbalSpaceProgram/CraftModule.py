import krpc
import time


class Craft:
    def __init__(self):
        self.connection = None
        self.vessel = None
        self.throttle = 0
        self.flight_info = None
        self.orbit_info = None
        self.srf_altitude = None
        self.position = None
        self.mean_altitude = None
        self.max_thrust = None
        self.thrust = None
        self.vertical_speed = None
        self.mass = None
        self.connect()
        self.craft_status()

    def connect(self):
        self.connection = krpc.connect(name='Craft Program',
                                       address='127.0.0.1')
        return True

    def disconnect(self):
        self.connection.close()
        return True

    def active_vessel(self):
        self.vessel = self.connection.space_center.active_vessel
        return True

    def set_throttle(self, throttle):
        '''
        throttle in decimal percentage and ranges from 0 to 1
        '''
        if throttle > 1:
            throttle = 1
        if throttle < 0:
            throttle = 0
        self.throttle = throttle
        self.vessel.control.throttle = throttle
        return True

    def max_thrust(self):
        '''
        Set throttle to 1
        '''
        throttle = 1
        self.throttle = throttle
        self.vessel.control.throttle = throttle
        return True

    def shutdown_engine(self):
        '''
        Set throttle to 0
        '''
        throttle = 0
        self.throttle = throttle
        self.vessel.control.throttle = throttle
        return True

    def craft_status(self):
        if self.vessel is None:
            self.active_vessel()
        self.flight_info = self.vessel.flight()
        self.orbit_info = self.vessel.orbit
        self.srf_altitude = self.connection.add_stream(getattr, self.flight_info, 'surface_altitude')
        # drag
        self.drag = self.connection.add_stream(getattr, self.flight_info, 'drag')
        # center of mass
        self.CoM = self.connection.add_stream(getattr, self.flight_info, 'center_of_mass')
        refframe = self.vessel.orbit.body.reference_frame
        self.position = self.connection.add_stream(self.vessel.position, refframe)
        self.mean_altitude = self.connection.add_stream(getattr, self.flight_info, 'mean_altitude')
        # mean_altitude: The altitude above sea level, in meters. Measured from the center of mass of the vessel.
        self.max_thrust = self.connection.add_stream(getattr, self.vessel, 'max_thrust')
        self.thrust = self.connection.add_stream(getattr, self.vessel, 'thrust')
        self.vertical_speed = self.connection.add_stream(getattr, self.vessel.flight(refframe), 'vertical_speed')
        self.mass = self.connection.add_stream(getattr, self.vessel, 'mass')
        return True

    def acceleration(self):
        ''' result in m/s^2'''
        Vi = self.vertical_speed()
        time.sleep(0.1)
        Vf = self.vertical_speed()
        return (Vf - Vi) / 0.1

    def is_flying(self):
        return self.vessel.situation == self.vessel.situation.flying

    def is_landed(self):
        return self.vessel.situation == self.vessel.situation.landed

    def engine_id(self):
        parts = self.vessel.parts.in_stage(self.vessel.control.current_stage - 1)
        engines = [part.engine for part in parts if part.engine != None]

    #       are there any engines in the current stage that have run out of fuel?

    def gravitational_acceleration(self, body_mass, radius, altitude):
        return 6.67430e-11 * (body_mass / pow((radius + altitude), 2))

    def wait_until(self, operation = 'eq', target = 0, current = 0):
        functions = {
            'eq': lambda target, current: target == current,
            'lt': lambda target, current: target > current,
            'gt': lambda target, current: target < current,
            'gt_eq': lambda target, current: target <= current,
            'le_eq': lambda target, current: target >= current
        }
        job = functions[operation]
        return job(target, current)

# we are trying to identify if the current asp stage has run out of fuel
# identify all the engines
# check if they still have fuel
# if not, cut the stage

# CelestialBody.name



# flight = craft.connection.space_center.active_vessel.flight(craft.connection.space_center.active_vessel.orbit.body.reference_frame)
# flight.atmosphere_density
# craft.connection.space_center.CelestialBody.density_at(craft.connection.space_center.active_vessel.flight().surface_altitude())