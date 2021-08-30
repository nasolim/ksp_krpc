import CraftModule
import time
from IPython.display import display, clear_output
import PIDController as pid


class HoverProtocol(CraftModule.Craft):
    def __init__(self):
        CraftModule.Craft.__init__(self)
        self.timestep = 0.001
        self.hover_alt = 1500
        self.kp = 2000
        self.ki = 2042.5531914894
        self.kd = 750
        self.controller = pid.PIDController(self.kp, self.ki, self.kd, self.hover_alt, self.timestep, self.max_thrust())

    # situation operator functions


    def continue_holding_pattern(self, powered_flight, elapsed_time, seconds):
        return powered_flight and elapsed_time < seconds

    #  flight information
    def elapsed_time(self, start, end):
        return end - start

    def determine_thrust_pct(self, thrust_in_newtons):
        return (thrust_in_newtons / self.max_thrust()) * 100.000

    def set_hover_altitude(self, altitude):
        self.hover_alt = altitude
        self.set_controller()

    def set_controller(self):
        self.controller = pid.PIDController(self.kp, self.ki, self.kd, self.hover_alt, self.timestep, self.max_thrust())

    def hover(self, seconds=1000, altitude=500):
        self.set_hover_altitude(altitude)
        start = time.time()
        try:
            while self.continue_holding_pattern(self.is_flying(), self.elapsed_time(start, time.time()), seconds):
                thrust = self.controller.compute(1500, self.mean_altitude())
                self.set_throttle(self.determine_thrust_pct(thrust))
                time.sleep(self.timestep)
                print('{} seconds remaining\n'.format(round(seconds - self.elapsed_time(start, time.time())), end="\r"))
                print(self.controller, end="\r")
            self.vessel.control.throttle = 0
            self.controller.graph()
        except ZeroDivisionError as err:
            self.vessel.control.throttle = 0
            self.controller.graph()
            print('{} has occurred'.format(err))
        # return df
