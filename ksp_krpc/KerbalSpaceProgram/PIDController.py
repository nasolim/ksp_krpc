import numpy as np
import matplotlib.pyplot as plt


class PIDController:
    def __init__(self, KP, KI, KD, target, timestep, maximum,label = 'Altitude Meters', type = 'Maximum'):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.maximum = maximum
        self.timestep = timestep
        self.setpoint = target
        self.error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.error_last = 0
        self.output = 0
        self.actual = 0
        self.times = np.array([])
        self.poses = np.array([])
        self.kpe = np.array([])
        self.kde = np.array([])
        self.kie = np.array([])
        self.thrust = np.array([])
        self.setting = np.array([])
        self.last_timestamp = 0
        self.label = label
        self.type = type

    def compute(self, target, actual):
        self.actual = actual
        self.setpoint = target
        self.calculate_error()
        self.calculate_integratal_error()
        self.calculate_derivative_error()
        self.calculate_output()
        self.set_last_error(self.error)
        if self.output_above_maximum_thrust():
            self.output = self.maximum
        if self.output_below_minimum_thrust():
            self.output = 0
        self.log()
        return self.output

    def output_above_maximum_thrust(self):
        if self.type == 'Minimum':
            return self.output <= self.maximum
        return self.output >= self.maximum

    def output_below_minimum_thrust(self):
        return self.output < 0

    def calculate_error(self):
        self.error = self.setpoint - self.actual
        return self.error

    def calculate_integratal_error(self):
        self.integral_error += self.error * self.timestep
        return self.integral_error

    def calculate_derivative_error(self):
        self.derivative_error = (self.error - self.error_last) / self.timestep
        return self.derivative_error

    def calculate_output(self):
        self.output = (self.kp * self.error) + (self.ki * self.integral_error) + (self.kd * self.derivative_error)
        return self.output

    def set_last_error(self, error):
        self.error_last = error

    def set_max_thrust(self, max):
        self.maximum = max

    def log(self):
        self.last_timestamp += self.timestep
        self.times = np.append(self.times, self.last_timestamp)
        self.poses = np.append(self.poses, self.actual)
        self.kpe = np.append(self.kpe, self.error)
        self.kde = np.append(self.kde, self.derivative_error)
        self.kie = np.append(self.kie, self.integral_error)
        self.thrust = np.append(self.thrust, self.output)
        self.setting = np.append(self.setting, self.setpoint)
        return True

    def get_kpe(self):
        return self.kp * self.error

    def get_kde(self):
        return self.kd * self.derivative_error

    def get_kie(self):
        return self.ki * self.integral_error

    def graph(self):
        fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, sharex=True)
        # fig.suptitle('antiwindup')
        ax1.set(ylabel=self.label)
        # ax1.axhline(y=self.setpoint + 50, color='r', linestyle='-')
        ax1.plot(self.times, self.poses, label=self.label)
        ax1.plot(self.times, self.setting, label='Target', color='red')
        ax2.set(ylabel='KP_error \n{}'.format(self.kp))
        ax2.plot(self.times, self.kpe, 'tab:red')
        ax3.set(ylabel='KD_error \n{}'.format(self.kd))
        ax3.plot(self.times, self.kde, 'tab:orange')
        ax4.set(ylabel='KI_error \n{}'.format(self.ki))
        ax4.plot(self.times, self.kie, 'tab:pink')
        ax5.set(ylabel='Thrust \nNewtons')
        ax5.plot(self.times, self.thrust, 'tab:brown')
        plt.show()

    def __repr__(self):
        rep = 'kp: {} \nki: {} \nkd: {} \nmax: {} \nsetpoint: {} \nact: {} \nerr: {} \nierr: {} \nderr: {} \nlerr:{} \noutput: {} \n{}'.format(
            self.kp,
            self.ki,
            self.kd,
            self.maximum,
            self.setpoint,
            self.actual,
            self.error,
            self.integral_error,
            self.derivative_error,
            self.error_last,
            self.output,
            '=' * 10)
        return rep

# Ziegler-nichols Approach
#


#  The proportional controller improves the transient response of the system.
#  Transient response: the time taken by a system to reach the steady state from its initial state.
# Steady state: the point at which the system output becomes settled and starts working in normal condition.
#    The proportional controller adds the product of the proportional gain (Kp) and error value to the process control output. In this case, the system response is directly proportional to the gain hence the error decreases with the increase in proportional gain.
#    The proportional controller always gives the steady-state error. If we increase the proportional gain more than the critical gain value, the output creates oscillation.

# https://www.csimn.com/CSI_pages/PIDforDummies.html
# For example, if an oven is cooler than required, the heat will be increased. Here are the three steps:
#
# Proportional tuning involves correcting a target proportional to the difference. Thus, the target value is never achieved because as the difference approaches zero, so too does the applied correction.
# Integral tuning attempts to remedy this by effectively cumulating the error result from the "P" action to increase the correction factor. For example, if the oven remained below temperature, “I” would act to increase the head delivered. However, rather than stop heating when the target is reached, "I" attempts to drive the cumulative error to zero, resulting in an overshoot.
# Derivative tuning attempts to minimize this overshoot by slowing the correction factor applied as the target is approached.


# A proportional integral derivative (PID) controller can be used as a means of controlling temperature, pressure, flow and other process variables. As its name implies, a PID controller combines proportional control with additional integral and derivative adjustments which help the unit automatically compensate for changes in the system.
