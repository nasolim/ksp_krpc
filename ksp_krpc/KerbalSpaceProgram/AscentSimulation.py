import turtle
import numpy as np
import matplotlib.pyplot as plt
import PIDController as pid
import time
from progress.bar import Bar

# GLOBAL PARAMS
TIMER = 0
TIME_STEP = 0.001
# SETPOINT = 1500
SETPOINT = 1500
SIM_TIME = 300000
INITIAL_X = 0
INITIAL_Y = 0
# MASS = 6300 #kg
MASS = 6300  # kg
# MAX_THRUST = 205783 #Newtons
MAX_THRUST = 205534  # Newtons
g = -9.80  # Gravitational constant
V_i = 0  # initial velocity
Y_i = 0  # initial height
# ---PID GAINS---
antiWindup = True
# ku = .025
#  250 ms
#  ki
# kd
KP = 2000
KI = 2042.5531914894
KD = 705
# ---------------


class Simulation(object):
    def __init__(self):
        self.Insight = TestRocket()
        self.pid = pid.PIDController(KP, KI, KD, SETPOINT, TIME_STEP, MAX_THRUST)
        self.sim = True
        self.timer = 0
        self.poses = np.array([])
        self.acc = np.array([])
        self.times = np.array([])
        self.consum = np.array([])
        self.kpe = np.array([])
        self.kde = np.array([])
        self.kie = np.array([])
        self.thrst = np.array([])

    def cycle(self):
        #  0 => 100.13
        #  100 => 101.01
        #  1000 => 109.30
        #  10000 => 240.52
        bar = Bar('Running Cycles', max=300000)
        ct = 0
        while (self.sim):
            # print(self.pid)
            self.Insight.set_ddy(MAX_THRUST)
            self.Insight.set_dy()
            self.Insight.set_y()
            # time.sleep(TIME_STEP)
            self.timer += 1
            if self.timer > SIM_TIME:
                print("SIM ENDED")
                self.sim = False
            elif self.Insight.get_y() > 200000:
                print("OUT OF BOUNDS {}m".format(self.Insight.get_y()))
                self.sim = False
            elif self.Insight.get_y() < -100:
                print("OUT OF BOUNDS")
                self.sim = False
            self.acc = np.append(self.acc,self.Insight.get_ddy())
            self.poses = np.append(self.poses,self.Insight.get_y())
            self.times = np.append(self.times,self.timer / 1000)
            self.consum = np.append(self.consum,self.Insight.consumption)
            bar.next()
        bar.finish()
        graph(self.times,self.poses, 'altitude (m)')
        graph(self.times,self.acc, 'acceleration (m/s**2)')
        graph(self.times,self.consum, 'consumption (kg)')

    def set_KP(self, value):
        global KP
        KP = value


class TestRocket:
    def __init__(self):
        global testRocket
        self.ddy = 0
        self.dy = V_i
        self.y = INITIAL_Y
        self.consumption = 0
        self.wet_mass = 6620
        self.dry_mass = 2620

    def set_ddy(self, thrust):
        # divided by a thousand because the timestep is in milliseconds,
        # so the thrust is converted to meters / millisecond
        thrust = thrust if self.consumption <= self.wet_mass - self.dry_mass else 0
        self.ddy = (g + thrust / self.get_mass()) / 1000
        self.consumption += ((thrust / 265) / ( -1 * g) ) / 1000

    def get_ddy(self):
        return self.ddy

    def set_dy(self):
        self.dy += self.ddy * TIME_STEP

    def get_dy(self):
        return self.dy

    def set_y(self):
        self.y += self.dy

    def get_y(self):
        return self.y

    def get_mass(self):
        return self.wet_mass - self.consumption





def graph(x,y, name):
    plt.plot(x,y)
    plt.title('{} time: {} ms'.format(name, SIM_TIME))
    plt.grid()
    plt.show()

def main():
    print('Prepping Sim')
    sim = Simulation()
    sim.cycle()

main()
