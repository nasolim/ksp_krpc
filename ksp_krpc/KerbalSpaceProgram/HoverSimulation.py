import turtle
import numpy as np
import matplotlib.pyplot as plt
import PIDController as pid
import time

# GLOBAL PARAMS
TIMER = 0
TIME_STEP = 0.001
# SETPOINT = 1500
SETPOINT = 1500
SIM_TIME = 60000
INITIAL_X = 0
INITIAL_Y = 0
# MASS = 6300 #kg
MASS = 6300  # kg
# MAX_THRUST = 205783 #Newtons
MAX_THRUST = 205534  # Newtons
g = -9.81  # Gravitational constant
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
        # self.screen = turtle.Screen()
        # self.screen.setup(800, 600)
        # self.marker = turtle.Turtle()
        # self.marker.penup()
        # self.marker.left(180)
        # self.marker.goto(10000, SETPOINT)
        # self.marker.color('red')
        self.sim = True
        self.timer = 0
        self.poses = np.array([])
        self.times = np.array([])
        self.kpe = np.array([])
        self.kde = np.array([])
        self.kie = np.array([])
        self.thrst = np.array([])

    def cycle(self):
        while (self.sim):
            thrust = self.pid.compute(SETPOINT, self.Insight.get_y())
            # print(self.pid)
            self.Insight.set_ddy(thrust)
            self.Insight.set_dy()
            self.Insight.set_y()
            # time.sleep(TIME_STEP)
            self.timer += 1
            print(self.timer)
            if self.timer > SIM_TIME:
                print("SIM ENDED")
                self.sim = False
            elif self.Insight.get_y() > 100000:
                print("OUT OF BOUNDS")
                self.sim = False
            elif self.Insight.get_y() < -100:
                print("OUT OF BOUNDS")
                self.sim = False
            self.poses = np.append(self.poses,self.Insight.get_y())
            self.times = np.append(self.times,self.timer / 1000)
        # self.pid.graph()
        graph(self.times,self.poses)

    def set_KP(self, value):
        global KP
        KP = value

class Rocket(object):
    def __init__(self):
        global Rocket
        self.Rocket = turtle.Turtle()
        self.Rocket.shape('circle')
        self.Rocket.color('black')
        self.Rocket.penup()
        self.Rocket.goto(INITIAL_X, INITIAL_Y)
        self.Rocket.speed(0)
        # physics
        self.ddy = 0
        self.dy = V_i
        self.y = INITIAL_Y

    def set_ddy(self, thrust):
        self.ddy = (g + thrust / MASS) / 1000

    def get_ddy(self):
        return self.ddy

    def set_dy(self):
        self.dy += self.ddy * TIME_STEP

    def get_dy(self):
        return self.dy

    def set_y(self):
        self.Rocket.sety(self.y + self.dy)

    def get_y(self):
        self.y = self.Rocket.ycor()
        return self.y


class TestRocket:
    def __init__(self):
        global testRocket
        self.ddy = 0
        self.dy = V_i
        self.y = INITIAL_Y

    def set_ddy(self, thrust):
        self.ddy = (g + thrust / MASS) / 1000

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




def graph(x,y):
    plt.plot(x,y)
    plt.title('kp: {} ki: {} kd: {} time: {} ms'.format(KP,KI,KD, SIM_TIME))
    plt.grid()
    plt.show()

def main():
    print('Prepping Sim')
    sim = Simulation()
    print('Running Cycles')
    sim.cycle()

main()
