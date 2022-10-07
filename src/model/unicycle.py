import math
from math import atan2, sin, cos, radians
import time
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from src.model.MPC_utils import *

class State():

    def __init__(self, x_=0, y_=0, theta_=0):
        self.x = x_
        self.y = y_
        self.theta = theta_

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.theta)

class Robot():
    def __init__(self, x_=0, y_=0, z_=0, R_=0.0325, L_=0.1, dt=2e-2):
        self.current = State(x_, y_, z_) # zero initialization
        self.R = R_  # in meter
        self.L = L_  # in meter
        self.dt = dt

    def fixAngle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def step(self, v, w):
# =============================================================================
# another dynamic model
#         theta_dt = w
#         old_theta = self.current.theta
#         self.current.theta = self.fixAngle(self.current.theta + self.fixAngle(theta_dt * self.dt))
#         self.current.x = self.current.x + (v / (w+1e-20)) * (sin(self.current.theta) - sin(old_theta))
#         self.current.y = self.current.y + (v / (w+1e-20)) * (cos(old_theta) - cos(self.current.theta))
# =============================================================================

#       dynamic model
        theta_dt = w
        x_dt = v * cos(self.current.theta)
        y_dt = v * sin(self.current.theta)
        self.current.theta = (self.current.theta + theta_dt * self.dt)

        self.current.x = self.current.x + x_dt * self.dt
        self.current.y = self.current.y + y_dt * self.dt

        return self.current

