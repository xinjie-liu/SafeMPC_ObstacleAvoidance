# Adding to my path
# =============================================================================
# import sys
# sys.path.insert(0, 'C:\\Users\\Vassil\\Desktop\\Personal\\TU Delft\\Msc Robotics\\forces_pro_client')
# =============================================================================
import math
from math import atan2, sin, cos, radians
import time
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from model.MPC_utils import *
#from MPC_unicy import MPC

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
        # self.u = np.array([0,0])
# =============================================================================
#     def uniToDiff(self, v, w):
#         vR = (2 * v + w * self.L) / (2 * self.R)
#         vL = (2 * v - w * self.L) / (2 * self.R)
#         return vR, vL
# 
#     def diffToUni(self, vR, vL):
#         v = self.R / 2 * (vR + vL)
#         w = self.R / self.L * (vR - vL)
#         return v, w
# =============================================================================

    def fixAngle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def step(self, v, w):
# =============================================================================
#         theta_dt = w
#         old_theta = self.current.theta
#         self.current.theta = self.fixAngle(self.current.theta + self.fixAngle(theta_dt * self.dt))
#         self.current.x = self.current.x + (v / (w+1e-20)) * (sin(self.current.theta) - sin(old_theta))
#         self.current.y = self.current.y + (v / (w+1e-20)) * (cos(old_theta) - cos(self.current.theta))
# =============================================================================
# =============================================================================
#        dynamic model (old one)
        theta_dt = w
        x_dt = v * cos(self.current.theta)
        y_dt = v * sin(self.current.theta)
        self.current.theta = self.fixAngle(self.current.theta + theta_dt * self.dt)
        
        self.current.x = self.current.x + x_dt * self.dt
        self.current.y = self.current.y + y_dt * self.dt

# =============================================================================
#         # In terms of u:
#         x_dt = (self.u[0]+self.u[1])*np.cos(self.current.theta)/2
#         y_dt = (self.u[0]+self.u[1])*np.sin(self.current.theta)/2
#         theta_dt = (self.u[1]-self.u[0])/(2*self.L)
# =============================================================================
# =============================================================================

        return self.current

