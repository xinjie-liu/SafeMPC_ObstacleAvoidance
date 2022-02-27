import math
from math import atan2, sin, cos, radians
import time
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from MPC_unicy import MPC

class State():

    def __init__(self, x_=0, y_=0, theta_=0):
        self.x = x_
        self.y = y_
        self.theta = theta_

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.theta)

class Robot():
    def __init__(self, R_=0.0325, L_=0.1):
        self.current = State(0, 0, 0) # zero initialization
        self.R = R_  # in meter
        self.L = L_  # in meter
        self.dt = 5e-2
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
        return atan2(sin(angle), cos(angle))

    def step(self, v, w):
# =============================================================================
        x_dt = v * cos(self.current.theta)
        y_dt = v * sin(self.current.theta)
        theta_dt = w
# =============================================================================
        # In terms of u:
        # x_dt = (self.u[0]+self.u[1])*np.cos(self.current.theta)/2
        # y_dt = (self.u[0]+self.u[1])*np.sin(self.current.theta)/2
        # theta_dt = (self.u[1]-self.u[0])/(2*self.L)

        self.current.x = self.current.x + x_dt * self.dt
        self.current.y = self.current.y + y_dt * self.dt
        self.current.theta = self.fixAngle(self.current.theta + self.fixAngle(theta_dt * self.dt))

        return self.current


# Starting point of the code
def main():

    def animate(i):
        line.set_xdata(real_trajectory['x'][:i + 1])
        line.set_ydata(real_trajectory['y'][:i + 1])
        line.set_3d_properties(real_trajectory['z'][:i + 1])
        point.set_xdata(real_trajectory['x'][i])
        point.set_ydata(real_trajectory['y'][i])
        point.set_3d_properties(real_trajectory['z'][i])

    env = Robot()
    mpc = MPC(20)
    real_trajectory = {'x': [], 'y': [], 'z': []}
    for iter in range(500):
        # state = env.step(0.5, 0.8)
        v, w = mpc.control(env.current, np.array([5., 0., 0.]))
        state = env.step(v, w)
        print(env.current)
        real_trajectory['x'].append(state.x)
        real_trajectory['y'].append(state.y)
        real_trajectory['z'].append(0)

    # plotting stuff
    fig = plt.figure()
    ax1 = p3.Axes3D(fig) # 3D place for drawing
    real_trajectory['x'] = np.array(real_trajectory['x'])
    real_trajectory['y'] = np.array(real_trajectory['y'])
    real_trajectory['z'] = np.array(real_trajectory['z'])
    point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'ro',
                      label='Robot', markersize=15)
    line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]],
                     label='Real_Trajectory')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.set_title('3D animate')
    ax1.set_xlim(-5., 5.)
    ax1.set_ylim(-5., 5.)
    ax1.set_zlim(0., 3.)
    ax1.legend(loc='lower right')

    ani = animation.FuncAnimation(fig=fig,
                                  func=animate,
                                  frames=len(real_trajectory['x']),
                                  interval=1,
                                  repeat=False,
                                  blit=False)
    plt.show()

if __name__ == "__main__":
    main()