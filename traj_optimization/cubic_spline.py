"""
File for cubic spline interpolation definition
"""
from numpy.linalg import norm
from scipy.interpolate import CubicSpline
from traj_optimization.min_snap_utils import *
from simulator_helpers import generate_env, init_simulation
from Obstacle import plot_three_dee_box
from matplotlib import pyplot as plt
import time


def cubic_spline(path_list, T):
    """
    Function for computing cubic spline interpolation using the function CubicSpline from scipy.interpolate
    :param path_list: list of 3D coordinates to be linked by the splines
    :param T: Total time required to complete the path
    :return: numpy arrays containing the position, velocity and acceleration along the given path in time steps of 0.01s
    """
    path_array = np.array(path_list)
    num = path_array.shape[0]

    # allocate the time for each segment based on distance with respect total
    dis_list = []
    for i in range(num-1):
        dis = norm(path_array[num-1-i, :]-path_array[num-i-2, :])
        dis_list.append(dis)
    t_array = np.zeros(num)
    for i in range(num-1):
        t_array[i] = sum(dis_list[0:i]) / sum(dis_list) * T
    t_array[num-1] = T

    # get the x, y, z coordinates
    x = path_array[:, 0]
    y = path_array[:, 1]
    z = path_array[:, 2]

    # Obtain the splines for each of the coordinates, given the allocated times for
    # each segment. The boundary type is set to be 'clamped', which means
    # the first derivative at curves ends are zero. 
    fx = CubicSpline(t_array, x, bc_type='clamped')
    t_new = np.linspace(0, T, int(T/0.01))          # The time is resampled at fixed intervals to get:
    x_new = fx(t_new)                               # Position
    vel_x = fx(t_new, 1)                            # Velocity
    acc_x = fx(t_new, 2)                            # and Acceleration
    jerk_x = fx(t_new, 3)
    snap_x = fx(t_new, 4)

    fy = CubicSpline(t_array, y, bc_type='clamped')
    y_new = fy(t_new)
    vel_y = fy(t_new, 1)
    acc_y = fy(t_new, 2)
    jerk_y = fy(t_new, 3)
    snap_y = fy(t_new, 4)

    fz = CubicSpline(t_array, z, bc_type='clamped')
    z_new = fz(t_new)
    vel_z = fz(t_new, 1)
    acc_z = fz(t_new, 2)
    jerk_z = fz(t_new, 3)
    snap_z = fz(t_new, 4)

    pos = np.vstack((x_new, y_new, z_new)).T   
    vel = np.vstack((vel_x, vel_y, vel_z)).T   
    acc = np.vstack((acc_x, acc_y, acc_z)).T
    jerk = np.vstack((jerk_x, jerk_y, jerk_z)).T
    snap = np.vstack((snap_x, snap_y, snap_z)).T

    return pos, vel, acc, jerk, snap
