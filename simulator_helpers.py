"""
File containing helper functions for frequently repeated actions in the simulator files for the different conditions
evaluated.
"""

import numpy as np
from model.quadrotor import Quadrotor
from Obstacle import Obstacle, plot_three_dee_box
import matplotlib.pyplot as plt
from matplotlib import animation
from scipy.spatial.transform import Rotation
from model.MPC_3D_trajectory_tracking import MPC_traj
from model.nonlinear_controller import GeometricController
import csv
import os


def init_simulation(mpc=True, time_horizon=50, obstacle=False):
    """
    Function for initializing multiple elements of the simulation which are generally common. Depending on the
    values of the parameters the policy will be adjusted
    :param mpc: boolean for determining whether a geometric controller or a model predictive controller is to be tested
    :param traj_tracking: boolean for specifying the type of MPC controller that will be used. Depending on whether
    path interpolation will be carried out or not
    :param time_horizon: time horizon for the minimum snap, determined in time steps
    :param obstacle:
    :return:
    """
    env = Quadrotor()
    if mpc:
        policy = MPC_traj(time_horizon, obstacle)
    else:
        policy = GeometricController()
    t0, dt, total_se, total_energy, penalty = 0, 1e-2, 0, 0, 2500
    return env, policy, t0, dt, total_se, total_energy, penalty


def find_closest(state, obstacles):
    """
    Function for computing the largest radius of the sphere in which the drone can move without colliding
    with any obstacle
    :param state: current state of the quadrotor
    :param obstacles: set of obstacles in the environment
    :return: radius of the aforementioned sphere
    """
    min_dist = 3            # Initial guess of the sphere radius
    is_free = False
    # Iterate by progressively reducing the sphere radius until no collision is found
    while is_free is False:
        is_free = True
        for obstacle in obstacles:
            check = obstacle.collision_check(state['x'], min_dist)
            if check:
                is_free = False
                if min_dist > 0.1:
                    min_dist -= 0.1
                else:
                    min_dist -= 0.01
                break
    return min_dist


def generate_env(scenario):
    """
    Function for generating the obstacles object for different scenarios in which the drone is to be tested
    and the figures and axes of the plotting. That is, more common elements for the simulations, but in this case will
    depend on the scenario selected.

    :param scenario: integer specifying the file from which the scenario parameters will be read
    :return: list of obstacles in the environment, figure and axis for plotting, the boundaries of the environment and
    numpy arrays with the initial and final position of the quadrotor in the workspace
    """

    # Open the file corresponding to the chosen scenario and read the data
    this_dir = os.path.dirname(os.path.abspath(__file__))
    filename = f"{this_dir}/scenarios/scenario_{scenario}.csv"
    print(f"Loading scenario from {filename}")
    file = open(filename)
    csvreader = csv.reader(file)

    for i in range(7):
        next(csvreader)
    rows = []
    for row in csvreader:
        rows.append(row)
    file.close()

    # Extract the obstacles
    boxes = list()
    for row in rows[16:]:
        box = np.array([[float(row[1]), float(row[2]), float(row[3])],
                        [float(row[5]), float(row[6]), float(row[7])]])
        boxes.append(box)

    # Convert the list of points in a list of obstacles
    obstacles = list()
    for box in boxes:
        obstacles.append(Obstacle(box[0, :], box[1, :]))

    # Define the figure and axis for plotting depending on the setup
    fig = plt.figure()
    axis = fig.add_subplot(111, projection="3d")
    axis.set_xlim(float(rows[0][1]), float(rows[0][2]))
    axis.set_ylim(float(rows[1][1]), float(rows[1][2]))
    axis.set_zlim(float(rows[2][1]), float(rows[2][2]))
    plt.rcParams['figure.figsize'] = float(rows[3][1]), float(rows[3][2])
    axis.set_xlabel(rows[4][1].lstrip())
    axis.set_ylabel(rows[5][1].lstrip())
    axis.set_zlabel(rows[6][1].lstrip())
    axis.set_title(rows[7][1].lstrip())
    axis.view_init(float(rows[8][1]), float(rows[8][2]))

    # Set the map boundaries for point search in RRT_star
    boundary = [float(rows[10][1]), float(rows[10][2]), float(rows[10][3])]

    # Set the initial and terminal positions, which may vary depending on the number of drones to simulate
    start_points = []
    end_points = []

    for idx in range(len(rows[12])//4):
        start_point = np.array([float(rows[12][1+4*idx]), float(rows[12][2+4*idx]), float(rows[12][3+4*idx])])
        end_point = np.array([float(rows[13][1+4*idx]), float(rows[13][2+4*idx]), float(rows[13][3+4*idx])])
        start_points.append(start_point)
        end_points.append(end_point)

    print("Loaded scenario successfully.")

    return obstacles, fig, axis, boundary, start_points, end_points


def show_scenario_no_solution(scenario):
    """
    Function for plotting a provided scenario. This function provides a static view of the map of the scenario,
    along with start and end position. No path or trajectories are plotted.
    :param scenario: int corresponding to the desired scenario.
    """
    obstacles, fig, axis, boundary, start_points, end_points = generate_env(scenario)
    for obstacle in obstacles:
        plot_three_dee_box(obstacle, ax=axis)
    for point in start_points:
        axis.plot(point[0], point[1], point[2], 'r.', zorder=100000)
    for point in end_points:
        axis.plot(point[0], point[1], point[2], 'b.', zorder=100000)
    plt.show()

def plot_all(fig, axis, obstacles, start, goal, path, trajectory, orientation, dynamic=False, obstacle_trajectory=None):
    """
    Function for plotting all the elements of the simulation

    :param fig: figure in which the plotting will be done
    :param axis: axes of the figure in which the plotting will be done
    :param obstacles: list of obstacles in the environment
    :param start: list of numpy arrays with the starting position of the drone of the simulation
    :param goal: list of numpy arrays with the goal position of the drone of the simulation
    :param path: numpy array with the points generated by the RRT_star algorithm
    :param trajectory: set of points followed in the simulation by the quadrotor
    :param orientation: set of orientations taken by the quadrotor during the simulation
    :param dynamic: boolean stating whether dynamic obstacles need to be plotted or not
    :param obstacle_trajectory: set of locations in the 3D space followed by the dynamic obstacle, if so
    :return: nothing
    """

    if dynamic:
        obstacle_trajectory = obstacle_trajectory[:len(trajectory)]
    # Plot the obstacles
    for box in obstacles:
        plot_three_dee_box(box, ax=axis)
    # Plot the start and goal points
    axis.plot([start[0]], [start[1]], [start[2]], 'go', markersize=5, label="Start")
    axis.plot([goal[0]], [goal[1]], [goal[2]], 'bo', markersize=5, label="End")

    # Plot the final path
    path_length = 0
    for i in range(len(path) - 1):
        if i == 0:
            axis.plot([path[i][0], path[i + 1][0]],
                      [path[i][1], path[i + 1][1]],
                      [path[i][2], path[i + 1][2]], c='b', linewidth=1, label="RRT_path")
        else:
            axis.plot([path[i][0], path[i + 1][0]],
                 [path[i][1], path[i + 1][1]],
                 [path[i][2], path[i + 1][2]], c='b', linewidth=1)
        path_length += np.linalg.norm(path[i] - path[i + 1])
    print('Length of path:', round(path_length, 2))

    # Plot the trajectory of the quadrotor
    rot = Rotation.from_quat(orientation[0, :]).as_matrix()
    print(rot)
    rotor_dists = np.array([[0.046 * np.sqrt(2), -0.046 * np.sqrt(2), 0],
                            [0.046 * np.sqrt(2), 0.046 * np.sqrt(2), 0],
                            [-0.046 * np.sqrt(2), 0.046 * np.sqrt(2), 0],
                            [-0.046 * np.sqrt(2), -0.046 * np.sqrt(2), 0]])
    rots = rotor_dists @ rot
    rots = rots + trajectory[0, :]
    points = np.vstack((trajectory[0, :], rots))
    point, = axis.plot(points[:, 0], points[:, 1], points[:, 2], 'r.', label='Quadrotor')
    # if dynamic:
    #     point_obstacle, = axis.plot(obstacle_trajectory[:, 0], obstacle_trajectory[:, 1], obstacle_trajectory[:, 2], 'y.', label='Obstacle')
    line, = axis.plot([trajectory[0, 0]], [trajectory[0, 1]], [trajectory[0, 2]], 'g', label='Real_Trajectory')
    print(points)
    print(point)
    print(line)

    # Helper function for the animation
    def animate(i):
        line.set_xdata(trajectory[:i + 1, 0])
        line.set_ydata(trajectory[:i + 1, 1])
        line.set_3d_properties(trajectory[:i + 1, 2])
        rot = Rotation.from_quat(orientation[i, :]).as_matrix()
        rots = rotor_dists @ rot
        rots = rots + trajectory[i, :]
        if dynamic:
            points = np.vstack((trajectory[i, :], rots, obstacle_trajectory[i, :]))
        else:
            points = np.vstack((trajectory[i, :], rots))
        point.set_xdata(points[:, 0])
        point.set_ydata(points[:, 1])
        point.set_3d_properties(points[:, 2])

    # Plot the legend and start the animation
    axis.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig, func=animate, frames=np.size(trajectory, 0), interval=1, repeat=False,
                                  blit=False)
    plt.show()


if __name__ == "__main__":
    """
    Small auxiliary main function for testing the helpers and seeing the empty scenario.
    """
    point = {'x': np.array([0, 0, 0])}
    obstacles, fig, axis, boundary, start_points, end_points = generate_env(2)
    print(find_closest(point, obstacles))
    for box in obstacles:
        plot_three_dee_box(box, ax=axis)
    axis.plot(point['x'][0], point['x'][1], point['x'][2], 'r.')
    plt.show()

    show_scenario_no_solution(2)

