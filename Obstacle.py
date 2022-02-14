"""
Contains the source code for defining the obstacle class and functions for collision detection and object plotting
"""
import numpy as np
import matplotlib.pyplot as plt


class Obstacle(object):
    """
    Obstacle object using AABB min-max coordinate representation
    In addition to the min-max point coordinates, it also stores such values for the collision object given
    a robot of given redius
    """
    def __init__(self, point_min=[0, 0, 0], point_max=[0, 0, 0], radius=0.05):
        self.point_min_ = point_min
        self.point_max_ = point_max
        self.collision_min_ = point_min - radius
        self.collision_max_ = point_max + radius
        self.r_ = radius

    def collision_check(self, point, r=None):
        """
        Function for checking collision with a given point
        :param point: location of the drone in space
        :param r: if given, collision with the obstacles given the new radius of the robot is calculated instead
        :return: True is collision is detected, False otherwise
        """
        if r is None:
            if self.collision_min_[0] <= point[0] <= self.collision_max_[0] and \
                self.collision_min_[1] <= point[1] <= self.collision_max_[1] and \
                    self.collision_min_[2] <= point[2] <= self.collision_max_[2]:
                return True
            else:
                return False
        else:
            if self.collision_min_[0] - r <= point[0] <= self.collision_max_[0] + r and \
                self.collision_min_[1] - r <= point[1] <= self.collision_max_[1] + r and \
                    self.collision_min_[2] - r <= point[2] <= self.collision_max_[2] + r:
                return True
            else:
                return False

    # Some getters which may come in handy
    def get_min(self):
        return self.collision_min_

    def get_max(self):
        return self.collision_max_


def collision_check_path(start_pos, goal_pos, obstacle_array, r = None):
    """
    Function for checking collision along the path connecting two points in a straight line
    :param start_pos: Initial point to start checking from
    :param goal_pos: final point to which finish the checking
    :param obstacle_array: set of obstacles in the environment, both in workspace and configuration space
    :return: True if collision is detected, False otherwise
    """
    start_pos = np.array(start_pos)
    goal_pos = np.array(goal_pos)
    n = 100
    direction = (goal_pos-start_pos)/n
    for i in range(n+1):
        current_pos = start_pos + i*direction
        # Given obstacle_array an array containing list of obstacle objects
        for obstacle in obstacle_array:
            collision_single_obs = obstacle.collision_check(current_pos, r)
            if collision_single_obs:
                return True
    return False


def plot_three_dee_box(points, ax=None, rgb=(0.75, 0.75, 0.75), opacity=0.75, show=False):
    """
    Given a set of 3D points (or obstacle objects), min-max AABB (axis-aligned bounding box) are created for each
    pair of points (or per obstacle), and plots them.
    :param points: the min-max box-defining points, format can be a np.array of shape (2n, 3), or an Obstacle object.
    :param ax: the parent plotting environment, if none is provided, one will be created automatically
    :param rgb: the color to use, by default red, format is a tuple of RGB values
    :param opacity: determines the opacity og the box, format is a float, must be between 0 and 1
    :param show: if set to True, the function will automatically plot the box
    :return: None
    """
    if type(points) == Obstacle:
        points = np.array([points.point_min_, points.point_max_])

    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    x = points[:, 0]    # the box's x-coordinate extremities
    y = points[:, 1]    # the box's y-coordinate extremities
    z = points[:, 2]    # the box's z-coordinate extremities

    yy_x, zz_x = np.meshgrid(y, z)      # the grids of points for the planes perpendicular to the x-axis
    xx_y, zz_y = np.meshgrid(x, z)      # the grids of points for the planes perpendicular to the y-axis
    xx_z, yy_z = np.meshgrid(x, y)      # the grids of points for the planes perpendicular to the z-axis

    # plotting all 6 faces of the cuboid
    ax.plot_surface(points[0, 0].reshape(1, 1), yy_x, zz_x, color=rgb, alpha=opacity)
    ax.plot_surface(points[1, 0].reshape(1, 1), yy_x, zz_x, color=rgb, alpha=opacity)
    ax.plot_surface(xx_y, points[0, 1].reshape(1, 1), zz_y, color=rgb, alpha=opacity)
    ax.plot_surface(xx_y, points[1, 1].reshape(1, 1), zz_y, color=rgb, alpha=opacity)
    ax.plot_surface(xx_z, yy_z, points[0, 2].reshape(1, 1), color=rgb, alpha=opacity)
    ax.plot_surface(xx_z, yy_z, points[1, 2].reshape(1, 1), color=rgb, alpha=opacity)

    if show:
        plt.show()
