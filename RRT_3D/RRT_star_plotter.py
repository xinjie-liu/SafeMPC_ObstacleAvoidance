"""
RRT_star class definition file
"""
import numpy as np
from numpy.linalg import norm
from Obstacle import collision_check_path, plot_three_dee_box
from simulator_helpers import generate_env
import matplotlib.pyplot as plt
import time


# Definition of Class Node
class Node:
    def __init__(self, pos, cost, parent_idx):
        self.pos = pos
        self.cost = cost                # cost from the start to the current node
        self.parent_idx = parent_idx    # parent's node index


# Definition of the RRT* search class
class RRT_star:
    """
    Given the start pos, goal pos and map boundary,
    RRT* method is used to generate a waypoints list, 
    return the path_list. This secondary file also plots
    the evolution of the tree with the iterations
    """

    # Class constructor given an initial pos
    def __init__(self, x_start, num_iter, obstacles, thr=0.5, margin=0, ax_anim=None):
        # Set parameters
        self.num_dim = 3        # number of dimensions to search for
        self.thr = thr          # threshold for final goal
        self.pathFind = False   # boolean for found path indication
        self.neigh_dist = 5     # threshold for termination
        self.num_iter = num_iter
        self.goal_idx = 0
        self.obstacle_array = obstacles
        self.tree_color = (1, 0, 0)
        self.ax = ax_anim
        # Add the first node
        self.node_list = [Node(pos=x_start, cost=0, parent_idx=-1)]
        self.safety_margin = margin
        if self.ax is not None:
            self.lines = self.ax.plot(x_start[0], x_start[1], x_start[2], "ro")
            for obstacle in obstacles:
                plot_three_dee_box(points=obstacle, ax=self.ax)

    def draw_growing_tree(self):
        self.lines.clear()
        self.ax.lines.clear()

        self.lines = self.ax.plot(self.node_list[0].pos[0],
                                  self.node_list[0].pos[1],
                                  self.node_list[0].pos[2], "ro")

        for node in self.node_list[1:]:
            nodeline = np.array([node.pos,
                                 self.node_list[node.parent_idx].pos])
            self.lines.append(self.ax.plot(nodeline[:, 0], nodeline[:, 1], nodeline[:, 2],
                                           color=self.tree_color,
                                           alpha=0.3, zorder=1500))
        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()

    # Method for adding new paths
    def find_path(self, x_goal, map_boundary):
        # draw the goal
        if self.ax is not None:
            self.ax.plot(x_goal[0], x_goal[1], x_goal[2], "ro")
        # Start iteration
        start_time = time.time()
        time_list = [start_time]
        inter_time_list = []
        count = 0
        for iteration in range(self.num_iter):
            # show the progress
            if (iteration + 1) % 100 == 0:
                time_list.append(time.time())
                inter_time = np.round(time_list[count + 1] - time_list[count], 2)
                inter_time_list.append(inter_time)
                print('Search iterations:', iteration + 1,
                      ", interval time: " + str(inter_time) + "s")
                count += 1

            # get a new sampled point and the index of the closest node in the list
            x_new, idx = self.new_and_closest(map_boundary)

            # check whether the new point is colliding or not
            if idx is None:
                continue
            else:
                # path collision checking, if there is a collision, skip the rest and go to the next iteration
                if collision_check_path(self.node_list[idx].pos, x_new, self.obstacle_array, self.safety_margin):
                    continue

            # Rewire the new node for optimal cost and get the close points
            neigh_list = self.rewire_new_node(x_new, idx)

            # Rewire the neighbouring points to the new sample, for existent nodes, pass its pos on the list rather
            # than their pos
            for j in neigh_list:
                self.rewire_node(j)

            # check arriving to the goal
            if (not self.pathFind) and norm(x_new - x_goal) < self.thr:
                self.pathFind = True
                # Add the final point to the node list
                self.node_list.append(Node(x_goal, self.node_list[-1].cost+norm(x_goal - x_new),
                                           len(self.node_list) - 1))
                self.goal_idx = len(self.node_list) - 1
                # break

            if self.ax is not None:
                self.draw_growing_tree()

        total_time = np.round(time.time() - start_time, 2)
        avg_time_per100 = np.round(np.mean(inter_time_list), 2)
        print("Total searching time: " + str(total_time) + 's, ' + "average interval time per 100 iters: "+ str(avg_time_per100) + 's')
        print("Total number of iterations:", iteration)

        return self.pathFind

    # Function for obtaining the path
    def get_path(self):
        # back search
        path_list = []
        if self.pathFind:
            # The index of the last element will be equal to the number of elements added
            path_idx = self.goal_idx
            # Iterate backwards appending the nodes and updating the index until the initial one
            while path_idx >= 0:
                path_list.append(self.node_list[path_idx].pos)
                path_idx = self.node_list[path_idx].parent_idx
        else:
            print('No path found')
        return np.flip(np.array(path_list), axis=0)

    # Function for obtaining a simplified path which avoid too many waypoints and zig-zags
    def get_straight_path(self):
        path = self.get_path()
        idx = []
        for i in range(np.size(path, axis=0)-2):
            if i not in idx:
                for j in range(i+2, np.size(path, axis=0)):
                    if not collision_check_path(path[i, :], path[j, :], self.obstacle_array, self.safety_margin):
                        idx.append(j-1)
                    else:
                        break
        path = np.delete(path, idx, axis=0)
        return path

    # Function for generating a new sample and the index of the closest neighbor
    def new_and_closest(self, map_boundary):
        # random sampling
        x_rand = np.random.uniform(0, 1, 3) * map_boundary

        # check for collision of the sample with an obstacle
        for obstacle in self.obstacle_array:
            if obstacle.collision_check(x_rand, self.safety_margin):
                return x_rand, None

        # find the nearest node
        nearest_dis = float('inf')
        nearest_node = None
        for i in range(len(self.node_list)):
            dis = norm(self.node_list[i].pos - x_rand)
            if dis < nearest_dis:
                nearest_node = i
                nearest_dis = dis

        return x_rand, nearest_node

    # Function for node rewiring, takes a bool to rewire depending on whether a new sample is given or an existing one
    def rewire_new_node(self, pos, closest_idx):
        neigh_list = []
        optimal_neigh = closest_idx
        lowest_cost = self.node_list[closest_idx].cost + norm(pos - self.node_list[closest_idx].pos)

        n = len(self.node_list)
        self.neigh_dist = 7*((np.log(n)/n)**(1/4))

        # Iterate over the list of nodes to find those within distance threshold and in collision-free connection
        for j in range(len(self.node_list)):
            if j == closest_idx:
                continue
            dist2xnew = norm(pos - self.node_list[j].pos)
            neighbor_cost = self.node_list[j].cost + dist2xnew
            if dist2xnew < self.neigh_dist:
                if not collision_check_path(self.node_list[j].pos, pos, self.obstacle_array, self.safety_margin):
                    neigh_list.append(j)
                    if neighbor_cost < lowest_cost:
                        lowest_cost = neighbor_cost
                        optimal_neigh = j

        # add x_new to the tree, the parent of x_new is at index optimal_node
        self.node_list.append(Node(pos, lowest_cost, optimal_neigh))

        return neigh_list

    # If the node is existent, check whether the connection with the new node [at index -1] minimizes the cost
    def rewire_node(self, node_idx):
        rewire_cost = self.node_list[-1].cost + norm(self.node_list[node_idx].pos - self.node_list[-1].pos)
        if rewire_cost < self.node_list[node_idx].cost:
            self.node_list[node_idx].cost = rewire_cost
            self.node_list[node_idx].parent_idx = len(self.node_list) - 1

    # Function for plotting the final tree of the algorithm
    def plotTree(self, ax, tree_color=(1, 0, 0), path_color=(0, 0, 1)):
        # note that argument "ax" is not self.ax. this argument is specifically for the static plot, and is therefore
        # not the attribute. the attribute is used to show the dynamic plot. Any axis object can be used to plot the
        # final tree.
        ax.lines.clear()
        ax.collections.clear()
        for obstacle in self.obstacle_array:
            plot_three_dee_box(points=obstacle, ax=ax)

        for node in range(1, len(self.node_list)):
            parent_idx = self.node_list[node].parent_idx

            xs = [self.node_list[parent_idx].pos[0], self.node_list[node].pos[0]]
            ys = [self.node_list[parent_idx].pos[1], self.node_list[node].pos[1]]
            zs = [self.node_list[parent_idx].pos[2], self.node_list[node].pos[2]]

            ax.plot(xs, ys, zs, color=tree_color, alpha=0.1)

        if self.pathFind:
            for node_idx in range(len(self.get_path())-1):

                xs = [self.get_path()[node_idx][0], self.get_path()[node_idx+1][0]]
                ys = [self.get_path()[node_idx][1], self.get_path()[node_idx+1][1]]
                zs = [self.get_path()[node_idx][2], self.get_path()[node_idx+1][2]]

                ax.plot(xs, ys, zs, color=path_color)


if __name__ == '__main__':
    # Define the obstacles, plotting figure and axis and other scenario properties
    scenario = 2
    num_iters = [1500, 2500, 4000]
    goal = 1
    for num_iter in num_iters:
        print("Number of iterations: " + str(num_iter))
        obstacles, fig, ax1, map_boundary, starts, ends = generate_env(scenario)

        if goal == 2:
            starts[0] = np.array([1, 2.5, 1])
            ends[0] = np.array([3, 0.5, 14])

        RRT_star_pathfinder = RRT_star(x_start=starts[0], num_iter=num_iter, obstacles=obstacles)
        path_exists = RRT_star_pathfinder.find_path(x_goal=ends[0], map_boundary=map_boundary)
        print("Is path found:", path_exists)

        RRT_star_pathfinder.plotTree(ax1)

        RRT_path = RRT_star_pathfinder.get_path()
        length = 0
        for i in range(1, np.size(RRT_path, 0)):
            length += np.linalg.norm(RRT_path[i, :] - RRT_path[i-1, :])
        print("length of complete path:"+str(length))
        RRT_simplified_path = RRT_star_pathfinder.get_straight_path()
        length = 0
        for i in range(1, np.size(RRT_simplified_path, 0)):
            length += np.linalg.norm(RRT_simplified_path[i, :] - RRT_simplified_path[i - 1, :])
        print("length of simplified path:" + str(length))
        save_file = False
        if save_file:
            np.savez('../full_results_and_more/front_end/RRT/RRT_points_scenario_'+str(scenario)+'_num_iter_'+str(num_iter)+
                    '_goal_'+str(goal), complete_path=RRT_path, simplified_path=RRT_simplified_path)

        plt.show(block=True)

