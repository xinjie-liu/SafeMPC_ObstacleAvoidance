"""
File containing the source code for multiple functions required for the minimum snap optimization algorithm in its
different versions (with and without time optimization, mainly)
"""
import numpy as np
import math


########################################################################################################################
# FUNCTIONS FOR MINIMUM SNAP Calculation (BOTH FOR OPTIMAL TIME ALLOCATION AND NORMAL)
########################################################################################################################
def get_q(n_seg, n_order, ts):
    """
    Function for obtaining the cost matrix of the objective function of minimum snap
    :param n_seg: number of segments in the path
    :param n_order: order of the polynomials connecting consecutive points
    :param ts: segment times
    :return: numpy array with the cost matrix with size depending on n_seg and n_order
    """
    q = np.zeros((n_seg * (n_order + 1), n_seg * (n_order + 1)))
    for k in range(n_seg):
        q_k = np.zeros((n_order + 1, n_order + 1))

        # Get the Q matrix for each segment of the trajectory
        for i in range(4, n_order + 1):
            for l in range(4, n_order + 1):
                q_k[i, l] = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)*pow(ts[k], (i+l-7)) / (i+l-7)
        # Assign it to the corresponding pos of the general Q matrix
        q[(8 * k):(8 * k + 8), (8 * k):(8 * k + 8)] = q_k
    return q


def obj_function(variables, n_seg, n_order, penalty, time_optimal, ts):
    """
    Function to be called by the optimizer as objective function to be minimized. Depending on the provided parameters
    (including whether time is to be optimized or not), the objective function is modified accordingly
    :param variables: array with the optimization variables, passed by the optimizer
    :param n_seg: number of segments in the path
    :param n_order: order of the polynomials connecting consecutive points
    :param penalty: boolean determining whether segment time optimization must be carried out or not
    :param time_optimal: the time_optimal boolean, required by the objective function and equality constraints of the
    solver
    :param ts: allocated time for each segment of the path, will be overwritten with the variables if time_optimal is
    set to True
    :return: the current cost for the values of the variables and cost matrix
    """
    # Un-pack the variables
    if time_optimal:
        ts = variables[:n_seg]
        xs = variables[n_seg: n_seg * (n_order + 1) + n_seg]
        ys = variables[n_seg * (n_order + 1) + n_seg: 2 * n_seg * (n_order + 1) + n_seg]
        zs = variables[2 * n_seg * (n_order + 1) + n_seg: 3 * n_seg * (n_order + 1) + n_seg]
    else:
        xs = variables[:n_seg * (n_order + 1)]
        ys = variables[n_seg * (n_order + 1): 2 * n_seg * (n_order + 1)]
        zs = variables[2 * n_seg * (n_order + 1): 3 * n_seg * (n_order + 1)]

    # Get the cost function matrix for the coefficients
    qs = get_q(n_seg, n_order, ts)

    # Depending on the value of time_optimal, include the penalty on the cost or not
    if time_optimal:
        obj = xs @ qs @ xs.reshape(-1, 1) + ys @ qs @ ys.reshape(-1, 1) + zs @ qs @ zs.reshape(-1, 1) \
            + penalty * np.sum(ts ** 2)
    else:
        obj = xs @ qs @ xs.reshape(-1, 1) + ys @ qs @ ys.reshape(-1, 1) + zs @ qs @ zs.reshape(-1, 1)
    return obj


def get_ab(n_seg, n_order, path, ts):
    """
    Function for computing the constraint matrices A and B for the equality constraints of the optimization problem
    :param n_seg: number of segments in the path
    :param n_order: order of the polynomials connecting consecutive points
    :param path: set of 3D points to which the optimization must be applied
    :param ts: allocated time for each segment of the path
    :return: numpy arrays with the A and B matrices for the equality constraints of the optimization.
    """
    n_all_poly = n_seg * (n_order + 1)

    # set initial and final point constraints (position equal to initial and final points, derivatives equal to zero)
    aeq_start = np.zeros((4, n_all_poly))
    aeq_start[:4, :4] = np.diag([1, 1, 2, 6])
    aeq_end = np.zeros((4, n_all_poly))
    for k in range(4):
        for i in range(k, n_order + 1):
            aeq_end[k, -(n_order + 1 - i)] = (math.factorial(i) * pow(ts[-1], (i - k))) / math.factorial(i - k)
    beq_start = np.array([path[0], 0, 0, 0])
    beq_end = np.array([path[-1], 0, 0, 0])

    # position constraints for waypoints
    aeq_wp = np.zeros((n_seg - 1, n_all_poly))
    for j in range(n_seg - 1):
        aeq_wp[j, 8 * (j + 1)] = 1
    beq_wp = path[1:-1]

    # pos continuity
    aeq_contp = get_aeq_cont(n_seg, n_order, ts, k=0)

    # velocity continuity
    aeq_contv = get_aeq_cont(n_seg, n_order, ts, k=1)

    # acceleration continuity
    aeq_conta = get_aeq_cont(n_seg, n_order, ts, k=2)

    # jerk continuity
    aeq_contj = get_aeq_cont(n_seg, n_order, ts, k=3)
    aeq_conts = get_aeq_cont(n_seg, n_order, ts, k=4)
    aeq_contc = get_aeq_cont(n_seg, n_order, ts, k=5)
    aeq_contpop = get_aeq_cont(n_seg, n_order, ts, k=6)

    # Combine the matrices in a single matrix
    aeq_cont = np.vstack((aeq_contp, aeq_contv, aeq_conta, aeq_contj, aeq_conts, aeq_contc, aeq_contpop))
    beq_cont = np.zeros(7 * (n_seg - 1), )
    aeq = np.vstack((aeq_start, aeq_end, aeq_wp, aeq_cont))
    beq = np.concatenate((beq_start, beq_end, beq_wp, beq_cont))
    return aeq, beq


# Helper function for obtaining the continuity constraint of given order (k)
def get_aeq_cont(n_seg, n_order, ts, k):
    """
    Helper function for the equality matrix for obtaining the portions of the matrix corresponding to the continuity
    constraints of order k
    :param n_seg: number of segments in the path
    :param n_order: order of the polynomials connecting consecutive points
    :param ts: allocated time for each segment of the path
    :param k: order of the polynomial which has to satisfy the continuity constraint
    :return: numpy array with the segment of the equality constriant matrix A corresponding to the continuity of the
    k-th order of the polynomial
    """
    aeq_cont = np.zeros((n_seg - 1, n_seg * (n_order + 1)))
    for j in range(n_seg - 1):
        for i in range(k, n_order + 1):
            aeq_cont[j, (n_order + 1) * j + i] = math.factorial(i) * pow(ts[j], i - k) / math.factorial(i - k)
        aeq_cont[j, (n_order + 1) * (j + 1) + k] = -math.factorial(k)
    return aeq_cont


def equal_constraint(variables, n_seg, n_order, path, time_optimal, ts):
    """
    Equality constraint function for the solver, following the same principles as obj_function
    :param variables: array with the optimization variables, passed by the optimizer
    :param n_seg: number of segments in the path
    :param n_order: order of the polynomials connecting consecutive points
    :param path: set of 3D points to which the optimization must be applied
    :param time_optimal: the time_optimal boolean, required by the objective function and equality constraints of the
    solver
    :param ts: allocated time for each segment of the path, will be overwritten with the variables if time_optimal is
    set to True
    :return: a vector with the resulting value of the equality constraint applied to the current value of the
    optimization variables.
    """
    # Unpack the different variables
    if time_optimal:
        ts = variables[:n_seg]
        xs = variables[n_seg: n_seg * (n_order + 1) + n_seg]
        ys = variables[n_seg * (n_order + 1) + n_seg: 2 * n_seg * (n_order + 1) + n_seg]
        zs = variables[2 * n_seg * (n_order + 1) + n_seg: 3 * n_seg * (n_order + 1) + n_seg]
    else:
        xs = variables[:n_seg * (n_order + 1)]
        ys = variables[n_seg * (n_order + 1): 2 * n_seg * (n_order + 1)]
        zs = variables[2 * n_seg * (n_order + 1): 3 * n_seg * (n_order + 1)]

    # Obtain the different constraint matrices
    aeq_x, beq_x = get_ab(n_seg, n_order, path[:, 0], ts)
    aeq_y, beq_y = get_ab(n_seg, n_order, path[:, 1], ts)
    aeq_z, beq_z = get_ab(n_seg, n_order, path[:, 2], ts)

    # Compute the equality value
    constraint = np.hstack((aeq_x @ xs - beq_x,
                            aeq_y @ ys - beq_y,
                            aeq_z @ zs - beq_z))
    return constraint


def get_point_values(coef_x, coef_y, coef_z, ts, n_seg, n_order, order_der=0, tstep=0.01):
    """
    Function for obtaining the values of the position and its derivatives given the coefficients of the polynomials
    obtained in the minimum snap optimization.
    :param coef_x: coefficients of the polynomials in the X direction of order order_der
    :param coef_y: coefficients of the polynomials in the Y direction of order order_der
    :param coef_z: coefficients of the polynomials in the > direction of order order_der
    :param ts: allocated time for each segment of the path
    :param n_seg: number of segments in the path
    :param n_order: order of the polynomials connecting consecutive points
    :param order_der: order of the derivative of the polynomial whose value will be calculated
    :param tstep: time step at which the values will be computed
    :return: numpy array containing the values at successive tsteps for the given coefficients corresponding to the
    order "order_der" of the position polynomials
    """
    values = []
    num_terms = n_order + 1 - order_der
    for i in range(n_seg):
        xi = coef_x[(num_terms*i):(num_terms*(i + 1))].tolist()
        yi = coef_y[(num_terms*i):(num_terms*(i + 1))].tolist()
        zi = coef_z[(num_terms*i):(num_terms*(i + 1))].tolist()
        for t in np.arange(0, ts[i], tstep):
            values.append(np.polyval(xi[::-1], t))
            values.append(np.polyval(yi[::-1], t))
            values.append(np.polyval(zi[::-1], t))

    return np.array(values).reshape((-1, 3))


def get_derivative_coef(coef_x, coef_y, coef_z, n_order, order_der):
    """
    Function for computing the polynomial coefficients of given order, given the coefficients of the polynomials of
    order "order_der - 1"
    :param coef_x: coefficients of the polynomials in the X direction of order order_der - 1
    :param coef_y: coefficients of the polynomials in the Y direction of order order_der - 1
    :param coef_z: coefficients of the polynomials in the > direction of order order_der - 1
    :param n_order: order of the polynomials connecting consecutive points
    :param order_der: order of the derivative of the position whose polynomials will be calculated
    :return: numpy arrays containing the coefficients of order "order_der" of the position for the three coordinates
    """
    num_terms = n_order + 2 - order_der
    der_coef_x = (coef_x.reshape((-1, num_terms))[:, 1:] * np.arange(start=1, stop=num_terms).reshape(1, -1)).reshape((-1,))
    der_coef_y = (coef_y.reshape((-1, num_terms))[:, 1:] * np.arange(start=1, stop=num_terms).reshape(1, -1)).reshape((-1,))
    der_coef_z = (coef_z.reshape((-1, num_terms))[:, 1:] * np.arange(start=1, stop=num_terms).reshape(1, -1)).reshape((-1,))
    return der_coef_x, der_coef_y, der_coef_z


########################################################################################################################
# FUNCTIONS SPECIFIC FOR OPTIMAL TIME ALLOCATION
########################################################################################################################
def bound(n_seg, n_var):
    """
    Function for getting the boundary values of the variables of the optimization for time-optimized minimum snap
    :param n_seg: number of segments of the path
    :param n_var: number of variables that will be used in the optimization process
    :return: tuple containing the pair of boundary values for each variable. In principle, only the time is bounded.
    """
    # set the boundary of the time distribution
    bound_tuple = ()
    for i in range(n_var):
        if i < n_seg:
            bound_tuple += ((0.01, 5),)
        else:
            bound_tuple += ((-np.inf, np.inf),)
    return bound_tuple


########################################################################################################################
# FUNCTIONS SPECIFIC FOR PROPORTIONAL TIME ALLOCATION
########################################################################################################################
def compute_proportional_t(path, total_time, n_seg):
    """
    Function for computing the time allocated to each segment of the path, as a fraction of a given total_time,
    proportional to the length of the segment out of the
    :param path: set of 3D points to which the optimization must be applied
    :param total_time: total time required to complete the path
    :param n_seg: number of segments which compose the path
    :return: numpy array containing the time allocated to each segment of the path
    """
    ts = np.zeros((n_seg,))
    dist = np.zeros((n_seg,))
    dist_sum, t_sum = 0, 0

    # Calculate the distance of each segment and the total distance of the path
    for i in range(n_seg):
        dist[i] = np.linalg.norm(path[i+1, :] - path[i, :])
        dist_sum += dist[i]

    # Assign the time for each segment based on its distance
    for i in range(n_seg-1):
        ts[i] = total_time*dist[i]/dist_sum
        t_sum += ts[i]
    ts[-1] = total_time - t_sum
    return ts


########################################################################################################################
# FUNCTIONS FOR ACTUATION CONSTRAINT
########################################################################################################################
def inequal_constraint(variables, n_seg, n_order):
    """
    Following the same principle as equal_constraints, this function is used for evaluating the inequality constraints
    of the optimizer.
    :param variables: array with the optimization variables, passed by the optimizer
    :param n_seg: number of segments in the path
    :param n_order: order of the polynomials connecting consecutive points
    :return: vector containing the values of applying the inequality (actuator) constraints to the current values
    of the optimization variables
    """
    # Extract the times and coefficients of the polynomials
    ts = variables[:n_seg]
    pos_coef_x = variables[n_seg:n_seg * (n_order + 1) + n_seg]
    pos_coef_y = variables[n_seg * (n_order + 1) + n_seg: 2 * n_seg * (n_order + 1) + n_seg]
    pos_coef_z = variables[2 * n_seg * (n_order + 1) + n_seg: 3 * n_seg * (n_order + 1) + n_seg]

    # Compute the derivatives of the polynomials
    vel_coef_x, vel_coef_y, vel_coef_z = get_derivative_coef(pos_coef_x, pos_coef_y, pos_coef_z, n_order, 1)
    acc_coef_y, acc_coef_x, acc_coef_z = get_derivative_coef(vel_coef_x, vel_coef_y, vel_coef_z, n_order, 2)
    jerk_coef_y, jerk_coef_x, jerk_coef_z = get_derivative_coef(acc_coef_x, acc_coef_y, acc_coef_z, n_order, 3)
    snap_coef_y, snap_coef_x, snap_coef_z = get_derivative_coef(jerk_coef_x, jerk_coef_y, jerk_coef_z, n_order, 4)

    # For a bigger time step than the one used for the final computation, check for collisions with obstacles
    tstep = 0.1
    acc = get_point_values(acc_coef_x, acc_coef_y, acc_coef_z, ts, n_seg, n_order, 2, tstep)
    jerk = get_point_values(jerk_coef_x, jerk_coef_y, jerk_coef_z, ts, n_seg, n_order, 3, tstep)
    snap = get_point_values(snap_coef_x, snap_coef_y, snap_coef_z, ts, n_seg, n_order, 4, tstep)

    # Obtain the maximum actuation for the computed accelerations, jerks and snaps and compute the difference with the
    # maximum one.
    _, max_speed = get_max_actuation(acc, jerk, snap)
    diff = np.full((4,), 2500) - max_speed
    return diff


def get_max_actuation(acc, jerk, snap):
    """
    For the given set of accelerations, jerks and snaps, compute the rotor actuation required at each step
    :param acc: numpy array containing the accelerations at fixed time steps
    :param jerk: numpy array containing the jerks at fixed time steps
    :param snap: numpy array containing the snaps at fixed time steps
    :return: location of the maximum rotor speed and the maximum speeds
    """
    max_idx, max_val, max_speed = 1e9, 0, np.zeros((4,))
    for i in range(len(acc)):
        rotor_speeds = get_input_from_ref(acc[i, :], jerk[i, :], snap[i, :])
        if np.max(rotor_speeds) > max_val:
            max_idx = i
            max_val = np.max(rotor_speeds)
            max_speed = rotor_speeds
    return max_idx, max_speed


def get_input_from_ref(acc, jerk, snap):
    """
    Function for computing the required actuation speeds of the quadrotor given the acceleration, jerk and snap at a
    given instant of time.
    :param acc: array containing the acceleration in the three coordinates for a given instant
    :param jerk: array containing the jerk in the three coordinates for a given instant
    :param snap: array containing the snap in the three coordinates for a given instant
    :return: numpy array containing the velocities of the four rotors of the quadrotor
    """
    m, g = 0.03, 9.81
    # Total thrust obtained from accelerations
    total_thrust = m*np.sqrt(acc[0]**2 + acc[1]**2 + (acc[2] - g)**2)

    # Pitch and roll angles considering zero yaw
    pitch = math.atan(acc[0]/(acc[2] - g))
    roll = math.asin(m*acc[1]/total_thrust)

    # Get the body frame axes
    xc = np.array([1, 0, 0])
    yc = np.array([0, 1, 0])
    zc = np.array([0, 0, 1])

    alpha = acc - g*zc
    xb = np.cross(yc, alpha)/np.linalg.norm(np.cross(yc, alpha))
    yb = np.cross(alpha, xb)/np.linalg.norm(np.cross(alpha, xb))
    zb = np.cross(xb, yb)

    # Compute angular velocities
    c = np.dot(zb, alpha)
    wb = np.zeros((3,))
    wb[0] = - np.dot(yb, jerk)/c
    wb[1] = np.dot(xb, jerk)/c
    wb[2] = wb[1]*np.dot(yc, zb)/np.linalg.norm(np.cross(yc, zb))

    # Compute angular accelerations
    c_dot = np.dot(zb, jerk)
    wb_dot = np.zeros((3,))
    wb_dot[0] = -(np.dot(yb, snap) - 2*c_dot*wb[0] + c*wb[1]*wb[2])/c
    wb_dot[1] = (np.dot(xb, snap) - 2*c_dot*wb[1] + c*wb[0]*wb[2])/c
    wb_dot[2] = (-wb[0]*wb[1]*np.dot(yc, yb) - wb[0]*wb[2]*np.dot(yc, zb) + wb_dot[1]*np.dot(yc, zb)) / \
                np.linalg.norm(np.cross(yc, zb))

    # Compute the moments
    inertia_mat = np.diag(np.array([1.43e-5, 1.43e-5, 2.89e-5]))
    m = np.dot(inertia_mat, wb_dot) + np.cross(wb, np.dot(inertia_mat, wb))

    # Convert to rotor speeds
    k_drag, k_thrust = 7.8e-11, 2.3e-08
    k = k_drag/k_thrust
    to_inputs = np.array([[1, 1, 1, 1],
                             [0, 0.046, 0, -0.046],
                             [-0.046, 0, 0.046, 0],
                             [k, -k, k, -k]])
    rotor_forces = np.dot(np.linalg.inv(to_inputs), np.array([total_thrust, m[0], m[1], m[2]]))
    rotor_speeds = np.sqrt(rotor_forces/k_thrust)
    return rotor_speeds


########################################################################################################################
# FUNCTIONS FOR COLLISION AVOIDANCE GUARANTEE
########################################################################################################################
def find_collisions(obstacles, pos):
    """
    Given a set of obstacles in workspace and configuration space, as well as the set of position through which the
    drone will pass, determines whether collisions are produced and where.
    :param obstacles: set of obstacles in the workspace and configuration space
    :param pos: numpy array with the coordinates in 3D space that will be visited by the quadrotor
    :return: locations in the pos array at which collisions are produced
    """
    idx = []
    # Iterate over the different positions and check for each of them whether it collides with any of the obstacles
    for i in range(np.size(pos, axis=0)):
        for obstacle in obstacles:
            if obstacle.collision_check(pos[i, :]):
                idx.append(i)
                break
    return idx


def extend_path(path, idx, ts, tstep):
    """
    Function for extending the path given the indeces of the positions at which collisions occur
    :param path: set of 3D points generated by the RRT algorithm which  will be used for extending the path
    :param idx: locations in the position array (positions generated at fixed time steps tstep) where collisions have
    been detected
    :param ts: numpy array containing the times allocated for each segment of the path
    :param tstep: time step which has been used for generating the positions out of the polynomials obtained in the
    optimization
    :return: new version of the path with additional waypoints in the middle of the path segments at which collisions
    have occurred
    """
    positions = []
    # Iterate over the index of the colliding positions
    for i in idx:
        # Extract, using the time step, the time at which the collision occurs
        idx_t = i*tstep
        # Find the segment of the path at which the collision occurs, and add it to a list
        for position, time in enumerate(ts):
            if idx_t > time:
                idx_t -= time
            else:
                if position+1 not in positions:
                    positions.append(position + 1)
                break
    # Expand the path attending to the segments with collisions by adding an extra waypoint
    for i, position in enumerate(positions):
        value = (path[position + i - 1, :] + path[position + i, :])/2
        path = np.insert(path, position+i, value, axis=0)
    return path
