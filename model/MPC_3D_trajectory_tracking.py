"""
File containing the class definition of the Nonlinear Model Predictive Controller
"""
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from matplotlib import animation
import forcespro
from model.quadrotor import Quadrotor
import casadi


class MPC_traj:
    """
    MPC class, instantiated with the horizon limit (N) and an option to expect or not obstacles on the trajectory
    (obstacle). Can be called with the "control" function, to which the current state and goal must be passed in
    addition to the size of the collision-free bounding box in which the drone can move
    """
    def __init__(self, N, obstacle=False):
        """
        Parameters of the class
        """
        self.mass = 0.030  # kg
        self.Ixx = 1.43e-5  # kg*m^2
        self.Iyy = 1.43e-5  # kg*m^2
        self.Izz = 2.89e-5  # kg*m^2
        self.arm_length = 0.046  # meters
        self.rotor_speed_min = 0  # rad/s
        self.rotor_speed_max = 2700  # rad/s, satisfy the constraint of max thrust
        self.k_thrust = 2.3e-08  # N/(rad/s)**2
        self.k_drag = 7.8e-11  # Nm/(rad/s)**2

        # Additional constants.
        self.inertia_casadi = casadi.SX([[self.Ixx, 0, 0], [0, self.Iyy, 0], [0, 0, self.Izz]])
        self.g = 9.81  # m/s^2

        # Precomputes
        k = self.k_drag / self.k_thrust
        L = self.arm_length
        self.to_TM = np.array([[1, 1, 1, 1],
                               [0, L, 0, -L],
                               [-L, 0, L, 0],
                               [k, -k, k, -k]])
        self.inv_inertia_casadi = casadi.SX([[1 / self.Ixx, 0, 0],
                                             [0, 1 / self.Iyy, 0],
                                             [0, 0, 1 / self.Izz]])
        self.weight = np.array([0, 0, -self.mass * self.g])
        self.t_step = 0.01
        self.dt = 0.01
        """
        Parameters for the MPC controller
        """
        self.model = forcespro.nlp.SymbolicModel(N)  # create an empty model with time horizon of 50 steps
        # objective (cost function)
        # cost matrix for tracking the goal point
        self._Q_goal = np.diag([
            100, 100, 100,      # x, y, z
            20, 20, 20,         # dx, dy, dz
            10, 10, 10, 10,     # qx, qy, qz, qw
            10, 10, 10])        # r, p, q
        self._Q_goal_N = np.diag([
            200, 200, 200,      # x, y, z
            10, 10, 10,         # dx, dy, dz
            10, 10, 10, 10,     # qx, qy, qz, qw
            10, 10, 10])        # r, p, q

        # cost: distance to the goal
        self.model.objective = self.objective

        # equality constraints (quadrotor model)
        # z[0:4] action z[4:14] state
        self.model.eq = lambda z: forcespro.nlp.integrate(self.continuous_dynamics, z[4:], z[0:4],
                                                          integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
        self.model.E = np.concatenate([np.zeros((13, 4)), np.eye(13)], axis=1)  # inter-stage equality Ek@ zk+1=f(zk,pk)
        #  upper/lower variable bounds lb <= z <= ub
        # Lower bounds are parametric (indices not mentioned here are -inf)
        self.model.lbidx = [0, 4, 5, 6]

        # Upper bounds are parametric (indices not mentioned here are +inf)
        self.model.ubidx = [0, 4, 5, 6]

        # General (differentiable) nonlinear inequalities hl <= h(x,p) <= hu
        if obstacle:
            self.model.ineq = lambda z, p: np.array([(z[4] - p[6])**2 + (z[5] - p[7])**2 + (z[6] - p[8])**2])
            self.model.hu = np.array([np.inf])
            self.model.hl = np.array([0.49])

        # set dimensions of the problem
        self.model.nvar = 17    # number of variables
        self.model.neq = 13     # number of equality constraints
        if obstacle:
            self.model.nh = 1      # number of inequality constraints functions
            self.model.npar = 9  # number of runtime parameters (pos and vel and pos_obstacle)
        else:
            self.model.npar = 6  # number of runtime parameters (pos and vel and pos_obstacle)
        self.model.xinitidx = range(4, 17)  # indices of the state variables

        # Set solver options
        self.codeoptions = forcespro.CodeOptions('FORCENLPsolver')
        self.codeoptions.maxit = 200  # Maximum number of iterations
        self.codeoptions.printlevel = 0
        self.codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
        self.codeoptions.cleanup = False
        self.codeoptions.timing = 1
        self.codeoptions.nlp.hessian_approximation = 'bfgs'  # when using solvemethod = 'SQP_NLP' and LSobjective, try out 'gauss-newton' here (original: bfgs)
        self.codeoptions.nlp.bfgs_init = 2.5 * np.identity(8)  # initialization of the hessian approximation
        self.codeoptions.solvemethod = "SQP_NLP"
        self.codeoptions.sqp_nlp.maxqps = 1  # maximum number of quadratic problems to be solved
        self.codeoptions.sqp_nlp.reg_hessian = 5e-5  # increase this if exitflag=-8
        self.codeoptions.nlp.stack_parambounds = True
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        self.solver = self.model.generate_solver(self.codeoptions)

        # Set initial guess to start solver
        self.inital_guess = np.zeros([self.model.nvar, 1])
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}

    def control(self, state, goal, bounding_box_size=2):
        """
        Sovling NLP prolem in N-step-horizon for optimal control, take the first control input
        """
        # Set initial guess
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}
        # Set initial condition
        state = Quadrotor._pack_state(state)
        x_current = np.transpose(state)
        self.problem["xinit"] = x_current
        # Set runtime parameters
        self.problem["all_parameters"] = np.transpose(np.tile(goal, (1, self.model.N)))
        # Set runtime constraints
        self.bounding_box_size = bounding_box_size
        # only bound first 5 steps (since MPC only executes the first control command)
        lb_first = np.tile([0, state[0]-self.bounding_box_size, state[1]-self.bounding_box_size, state[2]-self.bounding_box_size], (5,))
        lb_second = np.tile([0, state[0]-100, state[1]-100, state[2]-100], (self.model.N - 5, ))
        self.problem["lb"] = np.hstack((lb_first, lb_second))
        ub_first = np.tile([2.5*self.mass*self.g, state[0]+self.bounding_box_size, state[1]+self.bounding_box_size, state[2]+self.bounding_box_size], (5, ))
        ub_second = np.tile([2.5*self.mass*self.g, state[0]+100, state[1]+100, state[2]+100], (self.model.N - 5, ))
        self.problem["ub"] = np.hstack((ub_first, ub_second))

        # Time to solve the NLP!
        output, exitflag, info = self.solver.solve(self.problem)
        # Make sure the solver has exited properly.
        print("exitflag: ", exitflag)
        self.inital_guess = output['x01'][:, np.newaxis]
        u = np.zeros(4)
        u[0] = output['x01'][0]
        u[1] = output['x01'][1]
        u[2] = output['x01'][2]
        u[3] = output['x01'][3]

        gama = self.k_drag / self.k_thrust
        cof_temp = np.array([1, 1, 1, 1, 0, self.arm_length, 0, -self.arm_length, -self.arm_length, 0, self.arm_length,
                             0, gama, -gama, gama, -gama]).reshape(4, 4)

        f_i = np.matmul(np.linalg.inv(cof_temp), u)

        cmd_motor_speeds = np.zeros(4)
        cmd_moment = np.zeros(3)

        for i in range(4):
            if f_i[i] < 0:
                f_i[i] = 0
                cmd_motor_speeds[i] = self.rotor_speed_max
            cmd_motor_speeds[i] = np.sqrt(f_i[i] / self.k_thrust)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max

        cmd_thrust = u[0]
        cmd_moment[0] = u[1]
        cmd_moment[1] = u[2]
        cmd_moment[2] = u[3]

        control_input = {'cmd_rotor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment}

        return control_input

    def continuous_dynamics(self, s, u):
        # rotate_k function (third column of the rotation matrix configured by q)
        rotate_k = casadi.vertcat(2 * (s[6] * s[8] + s[7] * s[9]), 2 * (s[7] * s[8] - s[6] * s[9]), 1 - 2 * (s[6] ** 2 + s[7] ** 2))

        # quat_dot function
        (q0, q1, q2, q3) = (s[6], s[7], s[8], s[9])
        col1 = casadi.vertcat(q3, -q2, q1)
        col2 = casadi.vertcat(q2, q3, -q0)
        col3 = casadi.vertcat(-q1, q0, q3)
        col4 = casadi.vertcat(-q0, -q1, -q2)
        G = casadi.horzcat(col1, col2, col3, col4)
        # G = np.array([[q3, q2, -q1, -q0],
        #               [-q2, q3, q0, -q1],
        #               [q1, -q0, q3, -q2]])
        omega = casadi.vertcat(s[10], s[11], s[12])  # angular velocity
        quaternion = casadi.vertcat(s[6], s[7], s[8], s[9])
        quat_dot = 0.5 * G.T @ omega
        # Augment to maintain unit quaternion.
        quat_err = casadi.sum1(casadi.sum2(quaternion ** 2)) - 1
        quat_err_grad = 2 * quaternion
        quat_dot = quat_dot - quat_err * quat_err_grad

        # Angular velocity derivative
        moment = casadi.vertcat(u[1], u[2], u[3])
        w_dot = self.inv_inertia_casadi @ (moment - casadi.cross(omega, self.inertia_casadi @ omega, 1))

        return casadi.vertcat(s[3], s[4], s[5], (self.weight + u[0] * rotate_k) / self.mass, quat_dot, w_dot)

    def objective(self, z, goal):
        self.goal = np.array([goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], 0, 0, 0, 1, 0, 0, 0])
        return (z[4:] - self.goal).T @ self._Q_goal @ (z[4:]-self.goal) + 0.1 * (z[0]**2 + z[1]**2 + z[2]**2 + z[3]**2)
