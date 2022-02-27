"""
File containing the class definition of the Nonlinear Model Predictive Controller
"""
# Adding to my path
# =============================================================================
# import sys
# sys.path.insert(0, 'C:\\Users\\Vassil\\Desktop\\Personal\\TU Delft\\Msc Robotics\\forces_pro_client')
# =============================================================================
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from matplotlib import animation
import forcespro
from model.quadrotor import Quadrotor
import casadi

class MPC:
    """
    MPC class, instantiated with the horizon limit (N) and an option to expect or not obstacles on the trajectory
    (obstacle). Can be called with the "control" function, to which the current state and goal must be passed in
    addition to the size of the collision-free bounding box in which the drone can move
    """
    def __init__(self, N):
        """
        Parameters of the class
        """
        self.dt = 5e-3
        self.N = N # planning horizon
        """
        Parameters for the MPC controller
        """
        self.model = forcespro.nlp.SymbolicModel(N)  # create an empty model with time horizon of N steps

        self._Q_goal = np.diag([100, 100, 10])      # x, y, theta
        self._Q_goal_N = np.diag([500, 500, 100])      # x, y, theta

        # cost: distance to the goal
        self.model.objective = self.objective
        self.model.objectiveN = self.objectiveN

        # equality constraints (robot model)
        # z[0:2] action z[2:5] state
        self.model.eq = lambda z: forcespro.nlp.integrate(self.continuous_dynamics, z[2:], z[0:2],
                                                          integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
        self.model.E = np.concatenate([np.zeros((3, 2)), np.eye(3)], axis=1)  # inter-stage equality Ek@ zk+1=f(zk,pk)

        # set dimensions of the problem
        self.model.nvar = 5    # number of variables
        self.model.neq = 3     # number of equality constraints
        self.model.npar = 3    # x, y, theta
        self.model.xinitidx = range(2, 5)  # indices of the state variables

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
        self.codeoptions.sqp_nlp.reg_hessian = 5e-8  # increase this if exitflag=-8
        #self.codeoptions.nlp.stack_parambounds = True
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        self.solver = self.model.generate_solver(self.codeoptions)

        # Set initial guess to start solver
        self.inital_guess = np.zeros([self.model.nvar, 1])
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}

    def control(self, state, goal):
        """
        Sovling NLP prolem in N-step-horizon for optimal control, take the first control input
        """
        # Set initial guess
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}
        # Set initial condition
        state = np.array([state.x, state.y, state.theta])
        x_current = np.transpose(state)
        self.problem["xinit"] = x_current
        # Set runtime parameters
        self.problem["all_parameters"] = np.transpose(np.tile(goal, (1, self.model.N)))

        # Time to solve the NLP!
        output, exitflag, info = self.solver.solve(self.problem)
        # Make sure the solver has exited properly.
        print("exitflag: ", exitflag)
        self.inital_guess = output['x02'][:, np.newaxis]
        print(output['x01'])

        v = output['x01'][0]
        w = output['x01'][1]

        return v, w

    def fixAngle(self, angle):
        return casadi.atan2(casadi.sin(angle), casadi.cos(angle))

    # this is the discrete-time model for omega = 0 (straight line)
    def continuous_dynamics(self, s, u):
         v = u[0]
         w = u[1]
         x_dt = v * casadi.cos(s[2])
         y_dt = v * casadi.sin(s[2])
         theta_dt = w
    
         return casadi.vertcat(x_dt, y_dt, theta_dt)

# =============================================================================
#     # this is the discrete-time model for omega /= 0
#     def continuous_dynamics(self, s, u):
#         v = u[0]
#         w = u[1]
#         theta_dt = w
#         new_theta = self.fixAngle(s[2] + self.fixAngle(theta_dt * self.dt))
#         new_x = s[0] + (v/(w+1e-20)) * (casadi.sin(new_theta) - casadi.sin(s[2]))
#         new_y = s[1] + (v/(w+1e-20)) * (casadi.cos(s[2]) - casadi.cos(new_theta))
# 
#         return casadi.vertcat(new_x, new_y, new_theta)
# =============================================================================

    def objective(self, z, goal):
        self.goal = casadi.vertcat(goal[0], goal[1], goal[2])
        return (z[2:] - self.goal).T @ self._Q_goal @ (z[2:]-self.goal) + 0.1 * z[0]**2 + 0.1 * z[1]**2

    def objectiveN(self, z, goal):
        self.goal = casadi.vertcat(goal[0], goal[1], goal[2])
        return (z[2:] - self.goal).T @ self._Q_goal_N @ (z[2:]-self.goal) + 0.1 * z[0]**2 + 0.1 * z[1]**2

#######################################################################################################################
# You can check the results of a single optimization step here, be sure to comment the main function below
#######################################################################################################################
# =============================================================================
# mpc = MPC(20)
# 
# # # Set initial guess to start solver from (here, middle of upper and lower bound)
# x0i = np.zeros(5)
# x0 = np.transpose(np.tile(x0i, (1, mpc.model.N)))
# init_state = np.zeros(3)
# problem = {"x0": x0, "xinit": np.transpose(init_state)}
# # Set runtime parameters
# goal = np.array([0.5, 0.5, 0])
# problem["all_parameters"] = np.transpose(np.tile(goal, (1, mpc.N)))
# 
# # Time to solve the NLP!
# output, exitflag, info = mpc.solver.solve(problem)
# 
# # Make sure the solver has exited properly.
# print("exitflag: ", exitflag)
# print("FORCES took {} iterations and {} seconds to solve the problem.".format(info.it, info.solvetime))
# 
# print(output)
# print(output['x01'].shape)
# =============================================================================

