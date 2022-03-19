"""
File containing the class definition of the Nonlinear Model Predictive Controller
"""
# =============================================================================
# # Adding to my path
# import sys
# sys.path.insert(0, 'C:\\Users\\Vassil\\Desktop\\Personal\\TU Delft\\Msc Robotics\\forces_pro_client')
# sys.path.insert(0, 'C:\\Users\\Vassil\\Desktop\\Personal\\TU Delft\\Msc Robotics\\Model Predictive Control\\MPC_final_project')
# =============================================================================
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from model.unicycle import Robot
import forcespro
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
        self.dt = 2e-2
        self.N = N # planning horizon
        """
        Parameters for the MPC controller
        """
        self.model = forcespro.nlp.SymbolicModel(N)  # create an empty model with time horizon of N steps

        self._Q_goal = np.diag([100, 100, 0])      # x, y, theta
        self._Q_goal_N = np.diag([500, 500, 100])      # x, y, theta

        # cost: distance to the goal
        self.model.objective = self.objective
        self.model.objectiveN = self.objectiveN

        # equality constraints (robot model)
        # z[0:2] action z[2:5] state
        self.model.eq = lambda z: forcespro.nlp.integrate(self.continuous_dynamics, z[2:], z[0:2],
                                                          integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
        self.model.E = np.concatenate([np.zeros((3, 2)), np.eye(3)], axis=1)  # inter-stage equality Ek@ zk+1=f(zk,pk)

        # inequality constraints for collision avoidance
        self.model.ineq = lambda z: np.array([(z[2] - 5) ** 2 + (z[3] - 0) ** 2]) # squared distance
        self.model.hu = np.array([np.inf])
        self.model.hl = np.array([2.5])
        self.model.nh = 1 # # number of inequality constraints functions (collision avoidance)
        # terminal constraint
        # self.model.ineqN = lambda z: np.array([(z[2] - 5) ** 2 + (z[3] - 0) ** 2, (z[2] - 10) ** 2 + (z[3] - 0) ** 2] ) # squared distance
        # self.model.huN = np.array([np.inf, np.inf])
        # self.model.hlN = np.array([4, -np.inf])
        # self.model.nhN = 2 # # number of inequality constraints functions at the last stage

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
        self.problem["reinitialize"] = False # reuse the previous solution at every iteration

    def control(self, state, goal):
        """
        Sovling NLP prolem in N-step-horizon for optimal control, take the first control input
        """
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

# Starting point of the code
def main():
    def animate(i):
        line.set_xdata(real_trajectory['x'][:i + 1])
        line.set_ydata(real_trajectory['y'][:i + 1])
        line.set_3d_properties(real_trajectory['z'][:i + 1])
        point.set_xdata(real_trajectory['x'][i])
        point.set_ydata(real_trajectory['y'][i])
        point.set_3d_properties(real_trajectory['z'][i])

        heading.set_xdata(
            [real_trajectory['x'][i], real_trajectory['x'][i] + 0.8 * np.cos(real_trajectory['theta'][i])])
        heading.set_ydata(
            [real_trajectory['y'][i], real_trajectory['y'][i] + 0.8 * np.sin(real_trajectory['theta'][i])])
        return ax1

    env = Robot(-2, 0, 0)
    mpc = MPC(40)
    real_trajectory = {'x': [], 'y': [], 'z': [], 'theta': []}
    for iter in range(5000):
        # state = env.step(0.5, 0.)
        v, w = mpc.control(env.current, np.array([10., 0., 0.]))
        state = env.step(v, w)
        print(env.current)
        real_trajectory['x'].append(state.x)
        real_trajectory['y'].append(state.y)
        real_trajectory['z'].append(0)
        real_trajectory['theta'].append(state.theta)

    # plotting stuff
    fig = plt.figure()
    ax1 = p3.Axes3D(fig)  # 3D place for drawing
    real_trajectory['x'] = np.array(real_trajectory['x'])
    real_trajectory['y'] = np.array(real_trajectory['y'])
    real_trajectory['z'] = np.array(real_trajectory['z'])
    point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'ro',
                      label='Robot', markersize=15)

    heading, = ax1.plot([real_trajectory['x'][0], real_trajectory['x'][0] + 0.8 * np.cos(real_trajectory['theta'][0])], \
                        [real_trajectory['y'][0], real_trajectory['y'][0] + 0.8 * np.sin(real_trajectory['theta'][0])],
                        [real_trajectory['z'][0]], 'b')

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
                                  interval=50,
                                  repeat=False,
                                  blit=False)
    plt.show()


if __name__ == "__main__":
    main()