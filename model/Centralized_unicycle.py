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
from unicycle import Robot
import forcespro
from model.quadrotor import Quadrotor
import casadi
import mpl_toolkits.mplot3d.axes3d as p3

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
        self.N = N  # planning horizon
        """
        Parameters for the MPC controller
        """
        self.model = forcespro.nlp.SymbolicModel(N)  # create an empty model with time horizon of N steps

        self._Q_goal = np.diag([100, 100, 10, 100, 100, 10])  # x, y, theta
        self._Q_goal_N = np.diag([500, 500, 100, 500, 500, 100])  # x, y, theta

        # cost: distance to the goal
        self.model.objective = self.objective
        self.model.objectiveN = self.objectiveN

        # equality constraints (robot model)
        # z[0:2] action z[2:5] state
        self.model.eq = lambda z: forcespro.nlp.integrate(self.continuous_dynamics, z[4:], z[0:4],
                                                          integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
        self.model.E = np.concatenate([np.zeros((6, 4)), np.eye(6)], axis=1)  # inter-stage equality Ek@ zk+1=f(zk,pk)

        # inequality constraints for collision avoidance between agents
        # self.model.ineq = lambda z: np.array([(z[4] - z[7]) ** 2 + (z[5] - z[8]) ** 2]) # squared distance between robots (each robot has radius of 0.2m)
        # self.model.hu = np.array([np.inf])
        # self.model.hl = np.array([0.16])
        # self.model.nh = 1 # # number of inequality constraints functions (collision avoidance)

        # set dimensions of the problem
        self.model.nvar = 10  # number of variables
        self.model.neq = 6  # number of equality constraints
        self.model.npar = 6  # x, y, theta
        self.model.xinitidx = range(4, 10)  # indices of the state variables

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
        # self.codeoptions.nlp.stack_parambounds = True
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        self.solver = self.model.generate_solver(self.codeoptions)

        # Set initial guess to start solver
        self.inital_guess = np.zeros([self.model.nvar, 1])
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}

    def control(self, state1, state2, goal):
        """
        Sovling NLP prolem in N-step-horizon for optimal control, take the first control input
        """
        # Set initial guess
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}
        # Set initial condition
        state = np.array([state1.x, state1.y, state1.theta, state2.x, state2.y, state2.theta])
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

        v1 = output['x01'][0]
        w1 = output['x01'][1]
        v2 = output['x01'][2]
        w2 = output['x01'][3]

        return v1, w1, v2, w2

    def continuous_dynamics(self, s, u):

        v1 = u[0]
        w1 = u[1]
        x_dt1 = v1 * casadi.cos(s[2])
        y_dt1 = v1 * casadi.sin(s[2])
        theta_dt1 = w1

        v2 = u[2]
        w2 = u[3]
        x_dt2 = v2 * casadi.cos(s[5])
        y_dt2 = v2 * casadi.sin(s[5])
        theta_dt2 = w2

        return casadi.vertcat(x_dt1, y_dt1, theta_dt1, x_dt2, y_dt2, theta_dt2)

    def objective(self, z, goal):
        self.goal = casadi.vertcat(goal[0], goal[1], goal[2], goal[3], goal[4], goal[5])
        return (z[4:] - self.goal).T @ self._Q_goal @ (z[4:] - self.goal) + 0.1 * (z[0]**2 + z[1]**2 + z[2]**2 + z[3]**2)

    def objectiveN(self, z, goal):
        self.goal = casadi.vertcat(goal[0], goal[1], goal[2], goal[3], goal[4], goal[5])
        return (z[4:] - self.goal).T @ self._Q_goal_N @ (z[4:] - self.goal) + 0.1 * (z[0]**2 + z[1]**2 + z[2]**2 + z[3]**2)

def main():

    def animate(i):
        line1.set_xdata(real_trajectory['x1'][:i + 1])
        line1.set_ydata(real_trajectory['y1'][:i + 1])
        line1.set_3d_properties(real_trajectory['z1'][:i + 1])
        point1.set_xdata(real_trajectory['x1'][i])
        point1.set_ydata(real_trajectory['y1'][i])
        point1.set_3d_properties(real_trajectory['z1'][i])
        line2.set_xdata(real_trajectory['x2'][:i + 1])
        line2.set_ydata(real_trajectory['y2'][:i + 1])
        line2.set_3d_properties(real_trajectory['z2'][:i + 1])
        point2.set_xdata(real_trajectory['x2'][i])
        point2.set_ydata(real_trajectory['y2'][i])
        point2.set_3d_properties(real_trajectory['z2'][i])

    env1 = Robot(0, 0, 0)
    env2 = Robot(10, 10, 10)
    mpc = MPC(10) # centralized control
    real_trajectory = {'x1': [], 'y1': [], 'z1': [], 'x2': [], 'y2': [], 'z2': []}
    for iter in range(5000):
        #state = env.step(0.5, 0.)
        v1, w1, v2, w2 = mpc.control(env1.current, env2.current, np.array([10., 10., 10., 0., 0., 0.]))
        state1 = env1.step(v1, w1)
        state2 = env2.step(v2, w2)
        real_trajectory['x1'].append(state1.x)
        real_trajectory['y1'].append(state1.y)
        real_trajectory['z1'].append(0)
        real_trajectory['x2'].append(state2.x)
        real_trajectory['y2'].append(state2.y)
        real_trajectory['z2'].append(0)

    # plotting stuff
    fig = plt.figure()
    ax1 = p3.Axes3D(fig) # 3D place for drawing
    real_trajectory['x1'] = np.array(real_trajectory['x1'])
    real_trajectory['y1'] = np.array(real_trajectory['y1'])
    real_trajectory['z1'] = np.array(real_trajectory['z1'])
    real_trajectory['x2'] = np.array(real_trajectory['x2'])
    real_trajectory['y2'] = np.array(real_trajectory['y2'])
    real_trajectory['z2'] = np.array(real_trajectory['z2'])

    point1, = ax1.plot([real_trajectory['x1'][0]], [real_trajectory['y1'][0]], [real_trajectory['z1'][0]], 'ro',
                      label='Robot1', markersize=5)
    point2, = ax1.plot([real_trajectory['x2'][0]], [real_trajectory['y2'][0]], [real_trajectory['z2'][0]], 'go',
                      label='Robot2', markersize=5)

    line1, = ax1.plot([real_trajectory['x1'][0]], [real_trajectory['y1'][0]], [real_trajectory['z1'][0]],
                     label='Real_Trajectory1')
    line2, = ax1.plot([real_trajectory['x2'][0]], [real_trajectory['y2'][0]], [real_trajectory['z2'][0]],
                     label='Real_Trajectory2')

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
                                  frames=len(real_trajectory['x1']),
                                  interval=1,
                                  repeat=False,
                                  blit=False)
    plt.show()

if __name__ == "__main__":
    main()