"""
File containing the class definition of the Model Predictive Controller
"""

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from numpy.linalg import inv
from matplotlib import animation
from model.unicycle import Robot
import forcespro
from scipy.linalg import block_diag

"""
Parameters of the class
"""
class MPC():
    def __init__(self, N, dt=2e-2):
        self.dt = dt
        self.N = N  # planning horizon
        self.stages = forcespro.MultistageProblem(N)  # create the stages for the whole finite horizon
        self.nx = 3
        self.nu = 2
        self.Q = 100*np.diag([40, 40, 0.1])
        self.R = np.eye(self.nu)/100
        self.P = 0 * self.Q
        self.set_up_solver()
        import MPC_Project_FORCESPRO_py
        self.solver = MPC_Project_FORCESPRO_py
        self.n = 1

    def set_up_solver(self):
        for i in range(self.N):
            self.stages.dims[i]['n'] = self.nx + self.nu + 1  # number of stage variables (one slack variable per stage)
            self.stages.dims[i]['r'] = self.nx  # number of equality constraints
            self.stages.dims[i]['l'] = 3  # nx + nu  # number of lower bounds
            self.stages.dims[i]['u'] = 2  # nx + nu  # number of upper bounds

            # hyper-plane linear inequality constraints for collision avoidance
            self.stages.dims[i]['p'] = 1
            self.stages.ineq[i]['p']['A'] = np.zeros((1, self.nx+self.nu+1)) # Jacobian of linear inequality
            self.stages.ineq[i]['p']['b'] = np.zeros((1, ))  # RHS of linear inequality

            # lower bounds
            self.stages.ineq[i]['b']['lbidx'] = np.array([1, 2, 6])  # lower bound acts on these indices
            self.stages.ineq[i]['b']['lb'] = np.array([-3, -3, 0])  # lower bound for this stage variable
            # upper bounds
            self.stages.ineq[i]['b']['ubidx'] = np.array([1, 2])  # lower bound acts on these indices
            self.stages.ineq[i]['b']['ub'] = np.array([3, 3])  # lower bound for this stage variable


            # Cost/Objective function
            # V = sum_i(z(i)*H*z(i)) + z(N)*H*z(N) -> where z(i) = [u1,u2,x1,x2] at stage/step i.
            if i == self.N - 1:
                # For xN use the terminal cost P to penalise the xN state
                # self.stages.cost[i]['H'] = np.vstack(
                #     (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.P))))
                self.stages.cost[i]['H'] = block_diag(self.R, self.P, np.array([[3]]))
            else:
                # For i from 0 to N-1 use the stage cost Q to penalise the state
                # self.stages.cost[i]['H'] = np.vstack(
                #     (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.Q))))
                self.stages.cost[i]['H'] = block_diag(self.R, self.Q, np.array([[3]]))

            # f = np.zeros((self.nx + self.nu + 1, 1))
            # f[-1, 0] = 1000 # slack variable
            self.stages.cost[i]['f'] = np.zeros((self.nx + self.nu + 1, 1))

            # Equality constraints (expressed in the form of C*z(i) + D*z(i+1) = 0, where C = ( B(i) | A(i) ), D = (0 | -I)
            if (i < self.N - 1):
                self.stages.eq[i]['C'] = np.zeros([self.nx, self.nx+self.nu+1])
            if (i > 0):
                self.stages.eq[i]['c'] = np.zeros((self.nx, 1))
            self.stages.eq[i]['D'] = np.hstack((np.zeros([self.nx, self.nu]), -np.eye(self.nx), np.zeros((self.nx,1))))

        # parameter: initial state
        self.stages.newParam("xinit", [1], 'eq.c')  # 1-indexed
        # parameter: linearized model
        for i in range(self.N-1):
            self.stages.newParam("linear_model"+str(i+1), [i+1], 'eq.C')
        # parameter: collision avoidance constraints
        for i in range(self.N):
            self.stages.newParam("hyperplaneA"+str(i+1), [i+1], 'ineq.p.A')
            self.stages.newParam("hyperplaneb"+str(i+1), [i+1], 'ineq.p.b')
        # define the output
        self.stages.newOutput('output', range(1, self.N+1), range(1, self.nu + self.nx + 2))

        # solver settings
        self.stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
        self.stages.codeoptions['printlevel'] = 2
        self.stages.generateCode()

    def define_hyperplane(self, x1, y1, obsx=0.5, obsy=0):
        # # ================================================================
        # define a hyperlane that rotates around obstacle (Benjamin's method)
        r = 0.1  # safety radius of each robot
        pos_obs = np.array([obsx, obsy])
        p_c = pos_obs + 2*r*(np.array([x1, y1]) - pos_obs)/np.linalg.norm(np.array([x1, y1]) - pos_obs)
        sin_theta = np.sin(np.pi/2)
        cos_theta = np.cos(np.pi/2)
        rotation = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
        # rotation2 = np.array([[np.cos(0.6), -np.sin(0.6)], [np.sin(0.6), np.cos(0.6)]])
        v = rotation @ (np.array([x1, y1]) - pos_obs)
        v = v / np.linalg.norm(v)
        a = v[1] / v[0]
        b = p_c[1] - a*p_c[0]
        # a = np.round(a, decimals=2)
        # b = np.round(b, decimals=2)
        # a = -1
        # b = 10

        return a, b

    def collision_avoidance(self, X1, xref1):  # xref1, xref2 shape: (N, 7)
        # # ============================================================================================
        # define a hyperlane that rotates around obstacle (Benjamin's method)
        x1 = X1[0]
        y1 = X1[1]
        distance = np.sqrt((x1-0.5)**2 + (y1-0)**2)
        a, b = self.define_hyperplane(x1, y1)
        # ====================================================================
        # delete later
        print("hyperplane: y = {}x + {}".format(a, b))
        # ====================================================================
        # define linear constraints
        ineqA = np.zeros((self.N, self.n, self.nu + self.nx + 1))
        ineqb = np.zeros((self.N, self.n))
        for i in range(self.N):
            if distance < 0.35:
                ineqA[i] = np.array([[0, 0, a, -1, 0, -1]])
                ineqb[i][0] = -b - a * xref1[i, 0] + xref1[i, 1]
            else:
                ineqA[i] = np.zeros((1, 6))
                ineqb[i][0] = 100
        self.hyperplane = {"A": ineqA, "bs": ineqb}
        # ============================================================================================
        # # approach: linearize Euclidian distance to obstacle as linear constraint
        # x1 = X1[0]
        # y1 = X1[1]
        # xobs = 0.5
        # yobs = 0
        # r = 0.1 # safety radius
        # distance = np.sqrt((x1 - xobs) ** 2 + (y1 - yobs) ** 2)
        # ineqA = np.zeros((self.N, self.n, self.nu + self.nx + 1))
        # ineqb = np.zeros((self.N, self.n))
        # for i in range(self.N):
        #     # if distance < 0.4:
        #     ineqA[i] = np.array([[0, 0, 2*xobs-2*xref1[i,0], 2*yobs-2*xref1[i,1], 0, -1]])
        #     ineqb[i][0] = -r**2 + (xref1[i,0]-xobs)**2 + (xref1[i,1]-yobs)**2
        #     # else:
        #     #     pass
        # self.hyperplane = {"A": ineqA, "bs": ineqb}

    def control(self, state, Ads, Bds):
        self.problem = {"xinit": -state}  # eq.c = -xinit
        # set up linearized models as equality constraints
        for i in range(self.N - 1):
            A = Ads[i]
            B = Bds[i]
            self.problem["linear_model"+str(i+1)] = np.hstack((B, A, np.zeros((self.nx, 1))))
            self.problem["hyperplaneA"+str(i+1)] = self.hyperplane["A"][i]
            self.problem["hyperplaneb"+str(i+1)] = self.hyperplane["bs"][i]
        self.problem["hyperplaneA"+str(self.N)] = self.hyperplane["A"][self.N-1]
        self.problem["hyperplaneb"+str(self.N)] = self.hyperplane["bs"][self.N-1]
        self.output = self.solver.MPC_Project_FORCESPRO_solve(self.problem)
        control = self.output[0]['output'][:self.nu]

        return control

# #=======================================================
from model.MPC_utils import *
T = 10
dt = 2e-2
Xref = traj_generate(T/dt, T)
#Xref = line_traj_generate([0.,0., 0.], [10.,10.,0.], T/dt, dt) #+ np.array([0,0,0,0,0,0,0.2])*np.random.rand(10000,7)
Uref = get_ref_input(Xref)
linear_models = linearize_model_global(Xref, Uref, dt)
# #=========================================================
N = 15
nx = 3
x0 = np.array([1, 0, np.pi/2]) # This angle needs to be in standard notation (it gets wrapped later)
env = Robot(x0[0], x0[1], x0[2], dt)
mpc = MPC(N, dt=dt)
real_trajectory = {'x': [], 'y': [], 'z': [], 'theta': []}
uStore = []
error_t = np.zeros((nx,))
x_error = []
y_error = []

for i in range(int(T/dt)-N):
    # Find the new linearisation (from current step to current step + N
    Ads = linear_models[0][i:i+N]
    Bds = linear_models[1][i:i+N]
    # Calculate the new errors (current pose vs reference pose)
    error_t[:2] = np.array(x0[:2]-Xref[i,:2])
    error_t[2] =  wrapAngle(x0[2])-Xref[i,6]
    # Wrap the error too (otherwise there is a huge between -pi and pi)
    error_t[2] = wrapAngle(error_t[2])
    # set up the linear inequality constraints for collision avoidance
    mpc.collision_avoidance(x0, Xref[i:i+N+1])
    # Solve the MPC problem:
    control = mpc.control(error_t, Ads, Bds)
    if not mpc.output[1] == 1:
        break
    # Extract the first control input (for error correction) and add the reference input (for trajectory tracking)
    u = mpc.output[0]['output'][0:2] + Uref[i,:]
    uStore.append(u)

    # Simulate the motion
    state = env.step(u[0], u[1])
    x0 = np.array([state.x,state.y,state.theta])

    # Store the xy position for plotting:
    real_trajectory['x'].append(state.x)
    real_trajectory['y'].append(state.y)
    real_trajectory['z'].append(0)
    real_trajectory['theta'].append(state.theta)
    print('current position: x: ', state.x, ', y: ', state.y)

    x_error.append(error_t[0])
    y_error.append(error_t[1])
print(mpc)
# plot the robot position
xPos = np.array(real_trajectory['x'])
yPos = np.array(real_trajectory['y'])
fig1, ax1 = plt.subplots()
ax1.plot(xPos, yPos, 'r')
ax1.plot(Xref[:, 0], Xref[:, 1], 'g')
# plot the error
x_error = np.array(x_error)
y_error = np.array(y_error)
fig2, ax2 = plt.subplots()
ax2.plot(range(len(x_error)), x_error, 'b')
ax2.plot(range(len(y_error)), y_error, 'g')
plt.show()
# animation
plot_single_robot(real_trajectory)
