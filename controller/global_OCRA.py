"""
File containing the class definition of the Model Predictive Controller
"""

import numpy as np
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from scipy.linalg import block_diag
from model.unicycle import Robot
from model.MPC_utils import *
import forcespro
import get_userid
import casadi

"""
Parameters of the class
"""

class MPC():
    def __init__(self, N):
        self.dt = 2e-2
        self.N = N  # planning horizon
        self.stages = forcespro.MultistageProblem(N)  # create the stages for the whole finite horizon
        self.nx = 3 * 2
        self.nu = 2 * 2
        # self.Q = 100*np.diag([40, 40, 0.1, 40, 40, 0.1])
        # self.R = np.eye(self.nu)/100
        self.Q = 0.01*np.diag([4, 4, 0.1, 4, 4, 0.1])
        self.R = 0.001*np.eye(self.nu)
        self.P = 0 * self.Q
        self.set_up_solver()
        import MPC_Project_FORCESPRO_py
        self.solver = MPC_Project_FORCESPRO_py
        self.theta_err1 = None
        self.theta_err2 = None
        self.single_nx = 3
        self.single_nu = 2
        self.n = 2

    def set_up_solver(self):
        for i in range(self.N):
            self.stages.dims[i]['n'] = self.nx + self.nu  # number of stage variables
            self.stages.dims[i]['r'] = self.nx  # number of equality constraints
            self.stages.dims[i]['l'] = 2  # number of lower bounds
            self.stages.dims[i]['u'] = 2  # number of upper bounds

            # lower bounds
            self.stages.ineq[i]['b']['lbidx'] = np.array([1, 2])  # lower bound acts on these indices
            self.stages.ineq[i]['b']['lb'] = np.array([-10, -10])  # lower bound for this stage variable

            # upper bounds
            self.stages.ineq[i]['b']['ubidx'] = np.array([1, 2])  # upper bound acts on these indices
            self.stages.ineq[i]['b']['ub'] = np.array([10, 10])  # upper bound for this stage variable

            # collision avoidance between robots: section 8.8 of documentation(https://forces.embotech.com/Documentation/low_level_interface/index.html#cost-function)
            # QCQP problem
            # self.stages.ineq[i]['q']['idx'] = np.zeros((1,), dtype=object) # index vectors
            # self.stages.ineq[i]['q']['idx'][0] = np.array([5, 6, 8, 9])
            # self.stages.ineq[i]['q']['Q'] = np.zeros((1,), dtype=object) # Hessians, only one quadratic constraints
            # self.stages.ineq[i]['q']['Q'][0] = -np.array([[1, 0, -1, 0], [0, 1, 0, -1], [-1, 0, 1, 0], [0, -1, 0, 1]]) # square distance between robots
            # self.stages.ineq[i]['q']['r'] = -np.array([0.1])  # RHSs

            # hyper-plane linear inequality constraints for collision avoidance
            self.stages.dims[i]['p'] = 2 # two linear constraints for two robots
            self.stages.ineq[i]['p']['A'] = np.zeros((2, self.nx + self.nu)) # Jacobian of linear inequality
            self.stages.ineq[i]['p']['b'] = np.zeros((2, ))  # RHS of linear inequality

            # Cost/Objective function
            # V = sum_i(z(i)*H*z(i)) + z(N)*H*z(N) -> where z(i) = [u1,u2,x1,x2] at stage/step i.
            if i == self.N - 1:
                # For xN use the terminal cost P to penalise the xN state
                # z: [u10, u11, u20, u21, x1, y1, theta1, x2, y2, theta2]
                self.stages.cost[i]['H'] = np.vstack(
                    (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.P))))
            else:
                # For i from 0 to N-1 use the stage cost Q to penalise the state
                self.stages.cost[i]['H'] = np.vstack(
                    (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.Q))))

            self.stages.cost[i]['f'] = np.zeros((self.nx + self.nu, 1))  # there's no linear fT*z term in the cost, so set to 0

            # Equality constraints (expressed in the form of C*z(i) + D*z(i+1) = 0, where C = ( B(i) | A(i) ), D = (0 | -I)
            if (i < self.N - 1):
                self.stages.eq[i]['C'] = np.zeros([self.nx, self.nx+self.nu])
            if (i > 0):
                self.stages.eq[i]['c'] = np.zeros((self.nx, 1))
            self.stages.eq[i]['D'] = np.hstack((np.zeros([self.nx, self.nu]), -np.eye(self.nx)))

        # parameter: initial state
        self.stages.newParam("xinit", [1], 'eq.c')  # 1-indexed
        # parameter: terminal cost
        self.stages.newParam("terminal_cost", [N], 'cost.H')
        # parameter: linearized model
        for i in range(self.N-1):
            self.stages.newParam("linear_model"+str(i+1), [i+1], 'eq.C')
        # parameter: collision avoidance constraints
        for i in range(self.N):
            self.stages.newParam("hyperplaneA"+str(i+1), [i+1], 'ineq.p.A')
            self.stages.newParam("hyperplaneb"+str(i+1), [i+1], 'ineq.p.b')
        # define the output
        self.stages.newOutput('output', range(1, self.N+1), range(1, self.nu + self.nx + 1))

        # solver settings
        self.stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
        self.stages.codeoptions['printlevel'] = 2
        self.stages.generateCode()

    def define_hyperplane(self, x1, y1, x2, y2, theta2, Uref2):
        # # ================================================================
        # velocity obstacle
        r = 0.5  # safety radius of each robot
        distance = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        sin_theta = 2*r / distance
        if sin_theta >= 1:
            sin_theta = 1
        cos_theta = np.sqrt(1 - sin_theta**2)
        p_c = np.array([x2, y2]) - np.array([x1, y1])

        sin_ = np.sin(np.pi/2)
        cos_ = np.cos(np.pi/2)
        rotation = np.array([[cos_, sin_], [-sin_, cos_]]) # 90 degree, clockwise
        rotation2 = np.array([[cos_theta, sin_theta], [-sin_theta, cos_theta]])

        normal_vec = (rotation2 @ rotation @ p_c) / np.linalg.norm(p_c)
        print(normal_vec)

        # calculate normal_vec @ velocity_robot2
        print('theta2: ', theta2)
        vel2 = Uref2[0,0] # approximate the velocity of robot2 with its reference (for simplicity)
        R = np.array([[np.cos(theta2), -np.sin(theta2)], [np.sin(theta2), np.cos(theta2)]])
        vel2_vec = R @ np.array([vel2, 0]) # get the vector of velocity of robot2
        a = normal_vec @ vel2_vec
        return normal_vec, a
        # # ================================================================

    def collision_avoidance(self, X1, X2, Xref1, Xref2, Uref1, Uref2): # xref1, xref2 shape: (N, 7)
# ============================================================================================
        # define a hyperlane that rotates around obstacle (Benjamin's method)
        x1 = X1[0]
        y1 = X1[1]
        theta1 = wrapAngle(X1[2])
        # sin_ = np.sin(theta1 + Uref1[:,1] * self.dt)
        # cos_ = np.cos(theta1 + Uref1[:,1] * self.dt)
        x2 = X2[0]
        y2 = X2[1]
        distance = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        v, a = self.define_hyperplane(x1, y1, x2, y2, wrapAngle(X2[2]), Uref2) # normal vector in velocity space

        # define linear constraints
        ineqA = np.zeros((self.N, self.n, self.nu+self.nx))
        ineqb = np.zeros((self.N, self.n))
        for i in range(self.N):
            if not i == 0:
                theta1 += wrapAngle(Uref1[i-1,1]*self.dt)
            sin_ = np.sin(theta1)
            cos_ = np.cos(theta1)
            if distance < 3.50:
                ineqA[i] = np.array([[-v[0]*cos_-v[1]*sin_, v[0]*Uref1[i,0]*sin_*self.dt-v[1]*Uref1[i,0]*cos_*self.dt, 0, 0, 0, 0, 0, 0, 0, 0],\
                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
                ineqb[i, 0] = v[0]*Uref1[i,0]*cos_ + v[1]*Uref1[i,0]*sin_ - a
            else:
                pass
        self.hyperplane = {"A": ineqA, "bs": ineqb}
# =============================================================================================

    def control(self, state, Ads, Bds):
        self.problem = {"xinit": -state}  # eq.c = -xinit
        # set up linearized models as equality constraints
        for i in range(self.N - 1):
            A = Ads[i]
            B = Bds[i]
            self.problem["linear_model"+str(i+1)] = np.hstack((B, A))
            self.problem["hyperplaneA"+str(i+1)] = self.hyperplane["A"][i]
            self.problem["hyperplaneb"+str(i+1)] = self.hyperplane["bs"][i]
        self.problem["hyperplaneA"+str(self.N)] = self.hyperplane["A"][self.N-1]
        self.problem["hyperplaneb"+str(self.N)] = self.hyperplane["bs"][self.N-1]
        self.problem['terminal_cost'] = np.vstack(
            (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.P))))
        self.output = self.solver.MPC_Project_FORCESPRO_solve(self.problem)
        control = self.output[0]['output'][:self.nu]

#==========================================================================================
        # delete later
        # for i in range(self.N):
        #     print("predicted error of robot 1 at stage {}".format(i+1), self.output[i*(self.nx+self.nu) + self.nu], ' ', self.output[i*(self.nx+self.nu) + self.nu+1])
        #     error_vector1 = np.array([self.output[i*(self.nx+self.nu) + self.nu], self.output[i*(self.nx+self.nu) + self.nu+1]])
        #     print("left hand side of the constrainst: ", self.hyperplane["A"][0, 4:6] @ error_vector1)
#===========================================================================================
        return control

# #=======================================================
T = 10
dt = 1e-2
# Xref1 = traj_generate(T/dt, T)
# Xref2 = traj_generate(T/dt, T)
Xref1 = line_traj_generate([0.,0.,0.], [10.,6.,0.], T/dt, dt)
Xref2 = line_traj_generate([0.,6.,0.], [10.,0.,0.], T/dt, dt)
Uref1 = get_ref_input(Xref1)
Uref2 = get_ref_input(Xref2)
linear_models1 = linearize_model_global(Xref1, Uref1, dt)
linear_models2 = linearize_model_global(Xref2, Uref2, dt)
# #=========================================================
x1 = np.array([0., 0., np.pi/4]) # This angle needs to be in standard notation (it gets wrapped later)
env1 = Robot(x1[0], x1[1], x1[2], dt=dt)
x2 = np.array([0., 6., -np.pi/4]) # This angle needs to be in standard notation (it gets wrapped later)
env2 = Robot(x2[0], x2[1], x2[2], dt=dt)
N = 5
mpc = MPC(N)
nx = 3 # take care: nx here refers to nx for single robot!!
real_trajectory = {'x1': [x1[0]], 'y1': [x1[1]], 'z1': [0], 'theta1': [x1[2]], 'x2': [x2[0]], 'y2': [x2[1]], 'z2': [0], 'theta2': [x2[2]]}
uStore = []
error_t1 = np.zeros((mpc.N,nx))
error_t2 = np.zeros((mpc.N,nx))
x_error1 = []
y_error1 = []
x_error2 = []
y_error2 = []
Ps,_ = find_P(linear_models1[0], linear_models1[1], mpc.Q[:3,:3], mpc.R[:2,:2])

for i in range(int(T/dt)-N):
#for i in range(10):
    # Find the new linearisation (from current step to current step + N
    Ads1 = linear_models1[0][i:i+N]
    Bds1 = linear_models1[1][i:i+N]
    Ads2 = linear_models2[0][i:i+N]
    Bds2 = linear_models2[1][i:i+N]
    #mpc.P = np.zeros((2*nx, 2*nx)) # set the terminal cost (only for robot 1)
    mpc.P = block_diag(Ps[i + N - 1], 0*np.eye(nx))
    # Calculate the new errors (current pose vs reference pose)
    # robot 1
    error_t1[:,:2] = np.array([(x1[:2] - Xref1[i+k,:2]) for k in range(N)])
    error_t1[:,2] = np.array([wrapAngle(x1[2]) - Xref1[i+k,6] for k in range(N)])
    # Wrap the error too (otherwise there is a huge between -pi and pi)
    error_t1[:,2] = wrapAngle(error_t1[:,2]) #+ np.random.normal(-0.1, 1e-2)
    # robot 2
    error_t2[:,:2] = np.array([(x2[:2] - Xref2[i+k,:2]) for k in range(N)])
    error_t2[:,2] = np.array([wrapAngle(x2[2] - Xref2[i+k,6]) for k in range(N)])
    # Wrap the error too (otherwise there is a huge between -pi and pi)
    error_t2[:,2] = wrapAngle(error_t2[:,2])
    # Concatenate the errors and linearized models
    error_t = np.hstack((error_t1[0, :], error_t2[0, :]))
    zeros = np.zeros(Ads1.shape)
    Ads1 = np.concatenate((Ads1, zeros), axis=2)
    Ads2 = np.concatenate((zeros, Ads2), axis=2)
    Ads = np.concatenate((Ads1, Ads2), axis=1)
    zeros = np.zeros(Bds1.shape)
    Bds1 = np.concatenate((Bds1, zeros), axis=2)
    Bds2 = np.concatenate((zeros, Bds2), axis=2)
    Bds = np.concatenate((Bds1, Bds2), axis=1)
    # set up the linear inequality constraints for collision avoidance
    mpc.collision_avoidance(x1, x2, Xref1[i:i+N+1], Xref2[i:i+N+1], Uref1[i:i+N+1], Uref2[i:i+N+1])
    # Solve the MPC problem:
    control = mpc.control(error_t, Ads, Bds)
    if not mpc.output[1] == 1:
        break
    # Extract the first control input (for error correction) and add the reference input (for trajectory tracking)
    u1 = mpc.output[0]['output'][0:2] + Uref1[i,:]
    u2 = mpc.output[0]['output'][2:mpc.nu] + Uref2[i,:]
    uStore.append([u1, u2])
    # Simulate the motion
    state1 = env1.step(u1[0], u1[1])
    state2 = env2.step(u2[0], u2[1])
    x1 = np.array([state1.x,state1.y,state1.theta])
    x2 = np.array([state2.x,state2.y,state2.theta])
#==========================================================================
    print("true sin: ", np.sin(state1.theta))
    print("control input: ", u1)
#==========================================================================
    # Store the xy position for plotting:
    real_trajectory['x1'].append(state1.x)
    real_trajectory['y1'].append(state1.y)
    real_trajectory['z1'].append(0)
    real_trajectory['theta1'].append(state1.theta)
    real_trajectory['x2'].append(state2.x)
    real_trajectory['y2'].append(state2.y)
    real_trajectory['z2'].append(0)
    real_trajectory['theta2'].append(state2.theta)
    print('current position: x1: ', state1.x, ', y1: ', state1.y)
    print('current position: x2: ', state2.x, ', y2: ', state2.y)
    x_error1.append(error_t1[0, 0])
    y_error1.append(error_t1[0, 1])
    x_error2.append(error_t2[0, 0])
    y_error2.append(error_t2[0, 1])

# plot the robot position
xPos1 = np.array(real_trajectory['x1'])
yPos1 = np.array(real_trajectory['y1'])
xPos2 = np.array(real_trajectory['x2'])
yPos2 = np.array(real_trajectory['y2'])
fig1, ax1 = plt.subplots()
ax1.plot(xPos1, yPos1, 'r', label='robot_trajectory')
ax1.plot(Xref1[:, 0], Xref1[:, 1], 'g', label='reference_trajectory')
ax1.legend()
ax1.set_title('Robot1')
fig2, ax2 = plt.subplots()
ax2.plot(xPos2, yPos2, 'r', label='robot_trajectory')
ax2.plot(Xref2[:, 0], Xref2[:, 1], 'g', label='reference_trajectory')
ax1.set_title('Robot2')
ax2.legend()
fig3, ax3 = plt.subplots()
ax3.plot(xPos1, yPos1, 'r', label='robot1_trajectory')
ax3.plot(xPos2, yPos2, 'b', label='robot2_trajectory')
ax3.legend()
# plot the error
x_error1 = np.array(x_error1)
y_error1 = np.array(y_error1)
x_error2 = np.array(x_error2)
y_error2 = np.array(y_error2)
fig4, ax4 = plt.subplots()
ax4.plot(range(len(x_error1)), x_error1, 'b', label='x_error')
ax4.plot(range(len(y_error1)), y_error1, 'g', label='y_error')
ax4.set_title('Robot1')
ax4.legend()
fig5, ax5 = plt.subplots()
ax5.plot(range(len(x_error2)), x_error2, 'b', label='x_error')
ax5.plot(range(len(y_error2)), y_error2, 'g', label='y_error')
ax5.set_title('Robot2')
ax5.legend()
plt.show()
# animation
#plot_single_robot(real_trajectory)
plot_multi_robot(real_trajectory)
