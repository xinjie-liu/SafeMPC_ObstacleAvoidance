"""
File containing the class definition of the Model Predictive Controller
"""

import numpy as np
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
    def __init__(self, N,dt):
        self.dt = dt
        self.N = N  # planning horizon
        self.stages = forcespro.MultistageProblem(N)  # create the stages for the whole finite horizon
        self.nx = 3 * 2
        self.nu = 2 * 2
        self.Q = 50*np.diag([40, 40, 0.1, 40, 40, 0.1])
        self.R = np.eye(self.nu)/100
        self.P = 0 * self.Q
        self.set_up_solver()
        import MPC_Project_FORCESPRO_py
        self.solver = MPC_Project_FORCESPRO_py
        self.single_nx = 3
        self.single_nu = 2
        self.n = 2
        #self.theta_err = np.zeros((self.n,self.nx+self.nu))
        self.theta_err = np.zeros((self.n,self.N))
        self.theta = np.zeros((self.n,self.N))

    def set_up_solver(self):
        for i in range(self.N):
            self.stages.dims[i]['n'] = self.nx + self.nu  # number of stage variables
            self.stages.dims[i]['r'] = self.nx  # number of equality constraints
# =============================================================================
#             self.stages.dims[i]['l'] = 4  # number of lower bounds
#             self.stages.dims[i]['u'] = 4  # number of upper bounds
# 
#             # lower bounds
#             self.stages.ineq[i]['b']['lbidx'] = np.array([1, 2, 3, 4])  # lower bound acts on these indices
#             self.stages.ineq[i]['b']['lb'] = np.array([-10, -10, -10, -10])  # lower bound for this stage variable
# 
#             # upper bounds
#             self.stages.ineq[i]['b']['ubidx'] = np.array([1, 2, 3, 4])  # upper bound acts on these indices
#             self.stages.ineq[i]['b']['ub'] = np.array([10, 10, 10, 10])  # upper bound for this stage variable
# =============================================================================

            # collision avoidance between robots: section 8.8 of documentation(https://forces.embotech.com/Documentation/low_level_interface/index.html#cost-function)
            # QCQP problem
            
            self.stages.dims[ i ]['q'] =1 # number of quadratic constraints
            
            self.stages.ineq[i]['q']['idx'] = np.zeros((1,), dtype=object) # index vectors     (starting at 1)      
            self.stages.ineq[i]['q']['idx'][0] = np.array([5, 6, 8, 9]) # Add the constraint to e1,e2,e3,e4

            
            self.stages.ineq[i]['q']['Q'] = np.zeros((1,), dtype=object) # Hessians, only one quadratic constraint
            self.stages.ineq[i]['q']['Q'][0] = 0*np.array([[-1.0, 0., 1.0, 0.], [0., -1.0, 0., 1.0], [1.0, 0., -1.0, 0.], [0., 1.0, 0., -1.0]]) # square distance between robots

            
            self.stages.ineq[i]['q']['l'] = np.zeros((1,), dtype=object)
            self.stages.ineq[i]['q']['l'][0] = np.array([[0],[0],[0],[0]])
 
            self.stages.ineq[i]['q']['r'] = np.array([[0]])  # RHSs
            
# =============================================================================
#             # hyper-plane linear inequality constraints for collision avoidance
#             self.stages.dims[i]['p'] = 2 # two linear constraints for two robots
#             self.stages.ineq[i]['p']['A'] = np.zeros((2, self.nx + self.nu)) # Jacobian of linear inequality
#             self.stages.ineq[i]['p']['b'] = np.zeros((2,))  # RHS of linear inequality
# 
# =============================================================================



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
        # parameter: linearized model
        for i in range(self.N-1):
            self.stages.newParam("linear_model"+str(i+1), [i+1], 'eq.C')
        # parameter: collision avoidance constraints
        for i in range(self.N):
            self.stages.newParam("obstacleQ"+str(i+1), [i+1], 'ineq.q.Q')
            self.stages.newParam("obstaclel"+str(i+1), [i+1], 'ineq.q.l')
            self.stages.newParam("obstacleR"+str(i+1), [i+1], 'ineq.q.r')
        # define the output
        self.stages.newOutput('output', range(1, self.N+1), range(1, self.nu + self.nx + 1))

        # solver settings
        self.stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
        self.stages.codeoptions['printlevel'] = 2
        self.stages.generateCode()

# =============================================================================
#     def define_hyperplane(self, x1, y1, x2, y2):
#         # given position of two robots, compute the normal vector and the value of projection of boundary point onto the normal vector
#         r = 1 # safety radius of each robot
# 
#         
#         distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
#         
#         
#         
#         sin_theta = 2 * r / distance
#         # sin_theta = np.round(sin_theta, decimals=2)
#         if sin_theta > 1:
#             sin_theta = 1.
#             
#         theta =np.arcsin(sin_theta)
#         global storeN
#         storeN.append(theta)
#         V = (np.array([x2-x1,y2-y1]))
#         #v = V/np.linalg.norm(V)
#         
#         theta += np.pi/2
#         rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
# # =============================================================================
# #         cos_theta = np.sqrt(1 - sin_theta**2)
# #         rotation = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
# #         
# #         n = np.array([y2-y1, x1-x2])
# # =============================================================================
# 
# #================================================================
# # =============================================================================
# #         print('n: ', n)
# #         print('sin: ', sin_theta, 'cos: ', cos_theta)
# #         print('rotation matrix: ', rotation)
# # =============================================================================
# #================================================================
#         
#         n = rotation @ V
#         
#         a = n[0] * (x1+x2)/2 + n[1] * (y1+y2)/2
# #================================================================
# # =============================================================================
# #         # delete later
# #         print('x1, y1: ' , x1, ' ', y1)
# #         print('x2, y2: ', x2, ' ', y2)
# #         print()
# #         print('distance: ', distance)
# #         print('n: ', n)
# #         print('a: ', a)
# #         print('middle point: ', [(x1+x2)/2, (y1+y2)/2])
# # =============================================================================
# #=================================================================
#         return n, a
# =============================================================================

    def collision_avoidance(self, X1, X2, xref1, xref2): # xref1, xref2 shape: (N, 7)
        # define the hyper-plane for collision avoidance
        x1 = X1[0]
        y1 = X1[1]
        x2 = X2[0]
        y2 = X2[1]
        
        
        distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        
        self.theta_ref = np.zeros((self.n,self.N))
        self.theta_ref[0,:] = xref1[:,-1]
        self.theta_ref[1,:] = xref2[:,-1]
        
        self.theta_ref1 = xref1[:-1, -1]
        self.theta_ref2 = xref2[:-1, -1]
        # robot orientations calculated from the last time step
        self.theta = (self.theta_ref - self.theta_err)

        ineql = np.zeros([self.N+1,4])
        ineqQ = np.zeros([self.N+1,4,4])
        ineqr = 0*np.ones([self.N+1,1])
        
        safety_r =1# Distance to be kept between the two robots
        
        if distance < 3:

            for i in range(self.N):
                sin_1 = np.sin(self.theta[0,i])
                cos_1 = np.cos(self.theta[0,i])
                sin_2 = np.sin(self.theta[1,i])
                cos_2 = np.cos(self.theta[1,i])
    
# =============================================================================
#                 R_1_inv = np.array([[cos_1,-sin_1,-cos_2,sin_2]])    # R is the cross rotational vector
#                 R_2_inv = np.array([[sin_1,cos_1,-sin_2,-cos_2]]) 
# =============================================================================
                
                R_inv = np.array([[cos_1,-sin_1,0,0],[sin_1,cos_1,0,0],[0,0,cos_2,-sin_2],[0,0,sin_2,cos_2]])
                #
                #
                R_1_inv = (R_inv[0,:] - R_inv[2,:]).reshape(1,4)
                R_2_inv = (R_inv[1,:] - R_inv[3,:]).reshape(1,4)
                cx = (xref1[i,0] - xref2[i,0])
                cy = (xref1[i,1] - xref2[i,1])
                
                ineql[i] = -(-2*(cx*R_1_inv+cy*R_2_inv))
    
                ineqQ[i] = -(R_1_inv.T @ R_1_inv + R_2_inv.T @ R_2_inv)
           
                ineqr[i] = -np.array([[safety_r**2-cx**2-cy**2]])
                
                # Inequality should be in the form=>  -e.T*Q*e - l*e <= -r
        #print(ineqQ[0]," + ", ineql[0], "<=", ineqr[0])

        self.obstacle = {"Q": ineqQ, "l": ineql,"r": ineqr}
#==========================================================================================
    def control(self, state, Ads, Bds):
        self.problem = {"xinit": -state}  # eq.c = -xinit
        # set up linearized models as equality constraints
        for i in range(self.N - 1):
            A = Ads[i]
            B = Bds[i]
                       
            
            self.problem["linear_model"+str(i+1)] = np.hstack((B, A))
            self.problem["obstacleQ"+str(i+1)] = self.obstacle["Q"][i]
            self.problem["obstaclel"+str(i+1)] = self.obstacle["l"][i]
            self.problem["obstacleR"+str(i+1)] = self.obstacle["r"][i]
# # =============================================================================
            self.problem["obstacleQ"+str(self.N)] = self.obstacle["Q"][self.N]
            self.problem["obstaclel"+str(self.N)] = self.obstacle["l"][self.N]
            self.problem["obstacleR"+str(self.N)] = self.obstacle["r"][self.N]
# =============================================================================
        self.output = self.solver.MPC_Project_FORCESPRO_solve(self.problem)[0]['output']
        control = self.output[:self.nu]

        # get the angle error for calculating the collision avoidance constraints
        # Extract theta_err for the n robots from the output:
        self.theta_err[0,:] = (self.output[6::10])
        self.theta_err[1,:] = (self.output[9::10])
        
# =============================================================================
#         self.theta_err1 = []
#         self.theta_err2 = []
#         for i in range(self.N):
#             self.theta_err1.append(self.output[i*(self.nx+self.nu) + self.nu + self.single_nx - 1])
#             self.theta_err2.append(self.output[i*(self.nx+self.nu) + self.nu + self.n*self.single_nx - 1])
# =============================================================================
        # shift the error for one time step (since it will be used in the next step)
# =============================================================================
#         self.theta_err1 = self.theta_err1[1:]
#         self.theta_err2 = self.theta_err2[1:]
#         self.theta_err1.append(self.theta_err1[-1])
#         self.theta_err2.append(self.theta_err2[-1])
#         self.theta_err1 = np.array(self.theta_err1)
#         self.theta_err2 = np.array(self.theta_err2)
# =============================================================================
#==========================================================================================
        # delete later
        # for i in range(self.N):
        #     print("predicted error of robot 1 at stage {}".format(i+1), self.output[i*(self.nx+self.nu) + self.nu], ' ', self.output[i*(self.nx+self.nu) + self.nu+1])
        #     error_vector1 = np.array([self.output[i*(self.nx+self.nu) + self.nu], self.output[i*(self.nx+self.nu) + self.nu+1]])
        #     print("left hand side of the constrainst: ", self.hyperplane["A"][0, 4:6] @ error_vector1)
#===========================================================================================
        return control

# #=======================================================
plt.close("all")
storeN = []
T = 10
dt = 1e-2
#Xref1 = traj_generate(T/dt, T)
#Xref2 = traj_generate(T/dt, T)
Xref1 = line_traj_generate([0.,0.,0.], [10.,10.,0.], T/dt,dt)
Xref2 = line_traj_generate([10.,10.,0.], [0.,0.,0.], T/dt,dt)
Uref1 = get_ref_input(Xref1)
Uref2 = get_ref_input(Xref2)
linear_models1 = linearize_model(Xref1, Uref1, dt)
linear_models2 = linearize_model(Xref2, Uref2, dt)
# #=========================================================
x1 = np.array([0., 0, 0]) # This angle needs to be in standard notation (it gets wrapped later)
env1 = Robot(x1[0], x1[1], x1[2])
x2 = np.array([10., 10, np.pi]) # This angle needs to be in standard notation (it gets wrapped later)
env2 = Robot(x2[0], x2[1], x2[2])
N = 5
mpc = MPC(N,dt)
nx = 3 # take care: nx here refers to nx for single robot!!
real_trajectory = {'x1': [x1[0]], 'y1': [x1[1]], 'z1': [0], 'theta1': [x1[2]], 'x2': [x2[0]], 'y2': [x2[1]], 'z2': [0], 'theta2': [x2[2]]}
uStore = []
error_t1 = np.zeros((mpc.N,nx))
error_t2 = np.zeros((mpc.N,nx))
x_error1 = []
y_error1 = []
x_error2 = []
y_error2 = []

for i in range(int(T/dt)-N):
# for i in range(int(T/(1.65*dt))):
    # Find the new linearisation (from current step to current step + N
    Ads1 = linear_models1[0][i:i+N]
    Bds1 = linear_models1[1][i:i+N]
    Ads2 = linear_models2[0][i:i+N]
    Bds2 = linear_models2[1][i:i+N]
    # Calculate the new errors (current pose vs reference pose)
    # robot 1

    error_t1[:,:2] = np.array([np.array([[np.cos(x1[2]),np.sin(x1[2])],[-np.sin(x1[2]), np.cos(x1[2])]])@(Xref1[i+k,:2] - x1[:2]) for k in range(N)])
    error_t1[:,2] = np.array([Xref1[i+k,6] - wrapAngle(x1[2]) for k in range(N)])
    # Wrap the error too (otherwise there is a huge between -pi and pi)
    error_t1[:,2] = wrapAngle(error_t1[:,2])
    # robot 2
    error_t2[:,:2] = np.array([np.array([[np.cos(x2[2]),np.sin(x2[2])],[-np.sin(x2[2]), np.cos(x2[2])]])@(Xref2[i+k,:2] - x2[:2]) for k in range(N)])
    error_t2[:,2] = np.array([Xref2[i+k,6] - wrapAngle(x2[2]) for k in range(N)])
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
    mpc.collision_avoidance(x1, x2, Xref1[i:i+N], Xref2[i:i+N])
    # Solve the MPC problem:
    control = mpc.control(error_t, Ads, Bds)
    # Extract the first control input (for error correction) and add the reference input (for trajectory tracking)
    u1 = mpc.output[0:2] + Uref1[i,:]
    u2 = mpc.output[2:mpc.nu] + Uref2[i,:]
    uStore.append([u1, u2])
    # Simulate the motion
    state1 = env1.step(u1[0], u1[1])
    state2 = env2.step(u2[0], u2[1])
    x1 = np.array([state1.x,state1.y,state1.theta])
    x2 = np.array([state2.x,state2.y,state2.theta])
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
ax1.plot(xPos1, yPos1, 'r')
ax1.plot(Xref1[:, 0], Xref1[:, 1], 'g')
fig2, ax2 = plt.subplots()
ax2.plot(xPos2, yPos2, 'r')
ax2.plot(Xref2[:, 0], Xref2[:, 1], 'g')
fig3, ax3 = plt.subplots()
ax3.plot(xPos1, yPos1, 'r')
ax3.plot(xPos2, yPos2, 'b')
ax3.plot(Xref1[:, 0], Xref1[:, 1], 'g')
circle = plt.Circle((5,5),0.5,color='m',alpha=0.2)
ax3.add_patch(circle)
# plot the error
x_error1 = np.array(x_error1)
y_error1 = np.array(y_error1)
x_error2 = np.array(x_error2)
y_error2 = np.array(y_error2)
fig4, ax4 = plt.subplots()
ax4.plot(range(len(x_error1)), x_error1, 'b')
ax4.plot(range(len(y_error1)), y_error1, 'g')
fig5, ax5 = plt.subplots()
ax5.plot(range(len(x_error2)), x_error2, 'b')
ax5.plot(range(len(y_error2)), y_error2, 'g')
plt.show()
# animation
#plot_multi_robot(real_trajectory)
