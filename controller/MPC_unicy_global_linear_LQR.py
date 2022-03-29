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
import get_userid
import casadi
from scipy.io import savemat

"""
Parameters of the class
"""

class MPC():
    def __init__(self, N,dt):
        self.dt = dt
        self.N = N  # planning horizon
        self.stages = forcespro.MultistageProblem(N)  # create the stages for the whole finite horizon
        self.nx = 3
        self.nu = 2
        self.Q = .01*np.diag([4, 4, 0.1])
        self.R = .0001*np.eye(self.nu)
        # self.Q = 100 * np.diag([4, 40, 0.1])
        # self.R = np.eye(self.nu)/100
        self.P = 0 * self.Q
        self.Q_obs = 0*np.diag([4, 4, 0])
        self.set_up_solver()
        import MPC_Project_FORCESPRO_py
        self.solver = MPC_Project_FORCESPRO_py
        self.n = 1

    def set_up_solver(self):
        for i in range(self.N):
            self.stages.dims[i]['n'] = self.nx + self.nu  # number of stage variables
            self.stages.dims[i]['r'] = self.nx  # number of equality constraints
            self.stages.dims[i]['l'] = 2  # nx + nu  # number of lower bounds
            self.stages.dims[i]['u'] = 2  # nx + nu  # number of upper bounds
            # lower bounds
            self.stages.ineq[i]['b']['lbidx'] = np.array([1, 2])  # lower bound acts on these indices
            self.stages.ineq[i]['b']['lb'] = np.array([-8, -8])  # lower bound for this stage variable

            # upper bounds
            self.stages.ineq[i]['b']['ubidx'] = np.array([1, 2])  # upper bound acts on these indices
            self.stages.ineq[i]['b']['ub'] = np.array([8, 8])  # upper bound for this stage variable

            self.stages.dims[i]['p'] = 1 # number of polytopic (linear) constraints
            self.stages.ineq[i]['p']['A'] = np.zeros((1,self.nx + self.nu)) # Jacobian of linear inequality
            self.stages.ineq[i]['p']['b'] = np.zeros((1,))# RHS of linear inequality

            

            # Cost/Objective function
            # V = sum_i(z(i)*H*z(i)) + z(N)*H*z(N) -> where z(i) = [u1,u2,x1,x2] at stage/step i.
            if i == self.N - 1:
                # For xN use the terminal cost P to penalise the xN state
                self.stages.cost[i]['H'] = np.vstack(
                    (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.P))))
            else:
                # For i from 0 to N-1 use the stage cost Q to penalise the state
                self.stages.cost[i]['H'] = np.vstack(
                    (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.Q+self.Q_obs))))

            self.stages.cost[i]['f'] = np.zeros((self.nx + self.nu, 1))  # there's no linear fT*z term in the cost, so set to 0
            # Equality constraints (expressed in the form of C*z(i) + D*z(i+1) = 0, where C = ( B(i) | A(i) ), D = (0 | -I)
            if (i < self.N - 1):
                self.stages.eq[i]['C'] = np.zeros([self.nx, self.nx+self.nu])
            if (i > 0):
                self.stages.eq[i]['c'] = np.zeros((self.nx, 1))
            self.stages.eq[i]['D'] = np.hstack((np.zeros([self.nx, self.nu]), -np.eye(self.nx)))

        # parameter: initial state
        self.stages.newParam("xinit", [1], 'eq.c')  # 1-indexed
        self.stages.newParam("terminal_cost", [N], 'cost.H')
        # parameter: linearized model
        for i in range(self.N-1):
            self.stages.newParam("linear_model"+str(i+1), [i+1], 'eq.C')  
        for i in range(self.N):
            self.stages.newParam("linear_cost"+str(i+1), [i+1], 'cost.f',i+1)
            self.stages.newParam("hyperplaneA"+str(i+1), [i+1], 'ineq.p.A')
            self.stages.newParam("hyperplaneb"+str(i+1), [i+1], 'ineq.p.b')

        # define the output
        self.stages.newOutput('output', range(1, self.N+1), range(1, self.nu + self.nx + 1))
        # solver settings
        self.stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
        self.stages.codeoptions['printlevel'] = 2
        self.stages.codeoptions['overwrite'] = 1
        self.stages.generateCode()

    def collision_avoidance(self, X1, xref,start,goal,obs,timestep):
        # define the hyper-plane for collision avoidance
        

        
        heading = np.arctan2((obs[1]-X1[1]),(obs[0]-X1[0])) # heading towards obstacle
        
        if heading <= xref[0,-1]+np.sign(xref[0,-1])*np.pi/2 and heading >= -xref[0,-1]:#-np.pi/4:
            x1,y1 = start
            sign = -1
        else:
            x1,y1 = goal
            sign = 1
        
        #x1 = X1[0]
        #y1 = X1[1]
        # obstacle at 5,5
        distance = np.sqrt((x1 - obs[0]) ** 2 + (y1 - obs[1]) ** 2)       
        #distance_to_goal = np.sqrt((x1 - 10) ** 2 + (y1 - 10) ** 2) 
            
        ineqA = np.zeros((self.N, self.nu+self.nx))
        ineqb = np.zeros((self.N))
        
        safety_r =.5 # Distance to be kept between the two robots

        sin_theta = 2 * safety_r / distance
# =============================================================================
#         if np.abs(sin_theta) > 1:
#             sin_theta = np.sign(sin_theta)*1.
# =============================================================================
        

        
        V = sign*np.array([x1-obs[0],y1-obs[1]])
        
        v = (V/np.linalg.norm(V))
        
# =============================================================================
#         # Position of closest point on the circle (obstacle):
#         c = np.array([5,5]) + safety_r*np.array([x1-5,y1-5])/np.linalg.norm(np.array([5-x1,5-y1]))
#         
#         V = np.array([x1-c[0],y1-c[1]])
#         v = (V/np.linalg.norm(V))
# =============================================================================
        
        theta = sign*np.arcsin(sin_theta) + np.pi/2

        rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        
        n = rotation @ v
        #n = v
        a = n[0] * (x1 + obs[0]) / 2 + n[1] * (y1 + obs[1]) / 2 
        
        

        for i in range(self.N):
            ineqA[i] = np.array([0, 0, n[0], n[1], 0])
            ineqb[i] = a - n[0]*xref[i,0] - n[1]*xref[i,1]


        global storeConstraints
        if sign==-1:
            storeConstraints[0,:] = np.array([n[0],n[1],a])
        else:
            storeConstraints[1,:] = np.array([n[0],n[1],a]) 
        self.hyperplane = {"A": ineqA, "b": ineqb}
        self.distance = distance

# =============================================================================
#         ax3.plot(xx,yy,'b')
#         plt.show()
# =============================================================================
    
    def control(self, state, Ads, Bds,x0,xref):
        obs_avoidance = False
        self.problem = {"xinit": -state}  # eq.c = -xinit
        # set up linearized models as equality constraints
        #if np.sqrt((x0[0] - 5) ** 2 + (x0[1] - 5) ** 2) <= 2:
        for i in range(self.N - 1):
            A = Ads[i]
            B = Bds[i]
            xr = np.array([0,0,xref[i,0],xref[i,1],xref[i,-1]])
            self.problem["linear_model"+str(i+1)] = np.hstack((B, A))
# =============================================================================
#             Q_stacked = np.vstack((np.zeros((self.nu,self.nx+self.nu)), np.hstack((np.zeros((self.nx, self.nu)), self.Q_obs))))
#             f_cost = np.array([[2*np.array([0,0,5,5,0])@ Q_stacked - 2*xr @ Q_stacked]])
#            # print(f_cost)
#             self.problem["linear_cost"+str(i+1)] = f_cost
# =============================================================================
            if obs_avoidance:
                self.problem["hyperplaneA"+str(i+1)] = self.hyperplane["A"][i]
                self.problem["hyperplaneb"+str(i+1)] = self.hyperplane["b"][i]
        self.problem['terminal_cost'] = np.vstack(
            (np.hstack((self.R, np.zeros((self.nu, self.nx)))), np.hstack((np.zeros((self.nx, self.nu)), self.P))))
        self.output = self.solver.MPC_Project_FORCESPRO_solve(self.problem)[0]['output']
        control = self.output[:self.nu]

        # get the angle error for calculating the collision avoidance constraints
        # Extract theta_err for the n robots from the output:

        return control

# #=======================================================
from model.MPC_utils import *
plt.close("all")
storeConstraints = np.zeros((2,3))
T = 10
dt = 1e-2
Xref = traj_generate(T/dt, T)
#Xref = line_traj_generate([0.,0.,0], [10.,10.,0.], T/dt,dt)
obs = np.array([5,5])
Uref = get_ref_input(Xref)
linear_models = linearize_model_global(Xref, Uref, dt)
# #=========================================================
x0 = np.array([1., 0., 0.]) # This angle needs to be in standard notation (it gets wrapped later)
env = Robot(x0[0], x0[1], x0[2], dt=dt)

N = 10
beta = 5
trial = 2

nx = 3
mpc = MPC(N,dt)
real_trajectory = {'x': [], 'y': [], 'z': [], 'theta': []}
uStore = []
error_t = np.zeros((N,nx))
x_error = []
y_error = []
theta_error = []
Ps,Ks = find_P(linear_models[0], linear_models[1], mpc.Q, mpc.R)
for i in range(int(T/dt)-N):
    # Find the new linearisation (from current step to current step + N
    Ads = linear_models[0][i:i+N]
    Bds = linear_models[1][i:i+N]
    #mpc.P = beta*Ps[i + N - 1]  # set the terminal cost
    K = Ks[i]
    # Calculate the new errors (current pose vs reference pose)
    error_t[:,:2] = np.array([(x0[:2] - Xref[i+k,:2]) for k in range(N)])
    error_t[:,2] = np.array([wrapAngle(x0[2]) - Xref[i+k,6] for k in range(N)])
    # Wrap the error too (otherwise there is a huge between -pi and pi)
    error_t[:,2] = wrapAngle(error_t[:,2])
    # mpc.theta_err = error_t[2]
    start = Xref[0,0:2]
    goal = Xref[-1,0:2]
    # mpc.collision_avoidance(x0, Xref[i:i+N],start,goal,obs,i)

# =============================================================================
#             if (e.T@mpc.obstacle['Q'][k]@e + mpc.obstacle['l'][k].T@e <= mpc.obstacle['r'][k]) == False:
#                 break_loop = True
#             else:
#                 break_loop = False
# =============================================================================
    #if break_loop:
        #break
    # Solve the MPC problem:
    #control = mpc.control(error_t[0,:], Ads, Bds,x0,Xref[i:i+N])
    # Extract the first control input (for error correction) and add the reference input (for trajectory tracking)
    u = K @ error_t[0,:] + Uref[i,:]
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


    x_error.append(error_t[0,0])
    y_error.append(error_t[0,1])
    theta_error.append(error_t[0,2])
    
#print(mpc)
# plot the robot position
xPos = np.array(real_trajectory['x'])
yPos = np.array(real_trajectory['y'])
fig1, ax1 = plt.subplots()

#===================================================================
# plot of constraints and obstacles
circle = plt.Circle((obs[0],obs[1]),0.5,color='m',alpha=0.2)
#ax1.add_patch(circle)


line3 = ax1.plot(Xref[:, 0], Xref[:, 1], 'g', label='Reference Trajectory')
line4 = ax1.plot(xPos, yPos, 'r',label='Real Trajectory')
ax1.legend()
# plot the error
x_error = np.array(x_error)
y_error = np.array(y_error)
theta_error = np.array(theta_error)
#
#
#
#
# Save variables to .mat:

# Change save_var to True if you want to save the variables to a .mat file. Change trial number
# if you want to save multiple trials.
save_var = True


if save_var:
    data = {"x": real_trajectory['x'], "y":  real_trajectory['y'], "theta":  real_trajectory['theta'],
            "x_error": x_error, "y_error": y_error, "theta_error": theta_error,
            "ref_x": Xref[:,0], "ref_y": Xref[:,1], "ref_theta": Xref[:,-1],
            "uStore": uStore, "uref": Uref}
    savemat("experiment_LQR_"+str(trial)+"_data.mat", data)
    
plt.show()
# animation
# plot_single_robot(real_trajectory)
