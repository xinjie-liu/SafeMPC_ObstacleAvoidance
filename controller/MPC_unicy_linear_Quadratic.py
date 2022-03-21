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
        self.Q = 100*np.diag([4, 40, 0.1])
        self.R = np.eye(self.nu)/100
        self.P = 0 * self.Q
        self.set_up_solver()
        import MPC_Project_FORCESPRO_py
        self.solver = MPC_Project_FORCESPRO_py
        self.n = 1
        self.theta_err = np.zeros((self.n,self.N))
        self.theta = np.zeros((self.n,self.N))
    def set_up_solver(self):
        for i in range(self.N):
            self.stages.dims[i]['n'] = self.nx + self.nu  # number of stage variables
            self.stages.dims[i]['r'] = self.nx  # number of equality constraints
            self.stages.dims[i]['l'] = 2  # nx + nu  # number of lower bounds
            self.stages.dims[i]['u'] = 2  # nx + nu  # number of upper bounds
            # lower bounds
            self.stages.ineq[i]['b']['lbidx'] = np.array([1, 2])  # lower bound acts on these indices
            self.stages.ineq[i]['b']['lb'] = np.array([-10, -10])  # lower bound for this stage variable

            # upper bounds
            self.stages.ineq[i]['b']['ubidx'] = np.array([1, 2])  # upper bound acts on these indices
            self.stages.ineq[i]['b']['ub'] = np.array([10, 10])  # upper bound for this stage variable


            self.stages.dims[ i ]['q'] =1 # number of quadratic constraints
            
            self.stages.ineq[i]['q']['idx'] = np.zeros((1,), dtype=object) # index vectors     (starting at 1)      
            self.stages.ineq[i]['q']['idx'][0] = np.array([4,5]) # Add the constraint to e1,e2

            
            self.stages.ineq[i]['q']['Q'] = np.zeros((1,), dtype=object) # Hessians, only one quadratic constraint
            self.stages.ineq[i]['q']['Q'][0] = 0*np.array([[1.0, 0.], [0., 1.0]]) # square distance between robots

            
            self.stages.ineq[i]['q']['l'] = np.zeros((1,), dtype=object)
            self.stages.ineq[i]['q']['l'][0] = np.array([[0],[0]])
 
            self.stages.ineq[i]['q']['r'] = np.array([[2]])  # RHSs


            # Cost/Objective function
            # V = sum_i(z(i)*H*z(i)) + z(N)*H*z(N) -> where z(i) = [u1,u2,x1,x2] at stage/step i.
            if i == self.N - 1:
                # For xN use the terminal cost P to penalise the xN state
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
            self.stages.newParam("obstacleQ"+str(i+1), [i+1], 'ineq.q.Q',1)
            self.stages.newParam("obstaclel"+str(i+1), [i+1], 'ineq.q.l',1)
            self.stages.newParam("obstacleR"+str(i+1), [i+1], 'ineq.q.r')
        # define the output
        self.stages.newOutput('output', range(1, self.N+1), range(1, self.nu + self.nx + 1))
        # solver settings
        self.stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
        self.stages.codeoptions['printlevel'] = 2
        self.stages.generateCode()

    def collision_avoidance(self, X1, xref):
        # define the hyper-plane for collision avoidance
        x1 = X1[0]
        y1 = X1[1]

        
        distance = np.sqrt((x1 - 5) ** 2 + (y1 - 5) ** 2)
        
        self.theta_ref = xref[:,-1]
        
        # robot orientations calculated from the last time step
        self.theta = wrapAngle(self.theta_ref - self.theta_err)

        ineql = np.zeros((self.N,2,1))
        ineqQ = np.zeros((self.N,2,2))
        ineqr = 1*np.ones((self.N,1))
        
        safety_r =1 # Distance to be kept between the two robots
        
        if distance < 100:

            for i in range(self.N):
                
                #
                cx = 5 - xref[i,0]
                cy = 5 - xref[i,1]
                
                ineql[i] = -np.array([[2*cx],[2*cy]])
    
                ineqQ[i] = -np.array([[1,0],[0,1]])
           
                ineqr[i] = -np.array([[safety_r**2-cx**2-cy**2]])
                
                # Inequality should be in the form=>  -e.T*Q*e - l*e <= -r

        self.obstacle = {"Q": ineqQ, "l": ineql,"r": ineqr}
        
    def control(self, state, Ads, Bds):
        self.problem = {"xinit": -state}  # eq.c = -xinit
        # set up linearized models as equality constraints
        for i in range(self.N - 1):
            A = Ads[i]
            B = Bds[i]
            
            self.problem["linear_model"+str(i+1)] = np.hstack((B, A))
            self.problem["obstacleQ"+str(i+1)] = self.obstacle["Q"][i+1]
            self.problem["obstaclel"+str(i+1)] = self.obstacle["l"][i+1]
            self.problem["obstacleR"+str(i+1)] = self.obstacle["r"][i+1]
            
        self.output = self.solver.MPC_Project_FORCESPRO_solve(self.problem)[0]['output']
        control = self.output[:self.nu]

        # get the angle error for calculating the collision avoidance constraints
        # Extract theta_err for the n robots from the output:
       

        
        
        return control

# #=======================================================
from model.MPC_utils import *
plt.close("all")
T = 10
dt = 1e-2
#Xref = traj_generate(T/dt, T)
Xref = line_traj_generate([0.,0.,0], [10.,10.,0.], T/dt,dt) 
Uref = get_ref_input(Xref)
linear_models = linearize_model_global(Xref, Uref, dt)
# #=========================================================
x0 = np.array([1., 0., 0.]) # This angle needs to be in standard notation (it gets wrapped later)
env = Robot(x0[0], x0[1], x0[2], dt=dt)

N = 10
nx = 3
mpc = MPC(N,dt)
real_trajectory = {'x': [], 'y': [], 'z': [], 'theta': []}
uStore = []
error_t = np.zeros((mpc.N,nx))
x_error = []
y_error = []

for i in range(int(T/dt)-N):
    # Find the new linearisation (from current step to current step + N
    Ads = linear_models[0][i:i+N]
    Bds = linear_models[1][i:i+N]
    # Calculate the new errors (current pose vs reference pose)
    error_t[:,:2] = np.array([(x0[:2] - Xref[i+k,:2]) for k in range(N)])
    error_t[:,2] = np.array([wrapAngle(x0[2]) - Xref[i+k,6] for k in range(N)])
    # Wrap the error too (otherwise there is a huge between -pi and pi)
    error_t[:,2] = (error_t[:,2])
    # mpc.theta_err = error_t[2]
    mpc.collision_avoidance(x0, Xref[i:i+N])
    for k in range(N):
            e = error_t[k,:2].reshape(2,1)
            print(e.T@mpc.obstacle['Q'][k]@e + mpc.obstacle['l'][k].T@e <= mpc.obstacle['r'][k])
            if (e.T@mpc.obstacle['Q'][k]@e + mpc.obstacle['l'][k].T@e <= mpc.obstacle['r'][k]) == False:
                break_loop = True
            else:
                break_loop = False
    if break_loop:
        break
    # Solve the MPC problem:
    control = mpc.control(error_t[0,:], Ads, Bds)
    # Extract the first control input (for error correction) and add the reference input (for trajectory tracking)
    u = mpc.output[0:2] + Uref[i,:]
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
circle = plt.Circle((5,5),0.5,color='m',alpha=0.2)
ax1.add_patch(circle)
# plot the error
x_error = np.array(x_error)
y_error = np.array(y_error)
fig2, ax2 = plt.subplots()
ax2.plot(range(len(x_error)), x_error, 'b')
ax2.plot(range(len(y_error)), y_error, 'g')
plt.show()
# animation
# plot_single_robot(real_trajectory)
