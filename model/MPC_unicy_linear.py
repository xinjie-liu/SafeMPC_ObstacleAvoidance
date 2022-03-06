"""
File containing the class definition of the Model Predictive Controller
"""

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from numpy.linalg import inv
from matplotlib import animation
from unicycle import Robot
import forcespro
import get_userid
from quadrotor import Quadrotor
import casadi

"""
Parameters of the class
"""
# class MPC():
#     def __init__(self, N):
#         self.dt = 5e-3
#         self.N = N  # planning horizon
#         self.stages = forcespro.MultistageProblem(N)  # create the stages for the whole finite horizon
#         self.nx = 3
#         self.nu = 2

dt = 1e-3
N = 10  # planning horizon
stages = forcespro.MultistageProblem(N)  # create the stages for the whole finite horizon
nx = 3
nu = 2
Q = 100*np.diag([4, 40, 0.1])
R = np.eye(nu)/100
P = 0*Q

#=======================================================
# just for testing, remove later
from MPC_utils import *
Xref = traj_generate(10000, 10)
Uref = get_ref_input(Xref)
linear_models = linearize_model(Xref, Uref, 1e-3)
Ads = linear_models[0][:10]
Bds = linear_models[1][:10]
#=========================================================

for i in range(N):
    A = Ads[i]
    B = Bds[i]

    stages.dims[i]['n'] = nx + nu  # number of stage variables
    stages.dims[i]['r'] = nx  # number of equality constraints
    stages.dims[i]['l'] = 0 # nx + nu  # number of lower bounds
    stages.dims[i]['u'] = 0 # nx + nu  # number of upper bounds

    # Cost/Objective function
    # V = sum_i(z(i)*H*z(i)) + z(N)*H*z(N) -> where z(i) = [u1,u2,x1,x2] at stage/step i.
    if i == N-1:
        # For xN use the terminal cost P to penalise the xN state
        stages.cost[i]['H'] = np.vstack((np.hstack((R, np.zeros((nu, nx)))), np.hstack((np.zeros((nx, nu)), P))))
    else:
        # For i from 0 to N-1 use the stage cost Q to penalise the state
        stages.cost[i]['H'] = np.vstack((np.hstack((R, np.zeros((nu, nx)))), np.hstack((np.zeros((nx, nu)), Q))))

    stages.cost[i]['f'] =  np.zeros((nx+nu,1)) # there's no linear fT*z term in the cost, so set to 0

    # Equality constraints (expressed in the form of C*z(i) + D*z(i+1) = 0, where C = ( B(i) | A(i) ), D = (0 | -I)
    if (i < N - 1):
        stages.eq[i]['C'] = np.hstack((B, A))
    if (i > 0):
        stages.eq[i]['c'] = np.zeros((nx, 1))

    stages.eq[i]['D'] = np.hstack((np.zeros([nx, nu]), -np.eye(nx)))

# parameter: initial state
stages.newParam("xinit", [1], 'eq.c')  # 1-indexed
# define the output
stages.newOutput('control', range(1, 11), range(1, nu + nx + 1))
# # Set up the D matrix and c1 = -A*x0 as varying parameters:
# stages.newParam(('D_current'+str(i)), i, 'eq.D')

# solver settings
stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
stages.codeoptions['printlevel'] = 2
stages.generateCode()

import MPC_Project_FORCESPRO_py  # notice the _py suffix
problem = {"xinit": np.array([1, 0, -np.pi/2])}  # a dictionary of solver parameters
output = MPC_Project_FORCESPRO_py.MPC_Project_FORCESPRO_solve(problem)[0]['control']

xinit = output[2:5]
problem = {"xinit": xinit}
output2 = MPC_Project_FORCESPRO_py.MPC_Project_FORCESPRO_solve(problem)[0]['control']
print(output2)
