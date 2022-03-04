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
from quadrotor import Quadrotor
import casadi

"""
Parameters of the class
"""
dt = 5e-3
N = 10 # planning horizon
stages = forcespro.MultistageProblem(N) # create the stages for the whole finite horizon

nx = 2
nu = 2

for i in range(N):


    stages.dims[i]['n'] = nx + nu  # number of stage variables
    stages.dims[i]['r'] = nx  # number of equality constraints
    stages.dims[i]['l'] = nx + nu  # number of lower bounds
    stages.dims[i]['u'] = nx + nu  # number of upper bounds

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
        #stages.eq[i]['C'] = np.hstack(B, A)
    if (i > 0):
        stages.eq[i]['c'] = np.zeros((nx, 1))

    stages.eq[i]['D'] = np.hstack((0, -np.eye(nx)))


    # Set up the D matrix and c1 = -A*x0 as varying parameters:
    stages.newParam(('D_current'+str(i)), i, 'eq.D')

    stages.newParam('minusA_times_x0', [1], 'eq.c')
    # Define the output to be u0
    stages.newOutput('u0', 1, range(1,nu+1))

    # solver settings
    stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
    stages.codeoptions['printlevel'] = 0
    import get_userid
    stages.generateCode(get_userid.user_id)