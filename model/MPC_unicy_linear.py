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

class MPC():
    def __init__(self, N):
        self.dt = 5e-3
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

    def set_up_solver(self):
        for i in range(self.N):
            self.stages.dims[i]['n'] = self.nx + self.nu  # number of stage variables
            self.stages.dims[i]['r'] = self.nx  # number of equality constraints
            self.stages.dims[i]['l'] = 0  # nx + nu  # number of lower bounds
            self.stages.dims[i]['u'] = 0  # nx + nu  # number of upper bounds

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
        # define the output
        self.stages.newOutput('output', range(1, 11), range(1, self.nu + self.nx + 1))
        # # Set up the D matrix and c1 = -A*x0 as varying parameters:
        # stages.newParam(('D_current'+str(i)), i, 'eq.D')

        # solver settings
        self.stages.codeoptions['name'] = 'MPC_Project_FORCESPRO'
        self.stages.codeoptions['printlevel'] = 2
        self.stages.generateCode()

    def control(self, state, Ads, Bds):
        problem = {"xinit": -state}  # eq.c = -xinit
        # set up linearized models as equality constraints
        for i in range(self.N - 1):
            A = Ads[i]
            B = Bds[i]
            problem["linear_model"+str(i+1)] = np.hstack((B, A))
        self.output = self.solver.MPC_Project_FORCESPRO_solve(problem)[0]['output']
        control = self.output[:2]

        return control

# #=======================================================
# just for testing, remove later
from MPC_utils import *
Xref = traj_generate(10000, 10)
Uref = get_ref_input(Xref)
linear_models = linearize_model(Xref, Uref, 1e-3)
Ads = linear_models[0][:10]
Bds = linear_models[1][:10]
# #=========================================================

mpc = MPC(10)
control = mpc.control(np.array([1, 0, -np.pi/2]), Ads, Bds)
print(mpc.output)


