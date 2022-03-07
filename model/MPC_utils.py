import numpy as np

def line_traj_generate(start, goal, total_step): # start: (x, y, theta)
    x = np.zeros([total_step+1, ])
    y = np.zeros([total_step+1, ])
    theta = np.zeros([total_step+1, ])
    x_interval = (goal[0] - start[0])/total_step
    y_interval = (goal[1] - start[1])/total_step
    theta_interval = (goal[2] - start[2])/total_step
    for i in range(total_step+1):
        x[i] = start[0] + i * x_interval
        y[i] = start[1] + i * y_interval
        theta[i] = start[2] + i * theta_interval
    return x, y, theta

def traj_generate(total_step, T):
    t_interval = T/total_step
    t = np.arange(0, T, t_interval)
    # xr yr xrdot yrdot xrddot yrddot theta
    Xref = np.array([np.cos(t), np.sin(3*t), -np.sin(t), 3*np.cos(3*t), -np.cos(t), \
                     -9*np.sin(3*t), np.arctan2(3*np.cos(3*t), -np.sin(t))])
    return Xref.T

def get_ref_input(Xref):
    xrdot = Xref[:, 2]
    yrdot = Xref[:, 3]
    xrddot = Xref[:, 4]
    yrddot = Xref[:, 5]

    Uref = np.array([np.sqrt(xrdot ** 2 + yrdot ** 2), (xrdot * yrddot - yrdot * xrddot) / (xrdot ** 2 + yrdot ** 2)])

    return Uref.T

def linearize_model(Xref, Uref, dt):
    Ad = np.zeros([Xref.shape[0], 3, 3])
    Bd = np.zeros([Xref.shape[0], 3, 2])
    for i in range(Xref.shape[0]):
        Ad[i] = np.array([[1, Uref[i, 1]*dt, 0], [-Uref[i, 1]*dt, 1, Uref[i, 0]*dt], [0, 0, 1]])
        Bd[i] = np.array([[-dt, 0], [0, 0], [0, -dt]])
    return Ad, Bd
def wrapAngle(angle):
    return  np.arctan2(np.sin(angle), np.cos(angle))
    

# Xref = traj_generate(10000, 10)
# Uref = get_ref_input(Xref)
# linear_models = linearize_model(Xref, Uref, 1e-3)
# Ads = linear_models[0][:10]
# Bds = linear_models[1][:10]
# print(linear_models)
