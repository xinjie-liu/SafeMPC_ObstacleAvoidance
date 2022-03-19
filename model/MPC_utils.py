import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation



def plot_single_robot(real_trajectory):
    def animate(i):
        line.set_xdata(real_trajectory['x'][:i + 1])
        line.set_ydata(real_trajectory['y'][:i + 1])
        line.set_3d_properties(real_trajectory['z'][:i + 1])
        point.set_xdata(real_trajectory['x'][i])
        point.set_ydata(real_trajectory['y'][i])
        point.set_3d_properties(real_trajectory['z'][i])

        heading.set_xdata(
            [real_trajectory['x'][i], real_trajectory['x'][i] + 0.8 * np.cos(real_trajectory['theta'][i])])
        heading.set_ydata(
            [real_trajectory['y'][i], real_trajectory['y'][i] + 0.8 * np.sin(real_trajectory['theta'][i])])
        return ax1

    # plotting stuff
    fig = plt.figure()
    ax1 = p3.Axes3D(fig)  # 3D place for drawing
    real_trajectory['x'] = np.array(real_trajectory['x'])
    real_trajectory['y'] = np.array(real_trajectory['y'])
    real_trajectory['z'] = np.array(real_trajectory['z'])
    point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'ro',
                      label='Robot', markersize=15)

    heading, = ax1.plot([real_trajectory['x'][0], real_trajectory['x'][0] + 0.8 * np.cos(real_trajectory['theta'][0])], \
                        [real_trajectory['y'][0], real_trajectory['y'][0] + 0.8 * np.sin(real_trajectory['theta'][0])],
                        [real_trajectory['z'][0]], 'b')

    line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]],
                     label='Real_Trajectory')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.set_title('3D animate')
    ax1.set_xlim(-15., 15.)
    ax1.set_ylim(-15., 15.)
    ax1.set_zlim(0., 3.)
    ax1.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig,
                                  func=animate,
                                  frames=len(real_trajectory['x']),
                                  interval=50,
                                  repeat=False,
                                  blit=False)
    plt.show()

def plot_multi_robot(real_trajectory):

    def animate(i):
        line.set_xdata(real_trajectory['x1'][:i + 1])
        line.set_ydata(real_trajectory['y1'][:i + 1])
        line.set_3d_properties(real_trajectory['z1'][:i + 1])
        point.set_xdata(real_trajectory['x1'][i])
        point.set_ydata(real_trajectory['y1'][i])
        point.set_3d_properties(real_trajectory['z1'][i])
        line2.set_xdata(real_trajectory['x2'][:i + 1])
        line2.set_ydata(real_trajectory['y2'][:i + 1])
        line2.set_3d_properties(real_trajectory['z2'][:i + 1])
        point2.set_xdata(real_trajectory['x2'][i])
        point2.set_ydata(real_trajectory['y2'][i])
        point2.set_3d_properties(real_trajectory['z2'][i])

        # heading.set_xdata(
        #     [real_trajectory['x1'][i], real_trajectory['x1'][i] + 0.8 * np.cos(real_trajectory['theta1'][i])])
        # heading.set_ydata(
        #     [real_trajectory['y1'][i], real_trajectory['y1'][i] + 0.8 * np.sin(real_trajectory['theta1'][i])])
        return ax1

    # plotting stuff
    fig = plt.figure()
    ax1 = p3.Axes3D(fig)  # 3D place for drawing
    real_trajectory['x1'] = np.array(real_trajectory['x1'])
    real_trajectory['y1'] = np.array(real_trajectory['y1'])
    real_trajectory['z1'] = np.array(real_trajectory['z1'])
    real_trajectory['x2'] = np.array(real_trajectory['x2'])
    real_trajectory['y2'] = np.array(real_trajectory['y2'])
    real_trajectory['z2'] = np.array(real_trajectory['z2'])
    point, = ax1.plot([real_trajectory['x1'][0]], [real_trajectory['y1'][0]], [real_trajectory['z1'][0]], 'ro',
                      label='Robot1', markersize=10)
    point2, = ax1.plot([real_trajectory['x2'][0]], [real_trajectory['y2'][0]], [real_trajectory['z2'][0]], 'bo',
                      label='Robot2', markersize=10)

    # heading, = ax1.plot([real_trajectory['x1'][0], real_trajectory['x1'][0] + 0.8 * np.cos(real_trajectory['theta1'][0])], \
    #                     [real_trajectory['y1'][0], real_trajectory['y1'][0] + 0.8 * np.sin(real_trajectory['theta1'][0])],
    #                     [real_trajectory['z1'][0]], 'b')

    line, = ax1.plot([real_trajectory['x1'][0]], [real_trajectory['y1'][0]], [real_trajectory['z1'][0]],
                     label='Real_Trajectory1')
    line2, = ax1.plot([real_trajectory['x2'][0]], [real_trajectory['y2'][0]], [real_trajectory['z2'][0]],
                     label='Real_Trajectory2')

    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.set_title('3D animate')
    ax1.set_xlim(-15., 15.)
    ax1.set_ylim(-15., 15.)
    ax1.set_zlim(0., 3.)
    ax1.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig,
                                  func=animate,
                                  frames=len(real_trajectory['x1']),
                                  interval=5,
                                  repeat=False,
                                  blit=False)
    plt.show()

def line_traj_generate(start, goal, total_step):  # start: (x, y, theta)
    total_step = int(total_step)
    # xr yr xrdot yrdot xrddot yrddot theta
    x = np.zeros([total_step, ])
    y = np.zeros([total_step, ])
    x[0] = start[0]
    y[0] = start[1]
    x_interval = (goal[0] - start[0]) / (total_step - 1)
    y_interval = (goal[1] - start[1]) / (total_step - 1)
    for i in range(total_step - 1):
        x[i + 1] = start[0] + i * x_interval
        y[i + 1] = start[1] + i * y_interval
    xdot = np.diff(x)[1] * np.ones(len(x)) / 1e-3  # constant velocity
    ydot = np.diff(y)[1] * np.ones(len(y)) / 1e-3
    xddot = np.zeros(len(x))
    yddot = np.zeros(len(y))
    theta = np.arctan2(ydot, xdot)

    return np.array([x, y, xdot, ydot, xddot, yddot, theta]).T

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
#=================================================================
    # linearized model in robot local frame
    Ad = np.zeros([Xref.shape[0], 3, 3])
    Bd = np.zeros([Xref.shape[0], 3, 2])
    for i in range(Xref.shape[0]):
        Ad[i] = np.array([[1, Uref[i, 1]*dt, 0], [-Uref[i, 1]*dt, 1, Uref[i, 0]*dt], [0, 0, 1]])
        Bd[i] = np.array([[-dt, 0], [0, 0], [0, -dt]])
#=================================================================

    return Ad, Bd

def linearize_model_global(Xref, Uref, dt):
    # linearized model in global inertial frame
    Ad = np.zeros([Xref.shape[0], 3, 3])
    Bd = np.zeros([Xref.shape[0], 3, 2])
    for i in range(Xref.shape[0]):
        Ad[i] = np.array([[1, 0, -Uref[i, 0]*np.sin(Xref[i,-1])*dt], [0, 1, Uref[i, 0]*np.cos(Xref[i,-1])*dt], [0, 0, 1]])
        Bd[i] = np.array([[np.cos(Xref[i,-1])*dt, 0], [np.sin(Xref[i,-1])*dt, 0], [0, dt]])

    return Ad, Bd

def wrapAngle(angle):
    return  np.arctan2(np.sin(angle), np.cos(angle))

# Xref = traj_generate(10000, 10)
# Uref = get_ref_input(Xref)
# linear_models = linearize_model(Xref, Uref, 1e-3)
# Ads = linear_models[0][:10]
# Bds = linear_models[1][:10]
# print(linear_models)
