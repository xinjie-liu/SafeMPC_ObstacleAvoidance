import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import scipy.linalg as la
from control import dare
from matplotlib.patches import Ellipse
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
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
                      label='Robot', markersize=7)

    heading, = ax1.plot([real_trajectory['x'][0], real_trajectory['x'][0] + 0.8 * np.cos(real_trajectory['theta'][0])], \
                        [real_trajectory['y'][0], real_trajectory['y'][0] + 0.8 * np.sin(real_trajectory['theta'][0])],
                        [real_trajectory['z'][0]], 'b')

    line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]],
                     label='Real Trajectory')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.set_title('3D animate')
    ax1.set_xlim(-5., 12.)
    ax1.set_ylim(-5., 12.)
    ax1.set_zlim(0., 1.)
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
                      label='Robot 1', markersize=7)
    point2, = ax1.plot([real_trajectory['x2'][0]], [real_trajectory['y2'][0]], [real_trajectory['z2'][0]], 'bo',
                      label='Robot 2', markersize=7)

    # heading, = ax1.plot([real_trajectory['x1'][0], real_trajectory['x1'][0] + 0.8 * np.cos(real_trajectory['theta1'][0])], \
    #                     [real_trajectory['y1'][0], real_trajectory['y1'][0] + 0.8 * np.sin(real_trajectory['theta1'][0])],
    #                     [real_trajectory['z1'][0]], 'b')

    line, = ax1.plot([real_trajectory['x1'][0]], [real_trajectory['y1'][0]], [real_trajectory['z1'][0]], 'r',
                     label='Real Trajectory R1')
    line2, = ax1.plot([real_trajectory['x2'][0]], [real_trajectory['y2'][0]], [real_trajectory['z2'][0]],'b',
                     label='Real Trajectory R2')

    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.set_title('3D animate')
    ax1.set_xlim(-5., 12.)
    ax1.set_ylim(-5., 12.)
    ax1.set_zlim(0., 1.)
    ax1.legend(loc='lower right')
    ax1.view_init(elev=60.)
    ani = animation.FuncAnimation(fig=fig,
                                  func=animate,
                                  frames=len(real_trajectory['x1']),
                                  interval=5,
                                  repeat=False,
                                  blit=False)
    #fig.suptitle('Intersecting Trajectory With Terminal Cost', fontsize=14)
    writervideo = animation.FFMpegWriter(fps=60)
    ani.save('results.mp4', writer=writervideo, dpi=300)
    print('Animation')
    plt.show()
    fig.savefig('results_VO_non-conservative_terminal_3d.jpg',dpi=720)
def line_traj_generate(start, goal, total_step, dt):  # start: (x, y, theta)
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
    #xdot = np.sign(np.diff(x)[1])*0.5 * np.ones(len(x))  # constant velocity
    xdot = np.diff(x)[1]*np.ones(len(x))/(dt)
    #ydot = np.sign(np.diff(y)[1])*0.5 * np.ones(len(y))
    ydot = np.diff(y)[1]*np.ones(len(y))/(dt)
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
    Ad = np.zeros([Xref.shape[0], 3, 3])
    Bd = np.zeros([Xref.shape[0], 3, 2])
    for i in range(Xref.shape[0]):
        Ad[i] = np.array([[1, Uref[i, 1]*dt, 0], [-Uref[i, 1]*dt, 0, Uref[i, 0]*dt], [0, 0, 1]])
        Bd[i] = np.array([[-dt, 0], [0, 0], [0, -dt]])
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

def solve_dare(Ads, Bds, Q, R):
    P = np.zeros((len(Ads), *Q.shape))
    for i in range(len(Ads)):
        P[i] = la.solve_discrete_are(Ads[i], Bds[i], Q, R)

    return P

def find_P(Ads,Bds,Q,R):
    n = len(Ads)
    
    P = np.zeros((n,*Q.shape))
    K = np.zeros((n,Bds.shape[-1],Ads.shape[1]))
    P[-1],_,K[-1] = dare(Ads[-1],Bds[-1],Q,R)
    K[-1] = -K[-1]
    for i in range(n-1):
        _,_,Ki = dare(Ads[n-2-i],Bds[n-2-i],Q,R)
        Ki = -Ki
        # print(Ads[n-2-i].shape)
        # print(Bds[n-2-i].shape)
        # print(K)
        Q_k = Q + Ki.T @ R @ Ki
        A_k = Ads[n-2-i] + Bds[n-2-i] @ Ki
        P[n-2-i] = A_k.T @ P[n-1-i] @ A_k  + Q_k
        K[n-2-i] = Ki
    return P,K

def find_Terminal_set(c,P,K):
    lambd,v = la.eig(P)
    lambd = np.real(lambd)
    r = lambd*c
    origin = np.array([0,0,0])
    vertex = np.zeros([8,3])
    # Bottom 4 vertices
    vertex[0,:] = origin + r[0]*v[0] - r[2]*v[2]
    vertex[1,:] = origin + r[1]*v[1] - r[2]*v[2]
    vertex[2,:] = origin - r[0]*v[0] - r[2]*v[2]
    vertex[3,:] = origin - r[1]*v[1] - r[2]*v[2]
    # Top 4 vertices
    vertex[4,:] = origin + r[0]*v[0] + r[2]*v[2]
    vertex[5,:] = origin + r[1]*v[1] + r[2]*v[2]
    vertex[6,:] = origin - r[0]*v[0] + r[2]*v[2]
    vertex[7,:] = origin - r[1]*v[1] + r[2]*v[2]
    
    constraint = True 

    for vert in vertex:
        if np.linalg.norm(K@vert) > 10: # 10 is the constraint on u
            constraint = False
    return vertex, constraint


def find_final_terminal_set(Ads, Bds, Q, R):
    P,K = find_P(Ads, Bds, Q, R)
    c = 100 # Start with some large c
    for i in range(len(P)): # Iterate through all the terminal costs along the reference trajectory
        feasible = False
        while feasible == False: # If the set is infeasible reduce c by a small factor and try again
            c = c/1.01
            vertices, feasible = find_Terminal_set(c, P[i], K[i])
# =============================================================================
#     # Test that it works:
#     for i in range(len(P)):
#         vertices, feasible = find_Terminal_set(c, P[i], K[i])
#         print(feasible)
# =============================================================================
    return vertices,c

def plot_terminal_set(vertices):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # From https://stackoverflow.com/questions/63207496/how-to-visualize-polyhedrons-defined-by-their-vertices-in-3d-with-matplotlib-or
    hull = ConvexHull(vertices)
    # draw the polygons of the convex hull
    for s in hull.simplices:
        tri = Poly3DCollection(vertices[s])
        tri.set_color('red')
        tri.set_alpha(0.2)
        tri.set_edgecolor('none')
        ax.add_collection3d(tri)
    # draw the vertices
    ax.scatter(vertices[:4, 0], vertices[:4, 1], vertices[:4, 2], marker='o', color='red') # bottom vertices
    ax.scatter(vertices[4::, 0], vertices[4::, 1], vertices[4::, 2], marker='o', color='blue') # top vertices
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Theta axis')
    
    
    xlim = np.max(np.abs(vertices[:,0]))
    ylim = np.max(np.abs(vertices[:,1]))
    zlim = np.max(np.abs(vertices[:,2]))
    
    ax.axes.set_xlim3d(left=-xlim, right=xlim) 
    ax.axes.set_ylim3d(bottom=-ylim, top=ylim) 
    ax.axes.set_zlim3d(bottom=-zlim, top=zlim) 
    
    plt.show()

plt.close("all")
dt = 1e-2
Q = .01*np.diag([4, 4, 0.1])
R = .0001*np.eye(2)


Xref = traj_generate(10000, 10)
#Xref = line_traj_generate([0.,0.,0], [10.,10.,0.], 10000,dt)

Uref = get_ref_input(Xref)
linear_models = linearize_model_global(Xref, Uref, dt)
Ads = linear_models[0][:10]
Bds = linear_models[1][:10]


vertices,c = find_final_terminal_set(Ads, Bds, Q, R)
print("Final sublevel terminal set is at c = " + str(c))

plot = False
if plot:
    plot_terminal_set(vertices)
    

# =============================================================================
# from matplotlib.patches import Ellipse
# 
# delta = 45.0  # degrees
# 
# angles = np.arange(0, 360 + delta, delta)
# ells = [Ellipse((1, 1), 4, 2, a) for a in angles]
# 
# a = plt.subplot(111, aspect='equal')
# 
# for e in ells:
#     e.set_clip_box(a.bbox)
#     e.set_alpha(0.1)
#     a.add_artist(e)
# 
# plt.xlim(-2, 4)
# plt.ylim(-1, 3)
# 
# plt.show()
# =============================================================================
