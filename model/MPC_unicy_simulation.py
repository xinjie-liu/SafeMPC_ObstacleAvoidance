
# Starting point of the code
def main():
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

    env = Robot(-2, 0, 0)

    # simulate
    import MPC_Project_FORCESPRO_py

    problem = MPC_Project_FORCESPRO.MPC_Project_FORCESPRO_params

    real_trajectory = {'x': [], 'y': [], 'z': [], 'theta': []}
    T = 30
    dt = 1e-3

    refTraj = generateReferenceTrajectory()

    for k in range(T/dt):

        A, B = linearize_model(refTraj[k:k + N])

        for i in range(N): # Define the D =  (B | A) matrix over the whole horizon
            problem[('D_current'+str(i))] = np.hstack(B[i], A[i])

        state = env.step(v, w)
        print(env.current)
        real_trajectory['x'].append(state.x)
        real_trajectory['y'].append(state.y)
        real_trajectory['z'].append(0)
        real_trajectory['theta'].append(state.theta)

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
    ax1.set_xlim(-5., 5.)
    ax1.set_ylim(-5., 5.)
    ax1.set_zlim(0., 3.)
    ax1.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig,
                                  func=animate,
                                  frames=len(real_trajectory['x']),
                                  interval=50,
                                  repeat=False,
                                  blit=False)
    plt.show()


if __name__ == "__main__":
    main()