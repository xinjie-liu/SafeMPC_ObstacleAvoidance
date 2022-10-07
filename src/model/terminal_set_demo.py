from src.model.MPC_utils import *
# Run for an example of how the polyhedral terminal set is plotted

plt.close("all")
dt = 1e-2
Q = .01*np.diag([4, 4, 0.1])
R = .001*np.eye(2)


Xref = traj_generate(10000, 10)
#Xref = line_traj_generate([0.,0.,0], [10.,10.,0.], 10000,dt)

Uref = get_ref_input(Xref)
linear_models = linearize_model_global(Xref, Uref, dt)
# Choose how many linearisations to use for the terminal sets:
Ads = linear_models[0][:10]
Bds = linear_models[1][:10]


polyhedron,cs = find_final_terminal_set(Ads, Bds, Q, R)
#print("Final sublevel terminal set is at c = " + str(cs[-1]))

plot = True

terminalSet = 0 # Choose which set to plot:
if plot:
    plot_terminal_set(polyhedron[terminalSet],cs[terminalSet])
    print('C = '+ str(cs[terminalSet]))