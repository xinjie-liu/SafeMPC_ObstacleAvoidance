# MPC_final_project
Final project of course SC42125 Model Predictive Control
## 1. Video
[our project video can be found here](https://www.youtube.com/watch?v=nYDxWkKvzZ8)
## 2. How to run the files:

1. Make sure you have FORCESPRO installed and in the python PATH (tested on FORCESPRO 5.1.1).

2. There are multiple simulations you can run:

```./controller/MPC_unicy_global_linear_.py ``` contains an MPC controller + simulation for the global model of the unicycle, linearised about the trajectory. You can choose parameters such as trajectory, horizon and Q/R costs for the controller. You can also set it to avoid a static obstacle (how to do that is explained in the code comments) using the two linear constraints.

```./controller/MPC_unicy_global_linear_LQR.py ``` is the LQR controller, which is unconstrained and solves the DARE to find the K that follows the pre-defined trajectory.

```./controller/global_ORCA.py ``` is the two robot simulation where one of the robots avoids the other one by using a velocity constraint. Run this file to see how the controller performs. You can change the parameters for the trajectories to evaluate the performance on a shared or intersecting path.

```./model/MPC_utils.py ``` Open this file, uncomment the section at the bottom and run the file to see an example of how the outer approximation of the terminal set is calculated and plotted.
