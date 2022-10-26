# Safe Model Predictive Control Approach for Non-holonomic Mobile Robots in Dynamic Environment

This is a MPC project for motion planning & control of non-holonomic mobile robots

Have fun~~

### :smirk:  
#### :smirk:
##### :smirk:

## 1. Video & Paper

[our project video can be found here](https://www.youtube.com/watch?v=nYDxWkKvzZ8)

[our paper (preprint) can be found here](https://arxiv.org/abs/2207.12878)

## 2. How to run the files:

1. In order to run the files, please make sure you have FORCESPRO installed and in the python PATH (tested on FORCESPRO 5.1.1).

2. There are multiple simulations you can run:

```./controller/MPC_static_obstacle.py ``` contains an MPC controller + simulation for the global model of the unicycle, linearised about the trajectory. You can choose parameters such as trajectory, horizon and Q/R costs for the controller. You can also set it to avoid a static obstacle (how to do that is explained in the code comments) using the two linear constraints.

```./controller/LQR.py ``` is the LQR controller, which is unconstrained and solves the DARE to find the K that follows the pre-defined trajectory.

```./controller/MPC_dynamic_obstacle.py ``` is the two robots simulation where one of the robots avoids the other one by using a velocity constraint. Run this file to see how the controller performs. You can change the parameters for the trajectories to evaluate the performance on a shared or intersecting path.

```./model/MPC_utils.py ``` Open this file, uncomment the section at the bottom and run the file to see an example of how the outer approximation of the terminal set is calculated and plotted.
