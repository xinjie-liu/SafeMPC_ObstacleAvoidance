# MPC_final_project
Final project of course SC42125 Model Predictive Control
## 1. report
[our report can be found here](https://www.overleaf.com/3292615562fwdtpjvbbnwg)
## 2. some references (quadrotor)
* [(only an intuition about the proof process) A guiding vector field algorithm for path following, where they proved the existence of Lyapunov function](https://www.researchgate.net/publication/309191959_A_Guiding_Vector-Field_Algorithm_for_Path-Following_Control_of_Nonholonomic_Mobile_Robots/figures?lo=1)
* [(only an intuition about the task) High MPC: an interesting dynamic obstacle avoidance task for quadrotor](https://github.com/uzh-rpg/high_mpc)
* [An overview of MPC in quadrotor control](https://arxiv.org/pdf/2109.01365.pdf)
* [Non-linear MPC for quadrotor design, linearization, discretization](https://www.researchgate.net/profile/Mina-Samir-Kamel/publication/311545161_Model_Predictive_Control_for_Trajectory_Tracking_of_Unmanned_Aerial_Vehicles_Using_Robot_Operating_System/links/59682043458515e9af9eba66/Model-Predictive-Control-for-Trajectory-Tracking-of-Unmanned-Aerial-Vehicles-Using-Robot-Operating-System.pdf)
* [I don't know whether this could be useful but: parammetric MPC (some state variables of the quadrotor as parameters of others to reduce complexity of NLP)](https://journals-sagepub-com.tudelft.idm.oclc.org/doi/pdf/10.1177/0959651819847053)
* [maybe helpful for writing report: NMPC formulation for obstacle avoidance](https://arxiv.org/pdf/1812.04755.pdf)
* [might helps for searching for resources (especially analysis of linearized system): A survey of MPC in quadrotor](https://arxiv.org/pdf/2011.11104.pdf)
* [A cool group who did a lot of drone researches](https://rpg.ifi.uzh.ch/aggressive_flight.html)
## 3. some references (unicycle)
* [unicycle MPC (including stability analysis)](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7312445)
* [unicycle: a survey](https://www.cambridge.org/core/journals/robotica/article/nonholonomic-mobile-robots-trajectory-tracking-model-predictive-control-a-survey/6C977C1B5A025C8427F3FD3C7F63EE20)
* [distributed MPC](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8920881)
* [linearization of unicycle model](http://www.ece.ufrgs.br/~fetter/mechrob04_553.pdf)
## 4. collision avoidance
* [ellipse dynamic obstacle modelling and repulsive field by Bruno Brito](https://arxiv.org/pdf/2010.10190.pdf)
* [decentralized NMPC, with ellipse collision constraints (references of this paper may be useful)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8550178)
* [separating hyper-planes by Laura Ferranti](https://arxiv.org/pdf/2006.11492.pdf)
* [a linearization of nonlinear collision constraints: using first order Taylor approximation](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8550245)
## 5. experiments
* horizon length
* terminal set & terminal cost
* RoA
* input constraints
* old Q,R without terminal cost V.S. new Q, R (compare control input)
## 6. report fine tunning
* vector notations
* numerical results
