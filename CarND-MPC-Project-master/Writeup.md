# Model Predictive Control

This project implements a model predictive controller for controlling the car in simulator.
The simulator sends telemetry information to the MPC.The MPC is returning the steering angle and throttle to control the vehicle.

## Compilation

*Your code should compile.*

Code must compile without errors with cmake and make.

Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

## Implementation

### The Model

The model is implemented using kinematics equations.
The below equations were used - 

* x[t+1] = x[t] +v[t] &ast; cos(psi[t]) &ast; dt
* y[t+1] = y[t] +v[t] &ast; sin(psi[t]) &ast; dt
* psi[t+1] = psi[t] + v[t] &ast; delta[t]/Lf &ast;  dt
* v[t+1] = v[t] + a[t] &ast; dt
* cte[t+1] = f(x[t]) - y[t] + v[t] &ast; sin(epsi[t]) &ast; dt
* epsi[t+1] = psi[t] - psides[t] + v[t] &ast; delta[t] / Lf &ast; dt

The equations defines the state of the vehicle where x & y are the coordinates of the vehicle.
psi is the heading/direction of the car.
v is the velocity of the car.
cte and epsi are the cross track error and orientation error.
a is the accleration and delta is the steering angle of the vehicle.

Then a cost function is defined and our goal is to have an accleration and steering angle  with minimum cost .
The cost function is a sum of following values:
* Square of cross track error.
* Square of orientation error.
* Square of steering angle and accleration.
* Square of difference in two consecutive steering angles and accleration.

Each of the above values can be penalized for smoother turns and avoid overshooting.
I have penalized steering angle and difference in steering angles more for smoother steering transistions.
The cost function parameters are tuned by trial and error.Refer to line 61-76 in mpc.cpp for cost function.

### Timestep Length and Elapsed Duration (N & dt)

* N = 10
* t = 0.1

N and dt defines the horizon of the predicted values.
I have tried with different values of N and dt like (20,0.1) ,(10,0.5), (10,0.7).In all the cases,the mpc was not working properly. Finally I settled for the values (10,0.1) mentioned in classroom material and it worked well for my model.

### Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are converted to vehicle coordinate system.
A 3rd degree polynomial is fitted to the transformed waypoints.The coeffecients are used by the solver and to create reference trajectory.

### Model Predictive Control with Latency

To deal with latency, instead of applying actuators from previous timestep,they are applied from another timestep later.The equations are altered for this.Refer to lines 118-121 in mpc.cpp.


