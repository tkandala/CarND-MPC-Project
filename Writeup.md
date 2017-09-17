# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model Predictive Control

The purpose of this project is to use the Model Predictive Control to drive a car (inside a simulator). You are given a set of coordinates for waypoints that the car needs to go through and the current car position in the map, it's orientation and speed. Using these parameters, we need to calculate the steering angle and the speed with which we can "safely" navigate the car through the track.

## Implementation

### The Model

The implementation of the Controller was done inline with the lecture notes. MPC.cpp and MPC.h are the scripts for implementing the Controller. Inside MPP.cpp, we see the cost function implemented inside FG_eval class that will try to minimize the cost of multiple variables.

#### FG_eval

First we calculate the cost function based on the reference state using Cross-track error (CTE), Error in Orientation and speed. Next we calculate the cost of the actuators using the steering angle and acceleration. Finally, we also try to minimize the cost in the value gap between sequential actuation so the car moves smoothly with no sudden changes in acceleration or steering. All these costs are added so as to minimize these during iterations.

Next, I have selected N = 12 and dt=0.1. I have initially started with N=25 and dt=0.1 as per lecture notes but then realized that the cost function was being minimized with only half the timesteps. I have left the duration same.

Another change inside the FG_eval class is the degree of our state function which in the lecture notes was set to 1 but in our simulation, we set it to 3 - a cubic polynomial to fit curves. This changes the f0 and the psides0 variables.

#### MPC

The MPC class calculates the control set along with setting constraints on the variables that we are trying to calculate. We have a 6-element vector - x,y,psi,v,cte,epsi and 2 actuators - steer angle and throttle.

Since we require just the x & y points of the estimated path and the steer_angle & throttle, we return just these variables to main.cpp

#### main.cpp

To calculate the steer_angle and throttle to feed into the simulator, we need to convert the waypoint coordinates from map coordinate system to vehicle cordinate system using simple coordinate transformation - translation and rotation. By doing this, we simplofy our MPC controller and also feed those coordinates directly to the simulator which expects coordinates in vehicle coordinate system.

At this point, we only need to pass the cte and epsi values to our MPC solver to get our steering_angle and throttle values. Also, we need to make sure that the coefficients of the waypoints are calculated as a 3rd degree polynomial.

### Timestep length and elapsed duration

As discussed earlier, I started with a timestep length N as 25 and duration dt of 0.1, similar to what was used in lecture notes. By decreasing the timestep length N to 12, I haven't noticed any degredation in cost calculations so I left it at 12.

### Polynomial Fitting and MPC Preprocessing

As already discussed earlier, I have used a 3rd order polynomial to fit the waypoints which fits well to points on a curve. We can use a 2nd degree polynomial but the approximation will not be good enough. Going any higher will not improve our approximation compared to a 3rd degree polynomial.

Pre-processing of the waypoints was done by converting them from map coordinate system to vehicle coordinate system. This simplifies our MPC model.

After waypoints conervsion, I have calculated new state variables based on the speed and steer_angle values from the simulator because the state variables from simulator are again in map coordinate system.

### Model Predictive Control with Latency

Latency was introduced during the state variables initiation step before they are fed into the MPC solver. With a latency of 0.1, I have calculated the new state variables as shown in lines 128-132 in main.cpp.

## Final simulation

I have restricted the speed of the vehicle to 15mph (line 24 in MPC.cpp for ref_v variable) so that MPC has enough time to calculate the proper actuator values in time. By increasing the speed, we run into the problem of car running into the sides and I have found that for my controller setup, a speed of 15mph safely maneuver the car in the simulator. 



