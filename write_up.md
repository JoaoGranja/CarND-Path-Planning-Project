# **Path Planning** 

---

**Path Planning Project**

In this project, I will design a path planner in C++ that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

The goals / steps of this project are the following:
* Build a PID controller for the steering angle of the car so that the vehicle successfully drives a lap around the track.
* Describe the effect each of the P, I, D components had in your implementation
* Tune the PID hyperparameters and Describe how the final hyperparameters were chosen
* Summarize the results with a written report


[//]: # (Image References)

[video]: ./video/video.mov "Final video"

## Writeup Report

In this report I will address all steps of this project, explaining my approach and presenting some results obtained.

---
### Step 0 - Build a PID controller for the steering angle of the car so that the vehicle successfully drive a lap around the track

The main goal of this project is to build a PID controller for the steering angle of the car in order that:
* No tire may leave the drivable portion of the track surface;
* The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

For that, the PID class was completed. This class has the following functions:
* Init - Initialize PID control gains
* UpdateError - Update the PID error variables given cross track error
* TotalError - Calculate the total PID error used for determining the steering angle to apply to the car
* Twiddle - Update PID control gains based on cte error

### Step 1 - Describe the effect each of the P, I, D components had in your implementation

The PID controller is composed of three components P, I and D. The general effect and impact of each component on the controller is:

* P component - Increasing the proportional gain (Kp) has the effect of proportionally increasing the control signal for the same level of error. The fact that the controller will "push" harder for a given level of error tends to cause the closed-loop system to react more quickly, but also to overshoot more. Another effect of increasing Kp is that it tends to reduce, but not eliminate, the steady-state error.

* D component - The derivative term to the controller (Kd) adds the ability of the controller to "anticipate" error. With simple proportional control, if Kp is fixed, the only way that the control will increase is if the error increases. With derivative control, the control signal can become large if the error begins sloping upward, even while the magnitude of the error is still relatively small. This anticipation tends to add damping to the system, thereby decreasing overshoot. The addition of a derivative term, however, has no effect on the steady-state error.

* I component - the integral term to the controller (Ki) tends to help reduce steady-state error. If there is a persistent, steady error, the integrator builds and builds, thereby increasing the control signal and driving the error down. A drawback of the integral term, however, is that it can make the system more sluggish (and oscillatory) since when the error signal changes sign, it may take a while for the integrator to "unwind."


In this situation, I didn't notice a clear bias, steady-state error on the car through the simulation, so the I component could take 0 value. The P component has high importance to reduce the cross track error but high value for this term has the disadvantage of overshooting which can result in the car popping up onto ledges. Finally The d component helps the car not to overshoot, which means it counteracts the P component.


### Step 2 - Tune the PID hyperparameters and Describe how the final hyperparameters were chosen

After completing the PID class and understanding the effect of this PID controller term, the next task is to find the optimum control gains of the controller. The initial values for the control gains (Kp, Ki, Kd) were found through trial and error method. First trial was finding a good proportional gain Kp with remaining Ki and Kd with zero values. The goal was to find Kp so that the car starts having an oscillating behaviour.

With Kp = 0.1 the car starts oscillating around the center of the lane line. 

After that I started increasing the Kd to smooth the oscillation. I tried several values and I find Kd = 1 as a good gain for the differential gain. I tested the PID controller with Kd = 3 and Kd = 10 which also works but higher control gains means quick and high steering angle resulting on high 
yaws mainly when we increase the speed of the car.

As explained before for the control gain Ki, I didn't notice a clear bias caused by long time accumulation of cte. So I take Ki = 0.

I implemented a twiddle algorithm adapted to this situation which means the control gains are adapted after 500 ireation. However the track is not constant on one lap (there are curves), so the algorithm is very dependent on initial values and not efficient at finding control gains which minimizes the cte error.

So The final gains are: Kp = 0.1; Ki = 0; Kd = 1;

I included a P controller for speed, with reference speed = 50 mph.

The final video is:

![alt text][video]


