# PID Control Framework
This is a framework extended from the PID module proposed in [jckantor/CBE30338 Chemical Process Control](https://github.com/jckantor/CBE30338.git).
Before using this repository, it is recommended to take a look at [Chapter 4.0 PID Control](https://jckantor.github.io/CBE30338/04.00-PID_Control.html).

This repo allows you to have a PID-based control strategy over your connected devices.
The main purpose of this project is to reach steady-states of coupled processes.
Hence, some PID controllers are coupled and some are presented in a cascade control loop. 
#### PID loops
![pid_img_0](https://i.imgur.com/rHLQgGv.png)

Besides some basic parameters like Kp, Ki, and kd, other parameters are introduced to have a more comprehensive control strategy. In this PID control framework, you could adjust the following parameters. (You could locate these parameters in the code file, [PIDsim.py](https://github.com/PeterTsungYu/dev_iot/blob/dev/PIDsim.py).)
- Setpoint (SP)
- Minimum Manipulated Value (MVmin)
- Maximum Manipulated Value (MVmax)
- Proportional gain (Kp)
- Integral gain (Ki)
- Derivative gain (Kd)
- Proportional setpoint weight (beta)
- Derivative setpoint weight (gamma)
- Direction of Gain (DirectAction)
- Auto or manual (mode) 
- Response time (tstep)
- Increment of each step (SP_increment)
- Margin of steady-state (SP_range)
- Magnitude of responding kick (kick)

> If manual mode is on, the setpoint tracking is also activated to make SP tracks PV along the time.

## Examples
![pid_img_1](https://i.imgur.com/rvOjZq2.gif)
![pid_img_2](https://i.imgur.com/a2os7RM.gif)


