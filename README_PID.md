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
- Margin of steady state (SP_range)
- Magnitude of kicks (kick_prop)

> If manual mode is on, the setpoint tracking is also activated to make SP tracks PV along the time.

#### Example of an user interface for tuning
![pid_img_1](https://i.imgur.com/oDoADc2.png)

## Features
### Multiprocessing
In this project, PID controllers are running at the python environment installed in Rpi 3B+.
Each controller act as a subprocess under a main process.
There are 6 and more controllers running in a single Rpi with a quad-core processor.

### Setpoint tracking
If manual mode is on, the setpoint tracking is also activated to make SP tracks PV along the time.
#### [Snippet](https://github.com/PeterTsungYu/dev_iot/blob/faaae0b20436e31ef187fba7f2436a747c19b041/PIDsim.py#L304)
```python
if self.mode == 0:
    # Setpoint tracking
    self.SP_stepping = PV
```

### Setpoint weighting
To prevent impact of sudden setpoint changes or slips from a steady state, two weights are applied to proportional gain and derivative gain.
Typically, they are values ranging between 0~1 depending on each process.

- Proportional setpoint weight (beta)
```python
self.errorP1 = self.errorP0
self.errorP0 = self.beta*self.SP_stepping - self.PV # setpoint weighting
```

- Derivative setpoint weight (gamma)
```python
self.errorD2 = self.errorD1
self.errorD1 = self.errorD0
self.errorD0 = self.gamma*self.SP_stepping - self.PV # setpoint weighting
```

> There is no weighting on integral gain since one of the purposes of the integral gain is to subside to a steady state.

### Step increment
To reduce the impact of a sudden change on setpoints, a step increment parameter is introduced to generate a sequence of gradual setpoints.

For example, a current setpoint, 30, is set to a new setpoint, 70.
There is a 40 points difference.
If a step increment parameter, let's say 7, is given, then a sequence of gradual setpoints is set by steps.
In this example, they are 37, 44, 51, 58, 65, and 70.

```python
if self.SP_stepping < self.SP:
    self.SP_stepping += self.SP_increment
    if self.SP_stepping > self.SP:
        self.SP_stepping = self.SP
elif self.SP_stepping > self.SP:
    self.SP_stepping -= self.SP_increment
    if self.SP_stepping < self.SP:
        self.SP_stepping = self.SP 
```

### Manipulate proportional kicks during step changes 
There are times users would like to have kicks in manipulated values (MV) during step changes.
Thus, the kick_prop is introduced. It happens during the step changes so that users could have several kicks along the steps, which increases the impact of the kicks on the process.

As the setpoint weighting, the kick_props are applied to proportional gain and derivative gain.

```python
self.kick_prop = 1
if self.SP_stepping < self.SP:
    self.SP_stepping += self.SP_increment
    self.kick_prop = kick
    if self.SP_stepping > self.SP:
        self.SP_stepping = self.SP
        self.kick_prop = 1
elif self.SP_stepping > self.SP:
    self.SP_stepping -= self.SP_increment
    self.kick_prop = kick
    if self.SP_stepping < self.SP:
        self.SP_stepping = self.SP 
        self.kick_prop = 1 

...

self.deltaMV =  P*self.kick_prop + I + D*self.kick_prop
```

> if kick_prop stays as 1, then it is the same equation as the setpoint weighting.

### Margin of steady state
If a setpoint is restricted to a specific value within an unsteady process, one might experience a choppy control voyage.
Chances are a swinging state within a range is qualified as a steady state. 

Thus, the SP_range is applied to a conditional statement for a possible change on the MV.
If the PV is within a given range, then the MV is held as the previous.
Otherwise, the MV will be updated to a new value.

```python
if abs(self.SP_stepping - PV) <= self.SP_range:
    self.MV = MV
else:
    self.MV -= self.action*self.deltaMV
```

## Examples
![pid_img_2](https://i.imgur.com/rvOjZq2.gif)
![pid_img_3](https://i.imgur.com/a2os7RM.gif)


