# Need For Scenes - Point Cloud Pursuit

A Multi Object Tracking (MOT) via Sensor Fusion 3D interactive simulation using PCL

![game](https://github.com/AndreiMoraru123/NeedForScenes/assets/81184255/a20df907-f54c-448d-b955-c4319bab677c)

The Ego (player controller) car is designed using a 2D kinematic model accelerated with four states: x-position, y-position, velocity, and steering angle. 
These states are governed by the following equations implemented in the code:

$x(t + \Delta t) = x(t) + v(t) \cdot \cos(\theta(t)) \cdot \Delta t$

$y(t + \Delta t) = y(t) + v(t) \cdot \sin(\theta(t)) \cdot \Delta t$

$v(t + \Delta t) = v(t) + a(t) \cdot \Delta t - r \cdot v(t)$ if $a(t) = 0$

$\theta(t + \Delta t) = \theta(t) + v(t) \cdot s(t) \cdot \frac{\Delta t}{d}$

where:

- $x(t)$, $y(t)$ are the x and y coordinates of the car at time $t$.
- $v(t)$ is the velocity of the ego car at time $t$.
- $\theta(t)$ is the steering angle of the car at time $t$.
- $a(t)$ is the acceleration of the ego car at time $t$.
- $s(t)$ is the steering angle of the car's wheels at time $t$.
- $\Delta t$ is the time interval between state updates.
- $d$ is the distance between the front of the car and its center.
- $r$ is a rolling resistance constant that is applied when no acceleration is applied.


## Work in progress

![stewie2](https://user-images.githubusercontent.com/81184255/236611184-109bb765-766f-44a5-9f36-193eed3291d9.png)
