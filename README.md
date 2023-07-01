# Need For Scenes - Point Cloud Pursuit 🎮

#### A Multi Object Tracking (MOT) via Sensor Fusion 3D interactive simulation using the awesome [Point Cloud Library](https://pointclouds.org/).

![game](https://github.com/AndreiMoraru123/NeedForScenes/assets/81184255/a20df907-f54c-448d-b955-c4319bab677c)

## Ego Car
The green player-controlled car's motion model is designed using a 2D kinematic model accelerated with four states:
- x-position, y-position, velocity, and steering angle.

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

## Enemy Cars
  - These are autonomous vehicles in the game that act as dynamic obstacles for the player-controlled ego car.
  - Unlike the ego car, these adversary agent cars are not controlled by the user in real-time.
  - Instead, their movements are governed by predefined action vectors and encoded control laws that dictate their behavior throughout the game.
  - Action Vectors:
    - These vectors consist of a sequence of control inputs, including acceleration and steering commands, associated with specific time points.
    - They are designed to create various motion patterns for the adversary agent cars.
  - Encoded Control Laws:
    - These laws serve to implement the action vectors, translating the acceleration and steering commands into changes in the car's states based on the same physics-based kinematic model used for the ego car.
    - They also consider factors such as rolling resistance when the car is not being accelerated, mimicking the natural behavior of cars.
  - Adversary Agent Car Behavior:
    - They provide a dynamic and challenging environment for the player, who must carefully navigate the ego car while avoiding collisions with these moving obstacles.
    - Despite their autonomous nature, the adversary agent cars do not possess advanced decision-making capabilities or the ability to react to the player's actions.
  - Visualization Elements:
    - Green Sphere and Green Arrow:
      - The green sphere represents the estimated position of the adversary car, calculated by an Unscented Kalman filter as a tracking algorithm.
      - The green arrow represents the estimated velocity vector of the adversary car, also derived from the Kalman filter.
    - Red Sphere:
      - The red sphere represents the actual position of the adversary car, derived from the predefined action vectors and encoded control laws, but subject to a certain level of noise from simulated LIDAR measurement.
    - Magenta Arrow:
      - The magenta arrow illustrates the radar measurement from the ego car to the adversary car.
     
## Parking Spots
  - Parking spots are designated areas that are static, meaning they do not move or change position throughout the game.
  - They are strategically placed in the 3D environment and serve as the ultimate objective for the player.
  - The main goal of the player is to safely and efficiently maneuver the ego car into these parking spots.

## Obstacles
  - Obstacles are also static elements in the game that are randomly positioned in the 3D environment.
  - Their main function is to add complexity to the navigation task.
  - The player must skillfully maneuver the ego car around these obstacles to prevent collisions, which can impede the car's progress or result in game failure.

## Fictional Parking Spots and Obstacles
  - These objects are unique elements that visually resemble real parking spots and obstacles but have no gameplay implications.
  - They do not interact with or affect the movement of the ego car.
  - Instead, their primary role is to enhance the visual complexity of the scene, adding another level of depth to the game's 3D environment.
  - Their presence contributes to providing a more immersive and challenging experience for the player.

## Game Scenario

In the game environment, the encapsulation of the entire simulation domain is represented by the Scene class. This class is responsible for managing and coordinating all entities present in the game, including the adversary cars, obstacles, and parking spots. All these components are individually modeled objects using their respective classes: Car, Obstacle, and ParkingSpot.

The Scene class is designed to provide several key functionalities:

1. Scene Creation and Initialization:
   - The creation and configuration of the scene take place within the constructor of the class.
   - It initializes the random number generator used for the random placement of obstacles and parking spots in the 3D environment.
   - Additionally, it creates three adversary cars, setting their initial positions, dimensions, colors, and other parameters.
   - The constructor also associates a Tracker object with each car based on the values from the TrackCars matrix.

2. Scene Progression/Advancement:
   - The Scene class has a function responsible for progressing the state of the game environment.
   - It adjusts the position of each entity based on their velocities and the elapsed time.
   - As part of its operation, this function also checks for collisions between the player-controlled ego car and other entities in the game.
   - In case a collision is detected, the function takes corrective measures by bringing the ego car's velocity to zero and slightly adjusting its position to resolve the collision.

3. Collision Checks:
   - The Scene class includes a collision detection function that checks for any collisions between the ego car and any adversary car in the game environment.
   - This function returns a boolean value to indicate whether a collision has occurred.

4. Car Tracking:
   - If a car is set to be tracked, the scene advancement function utilizes both lidar and radar sensors to determine the position and velocity of the car.
   - This data is then passed to the associated Tracker object.
   - The function also calculates the actual position and velocity of the car as well as the estimated position and velocity from the tracking device.
   - These values are then used to calculate the Root Mean Square Error (RMSE) to evaluate the tracking accuracy.

5. Accuracy Evaluation:
   - The RMSE values are evaluated against predefined thresholds to determine the tracking accuracy.
   - If the RMSE values exceed these thresholds, indicating a tracking error beyond acceptable limits, a failure message is displayed to the user.

6. Car Control:
   - The control of each adversary car is determined by a sequence of control objects.
   - These objects specify the speed and steering angle of the car for specific periods.
   - The Car class has a control function used to set these controls.

## Unscented Kalman Filter (UKF)

In this project, the Unscented Kalman Filter (UKF) serves as the primary tool for state estimation under the simulation's non-linear circumstances.

### Estimated State

The tracked state of the car in this code includes:

1. **Position** (x and y): This is the current 2D position of the vehicle. Both lidar and radar measurements can provide information on position. The lidar provides a direct measurement in Cartesian coordinates, while the radar provides a measurement in polar coordinates that can be converted to Cartesian.

2. **Velocity** (v): This is the magnitude of the 2D velocity. While the lidar does not directly measure velocity, radar can provide the radial velocity of the vehicle (the component of the velocity towards or away from the radar sensor). 

3. **Yaw Angle** (ψ): This is the orientation of the vehicle in the 2D plane, relative to the positive x-axis.

4. **Yaw Rate** (ψ_dot): This is the rate of change of the yaw angle, which describes how quickly the vehicle is changing its orientation.


### Prediction Step

1. **Sigma Point Generation**: The UKF starts by generating sigma points from the current state estimate and its covariance. The process includes adding and subtracting a scaled square root of the covariance matrix from the state estimate. This step produces a cloud of points that encapsulates the possible states the system could be in, considering the uncertainty.

2. **Sigma Point Propagation**: Each generated sigma point is then propagated through the non-linear system dynamics. The specific dynamics depend on the system being modeled. For a mobile robot, it might be a kinematic bicycle model, for instance. 

3. **Mean and Covariance Estimation**: Once propagated, the sigma points are recombined to form a new predicted state estimate and covariance. The combination process involves taking a weighted sum of the sigma points, where the weights depend on the chosen sigma point generation method.

### Update Step

1. **Sigma Point Generation for the Measurement**: Just like in the prediction step, sigma points are generated but this time, these sigma points are transformed into the measurement space using the measurement function.

2. **Cross-Correlation Matrix Calculation**: The cross-correlation matrix is calculated using the difference between the mean of the predicted sigma points and each predicted sigma point in the state space, and the difference between the mean of the transformed sigma points and each transformed sigma point in the measurement space.

3. **Kalman Gain Calculation**: The Kalman gain is calculated using the cross-correlation matrix and the covariance matrix of the predicted sigma points in the measurement space.

4. **State Update**: The final step is to calculate the residual (the difference between the actual measurement and the transformed mean of the sigma points), multiply it with the Kalman gain and add this to the predicted state to obtain the updated state.

5. **Covariance Update**: Similarly, the covariance is updated by subtracting the Kalman gain times the covariance of the predicted sigma points in the measurement space times the transpose of the Kalman gain from the predicted covariance.

This process repeats in a loop for each incoming measurement, continuously refining the state estimate in response to new data. Thus, the UKF allows our system to maintain an accurate state representation despite working under non-linear conditions.


## ___Painless___ installation of the Point Cloud Library on Windows
### (if you want to try out this code):

* Install vcpkg, Microsoft's unofficial C++ package manager

```bash
> git clone https://github.com/microsoft/vcpkg
> .\vcpkg\bootstrap-vcpkg.bat
```

There is a lot more info to be found at the original [repo](https://github.com/microsoft/vcpkg), but really, this is all you need

* Install PCL x64

```bash
> vcpkg install pcl[vtk]:x64-windows --featurepackages --recurse
```

> **Warning**
> If you do not explicitly declare an x64 build, vcpkg will default it to x86 and cmake will probably fail.

* Next you need to specify the toolchain path to the cmake file

```
-DCMAKE_TOOLCHAIN_FILE=C:\Path\vcpkg.cmake
```

* If you are on a Windows machine, vcpckg will also default your toolchain to MSVC, so you will have to download Visual Studio and use it as the compiler

It looks like this for me (CLion):

![image](https://user-images.githubusercontent.com/81184255/197364009-78660d22-a0e9-4105-8327-9405d300993e.png)
