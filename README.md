# T3003B_IntelligentRobotics
Class T3003B: Integration of Robotics and Intelligent Systems

---

### Final Implementation

Robot specifications:
- Puzzlebot robot with Nvidia Jetson Nano.
- ROS1 implementation.
- 1D profile lidar sensor used for object detection.

Localisation:
- Encoder-based odometry and sensor fusion using landmark-based position estimates and Extended Kalman Filter (EKF) correction.

Navigation algorithms:
- Image-Based Visual Control Law for a Differential Drive Mobile Robot (Siradjuddin, I., Siradjuddin, I. A., & Adhisuwignjo, S. (2015)) to move to identified aruco blocks.
- Smooth control law for graceful motion of differential wheeled mobile robots (Park, J., & Kuipers, B. (2011)) for A to B coordinate navigation.
- Bug0 and Bug2 reactive algorithms for collision avoidance. 

---

### Localisation: Odometry, uncertainty propagation and extended kalman filter corrections

Odometry from wheel encoders was fused with landmark detection using an Extended Kalman Filter (EKF) to provide continuous localization and position correction by integrating motion predictions with measurement updates.

**Watch simulation results on youtube:**

One landmark, no obstacles:

[![YouTube Video](https://img.youtube.com/vi/ifPmzuSDDSw/0.jpg)](https://www.youtube.com/watch?v=ifPmzuSDDSw)

Four landmarks and obstacles:

[![YouTube Video](https://img.youtube.com/vi/r2-SB52dr-w/0.jpg)](https://www.youtube.com/watch?v=r2-SB52dr-w)

---

### Navigation: A smooth control law for graceful motion of differential wheeled mobile robots

Implementation of the smooth control law for graceful motion of differential wheeled mobile robots, as described in ‘A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment’ by Park and Kuipers

##### Reference
Park, J., & Kuipers, B. (2011). *A smooth control law for graceful motion of differential wheeled mobile robots in 2D environment*. Proceedings - IEEE International Conference on Robotics and Automation. [DOI: 10.1109/ICRA.2011.5980167](https://doi.org/10.1109/ICRA.2011.5980167)

---

### Vision & Control: An Image Based Visual Control Law for a Differential Drive Mobile Robot
Implementation of the Image-Based Visual Servoing (IBVS) control law, as proposed in ‘An Image Based Visual Control Law for a Differential Drive Mobile Robot’ by Siradjuddin et al., utilizing a set of reference image points to map visual errors in the image space to robot actuation.”

Watch simulation results on youtube:

[![YouTube Video](https://img.youtube.com/vi/0MXhxT5HQDc/0.jpg)](https://www.youtube.com/watch?v=0MXhxT5HQDc)

##### Reference

Siradjuddin, I., Siradjuddin, I. A., & Adhisuwignjo, S. (2015). An Image Based Visual Control Law for a Differential Drive Mobile Robot. *International Journal of Mechanical & Mechatronics Engineering*, 15(6).

---
### Week 2: Control and Odometry
Control, odometry, and simulation in RViz of the PuzzleBot movement while following a waypoint-defined trajectory, based on the robot's kinematic state representation.

- Simulation of the robot's dynamics under its kinematic model.
- Odometry by Dead Reckoning using RK4 numeric integration.
- Implementation of the linear controller (PID) to control the robot's linear and angular velocity in the robot frame.

#### Robot model
Kinematic model of the PuzzleBot differential robot:
```math
\dot{x} = v \cos{\theta}
```
```math
\dot{y} = v \sin{\theta}
```
```math
\dot{\theta} = \omega
```
Where $v$ and $w$ are linear and angular velocities in robot frame, whilst $x$, $y$, and $\theta$ are the state variables describing the pose in the 2D inertial frame.

The relationship between a given linear and angular velocity control input and the robot wheels' speeds is:

```math
     u = \begin{bmatrix} V \\ \omega \\ \end{bmatrix} = \begin{bmatrix}
\frac{r}{2} & \frac{r}{2} \\
\frac{r}{l} & -\frac{r}{l}
\end{bmatrix} \begin{bmatrix} \omega_r \\ \omega_l \\ \end{bmatrix}
```
Where $r$ is the wheel radius and $l$ is the robot's track length.


Robot model diagram VS Time|
:-------------------------:|
<img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week2Challenge/rosgraph.png" width="1000">|


Robot model diagram VS Time             |TF tree
:-------------------------:|:-------------------------:
<img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week2Challenge/robot_model.png" width="500"> | <img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week2Challenge/tf_tree.png" width="500">

Rviz simulation |Control Plots
:-------------------------:|:-------------------------:
<img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week2Challenge/rviz_view.png" width="500"> | <img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week2Challenge/plots.png" width="500">

<a href="https://www.youtube.com/embed/dPIq9fL0Sxc?si=p-V4RuxIzM52Lr4Z">Watch simulation results on youtube<a/>

---

### Week 1: State-space representation of the dynamics of a pendulum system:
```math
J \ddot{q} + k\, \dot{q} + mga\cos{(q)} = \tau
```
Let  $x_1$ = $q$  and  $\dot{x} = x_2$,

```math
\dot{x_1} = x_2
```

```math
\dot{x_2} = \frac{1}{J}(\tau - mga\cos{(x_1)} - k\,x_2)
```
Where link has inertia $J$ and the centre of mass, $m$, is located at $\textbf{x}$, a distance $a$ from the origin (point of rotation), and $k$ is the friction coefficient.

```math
J = \frac{4}{3}ma^{2}
```
For this case, since it is a uniform rod, the center of mass is located at the center of the rod. Then:

```math
a = \frac{l}{2}
```

##### 

Angular position and angular velocity VS Time             |  Phase Diagram for angular postion and angular velocity
:-------------------------:|:-------------------------:
<img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week1Challenge/visuals/q_qdot_vs_time.png" width="500"> | <img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week1Challenge/visuals/phase_diagram.png" width="500">


<a href="https://www.youtube.com/embed/bWifFmhGT0s?si=SvydODJ1PvwrKaqn">Watch simulation and live plotting on youtube<a/>

---
