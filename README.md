# T3003B_IntelligentRobotics
Class T3003B: Integration of Robotics and Intelligent Systems

---
### Week 2: Challenge
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

The relationship between a given linear and angular velocity control input $u = [v\;\omega]$ and the robot wheels' speeds is:

```math
     \begin{bmatrix} V \\ \omega \\ \end{bmatrix} = \begin{bmatrix}
\frac{r}{2} & \frac{r}{2} \\
\frac{r}{l} & -\frac{r}{l} \\
\end{bmatrix} \begin{bmatrix} \omega_r \\ \omega_l \\ \end{bmatrix}
```
Where $r$ is the wheel radius and $l$ is the robot's track length.

$
<a href="https://www.youtube.com/embed/dPIq9fL0Sxc?si=p-V4RuxIzM52Lr4Z">Watch simulation results on youtube<a/>

---

### Week 1: Challenge

State-space representation of the dynamics of a pendulum system:
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
