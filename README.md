# T3003B_IntelligentRobotics
Class T3003B: Integration of Robotics and Intelligent Systems

### Week 1: Challenge

---
State-space representation of the dynamics of a pendulum system
```math
J \ddot{q} + k\, \dot{q} + m\, g \, a \, \cos{(q)} = \tau
```
Let  $x_1$ = $q$  and  $x_2 = 2$:

```math
\dot{x_1} = x_2
```

```math
\dot{x_2} = \frac{1}{J}(\tau - mga\cos{(x_1)} - k\,x_2)
```

##### Angular position and velocity vs Time
<img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week1Challenge/visuals/q_qdot_vs_time.png" width="500">

##### Phase Diagram for angular postion and velocity
<img src="https://github.com/edgarcancinoe/T3003B_IntelligentRobotics/blob/main/Week1Challenge/visuals/phase_diagram.png" width="500">

<a href="https://youtu.be/bWifFmhGT0s">Watch on youtube<a/>

---
