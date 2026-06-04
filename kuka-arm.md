---
layout: single
title: "KUKA 7-DOF Manipulator Trajectory & Kinematics Control"
permalink: /projects/kuka-arm/
sidebar:
  nav: "main"
---

Developed, simulated, and validated a joint-space trajectory control framework for a KUKA 7-DOF redundant robotic manipulator using MATLAB[cite: 1]. This project showcases the mathematical foundations required for complex robotic manipulation and obstacle avoidance in industrial spaces[cite: 1].

## Core Engineering Achievements
* **Kinematics Engine:** Formulated and implemented complete forward and inverse kinematics using analytical and numerical methods[cite: 1].
* **Differential Kinematics:** Developed the system Jacobian matrix to map joint velocities to operational space, incorporating singularity avoidance protocols[cite: 1].
* **Redundancy Resolution:** Utilized null-space projection techniques to exploit the arm's 7th degree of freedom, allowing the manipulator to maintain its end-effector pose while simultaneously optimizing secondary objectives (such as joint limit avoidance)[cite: 1].
* **Obstacle Avoidance:** Implemented real-time distance-tracking algorithms to seamlessly steer the manipulator link geometries away from workplace obstructions during active trajectory execution[cite: 1].

## Technical Stack
* **Software/Environment:** MATLAB[cite: 1]
* **Theory Applied:** Forward/Inverse Kinematics, Jacobian Control, Null-Space Redundancy, Trajectory Generation[cite: 1]

---

### Mathematical Implementation

To achieve redundancy resolution, the joint velocities $\dot{q}$ were calculated using the pseudoinverse of the Jacobian $J^{\dagger}$ along with a null-space projection to optimize a secondary task vector $q_0$:

$$\dot{q} = J^{\dagger}v + (I - J^{\dagger}J)\dot{q}_0$$

This mathematical foundation guaranteed that primary task space velocities $v$ were tracking accurately while ensuring the physical link structures autonomously pivoted away from obstacles[cite: 1].

---

*Note: Code repository access and simulation videos can be provided upon request during interview evaluation.*
