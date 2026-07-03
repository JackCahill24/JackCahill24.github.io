---
layout: single
title: "KUKA 7-DOF Manipulator Trajectory & Kinematics Control"
permalink: /projects/kuka-arm/
classes: wide project-full
author_profile: false
sidebar: false
---

### Project Overview

This project involved programming a **KUKA LBR iiwa 7 R800** robotic arm to complete a precision stamping task across multiple target locations. The robot had to move from an initial configuration through a sequence of target poses while keeping the stamping tool properly aligned, avoiding the table surface, and staying within joint position and velocity limits.

The main challenge was converting camera-detected target locations into executable robot motion. To solve this, I built a MATLAB-based trajectory planning workflow that combined robot kinematics, coordinate frame transformations, inverse kinematics, and trajectory validation.

---

### 🛠️ Technical Implementation & Software Architecture

I modeled the KUKA arm using **forward kinematics** to calculate the position and orientation of the robot from its seven joint angles. Since the task depended on the rectangular stamping tool, I also accounted for the offset between the robot flange and the actual tool frame.

Target locations were determined using camera and **ArUco marker** data. I transformed these detected target poses into the robot base frame so the arm could plan motion relative to its workspace.

To reach each target, I implemented a **Jacobian-based inverse kinematics solver**. The solver iteratively adjusted the robot’s joint angles until the stamping tool aligned with the desired pose. Because the KUKA arm is redundant, I used **null-space control** to improve the motion without disrupting the main task.

Key methods included:

- Forward and inverse kinematics
- Analytical Jacobian calculations
- Coordinate frame transformations
- Null-space redundancy control
- Joint-limit and singularity avoidance
- Table-clearance checking
- Smooth trajectory generation in MATLAB

The final trajectory was divided into an approach phase, three target-to-target motion segments, and a final hold phase. The result was a **20-second trajectory** sampled every **0.005 seconds**, producing **4,000 joint-command rows** for the robot.

---

### 📊 Experimental Validation & Results

The trajectory was first validated in MATLAB to confirm that all joint positions and velocities stayed within the robot’s physical limits. I also simulated the robot motion to check the tool path, target sequence, and table clearance before hardware execution.

The final trajectory was then implemented on the physical KUKA arm. The robot successfully executed the planned stamping motion while satisfying the timing, alignment, and safety constraints.

#### Simulation

<div style="display: flex; justify-content: center; margin: 20px 0;">
  <iframe 
    src="https://drive.google.com/file/d/1UbjBZei6eJGbB0FGIInRFfcQ0QuDg34q/preview"
    width="100%" 
    height="500"
    allow="autoplay"
    style="border: none; border-radius: 6px;">
  </iframe>
</div>

#### Physical Robot Execution

<div style="display: flex; justify-content: center; margin: 20px 0;">
  <iframe 
    src="https://drive.google.com/file/d/17fIEDTm9jUgk6gYfdggDzGwm3PZidTQ3/preview"
    width="100%" 
    height="600"
    allow="autoplay"
    style="border: none; border-radius: 6px;">
  </iframe>
</div>

---

### Code

#### Main Trajectory Generation Script

This MATLAB script builds the full motion-planning pipeline, including target-pose construction, inverse kinematics, trajectory generation, and joint-limit validation.

[Open Code](/assets/code/kuka-arm/Final_Project_New.m){: .btn .btn--primary}

#### Trajectory Simulation Script

This MATLAB script loads the generated joint trajectory file and visualizes the KUKA arm motion, stamp path, and workspace setup in simulation.

[Open Code](/assets/code/kuka-arm/Final_Project_Sim_New.m){: .btn .btn--primary}