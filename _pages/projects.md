---
layout: single
title: "Engineering & Robotics Projects"
permalink: /projects/
classes: wide
---

A selection of robotics software packages, mechatronic prototypes, and automated systems developed throughout my academic and research career, emphasizing the intersection of physical design and software implementation. Click each project title for an in-depth walkthrough of the specific project.

---

### [KUKA 7-DOF Manipulator Trajectory & Kinematics Control](/projects/kuka-arm/)

<div style="display: flex; align-items: center; gap: 20px; margin-bottom: 15px;">
  <div style="flex: 1; max-width: 300px; height: 180px; display: flex; align-items: center; justify-content: center; background-color: transparent;">
    <img src="/assets/images/kuka_stock.png" alt="KUKA Manipulator Simulation" style="max-width: 100%; max-height: 100%; object-fit: contain; border-radius: 4px;">
  </div>
  <div style="flex: 2;">
    <strong>Core Stack:</strong><br>
    MATLAB, Robot Kinematics, Analytical Jacobians, Trajectory Generation
  </div>
</div>

Programmed a motion control framework for a 7-DOF KUKA LBR iiwa robotic arm to execute an automated precision stamping task. Developed the forward and inverse kinematics solvers, Jacobian calculations, and control loops necessary to translate camera-detected targets into fluid joint movements. Structured the system's trajectory generation to smoothly guide the arm through multiple motion phases while maintaining workspace obstacle clearance and satisfying joint safety limits.

<div style="clear: both;"></div>

---

### [Tactile Sensing System for Quadrupedal Interaction](/projects/quadruped-sensor/)

<div style="display: flex; align-items: center; gap: 20px; margin-bottom: 15px;">
  <div style="flex: 1; max-width: 300px; height: 180px; display: flex; align-items: center; justify-content: center; background-color: transparent;">
    <img src="/assets/images/headon_go1.png" alt="Quadruped Tactile Helmet Prototype" style="max-width: 100%; max-height: 100%; object-fit: contain; border-radius: 4px;">
  </div>
  <div style="flex: 2;">
    <strong>Core Stack:</strong><br>
    Embedded C++, Mechatronic Design, Embedded Systems, Data Communication Protocols
  </div>
</div>

Designed and evolved a multi-axis tactile helmet system for quadrupedal robots to enable active physical environment interaction. Developed the low-level firmware, micro-hardware networking, and data communication pipelines to continuously stream feedback from magnetic-based sensors that detect contact and applied forces via deformation-induced magnetic field changes. Expanded the system to feature a high-density, continuous sensing surface using a custom compressible lattice structure to map physical interactions across the robot's entire contact interface.

**Reference:** Stergios Bachoumas, John Cahill and Panagiotis Artemiadis, “Head-On: Robust Tactile Force Sensing for Quadrupedal Interaction,” 2026 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2026 (to appear).

<div style="clear: both;"></div>

---

### [Vehicle-Mounted UAS Automated Flight Testing Rig](/projects/uas-test-rig/)

<div style="display: flex; align-items: center; gap: 20px; margin-bottom: 15px;">
  <div style="flex: 1; max-width: 300px; height: 180px; display: flex; align-items: center; justify-content: center; background-color: transparent;">
    <img src="/assets/images/uas_rig.png" alt="UAS Flight Testing Rig" style="max-width: 100%; max-height: 100%; object-fit: contain; border-radius: 4px;">
  </div>
  <div style="flex: 2;">
    <strong>Core Stack:</strong><br>
    3D CAD Modeling, Finite Element Analysis (FEA), Structural Load Analysis, Data Acquisition (DAQ) Systems
  </div>
</div>

Designed and validated a custom, roof-mounted aerodynamic testing platform for in-house Unmanned Aerial System (UAS) calibration and flight testing. Modeled the complete structural assembly in CAD and conducted structural FEA to ensure stability at high operational vehicle speeds. Integrated and programmed a specialized multi-axis load cell and sensor instrumentation stack to capture real-time aerodynamic force profiles and telemetry data during testing runs.

<div style="clear: both;"></div>

---

### [Automated Pill Bottling & Descrambling Machine](/projects/pill-bot/)

<div style="display: flex; align-items: center; gap: 20px; margin-bottom: 15px;">
  <div style="flex: 1; max-width: 300px; height: 180px; display: flex; align-items: center; justify-content: center; background-color: transparent;">
    <img src="/assets/images/auto_pill.png" alt="Automated Pill Machine" style="max-width: 100%; max-height: 100%; object-fit: contain; border-radius: 4px;">
  </div>
  <div style="flex: 2;">
    <strong>Core Stack:</strong><br>
    Automation Logic, Embedded C++, Microcontrollers & Drivers, Electrical System Wiring
  </div>
</div>

Designed and engineered an automated mechatronic system to descramble, fill, and inspect medical pill bottles on a continuous assembly line. Programmed the central automation logic and state machine using Arduino microcontrollers to synchronize conveyor sorting, gating mechanisms, and part-rejection modules. Architected the complete electrical system layout, managing hardware sensor integration, signal routing, and motor driver wiring to achieve precise timing and continuous speed regulation across the conveyor handling lines.

<div style="clear: both;"></div>

---

### [Autonomous Navigation Platform & Curriculum](/projects/turtlebot-nav/)

<div style="display: flex; align-items: center; gap: 20px; margin-bottom: 15px;">
  <div style="flex: 1; max-width: 300px; height: 180px; display: flex; align-items: center; justify-content: center; background-color: transparent;">
    <img src="/assets/images/turtlebot.png" alt="TurtleBot3 Autonomous Navigation" style="max-width: 100%; max-height: 100%; object-fit: contain; border-radius: 4px;">
  </div>
  <div style="flex: 2;">
    <strong>Core Stack:</strong><br>
    ROS 2, Mobile Robot Control, Git/GitHub Version Control, Technical Writing & Software Deployment
  </div>
</div>

Architected from the ground up a graduate-level autonomous navigation project framework for differential-drive platforms utilized in a Master's level digital controls course. Built the full ROS 2 software infrastructure, custom tracking-controller plugin templates, and low-compute simulation/evaluation tools to enable rapid integration of student control laws with hardware LiDAR, IMU, and encoder feedback. Authored dozens of pages of extensive, production-grade technical guides and version-controlled deployment pipelines to support seamless system onboarding and physical testing loops.

<div style="clear: both;"></div>