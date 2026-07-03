---
layout: single
title: "Autonomous Navigation Platform & Curriculum"
permalink: /projects/turtlebot-nav/
classes: wide project-full
author_profile: false
sidebar: false
---

### Project Overview

This project involved designing a graduate-level **autonomous navigation platform and curriculum** for the **TurtleBot3 Burger**, focused on mobile robot control, path-following, and ROS 2 development. I built the project architecture to allow students to design, implement, test, and evaluate custom navigation controllers on a physical differential-drive robot.

The project uses **ROS 2 Humble** and the **Nav2** navigation stack. Students implement their own custom controller plugins in C++, then test them on a TurtleBot3 navigating through a mapped arena with static obstacles. The goal was to create an open-ended robotics project that emphasized real-world constraints, embedded hardware limitations, sensor feedback, and controller tuning.

My primary contributions focused on project design, ROS 2 software architecture, controller-plugin templates, technical documentation, evaluation tools, GitHub deployment, and instructional video resources.

---

### 🛠️ Technical Implementation & Software Architecture

The platform is built around a **nonholonomic differential-drive robot** equipped with LiDAR, wheel encoders, and an IMU. The TurtleBot3 runs ROS 2 on an onboard Raspberry Pi 4, while a separate Raspberry Pi 4 remote PC is used for development, visualization, SLAM, Nav2, and controller testing.

I developed the software workflow for creating and testing custom **Nav2 controller plugins**. This included package structure, launch files, plugin templates, setup instructions, and testing procedures so students could focus on implementing their own feedback-control laws rather than building the entire ROS 2 architecture from scratch.

I also prepared extensive documentation covering robot setup, ROS 2 Humble installation, remote-PC configuration, TurtleBot3 bringup, SLAM, map generation, Nav2 operation, and custom controller development. To support adoption, I created video tutorials and used GitHub to version, develop, and deploy the software directly to the robot platforms.

Key methods and components included:

* ROS 2 Humble development
* Nav2 controller plugin architecture
* Differential-drive mobile robot control
* LiDAR, IMU, and encoder integration
* SLAM, mapping, and localization workflows
* Custom C++ controller templates
* GitHub-based version control and deployment
* Technical documentation and video tutorials

---

### 📊 Experimental Validation & Results

The project was validated by deploying the software to physical TurtleBot3 platforms and testing controller behavior in a mapped arena with static obstacles. Students can use the framework to implement a controller, tune parameters, run trials, and compare navigation performance.

As part of my own completed controller project, I implemented and tuned a custom nonlinear trajectory-tracking controller in Nav2. The controller followed dynamically updating global paths while correcting tracking error in real time, with testing performed across increasingly difficult trials involving obstacle avoidance and final goal alignment.

The final platform provides a scalable instructional framework for teaching autonomous navigation through hands-on robotics development. It gives students experience with real ROS 2 software, custom controller development, sensor feedback, embedded compute limitations, and the practical challenges of deploying autonomy on physical robots.

#### Final Competition: Robot Maze Navigation

<div style="max-width: 750px; margin: 20px auto;">
  <iframe 
    src="https://www.youtube.com/embed/_Is6nM2Z3Zw"
    title="Final Competition: Robot Maze Navigation"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width: 100%; aspect-ratio: 16 / 9; border: none; border-radius: 6px;">
  </iframe>
</div>

#### Custom Controller Plugin Tutorial

<div style="max-width: 750px; margin: 20px auto;">
  <iframe 
    src="https://www.youtube.com/embed/20ixO3rp3DQ"
    title="Custom Controller Plugin Tutorial"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen
    style="width: 100%; aspect-ratio: 16 / 9; border: none; border-radius: 6px;">
  </iframe>
</div>

---

### Code

#### GitHub Repository

This repository contains the ROS 2 project framework, controller plugin templates, launch files, documentation, and evaluation tools for the TurtleBot3 autonomous navigation project.

[Open GitHub Repository](https://github.com/HORC-Lab/turtlebot3_navproject_public.git){: .btn .btn--primary}
