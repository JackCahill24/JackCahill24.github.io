---
layout: single
title: "Tactile Sensing System for Quadrupedal Interaction"
permalink: /projects/quadruped-sensor/
classes: wide project-full
author_profile: false
sidebar: false
---

### Project Overview

This project involved developing a **magnetometer-based tactile sensing system** for a **Unitree Go1** quadruped robot to support physical interaction through the robot’s head. The goal was to turn the front of the robot into an active contact interface capable of detecting contact, estimating interaction forces, and supporting pushing-based environment interaction.

This work contributed to **Head-On**, an accepted paper at the **IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2026)**, where I am listed as second author. My primary contributions were on the physical and embedded systems side of the project, including custom helmet design, tactile sensor fabrication, electronics integration, microcontroller setup, low-level firmware, and multi-sensor data communication. I also contributed to the paper writing and to the broader system development effort supporting contact detection, dynamic rebaselining, and force-estimation experiments.

---

### 🛠️ Technical Implementation & Software Architecture

I designed the custom helmet from scratch to fit securely around the Unitree Go1 head while avoiding interference with the robot’s cameras, sensors, and normal motion. The helmet needed to be compact, removable, mechanically stable, and strong enough to withstand repeated contact during pushing experiments.

The initial tactile interface used **AnySkin magnetometer-based tactile sensors**, which detect contact through deformation-induced changes in magnetic field measurements. I helped fabricate and prepare the sensors, then integrated them into the helmet as a multi-sensor contact surface.

To support multiple sensors, I built the embedded electronics architecture around an **ESP32 microcontroller** and the **I²C communication protocol**. Since identical sensors shared the same I²C addresses, I incorporated an **I²C multiplexer** to isolate each sensor channel and allow the microcontroller to read from them sequentially.

I also wrote and tested the low-level firmware used to initialize the sensors, cycle through active channels, collect magnetic field data, and stream the combined measurements to the robot’s onboard computer. This hardware and firmware pipeline supported the higher-level contact detection and dynamic rebaselining algorithms that enabled more reliable force and contact estimation during repeated physical interaction.

Key methods and components included:

* Custom CAD helmet design
* Sensor fabrication and mechanical integration
* ESP32 microcontroller integration
* I²C communication and multiplexing
* Low-level embedded firmware
* Serial/UDP data streaming support
* Contact detection and dynamic rebaselining support
* Hardware debugging and system bring-up

---

### 📊 Experimental Validation & Results

The tactile helmet was validated through controlled pushing experiments with the Unitree Go1. During testing, the robot pressed the sensorized helmet against an external force-measurement setup while the tactile sensors streamed magnetic field data to the onboard computer.

The system’s main capability was enabling the robot to distinguish contact from no-contact states and dynamically rebaseline the sensor readings to reduce drift. This was important because magnetometer-based tactile sensors can accumulate measurement drift during repeated loading, unloading, and robot motion. By combining the physical sensing platform with contact estimation and dynamic rebaselining, the system supported more accurate and stable force estimation during repeated pushing interactions.

The hardware system successfully supported repeated physical contact while maintaining stable communication across multiple tactile sensors. The custom helmet stayed securely mounted during experiments and provided a repeatable platform for collecting tactile data during dynamic robot interaction.

#### Current Development

The system is currently being further developed beyond the original AnySkin patch-based design. The updated approach replaces discrete tactile patches with a **compressible lattice structure containing embedded permanent magnets**, creating a more continuous sensing surface across the front of the Go1.

This newer design has improved sensing accuracy and made the interface more versatile by allowing contact to be detected across a continuous surface rather than only at separate sensor patches. The system has also been expanded with more magnetometers, a larger electronics architecture, an improved data-processing pipeline, and a redesigned helmet that is more modular and accessible for testing and future development.

#### Helmet Prototype

<div style="display: flex; justify-content: center; margin: 20px 0;">
  <img src="/assets/images/headon_go1.png" 
       alt="Quadruped Tactile Helmet Prototype" 
       style="width: 60%; max-width: 700px; border-radius: 6px;">
</div>

<div style="display: flex; justify-content: center; margin: 20px 0;">
  <img src="/assets/images/jack_go1.jpeg" 
       alt="Quadruped Tactile Helmet Prototype" 
       style="width: 60%; max-width: 700px; border-radius: 6px;">
</div>

---

### Code & Paper

#### Project Code

The code for this project is currently confidential while the research work is being prepared for public release.

<span class="btn btn--primary disabled">GitHub Repository Coming Soon</span>

#### Research Paper

The full IROS 2026 paper is not publicly available yet, but it will be linked here once publication access is available.

<span class="btn btn--primary disabled">Paper Coming Soon</span>