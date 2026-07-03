---
layout: single
title: "Automated Pill Bottling & Descrambling Machine"
permalink: /projects/pill-bot/
classes: wide project-full
author_profile: false
sidebar: false
---

### Project Overview

This project involved designing and building a fully autonomous **pill bottle descrambling and filling system** for small-scale supplement manufacturers, sponsored by [Omega Design](https://www.omegadesign.com/) and [Norwalt Design](https://norwalt.com/). The system was designed to descramble bottles, transfer them into position, dispense exactly **30 pills**, and reject bottles that were incorrectly oriented or improperly filled.

The final machine operated as a closed-loop automation system driven by sensor feedback. During validation testing, the system achieved a **96% success rate**, the highest in the class, and earned the **Best Descrambler Award** for its robust bottle orientation design.

My primary contributions focused on system intelligence, automation logic, sensor integration, and closed-loop control. I also contributed to the physical rejection mechanism used to remove faulty bottles from the production path.

---

### 🛠️ Technical Implementation & Software Architecture

I developed the core system operation logic in **C++ on an Arduino Mega**, defining how the machine autonomously transitioned between descrambling, transfer, filling, inspection, and rejection. The control program used sensor feedback to determine when each subsystem should start, stop, or wait for the next condition to be met.

A major part of my work was determining how sensors would be used throughout the system. Break-beam sensors were placed to detect bottle orientation, bottle position, transfer timing, and pill count. These inputs allowed the machine to make decisions without manual intervention, including whether a bottle should continue to filling, wait for the next stage, or be rejected.

The automation logic coordinated multiple motors, conveyors, sensors, and the pill dispensing sequence into one closed-loop workflow. Rather than relying on fixed timing alone, the system responded to live feedback from the machine, making the process more reliable across repeated runs.

I also helped design the **linear-actuator-based rejection mechanism**, which removed bottles that were incorrectly oriented or had an incorrect pill count. This mechanism was integrated into the control logic so faulty bottles could be automatically ejected from the line.

Key methods and components included:

* Arduino Mega control system
* C++ automation logic
* Break-beam sensor feedback
* Conveyor and motor coordination
* Closed-loop state-based control
* Linear actuator rejection control
* Electrical wiring and hardware integration

---

### 📊 Experimental Validation & Results

The system was validated through repeated full-cycle testing, where bottles were descrambled, transferred, filled, inspected, and either passed or rejected automatically. During final validation, the machine achieved a **96% success rate**, the highest in the class.

The final system demonstrated reliable autonomous operation, accurate pill counting, and robust fault handling. The control logic coordinated each subsystem based on sensor feedback, allowing the machine to complete the full sequence without manual intervention.

The completed machine provided a compact and cost-effective automation solution for small manufacturing environments, showing how sensor-driven control can improve accuracy, reliability, and quality control in bottle filling operations.

#### Final System Image

<div style="display: flex; justify-content: center; margin: 20px 0;">
  <img src="/assets/images/pill_labeled.jpeg" 
       alt="Automated Pill Bottling Machine" 
       style="width: 100%; max-width: 750px; border-radius: 6px;">
</div>

#### Wiring Diagram

<div style="display: flex; justify-content: center; margin: 20px 0;">
  <img src="/assets/images/wiring.jpeg" 
       alt="Automated Pill Bottling Machine Wiring Diagram" 
       style="width: 70%; max-width: 750px; border-radius: 6px;">
</div>

#### System Demonstration

<div style="display: flex; justify-content: center; margin: 20px 0;">
  <iframe 
    src="https://drive.google.com/file/d/1EiGO4iyOqOTDFHwRJEw7SdkGgB1DUABn/preview"
    width="100%" 
    height="500"
    allow="autoplay"
    style="border: none; border-radius: 6px;">
  </iframe>
</div>

---

### Code

#### Automation Control Code

This C++ file contains the Arduino Mega control logic for coordinating sensor feedback, subsystem timing, pill counting, and automatic bottle rejection.

[Open Code](/assets/code/pill-bot/pill_auto){: .btn .btn--primary}