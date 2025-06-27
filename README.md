<img src="https://upload.wikimedia.org/wikipedia/commons/0/0a/Stairs_icon.png" align="right" width="280" alt="Bose header"/>

# Bose — Stair-Climbing Hybrid-Terrain Robot

**Bose** is a hybrid-terrain mobile robot capable of ascending stairs and navigating flat surfaces using a dual-locomotion system. It features custom tri-helix wheels for vertical movement and retractable micro-wheels for horizontal traversal. Designed as a university project in computer engineering, this prototype integrates multi-sensor perception and embedded control in a low-cost, laser-cut mechanical platform.

![ProjectStatus](https://img.shields.io/badge/status-in%20progress-yellow)
![License](https://img.shields.io/badge/license-Academic--Use--Only-blue)


## Table of Contents
- [Overview](#overview)
- [Objectives](#objectives)
- [Repository Structure](#repository-structure)
- [Key Features](#key-features)
- [Documentation](#documentation)
- [Project Status](#project-status)
- [Team and Affiliation](#team-and-affiliation)
- [Acknowledgment](#acknowledgment)
- [License](#license)


## Overview

Bose is designed for indoor environments with stairs and flat ground. Its goal is to combine affordability, mechanical simplicity, and autonomy. Its locomotion system adapts in real time based on terrain using ultrasonic sensing, monocular SLAM, and encoder feedback. A Raspberry Pi 4B runs the entire stack under Ubuntu 22.04 + ROS 2 Humble.

> 🧪 *This project was developed as part of the Robotics, Language and Planning course (Spring 2025), UAB.*


## Objectives

- Design a low-cost stair-climbing robot prototype
- Combine mechanical and sensor-based terrain adaptation
- Enable SLAM-based indoor navigation
- Achieve smooth transitions between locomotion modes


## Repository Structure
```bash
├── docs/                # Final report, design logs, decisions
│   ├── logs/            # Daily development logs
│   ├── media/           # Images, CAD screenshots, diagrams
│   ├── decisions.md     # Engineering decisions with rationale
│   └── final_report.md  # Academic paper (Markdown format)
│
├── software/            # Python scripts, ROS 2 nodes, control logic
│
├── hardware/            # CAD files, wheel models, circuit diagrams
│
└── README.md            # Project overview (this file)
```


## Key Features

- 🌀 **Tri-helix wheels** — laser-cut spiral geometry for stair ascent
- 🤖 **Retractable wheel system** — servo-lifted micro wheels for ground mobility
- 🧠 **Onboard autonomy** — Pi 4B with ROS 2 and sensor fusion
- 👁️ **Monocular SLAM** — ORB-based tracking and sparse point cloud mapping
- 📡 **Sensor suite** — LiDAR, camera, IMU, encoders, ultrasonic sensor
- 🔧 **Low-cost fabrication** — plywood chassis, 3D-printed mounts


## Documentation

You can explore:
- [`decisions.md`](docs/decisions.md) — rationale for major technical choices
- [`logs/`](docs/logs) — chronological logs of design & testing progress
- [`final_report.md`](docs/final_report.md) — scientific paper detailing the project


## Project Status

> ⚠️ **This project is currently in course.**  
> Final submission deadline: **June 2025**  
> Demo video available on YouTube (link in final report).


## Team and Affiliation

**Contributors:**
- Arun Sharma  
- Pau Domínguez Ruiz  
- Gerard Souto Eslava  
- Chengjie Peng Lin  

**Supervision:**  
Prof. Fernando L. Vilariño Freire  
Prof. Carlos G. Calvo  
Prof. Vernon S. Albayeros Duarte

**Affiliation:**  
Bachelor’s in Computer Engineering  
Universitat Autònoma de Barcelona (UAB)  
Open Labs — *Laboratoris d’Innovació Oberta, Escola d’Enginyeria, UAB*


## Acknowledgment

The authors gratefully acknowledge:
- The instructors and Open Labs technical staff at UAB
- Maxon Group Spain for motor specification support
- Robot Electronics for help with motor driver debugging

The name **Bose** is inspired by the *Bose–Einstein distribution*, metaphorically representing state synchronization across heterogeneous environments.


## License

This academic repository is for educational and research purposes only.  
A formal open-source license will be attached after final evaluation.  
Expected: MIT or CC BY-SA.
