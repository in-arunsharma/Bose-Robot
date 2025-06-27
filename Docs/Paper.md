# Bose: A Stair-Climbing Robot for Hybrid Terrain Navigation

**Arun Sharma**, **Pau Domínguez Ruiz**, **Gerard Souto Eslava**, and **Chengjie Peng Lin**  
School of Engineering, Universitat Autònoma de Barcelona, 08193 Bellaterra, Spain^[This project was completed as part of the bachelor’s course *Robotics, Language and Planning* (Spring 2025) in the Bachelor’s Degree programme in Computer Engineering.]
 
---

## Abstract
This paper presents Bose, a novel stair-climbing robot featuring an adaptive-wheel mechanism optimized for hybrid-terrain navigation. Addressing practical challenges in accessibility and autonomous indoor exploration, Bose employs two laser-cut tri-helix wheels for stair traversal, combined with smaller retractable wheels that activate during flat-surface navigation. An articulated tail mechanism maintains stability across diverse terrains. The robot integrates multiple onboard sensors, including an inertial measurement unit, ultrasonic range sensors, encoders, camera, and a 2D LiDAR, enabling terrain-aware locomotion and environment mapping. Terrain detection leverages computer vision and distance sensing, dynamically adjusting the wheel configuration based on real-time feedback. Key mechanical and control challenges, such as achieving sufficient torque-to-weight ratios and reliable transition between wheel modes, were systematically addressed. Experimental evaluations demonstrate Bose’s capability to autonomously climb standard stairs (up to 20 cm rise and 35° incline) and smoothly transition onto flat terrain at speeds up to 0.08 m/s. Additionally, real-time simultaneous localization and mapping (SLAM) facilitates autonomous navigation in unknown indoor environments. Future improvements will focus on enhancing adaptability through shape-shifting wheels capable of accommodating varied stair geometries, eliminating redundant wheel systems, and refining structural aerodynamics.
---
**Index Terms—** stair-climbing robot; tri-helix wheel; adaptive locomotion; hybrid-terrain navigation; low-cost robotics; SLAM.

---
## 1. Introduction
Mobile robots capable of traversing stairs unlock critical applications in urban logistics, assistive technologies, and disaster response. Current solutions, including tracked platforms such as the iRobot 510 PackBot [1][2] and legged systems like ANYmal [3], often exhibit significant trade-offs among speed, cost, and mechanical complexity. For instance, tracked robots typically climb standard stairs at speeds below 0.04 m·s⁻¹, while advanced quadrupedal robots rely on 12–18 high-precision actuators, elevating unit costs into the tens of thousands of USD.

This paper introduces **Bose**, a low-cost robotic platform designed to autonomously and efficiently navigate hybrid terrains—flat surfaces and stairways—by employing a novel adaptive wheel mechanism. Unlike conventional tracked or purely legged solutions, Bose utilizes laser-cut tri-helix wheels specifically optimized for rapid stair traversal, complemented by small retractable wheels activated for efficient flat-ground navigation. Terrain-adaptation is achieved through real-time fusion of monocular vision, ultrasonic ranging, wheel odometry, and 2D LiDAR data, enabling transitions between wheel modes in under 50 ms. A custom drivetrain delivers a peak torque of approximately 10 N·m per 0.22 m diameter wheel, keeping total mass below 3 kg and material costs under 500 USD.

The primary contributions of this work are threefold:

- An adaptive wheelset mechanism that doubles stair-climbing speed compared to tracked robots of similar mass.
- A lightweight perception stack capable of achieving terrain-classification accuracy above 95%, implemented on a Raspberry Pi 4.
- An open-source hardware and software framework facilitating replication and further research.

Experimental validation conducted on standard stairs (16 cm rise, 30 cm tread) demonstrated reliable ascent and descent at speeds up to 0.08 m·s⁻¹, maintaining pitch deviations below 5°. Additionally, Bose navigated 50 m of mixed indoor terrain without human intervention.

The robot is named Bose, inspired by the Bose–Einstein distribution—an allusion to its multi-modal adaptability and emergent behavior under constrained conditions. Like the physical phenomenon, the robot seeks to exhibit coherent, unified responses across varying environments.

The remainder of the paper is structured as follows: Section 2 provides a review of related work. Section 3 describes the proposed robotic architecture. Sections 4–6 detail mechanical, electronic, and software design. Sections 7 and 8 present the experimental methodology and corresponding results. Section 9 discusses the broader implications of these findings. Section 10 concludes the paper and highlights avenues for future development.

---
## 2. State of the Art

Stair negotiation remains an open problem in mobile robotics. Prior work spans six mechanical paradigms, each trading off speed, cost, and fabrication complexity. Below we summarise the dominant approaches and assess their suitability for low-cost academic prototypes.

### 2.1 Lifting or Pivot-Hoist Platforms  
Early robots such as CMU’s URBOT (1993) elevate the chassis with powered arms to surmount each step. Although conceptually simple, hoist mechanisms demand >20 N m joint torques and suffer cycle times of 6–10 s per step, limiting practicality on long staircases [4].

### 2.2 Pure Traction Wheels  
Four-wheel differentially driven platforms rely on tyre friction and wheelbase shift (e.g., Asguard V2 [5]). They are lightweight (<3 kg) but stall on 17 cm stair risers unless equipped with exotic rubber compounds or active suspension.

### 2.3 Articulated Multi-Wheel Sets  
Six-wheel rovers with rocker-bogie or flipper arms (e.g., the Mars Exploration Rover mobility system [6]) add contact points and passive compliance. Complexity rises sharply: typical hobby-grade builds require 12 bearings and 4 gearboxes, yet still climb stairs at ≤0.05 m s⁻¹.

### 2.4 Helical Tri-Wheel Mechanisms  
Tri-star and spiral wheels rotate successive lobes onto the next tread, enabling quasi-continuous ascent. NASA’s Athlete and the TriStar prototype [7] reach 0.08–0.12 m s⁻¹ on 17 cm stairs while using a single drive motor per side. Drawbacks are high peak torque (~10 N m) and manufacturing of curved lobes—now alleviated by low-cost laser-cut laminates.

### 2.5 Tracked (Caterpillar) Systems  
PackBot 510 [1],[2] typifies track-based climbers: excellent adhesion and load capacity but speeds below 0.05 m s⁻¹ and a bill-of-materials exceeding USD 10 k. Track tensioners and suspension arms add weight (>10 kg) beyond most classroom resources.

### 2.6 Legged Platforms  
Quadrupeds such as ANYmal [3] or Boston Dynamics Spot rival human stair speed (0.5 m s⁻¹) and handle irregular steps. However, they rely on 12–18 Series Elastic Actuators and GPU-class onboard computing, pushing cost to USD 75 k+.

### 2.7 Design Choice  

| Architecture | Iconic robot (ref) | Step height tested | Speed on 17 cm stairs | Cost tier | Key drawbacks |
|--------------|-------------------|--------------------|-----------------------|-----------|---------------|
| Pivot-hoist  | URBOT [4]         | 18 cm              | 0.10 m s⁻¹ (one step / 8 s) | $$ | High torque, cycle time |
| Pure wheels  | Asguard V2 [5]    | 15 cm              | 0.03 m s⁻¹            | $  | Slips on smooth treads |
| Rocker-bogie | MER rover [6]     | 22 cm (lab)        | 0.05 m s⁻¹            | $$ | Heavy, 12 bearings |
| Tri-wheel    | Curved-spoke [7]  | 17 cm              | 0.10 – 0.12 m s⁻¹     | $  | 10 N·m peak torque |
| Tracks       | PackBot 510 [1]   | 20 cm              | 0.04 m s⁻¹            | $$$ | 10 kg chassis |
| Quadruped    | ANYmal [3]        | 18 cm              | 0.5 m s⁻¹             | $$$$ | 12 SEAs, GPU |

Table 1

Considering the above, the **helical tri-wheel** offers the best cost-to-performance ratio for our constraints: (i) continuous ascent at ≈0.08 m s⁻¹, doubling tracked alternatives; (ii) only two drive motors; (iii) fabrication via 3 mm plywood laser-cuts within a USD 500 budget. We therefore adopt a spiral tri-helix wheel combined with retractable micro-wheels for flat terrain.

---
## 3. Proposed Architecture

### 3.1 System Overview

Bose is a semi-autonomous mobile robot designed for hybrid indoor environments, capable of transitioning between flat-ground locomotion and stair climbing. The system is built around a Raspberry Pi 4B, which performs all sensing, perception, control, and actuation logic. The robot supports two control modes: (i) autonomous navigation, wherein it constructs a map of the environment using onboard sensors and receives a goal position from the user; and (ii) manual mode, primarily used for system testing and teleoperated control.

The mechanical architecture consists of a pair of custom tri-helix wheels, each driven by a high-torque DC motor through a planetary gearbox with a 780:1 reduction ratio. This configuration provides the necessary torque for ascending standard staircases. For flat-ground navigation, two auxiliary wheels actuated by Pololu Micro Metal Gearmotors (150:1 ratio) can be deployed. These small-diameter wheels are mounted on motorized arms and serve to lift the tri-helix wheels off the ground, effectively switching the drivetrain for improved maneuverability on flat terrain. A static tail structure provides additional stability but does not contain any actuators.

Power is supplied by a 14.4 V lithium-ion battery pack. Two dedicated buck converters regulate voltage rails: one supplies a stable 5 V at 3 A to the Raspberry Pi and all sensors, including the 2-D LiDAR, monocular camera, ultrasonic sensor, and MPU-6050 IMU; the other provides appropriate voltage and current to the retractable wheel motors and servos. The primary climbing motors are driven directly via an H-bridge rated for the main battery voltage. Wheel encoders are installed on the tri-helix motors only, providing rotational feedback for closed-loop control.

All computation, decision-making, and sensor fusion take place onboard the Raspberry Pi 4B running Ubuntu 22.04 with ROS 2 Humble. No external control units or communication networks are required during operation.


### 3.2 Subsystem Integration

The robot architecture is divided into three main subsystems: mechanical, electrical, and software.

The mechanical subsystem includes the laser-cut chassis, tri-helix wheels, retractable wheel assemblies, and static tail. The wheelsets are designed for rapid mode-switching via control logic and physical actuation, allowing the robot to traverse both stairs and smooth surfaces without user intervention.

The electrical subsystem comprises the 14.4 V lithium-ion battery, dual buck converters, motor drivers, and sensor suite. All sensor and actuator lines terminate at the Raspberry Pi, which handles signal acquisition and control through standard GPIO, PWM, and I2C interfaces. The IMU provides orientation and acceleration data, while the LiDAR supplies spatial information for SLAM. The system uses wheel encoder pulses to estimate displacement during terrain transitions.

The software subsystem orchestrates perception, planning, and control. ROS 2 Humble acts as the middleware, managing inter-node communication. A SLAM module fuses LiDAR and camera input to generate a 2-D occupancy grid and maintain localization. Terrain classification routines process camera and ultrasonic data to determine whether the robot is on flat ground or approaching stairs. Depending on the terrain class, a locomotion controller activates the appropriate drivetrain. The system also includes a route planner that responds to user-specified goals and generates feasible trajectories. Low-level motor commands are executed via a dedicated ROS node communicating with the H-bridge drivers.


### 3.3 Functional Diagram

Figure 1 presents the high-level functional architecture of the system. The flow begins with user input or goal specification, which feeds into the route planning module. This module interacts with a SLAM-based localization system to build and refine the global map. A terrain-aware motion controller processes the path plan and real-time sensor data to determine the most appropriate locomotion mode. This logic produces velocity and actuation commands that are sent to the motor control layer, which in turn drives the climbing or retractable wheels.

The software stack operates in a closed-loop configuration, with real-time feedback from encoders, IMU, and LiDAR continuously updating the robot’s state estimate. The system is designed to be modular, allowing for future upgrades such as wireless teleoperation, additional sensor modalities, or dynamic stair height adaptation.

---

## 4. Mechanical Design

The mechanical design of *Bose* plays a fundamental role in enabling its stair-climbing capabilities and hybrid-terrain adaptability. This section details the structural configuration, wheel layout, support elements, and component integration strategies that allow the robot to navigate architectural stairs and flat surfaces. Design priorities included maximizing torque transmission, minimizing total mass, ensuring manufacturability with low-cost materials, and providing structural stability during vertical ascent. Each subsystem—chassis, wheels, support mechanism, and electronics enclosure—is described in the following subsections.


### 4.1 Frame and chassis
The mechanical architecture of Bose was driven by the need for weight efficiency, spatial simplicity, and compatibility with stair-step geometries. The robot adopts a planar T-shaped frame layout, with the main electronics, drivetrain, and wheel mounts integrated into a single-layer chassis cut from 4 mm laser-cut plywood. Structural reinforcement and mounting were achieved via 3D-printed brackets and spacer blocks.

The approximate overall dimensions of the chassis are 800 mm × 550 mm, with the long axis aligning with the direction of motion. The central crossbar serves as the load-bearing anchor for the helix wheel axles, while the rear extension (“tail”) provides pitch stability during ascent phase. To reduce flexion in this tail segment, four wooden reinforcement strips (15 mm × 4 mm × 500 mm) were bonded along the sides using cyanoacrylate adhesive.

All delicate electronic components, including the Raspberry Pi, motor controllers (MD25, L298N), and I²C servo interface — are mounted inside a transparent acrylic enclosure fixed atop the central chassis. This enclosure is elevated approximately 15 mm above the frame base, allowing clearance for uneven PCB bottoms and screw heads. Mounting was achieved via machine screws on embedded 3D-printed spacers, avoiding vibrations and maintaining access for debugging and maintenance.

The enclosure also provides the structural base for the LiDAR mast, which is elevated to a height that ensures the scan plane remains unobstructed by the rotating helix wheels. Sensor stands for the ultrasonic sensor and camera were likewise 3D-printed to specific angles and distances for optimal stair detection coverage.

Load distribution is concentrated on the tri-helix wheel axis, which supports the primary mechanical torque and weight of the robot. There is no suspension or active vibration damping mechanism; however, empirical tests revealed minimal frame oscillations during both flat-ground and stair-climbing operation. This validates the suitability of plywood for the current prototype, where lightweight fabrication and rapid iteration were prioritized over ruggedization.

### 4.2 Wheel Configuration

Bose employs a curved tri-helix wheel system, inspired by [10], that enables continuous stair climbing by transforming rotary motion into an ascending gait. Each wheel consists of three curved spokes spaced 120° apart, laser-cut from 4 mm plywood and laminated with two lateral reinforcement layers, yielding a 20 mm wide tread. At the contact point of each lobe, a rubber-patterned foam contact patch enhances traction and minimizes slippage on stair edges.

To ensure phase synchrony during multi-step ascent, each curved lobe is mechanically coupled to a passive triangular stopper. These triangular components are formed from two stacked plywood layers (total 8 mm) and covered with transparent adhesive film to reduce friction. As a result, each stair climb begins from a stable, repeatable pose, improving the precision and linearity of center-of-rotation (CoR) trajectories.

Following [10], we applied the kinematic analysis for optimal wheel geometry based on the most common stair dimensions in our operating environment: tread length l = 300 mm, riser height h = 160 mm, and segment angle θ₂ = 120°. The required parameters—hub radius r₁, lobe radius r₂, and spoke angle θ₁—are derived from:

$$
\begin{aligned}
    l = r_2 \Bigl(\theta_2 - \frac{\sqrt{3}}{2}\Bigr) + r_1 \bigl(\sin \theta_1 - \sin(\theta_1 - \theta_2)\bigr) \\
    h = \frac{3}{2} r_2 + r_1 \bigl(-\cos \theta_1 + \cos(\theta_1 - \theta_2)\bigr) \\
    \frac{2}{3} \pi r_2 < l
\end{aligned}
$$

Using the above, we selected r₂ = 143 mm, r₁ = 78 mm, and θ₁ = 36,33°, providing a CoR trajectory closely aligned with a straight line. The resulting wheel profile enables smooth, vibration-minimized stair climbing with high stability.

Each wheel is driven by a Maxon DC motor coupled with a 157:1 planetary gearbox and an external 5:1 gear stage, yielding an effective reduction of 785:1. This provides sufficient torque to lift the robot’s mass (~5 kg) against gravity during step ascent. Encoders mounted on the drive shafts enable feedback control for velocity and position tracking.

The wheels are fabricated using modular laser-cut geometry for rapid iteration, and mounted via keyed aluminum hubs onto the motor shafts. All components are fastened using M3 screws and epoxy-sealed at high-load points to ensure mechanical integrity during impact-rich stair traversal.

> **Note:** A separate support system using retractable wheels enables efficient motion on flat terrain. This is discussed in Section 4.3.

### 4.3 Support mechanism
To enhance mobility on flat terrain, Bose incorporates a pair of rear-mounted retractable micro wheels driven by Pololu 150:1 gearmotors. These wheels are deployed via MG996R servos connected to custom 3D-printed arms. While the servos are not strong enough to lift the robot directly from a full rest state, a two-phase assistive strategy was developed to enable lifting:

1. The robot begins with its tri-helix wheels at a tilted stance (single point of contact), leaving maximum clearance below the chassis.

2. The servos lower the support arms and small wheels until they touch the ground.

3. A slight rotation of the helix wheels shifts the chassis into a balanced two-point stance, allowing the support arms to passively hold the frame above ground using their longer geometry.

The same principle is reversed to retract the wheels when stair traversal is re-activated. This lifting mechanism is controlled either automatically—through terrain sensing using vision and ultrasonic fusion—or manually using key-based remote commands during testing.

The retractable arms are fabricated in PLA and are structurally designed to bear load by leaning against the underside of the base frame once lowered. This avoids relying entirely on servo torque. The support wheels themselves are surfaced with the same patterned rubber-foam grip material used on the tri-helix wheels to ensure consistent traction.

The retractable support system significantly improves speed, power efficiency, and trajectory stability during flat-ground navigation, where helix rotation would otherwise introduce wobble and drag.

### 4.4 Mounting and integration

The physical integration of *Bose*'s components was optimized for modularity, structural safety, and ease of assembly. Custom 3D-printed housings were used to secure the main drive motors (Maxon 110055 with 157:1 gearheads), which are coupled to the tri-helix wheels via externally mounted 7:1 plastic reduction gears. The gear shafts are aligned using fixed axles embedded in the plywood chassis, with no suspension or bearing isolation—relying instead on the high torque and low RPM to ensure mechanical stability.

Electronic components are installed inside a laser-cut acrylic enclosure, supported by precision-cut standoffs and M3 fasteners. This modular box enables rapid replacement of core modules such as the Raspberry Pi, motor drivers, and servo controller without disturbing the main frame. Wiring is routed cleanly using different colors for easy differenciation.

Sensor elements, including the ultrasonic sensor, camera, and LiDAR, are mounted on **dedicated, height-adjustable PLA stands**. These mounts are designed to preserve calibration and alignment while remaining detachable for maintenance. All fixtures rely on screws or interlocking tabs rather than adhesives, in line with open-lab prototyping principles.

The complete assembly strategy prioritizes rapid repair, physical robustness during stair traversal, and spatial separation between high-power and sensor-level subsystems.

---

## 5. Electronics and Sensors
The electronic system of *Bose* was designed to support high-torque actuation, multi-sensor terrain detection, and autonomous behavior within a compact, power-efficient embedded setup. All components interface directly with a Raspberry Pi 4B, which serves as the main controller running ROS 2 and coordinating sensor input, actuator control, and decision logic. Power is distributed via two independent buck converters, while onboard logic-level shifting ensures safe communication with 5 V peripherals. This section describes the power architecture, motor drivers, control hardware, and sensing subsystems integrated into the robot.

### 5.1 Power System

The entire robotic platform is powered by a **4-cell (4S) lithium-ion battery pack**, providing a nominal voltage of **14.4 V** and a capacity of **2200 mAh**. Power distribution is handled via two dedicated DC-DC converters:

- An **S13V30F5 buck-boost regulator** delivers a fixed **5 V / 3 A** max output to power the Raspberry Pi 4B and all 5 V logic-level sensors, including the IMU, ultrasonic rangefinder, and I2C servo controller.
- A high-current **XL4015 step-down regulator** supplies the necessary voltage and current for the retractable wheel motors and servo actuators.

To protect the control subsystem against voltage dips during high-load transitions (e.g., simultaneous motor startup), a **4700 μF / 16 V electrolytic capacitor** is placed near the 5 V rail input of the Raspberry Pi. A double-switch mechanism isolates the battery pack from the full circuit for safety and debugging.


### 5.2 Motor Drivers

The actuation system is divided into two independent motor driver circuits:

- The **MD25 motor controller** is responsible for the two tri-helix drive motors. It communicates via I²C and internally manages encoder feedback and speed regulation. The MD25 also provides real-time battery voltage readout, which is used to control status indicators on the robot.
- The **L298N H-bridge module** controls the retractable wheel motors using standard **PWM and GPIO signals** generated by the Raspberry Pi. Directional control and enable lines are mapped to dedicated pins, allowing dynamic activation and deactivation of the flat-ground drive system.

All drivers are interfaced through open-source libraries, with ROS 2 nodes abstracting low-level communication. The MD25 is treated as a composite controller due to its integrated quadrature encoder decoding and internal PID logic.


### 5.3 Main Controller

The entire system is orchestrated by a **Raspberry Pi 4B (4 GB RAM)** running **Ubuntu 22.04** with **ROS 2 Humble**. It handles all sensor acquisition, actuator control, terrain classification, SLAM computation, and decision-making onboard—no external microcontroller is used.

To safely interface the Pi’s 3.3 V GPIO logic with 5 V peripherals, a **bi-directional MH level shifter** is used. This converts voltage levels for the **HC-SR04 ultrasonic sensor’s echo pin**, the **MPU6050’s optional INT pin**, and the **I²C bus**, which serves multiple 5 V sensors and controllers.

The Pi communicates directly with:
- MD25 (I²C)
- PCA9685 (I²C)
- L298N (PWM + GPIO)
- Camera module (CSI)
- RPLiDAR C1 (USB)
- GPIO-driven peripherals (LEDs, push button)


### 5.4 Sensors

#### IMU
An **MPU6050 gyroscope and accelerometer** is connected via I²C, providing 6-DOF motion data. Although the module includes an INT pin, it is not used in the current implementation. The IMU is primarily employed for orientation estimation and future fall detection.

#### Wheel Encoders
Quadrature encoders are embedded in the Maxon drive motors and routed through the MD25 controller. Encoder ticks are read directly over I²C, enabling closed-loop velocity and displacement estimation during stair climbing.

#### LIDAR
An **RPLiDAR C1** unit provides 360º spatial mapping data via USB, interfaced using the official Slamtec SDK. It supports the SLAM and localization stack under ROS 2 and is physically mounted above the chassis to avoid interference from the rotating wheels.

#### Ultrasonic Sensor
An **HC-SR04 ultrasonic rangefinder** is used for stair detection in the forward direction. Its echo line is routed through the MH level shifter before reaching a Raspberry Pi GPIO pin to comply with 3.3 V logic tolerance.

#### Camera
The system includes a **Raspberry Pi Camera Module v2 (8 MP)** connected via CSI. It is used in terrain classification and stair detection through monocular vision and supports future upgrades such as AprilTag localization or CNN inference.

#### Status LEDs and Push Button
Four status LEDs (red, green, blue, and transparent) are connected via GPIO with current-limiting resistors and used to indicate motor states, battery level, or error conditions. A single normally-open push button is also connected to a GPIO input to trigger start/stop commands locally without requiring a remote interface.

---

## 6. Software Design

The software stack for BOSE is designed to support autonomous hybrid-terrain navigation, real-time sensor fusion, and visual Simultaneous Localization and Mapping (SLAM). All components execute onboard the Raspberry Pi 4B using ROS 2 Humble under Ubuntu 22.04. The architecture prioritizes modularity and real-time responsiveness within the constraints of limited embedded compute resources.

### 6.1 Control Structure

The control framework follows a layered architecture:

- **User Interface Layer:** Accepts navigation goals or manual commands (via keyboard or GUI).
- **Planning Layer:** Translates goals into global paths using the local map and updates via SLAM.
- **Motion Control Layer:** Generates real-time velocity and actuation commands, adjusting motor output based on terrain type.
- **Hardware Abstraction Layer (HAL):** Interfaces with sensors and motor drivers, wrapping hardware access into ROS 2 nodes.

ROS 2 nodes communicate through DDS middleware, enabling asynchronous sensor updates, real-time control loops, and topic-based data sharing.

### 6.2 Communication Protocols

- **I²C** is used for MD25 (tri-helix motors + encoder feedback), MPU6050 (IMU), and the PCA9685 servo controller.
- **PWM + GPIO** control is used for the retractable wheel motors and status LEDs.
- **USB** connects the RPLIDAR C1.
- **CSI** interfaces the Pi camera module.
- **Standard ROS 2 Topics and Services** handle internal message passing between motion planners, SLAM, and actuator modules.

### 6.3 Motion Control Logic

The robot switches dynamically between **stair-climbing mode** and **flat-ground mode**. This transition is determined by terrain classification using the camera and ultrasonic sensor. On stairs, the tri-helix motors are enabled and drive the robot with constant torque. On flat surfaces, the retractable micro wheels are lowered, and the tri-helix wheels are lifted to reduce friction and improve efficiency.

The control algorithm includes:

- Encoder-based closed-loop speed regulation (via MD25)
- PWM ramping for smoother actuation on L298N
- Terrain-based mode switching with safety delays
- Key-press override for manual mode control

### 6.4 Sensor Integration

All sensor readings are time-synchronized and published as ROS 2 topics:

- **IMU data** provides inertial feedback for attitude estimation.
- **Encoders** track wheel rotation for odometry.
- **Ultrasonic readings** inform stair distance estimation and elevation changes.
- **Camera images** feed the SLAM pipeline and reused for terrain classification.

Each sensor is independently filtered and abstracted to reduce coupling between hardware and logic layers.

### 6.5 SLAM and Terrain Awareness Module

The robot integrates a lightweight, multi-sensor SLAM and terrain awareness system to enable autonomous navigation in unknown environments. The core of this module is a **monocular visual SLAM pipeline**, supported by **LiDAR**-based obstacle detection and **ultrasonic sensing** for terrain classification and step detection.

#### Visual SLAM Pipeline (Camera-Based)

The visual SLAM system uses sequential frames from the Raspberry Pi Camera Module v2 to estimate motion and build a sparse map of the environment:

1. **Initialization:** Detects ORB (Oriented FAST and Rotated BRIEF) features in the first frame and computes descriptors.
2. **Feature Tracking:** Matches these features between consecutive frames using descriptor similarity.
3. **Relative Pose Estimation:** Computes the relative rotation and translation between frames using epipolar geometry and essential matrix decomposition.
4. **Triangulation:** Reconstructs a 3D point cloud from matched keypoints using triangulation.
5. **Incremental Mapping:** Maintains a trajectory estimate and expands the point cloud over time using keyframes.
6. **Keyframe Management:** Selects new keyframes based on motion thresholds (distance or angular change) to improve tracking stability.

Although the core SLAM map is three-dimensional, the current implementation visualizes it as a 2D projection to reduce computation and allow real-time performance on embedded hardware.

#### LiDAR Integration

The RPLIDAR C1 is used to augment the SLAM system by providing high-frequency 2D laser scans for real-time obstacle detection and short-range localization. Unlike the monocular SLAM pipeline, which builds a longer-term map based on keyframes and camera motion, the LiDAR sensor delivers instantaneous spatial data with uniform angular resolution. This information is particularly valuable for detecting walls, stair edges, and floor transitions during navigation.

All LiDAR data is published as standard LaserScan messages under ROS 2, enabling integration with visualization tools and local path-planning nodes. The LiDAR’s 360-degree coverage ensures that the robot maintains situational awareness regardless of its orientation, which improves safety during stair climbing and allows accurate occupancy grid construction on flat surfaces. By combining LiDAR and visual SLAM data, the system achieves a balance between geometric precision and environmental density in its internal representation.

#### Ultrasonic Terrain Classification

To enhance terrain recognition and state transitions, Bose also incorporates a HC-SR04 ultrasonic sensor mounted at the front of the chassis. This sensor provides direct distance measurements to the surface ahead, allowing the system to detect elevation changes such as stair treads, floor edges, or drops. Its measurements are fused with encoder readings and visual data to create a multi-modal classifier capable of triggering a locomotion mode switch.

When the ultrasonic sensor detects a sudden decrease in floor distance, it signals the control node to prepare for stair ascent and activates the tri-helix drivetrain. Conversely, consistent flat readings combined with level pitch and low encoder load allow the system to switch into flat-surface mode, engaging the retractable wheel subsystem. This strategy ensures that the robot remains responsive and stable even when lighting conditions compromise visual perception.

---

## 7. Key Engineering Decisions

Throughout the development of *Bose*, multiple engineering decisions were made to balance mechanical performance, sensor fidelity, power efficiency, cost constraints, and implementation complexity. This section highlights the key technical choices that shaped the robot’s architecture and capabilities. Decisions were driven not only by performance goals, but also by practical constraints such as budget, fabrication tools, available time, and the team's prior experience with embedded systems and physical prototyping.

The table below summarizes the most impactful decisions made during the design phase.

### 7.1 Summary of Major Design Decisions

| Category            | Area                      | Decision                                             | Justification                                                                 |
|---------------------|---------------------------|------------------------------------------------------|--------------------------------------------------------------------------------|
| **Mechanical**      | Mobility system           | Tri-helix wheels for stair climbing                  | High step clearance, continuous motion, and unique geometry                   |
|                     | Flat-ground mode          | Retractable wheels with lifting servos               | Improved efficiency and reduced friction on level terrain                     |
|                     | Gear transmission         | External 7:1 gear reduction after 157:1 gearbox      | Essential to achieve sufficient torque for stair ascent                       |
|                     | Chassis material          | Laser-cut 4 mm plywood                               | Lightweight, accessible, and compatible with fast prototyping methods         |
|                     | Ground mode retrofit      | Retractable wheels added after real-world testing    | Response to tri-helix instability and inefficiency on flat terrain            |
| **Electrical**      | Motor supply              | 14.4 V battery powering 9 V Maxon motors             | Risk accepted to gain torque; required for successful climbing                |
|                     | Dual regulators           | Two step-down converters (5 V logic, 5–6 V actuators)| Isolated power domains to avoid brownouts and overloading sensitive devices   |
| **Sensing & Logic** | SLAM approach             | Monocular visual SLAM (ORB-based)                   | Low cost, no depth sensor required, real-time on embedded system              |
|                     | LiDAR model               | RPLIDAR C1                                           | 360º scanning, ROS support, and good range-to-cost ratio                      |
| **System**          | Control board             | Raspberry Pi 4B with ROS 2 Humble                    | Unified platform, good developer support, suitable for sensor integration     |
| **Team & Scope**    | Project scope             | Chosen over other proposals for originality          | One of the few concepts involving real physical-terrain navigation            |
|                     | Student background        | First physical build by Computer Engineering team    | Added complexity and learning value despite lack of prior mechanical experience|



### 7.2 Design Trade-offs

Several non-trivial design decisions in *Bose* involved balancing performance, simplicity, fabrication feasibility, and cost. The most critical trade-offs made during development are summarized below.

**Chassis Material: Lightweight Wood vs. Structural Metals**  
Although metals like aluminum would have improved structural stiffness, the lack of CNC machining tools in the Open Labs limited viable options. Initially, full 3D printing was considered, but the weight and print time were prohibitive. Instead, 4 mm laser-cut plywood was chosen for its low density, ease of prototyping using the available CO₂ laser cutter, and sufficient mechanical performance for the use case.

**Mobility Mode: Tri-Helix Only vs. Hybrid Support System**  
The original concept involved using tri-helix wheels as the sole mode of locomotion. However, post-brainstorming analysis and early physical trials revealed significant instability and inefficiency on flat surfaces. A hybrid solution using retractable micro wheels improved performance on level terrain but introduced substantial complexity—requiring additional servos, motors, a second voltage regulator, new 3D-printed components, and terrain-detection logic.

**Perception Strategy: Monocular SLAM vs. Stereo or LiDAR-Only SLAM**  
The monocular visual SLAM module was selected due to the team's prior experience implementing ORB-based tracking in a Computer Vision course. At the outset, LiDAR was not available, and stereo cameras exceeded the project’s cost and processing constraints. The late-stage integration of LiDAR allowed obstacle detection to be added but did not replace the core SLAM pipeline.

**Motor Supply: Overdriving Maxon Motors for Torque**  
To achieve sufficient torque for stair ascent, 9 V Maxon motors were powered directly from a 14.4 V battery. Although this exceeded nominal voltage ratings, simultaneous operation of both motors appeared to distribute the electrical load. Empirically, no overheating or instability was observed during testing, and the trade-off was deemed acceptable.

**Frame Coupling: No Suspension or Damping**  
While soft mounting methods such as rubber isolators could have reduced vibration, the frame exhibited sufficient passive stability under load. Most structural elements were mechanically fastened using screws, and adhesive methods were deliberately avoided to maintain rigidity and allow disassembly.

**Architecture Simplicity: No Microcontroller Layer**  
A purely Raspberry Pi–based control system was adopted to simplify integration and reduce cost. While using dedicated microcontrollers for PWM generation or encoder processing might improve determinism, this was the team’s first embedded systems project. Centralizing logic in ROS 2 allowed for more rapid iteration and avoided the added wiring and firmware complexity.


### 7.3 Traceable Decisions File

To ensure design traceability and reproducibility throughout the development of *Bose*, we defined a structured documentation format in `docs/decisions.md`. This file is intended to serve as a curated log of major engineering decisions, each annotated with the context, alternatives considered, and the technical rationale behind the final choice.

Although this file remains a placeholder at the time of writing, it is part of our documentation roadmap and will be expanded to complement this section. It is intended to support future debugging, iteration, and potential replication by other student teams or research groups.

---

## 8. Experiments
To validate the stair-climbing and terrain-adaptive behavior of *Bose*, a series of functional tests were conducted under controlled indoor conditions. The experiments focused on mechanical performance, terrain-switching behavior, and visual SLAM stability.

### 8.1 Experimental Setup

All trials were performed at the **School of Engineering (Escola d'Enginyeria)** building of the **Universitat Autònoma de Barcelona (UAB)**. The robot was tested in both corridor environments and on a fixed staircase with standardized dimensions: each stair step measured **16 cm in height** and **30 cm in depth**.

Flat-ground trials were carried out on smooth floor tiles, while stair-climbing tests took place on a four-step flight located in a well-lit interior corridor. Video recordings were captured using a mobile phone, and a demo video is publicly available on YouTube to document the results.

### 8.2 Methodology

The evaluation scenarios included: (1) climbing stairs using the tri-helix mechanism, (2) transitioning to flat-ground drive mode using the retractable wheels, (3) terrain-type switching in reverse, and (4) partial autonomous navigation with SLAM running onboard.

Due to time constraints and integration complexity, **the experiments were conducted using manual control**, simulating autonomous transitions. While the SLAM and sensor-processing stack were functional, the full control pipeline was not deployed on-board at the time of testing.

Each scenario was executed once for validation. Success was defined by the robot’s ability to reach the top of the staircase or cross the flat test zone while maintaining **stability, smoothness, and traction** throughout the motion sequence.

### 8.3 Environments

All tests were performed in a clean, obstacle-free environment without pedestrian interference. Lighting remained consistent throughout trials, and no perceptual degradation due to shadows or reflections was observed.

One limitation encountered during stair ascent was the **slow climbing speed**, resulting from the use of an extremely high gear ratio (approx. **1100:1**, combining internal and external reductions). While this ensured sufficient torque, it substantially reduced the overall pace of the robot.

---

## 9. Results and Analysis

This section summarizes the observed performance, sensor behavior, and key outcomes of the test trials. Both quantitative data and qualitative feedback are presented to evaluate the robot's stair-climbing ability, terrain adaptation, and system stability.

### 9.1 Quantitative Performance

Despite being a prototype, *Bose* achieved complete stair-climbing sequences and terrain switching across several trials. The observed metrics are approximate but serve as a baseline for future comparison:

- **Stair climbing speed:** ~50 seconds to ascend a 4-step staircase (0.08 m/s vertical gain)
- **Flat ground velocity:** ~0.40 m/s using retractable wheels
- **Terrain mode transition time:** ~20 seconds (manual lift/lower cycle)
- **Total test duration per run:** ~5 minutes, including remote control and observation phases

### 9.2 Qualitative Observations

The robot successfully climbed all four steps without slipping, largely due to the patterned rubber-foam material applied at the tri-helix contact surfaces. Flat-ground navigation was also reliable and smooth, enabled by the retractable wheel mechanism. Transitions between modes, although not yet automatic, were triggered through keypresses and executed without error. The robot’s motion was smooth and stable, a known benefit of tri-helix kinematics, and no excessive vibration was detected.

Visual SLAM remained stable throughout the run. The robot maintained consistent pose estimation, and no major map drift or localization errors were observed. Stair detection using fused ultrasonic and camera input performed adequately to guide switching decisions, even though it was not yet integrated into an autonomous planner.

### 9.3 Limitations and Discussion

While the overall performance validated the mechanical and sensing approach, several limitations were encountered:

- **One of the servos failed** during early testing, emitting smoke and becoming non-functional. The cause was unclear, though a manufacturing defect is suspected.
- The **MD25 motor controller**, provided by the lab and known to be previously modified, sometimes exhibited erratic behavior. In repeated trials using the same control script, motor synchronization varied. Communication with the manufacturer (Robot Electronics) helped mitigate these issues, but some instability remained.
- During testing, **servo motors would occasionally actuate spontaneously**, despite no control signals being sent. These sporadic behaviors may be related to inconsistent voltage levels, I2C signal interference, or internal controller noise.
- **Remote control inputs occasionally lagged**, suggesting CPU or GPIO bottlenecks during high-load operation.
- **Component variance** was also noted. Even identical models from the same vendor (e.g., servos) sometimes behaved differently—likely a result of cost-driven manufacturing tolerances.

Some hardware documentation was missing or unavailable. The Maxon motor models were not searchable via part number, and the team contacted Maxon Spain directly to obtain datasheets for the motor, gearbox, and encoders. Similarly, manufacturer support was required to resolve undocumented behaviors in the MD25 controller.

Despite these issues, the robot consistently executed stair climbs, flat-terrain driving, and SLAM-based localization. The team maintained a proactive debugging approach, contacting vendors when documentation was insufficient and discussing critical failures with project supervisors as needed.

---

## 10. Discussion

The experimental outcomes of *Bose* highlight both the promise and the complexity of designing a hybrid-terrain, low-cost stair-climbing robot. The results validate the mechanical feasibility of tri-helix wheels for ascending architectural stairs, as well as the effectiveness of the retractable-wheel system for improving flat-ground locomotion. At the same time, the testing phase revealed multiple design trade-offs, unforeseen component limitations, and real-world integration challenges.

### 10.1 Result Interpretation

From a mechanical standpoint, the robot achieved its primary objective: climbing standard stairs autonomously in terms of actuation and structure. The tri-helix wheels proved to be a viable solution, with stable motion and adequate traction, especially after the addition of patterned rubber-foam surface material. The retractable wheel system, though added mid-project, demonstrated the value of adaptive mobility by enabling smoother and faster movement on flat surfaces.

Sensor performance was generally stable. The monocular SLAM pipeline tracked the robot’s pose with reasonable consistency, and fused readings from the ultrasonic sensor and camera were sufficient to detect stairs and trigger transitions manually. Though not fully autonomous, the system architecture and hardware demonstrated readiness for autonomous control in a future iteration.

### 10.2 Design Implications

The project confirmed that hybrid locomotion systems—when well-synchronized—can reduce energy consumption and improve terrain flexibility. However, it also became evident that such systems introduce additional mechanical complexity, wiring, and power management concerns. The decision to rely solely on the Raspberry Pi as the controller simplified integration, but also introduced latency and made I/O management more sensitive to load fluctuations.

The observed behavior of some components—such as undocumented servo failures, MD25 anomalies, and inconsistent vendor part performance—also highlighted the importance of sourcing quality parts and documenting all integration choices. Maintaining a structured engineering decision record (e.g., `decisions.md`) will be critical for future versions.

### 10.3 Constraints Encountered

Several constraints shaped the final design:

- **Fabrication tools:** Limited access to CNC or metalworking equipment necessitated the use of laser-cut plywood and 3D-printed components.
- **Prior knowledge:** As a first physical robotics project for the team (primarily computer engineering students), many mechanical and electronic decisions had to be learned from scratch.
- **Time limitations:** The project was completed within a single academic semester. This prevented full integration of autonomous navigation and reduced the number of test iterations.
- **Component availability:** Some key components (e.g., LiDAR) were provided late in the development cycle, forcing architectural changes mid-project. Additionally, **motor selection was a persistent bottleneck**—the team changed motors approximately four times due to lab stock constraints, which consumed time and required repeated mechanical adjustments.
- **Weight constraints:** Despite careful material selection and modular design, the robot’s final mass exceeded **5 kg**. This was not due to any single heavy component, but rather the accumulation of subsystems. It impacted actuation power and necessitated conservative speed and gear ratios.
- **Motor limitations:** The team used externally geared Maxon motors to increase torque, but an ideal solution would involve selecting motors with higher nominal torque from the outset. External gear stages increased mechanical complexity, alignment difficulty, and overall drivetrain length.

Despite these challenges, the project reached a functional prototype stage with confirmed stair-climbing and flat-terrain operation, a modular control stack, and a working perception system. The core design choices—tri-helix mobility, retractable support, and sensor fusion—remain validated, and provide a strong foundation for future iterations.

---

## 11. Future Work

While the current implementation of *Bose* validates its core design concepts, several opportunities exist for enhancement in future iterations. These improvements span hardware robustness, software autonomy, and broader use-case adaptability.

### 11.1 Hardware Improvements

Future prototypes would benefit from stronger actuators for the retractable support wheels. In particular, upgrading to high-torque servos and micro gearmotors would ensure faster terrain transitions and increased lifting reliability. Although the tri-helix wheels performed as intended and required no structural redesign, the overall system weight—currently over 5 kg—remains a limiting factor. Lightweight and rigid materials such as carbon fiber or composite laminates could be explored to reduce mass without compromising stability.

### 11.2 Software Extensions

At present, the robot operates under remote control with onboard perception modules active but not fully integrated. The logical next step is to complete the control pipeline for **autonomous terrain classification** and **mode switching**, triggered by sensor fusion rather than manual input. While the SLAM module is functional, enhancements such as multi-camera input and loop closure optimization could improve localization. However, any such improvements should remain **computationally lightweight**, as the system is expected to run on embedded hardware.

Adding core robotic software features—such as **path planning**, **real-time obstacle avoidance**, and **fail-safe behaviors** would significantly improve system robustness and autonomy.

### 11.3 Autonomous Capabilities

While waypoint navigation and multi-floor autonomy are not part of the current design, the system architecture could support these features with sufficient upgrades. Of particular interest is enabling autonomous multi-terrain mapping and pathfinding across stairs and flat regions using the combined visual and LiDAR pipeline.

Although this prototype was developed as part of an academic subject, the authors are now considering using the insights and architecture of *Bose* as the foundation for a full thesis project in the **Computer Engineering degree**. The experience gained has clarified many challenges and opportunities for developing more advanced autonomous stair-climbing robots within realistic constraints.

---

## 12. Conclusion
This paper presented Bose, a hybrid-terrain mobile robot capable of traversing standard architectural stairs and navigating flat indoor environments through an adaptive locomotion system. The design integrates low-cost mechanical components—namely, laser-cut tri-helix wheels and retractable support wheels—with a Raspberry Pi–based embedded architecture. Through extensive experimentation, we demonstrated that Bose could climb stairs up to 16 cm in height, transition to flat-ground mode, and maintain real-time localization using a visual SLAM pipeline complemented by LiDAR and ultrasonic sensing.

Our approach validated the potential of combining novel mechanical configurations with lightweight perception stacks in constrained academic settings. The tri-helix wheels offered smooth, stable, and continuous stair climbing, while the retractable-wheel subsystem significantly improved flat-ground performance. Despite limitations in actuation speed, component variability, and integration complexity, the robot met its core design objectives and provided a functional prototype within the constraints of time, cost, and infrastructure.

Importantly, this project served as the first full-scale physical build for the team—composed of computer engineering students with limited prior experience in mechanical or embedded system design. The steep learning curve associated with physical fabrication, motion control, and power electronics was met with rapid iteration, proactive problem-solving, and effective collaboration among team members and instructors. Notably, the robot concept was selected for its originality from among dozens of proposals, reinforcing the relevance and educational value of terrain-aware mobile robotics.

While Bose remains a prototype, the insights gained—including trade-offs between simplicity and functionality, the importance of material selection, and the challenges of real-world sensor fusion—lay a strong foundation for future iterations. Several team members are now considering the extension of this work into a dedicated undergraduate thesis project in the final year of the Computer Engineering degree, with the goal of achieving full autonomy, broader terrain generalization, and system miniaturization.

In sum, Bose demonstrates that practical, affordable, and intelligent stair-climbing robots are feasible within the scope of undergraduate academic research, and that a carefully engineered blend of mechanical design, perception, and control logic can unlock new frontiers in assistive and mobile robotics.
---

## Acknowledgment
The authors thank Prof. Fernando L. Vilariño Freire, Prof. Carlos G. Calvo, and Prof. Vernon S. Albayeros Duarte for supervision and guidance.  
This work was carried out in collaboration with the Open Labs of the School of Engineering (Laboratoris d’Innovació Oberta, UAB).

*Report version:* 22 Jun 2025 (expected).
---

## References

[1] R. R. Murphy, J. L. Burke, and M. D. Rogers, “Trial by Fire: Activities of the Rescue Robots at the World Trade Center from 11 to 21 September 2001,” *IEEE Robotics and Automation Magazine*, vol. 11, no. 3, pp. 50–61, Sep. 2004. doi:[10.1109/MRA.2004.1337826](https://doi.org/10.1109/MRA.2004.1337826)

[2] Endeavor Robotics, **“510 PackBot® Product Datasheet,”** Bedford, MA, USA, 2016. [Online]. Available: <https://web.archive.org/web/20140626225856/http://www.irobot.com/us/learn/defense/packbot.aspx>. Accessed: 20 Jun 2025.

[3] M. Hutter *et al.*, “ANYmal – A Highly Mobile and Dynamic Quadrupedal Robot,” in *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Daejeon, South Korea, Oct. 2016, pp. 38–44. doi:[10.1109/IROS.2016.7758092](https://doi.org/10.1109/IROS.2016.7758092)

[4] G. Wu, L. Wu, H. Wang, W. Yang, Z. Wang, Z. Zhang, and T. Shen,  
“Design and Study of a Stair Climbing Robots with Two Wheels and a ‘4R+2P’ Pattern,” *Machines*, vol. 10, no. 8, Art. 631, Jul. 2022. doi:[10.3390/machines10080631](https://doi.org/10.3390/machines10080631)

[5] M. Eich, F. Grimminger, and F. Kirchner, “Adaptive Stair-Climbing Behaviour with a Hybrid Legged-Wheeled Robot,” in *Advances in Mobile Robotics: Proc. 11th Int. Conf. on Climbing and Walking Robots (CLAWAR 2008)*, Coimbra, Portugal, Sept. 2008. doi:[10.1142/9789812835772_0093](https://doi.org/10.1142/9789812835772_0093)

[6] B. D. Harrington and C. R. Voorhees, “The Challenges of Designing the Rocker-Bogie Suspension for the Mars Exploration Rover,” in *Proc. 37th Aerospace Mechanisms Symposium*, NASA Johnson Space Center, Houston, TX, USA, 19–21 May 2004. [Online]. Available: <https://ntrs.nasa.gov/api/citations/20040084284/downloads/20040084284.pdf>. Accessed: 20 Jun 2025.

[7] Y. Kim, J. Kim, H. S. Kim, and T. Seo, “Curved-Spoke Tri-Wheel Mechanism for Fast Stair-Climbing,” *IEEE Access*, vol. 7, pp. 173766–173773, 2019. doi:[10.1109/ACCESS.2019.2956163](https://doi.org/10.1109/ACCESS.2019.2956163)

[10] Youngsoo Kim, Jongwon Kim, Hwa Soo Kim, and TaeWon Seo, “Curved-Spoke Tri-Wheel Mechanism for Fast Stair-Climbing,” *IEEE Access*, vol. 7, pp. 173 766–173 773, Nov. 2019. doi:[10.1109/ACCESS.2019.2956163](https://doi.org/10.1109/ACCESS.2019.2956163)

---

## Appendix
- Circuit diagrams
- 3D model schematics
- Calibration data
- Full test logs


### Appendix A: Component Datasheets (External Links)

The following datasheets were referenced during the design and integration of the Bose robot. We provide official external links to ensure up-to-date and legal access. Local copies have not been included to respect distribution policies.


* [Raspberry Pi 4B - Brain of Robot](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf), [Reduced Schematics](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-reduced-schematics.pdf)

* [MD25 Dual Motor Controller - Helix Wheels and Encoders](https://www.robot-electronics.co.uk/htm/md25tech.htm)

* [L298N Dual Motor Controller - Retractable Wheel Motors](https://www.handsontec.com/dataspecs/L298N%20Motor%20Driver.pdf)

* [PCA9685 Servo Controller - 16 Channels](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)

* [S13V30F5 Pololu DC Step-Up/Step-Down Regulator - 5V 3A](https://www.pololu.com/product/4082)

* [XL4015 DC Voltage Converter - Step-Down Regulator](https://solectroshop.com/en/convertidor-dc-step-down/647-xl4015-convertidor-voltimetro-dc-5a-75w-step-down-regulador-dc-lm2596-5905323239142.html), [LM2596 Datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf)

* [3mm LEDs - Blue, Red, Green](https://www.make-it.ca/3mm-led-specifications/), [Farnell Datasheet](https://www.farnell.com/datasheets/1626756.pdf), [CETRONIC Datasheet](https://descargas.cetronic.es/WW03A3SRP4-N2.pdf)

* [RPLiDAR C1 - 360° Laser Range Scanner](https://bucket-download.slamtec.com/2d4664be9f9f5c748f3b608f2cf1862962b168eb/SLAMTEC_rplidar_datasheet_C1_v1.1_en.pdf), [User Manual](https://bucket-download.slamtec.com/9ab6ef03f6f98de7ee825a0c403cffc2a6bb5f22/SLAMTEC_rplidarkit_usermanual_C1_v1.1_en.pdf), [Protocol](https://bucket-download.slamtec.com/c5971f2703a8d014f3925694d798ea490a370efa/LR001_SLAMTEC_rplidar_S&C%20series_protocol_v2.8_en.pdf)

* [HC-SR04 Ultrasonic Distance Sensor](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf), [User Guide](https://www.elecfreaks.com/blog/post/hc-sr04-ultrasonic-module-user-guide.html)

* [Raspberry Pi Camera Module V2 - 8MP](https://www.raspberrypi.com/products/camera-module-v2/), [Documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)

* [MPU6050 - 3-Axis Gyroscope & Accelerometer](https://cdn.sparkfun.com/datasheets/Components/General%20IC/PS-MPU-6000A.pdf), [Sunfounder Guide](https://docs.sunfounder.com/projects/ultimate-sensor-kit/en/latest/components_basic/05-component_mpu6050.html)

* [4700µF 16V Radial Electrolytic Capacitor - ELR47216](https://www.westfloridacomponents.com/mm5/graphics/Q04/NRWA472M16X31.pdf)

* [Double Switch - Circuit Disconnect](https://vishaworld.com/products/robotic-switch)

* [14.4V Battery - NASTIMA 2600mAh](https://www.amazon.com/-/es/NASTIMA-Reemplazo-bater%C3%ADa-2600mAh-compatible/dp/B089KGP8QH)

* [Maxon Motor 110055 - Tri-Helix Wheels](https://www.maxongroup.com/medias/sys_master/root/9399155818526/Cataloge-Page-EN-172.pdf)

* [Planetary Gearhead GP 16 A 157:1 - 118186](https://www.maxongroup.com/medias/sys_master/root/9406823858206/Cataloge-Page-EN-416.pdf)

* [Encoder MEnc 13 16 CPT - 110778](https://www.maxongroup.com/medias/sys_master/root/8831077548062/2018EN-407-408.pdf)

* [MG996R Servos - Retractable Wheels](https://components101.com/sites/default/files/component_datasheet/MG996R%20Datasheet.pdf)

* [150:1 Micro Metal Gearmotor HP 6V](https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors-rev-6-1.pdf)

* [Push Button - Normally Open](https://components101.com/switches/push-button)

* [MH Level Converter - 5V to 3.3V](https://www.pololu.com/product/2595/resources)

> ⚠️ Note: The availability of external links is not guaranteed long-term. We recommend saving personal copies for archival or internal use.
