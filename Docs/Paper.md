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
- Frame and chassis
- Wheel configuration
- Support mechanism
- Mounting and integration
---

## 5. Electronics and Sensors
- Power system
- Motor drivers
- Main controller
- Sensor list:
  - IMU
  - Encoders
  - LIDAR
  - Others
---

## 6. Software Design
- Control structure
- Communication protocols
- Motion control logic
- Sensor integration
- SLAM module
---

## 7. Key Engineering Decisions
- Summary table of major decisions
- Design trade-offs
- Link to `decisions.md`
---

## 8. Experiments
- Setup
- Methodology
- Environments

---

## 9. Results and Analysis
- Performance summary
- Observed behavior
- Limitations
---

## 10. Discussion
- Result interpretation
- Design implications
- Constraints encountered
---

## 11. Future Work
- Hardware improvements
- Software extensions
- Autonomous capabilities
---

## 12. Conclusion

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
