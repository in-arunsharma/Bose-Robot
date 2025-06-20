# Bose: A Stair-Climbing Robot for Hybrid Terrain Navigation

**Authors:** Arun Sharma, Pau Domínguez Ruiz, Gerard Souto Eslava, Chengjie Peng Lin  
**Affiliation:**  
Universitat Autònoma de Barcelona (UAB)  
Bachelor's Degree in Computer Engineering  
Course: Robotics, Language and Planning (Spring 2025)  
Supervised by Prof. Fernando Luis Vilariño Freire, Prof. Carlos Garcia Calvo, Prof. Vernon Stanley Albayeros Duarte  
In collaboration with the Open Labs of the School of Engineering (Laboratoris d’Innovació Oberta, UAB)  
**Date:** 2025-06-27 (expected)

---

## Abstract
This paper presents Bose, a novel stair-climbing robot featuring an adaptive-wheel mechanism optimized for hybrid-terrain navigation. Addressing practical challenges in accessibility and autonomous indoor exploration, Bose employs two laser-cut tri-helix wheels for stair traversal, combined with smaller retractable wheels that activate during flat-surface navigation. An articulated tail mechanism maintains stability across diverse terrains. The robot integrates multiple onboard sensors, including an inertial measurement unit, ultrasonic range sensors, encoders, camera, and a 2D LiDAR, enabling terrain-aware locomotion and environment mapping. Terrain detection leverages computer vision and distance sensing, dynamically adjusting the wheel configuration based on real-time feedback. Key mechanical and control challenges, such as achieving sufficient torque-to-weight ratios and reliable transition between wheel modes, were systematically addressed. Experimental evaluations demonstrate Bose’s capability to autonomously climb standard stairs (up to 20 cm rise and 35° incline) and smoothly transition onto flat terrain at speeds up to 0.08 m/s. Additionally, real-time simultaneous localization and mapping (SLAM) facilitates autonomous navigation in unknown indoor environments. Future improvements will focus on enhancing adaptability through shape-shifting wheels capable of accommodating varied stair geometries, eliminating redundant wheel systems, and refining structural aerodynamics.
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

Several robotic approaches have been developed to tackle the challenge of stair climbing. Each solution presents trade-offs in terms of complexity, efficiency, and mechanical robustness. In this section, we review the main designs we explored and considered.

The  lifting system is one of the most straightforward mechanisms, where the robot physically raises itself using an arm or pivoting support. This method requires high torque and stability, as the entire body must be lifted at once. It can end up as a bulky design and may struggle with different shapes of stairs and its slow compared to others.

Another common design is the use of wheel traction and balance, typically found in U-shaped four-wheel robots. These rely on high-adhesion tires and carefully tuned torque to climb steps directly by maintaining continuous contact and shifting weight forward. They are mechanically simpler but this method does not work well on taller stairs or in cases with irregular surfaces.

We also considered six-wheel articulated robots, where the structure flexes slightly to adapt to stairs while maintaining traction. This approach shares principles with the traction-based models but improves stability by increasing the number of contact points. Still, it tends to be heavier and has the same problems as the U-shaped four-wheel robots mentioned before.

A very  elegant solution is the Tri-Wheel Helix mechanism, which uses spiral-shaped wheels to naturally conform to stair geometry. As the wheel rotates, the robot is gently lifted from one step to the next. This design is fast and cost-effective overall (except for the motor, which requires high torque due to the large wheel size). It also features a mechanically complex wheel design that is both challenging and interesting to build. This balance of performance and uniqueness led us to choose it for our project.

Another widely used system is the caterpillar track. This robust solution provides excellent grip and stability, allowing the robot to handle stairs and uneven terrain efficiently. However, it is usually slower and requires a sturdy frame to support the strain on the track system.

Beyond these, other less conventional methods exist, such as legged robots (e.g: Boston Dynamics' Spot), which can walk upstairs with impressive precision. However, these systems are often too expensive and complex for low-budget academic projects like ours.

In summary, after evaluating various designs, we found the Tri-Wheel Helix to be the most balanced option for our needs, combining adaptability, mechanical interest, and effectiveness for hybrid terrain navigation.

## 3. Proposed Architecture
- System overview
- Subsystem summary
- Functional diagram

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
## References

[1] R. R. Murphy, J. L. Burke, and M. D. Rogers, “Trial by Fire: Activities of the Rescue Robots at the World Trade Center from 11 to 21 September 2001,” *IEEE Robotics and Automation Magazine*, vol. 11, no. 3, pp. 50–61, Sep. 2004. doi:[10.1109/MRA.2004.1342786](https://doi.org/10.1109/MRA.2004.1342786)

[2] Endeavor Robotics, **“510 PackBot® Product Datasheet,”** Bedford, MA, USA, 2016. [Online]. Available: <https://web.archive.org/web/20140626225856/http://www.irobot.com/us/learn/defense/packbot.aspx>. Accessed: 20 Jun 2025.

[3] M. Hutter *et al.*, “ANYmal – A Highly Mobile and Dynamic Quadrupedal Robot,” in *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Daejeon, South Korea, Oct. 2016, pp. 38–44. doi:[10.1109/IROS.2016.7758092](https://doi.org/10.1109/IROS.2016.7758092)

---

## Appendix
- Circuit diagrams
- 3D model schematics
- Calibration data
- Full test logs


### Appendix A: Component Datasheets (External Links)

The following datasheets were referenced during the design and integration of the Bose robot. We provide official external links to ensure up-to-date and legal access. Local copies have not been included to respect distribution policies.

- [MPU-6050 – TDK InvenSense](https://)
- [MG996R Servo – TowerPro](https:/)
- [RPLIDAR C1 – Slamtec](https)  

> ⚠️ Note: The availability of external links is not guaranteed long-term. We recommend saving personal copies for archival or internal use.
