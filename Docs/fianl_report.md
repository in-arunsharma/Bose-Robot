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

---

## 1. Introduction
- Motivation
- Problem definition
- Project objective
- Structure of the report
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
