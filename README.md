# Bose: A Stair-Climbing Robot for Hybrid Terrain Navigation

**An open-source, low-cost robot designed to autonomously climb stairs and navigate flat indoor environments using adaptive locomotion and onboard perception.**

---

![Bose Hero Image](docs/images/bose_hero.jpg)

---

## 🚀 Overview

**Bose** is a hybrid-terrain mobile robot capable of transitioning between stair-climbing and flat-ground modes through a custom tri-helix wheel system and retractable support wheels. Built around a Raspberry Pi 4 and designed for reproducibility and education, Bose combines real-time terrain classification, SLAM, and autonomous control in a lightweight, low-cost platform.

> 🧠 The robot is named after the **Bose-Einstein distribution**, reflecting its multi-modal adaptability and emergent behavior under constrained environments.

---

## 🎯 Key Features

- ✅ Tri-helix wheel mechanism for stair traversal (up to 20 cm rise)
- ✅ Retractable micro wheels for efficient flat-ground navigation
- ✅ Real-time 2D SLAM with RPLIDAR C1 and monocular vision
- ✅ Terrain classification with ultrasonic sensor + camera
- ✅ Fully embedded system: runs on Raspberry Pi 4B (no external microcontroller)
- ✅ Modular, laser-cut chassis and 3D-printed mounts
- ✅ Reproducible architecture with open-source hardware and software

---

## 📸 Media & Demos

| Stair Climb | Terrain Switch | Visual SLAM |
|------------|----------------|-------------|
| ![Stair Climbing](docs/images/stair_climb.gif) | ![Terrain Switching](docs/images/switch_mode.gif) | ![SLAM Map](docs/images/slam_map.png) |

> 🎥 See the full [demo video on YouTube](https://youtube.com/your_video_url_here)

---

## 🧱 System Architecture

### Block Diagram

![System Architecture](Media/system_block_diagram.png)

### Hardware Stack

- Raspberry Pi 4B (4 GB RAM)
- MD25 Motor Controller (tri-helix wheels)
- L298N H-Bridge (retractable wheels)
- PCA9685 Servo Controller
- RPLIDAR C1 (USB)
- HC-SR04 Ultrasonic Sensor
- MPU6050 IMU
- Pi Camera v2 (8MP)
- 14.4V Li-ion Battery
- XL4015 and Pololu 5V 3A Buck Converters
- LEDs, push button, and MH Level Shifter

---

## 🛠️ Build Instructions

### 📦 Installation Requirements

- Raspberry Pi 4B with Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+

Install dependencies:

```bash
sudo apt update && sudo apt install python3-pip
pip install adafruit-circuitpython-pca9685 smbus2 numpy opencv-python rplidar-robotics
```

## 📁 Repository Structure

```
├── docs/
│   └── images/            # Photos, schematics, SLAM maps
├── hardware/
│   ├── fusion360/         # CAD models, STL files
│   └── wiring_diagrams/   # Fritzing schematics
├── software/
│   ├── control/           # Python control scripts
│   ├── slam/              # Visual + LiDAR SLAM
│   └── ros2_ws/           # ROS 2 nodes
├── scripts/
│   └── interactive_control.py
├── README.md
└── decisions.md           # Traceable engineering decisions
```

---

## 🔌 Wiring and Setup

See `hardware/wiring_diagrams/raspberry_pi_wiring.pdf` for pinout.

> 🛡️ **Tip:** Use a 4700 μF capacitor near the Pi 5V rail to prevent brownouts during motor actuation.

---

## 🕹️ Usage

### 🧪 Manual Control (Interactive Script)

Run:

```bash
python3 scripts/interactive_control.py
```

**Controls:**

- Arrow keys: Helix motors  
- W / A / S / D: Retractable wheel motors  
- O / L: Lower / lift support wheels  
- Q: Quit

---

### 🤖 SLAM and Navigation (ROS 2)

Launch ROS nodes:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bose bringup.launch.py
```

**ROS 2 Topics:**

- `/scan` – RPLIDAR laser scan  
- `/odom` – Encoder-based odometry  
- `/imu/data_raw` – IMU orientation  
- `/camera/image_raw` – Pi camera feed

---

## 🔍 Components List

| Component                | Description                                |
|--------------------------|--------------------------------------------|
| Raspberry Pi 4B          | Main controller                            |
| MD25 Motor Driver        | Tri-helix wheels + encoders                |
| L298N                    | Retractable wheel motors                   |
| PCA9685                  | 16-channel servo controller                |
| S13V30F5                 | 5V 3A buck-boost converter                 |
| XL4015                   | Power to servos and support motors         |
| HC-SR04                  | Ultrasonic sensor for stair detection      |
| MPU6050                  | 6-DOF IMU                                  |
| RPLIDAR C1               | 360º 2D LiDAR for SLAM                     |
| Pi Camera v2             | Monocular vision                           |
| MG996R                   | Servos for retractable wheels              |
| Maxon 110055 + Gearbox   | Tri-helix motor system                     |
| LEDs, push button        | Status and local control                   |

📄 Full datasheets in `docs/appendix_components.md`

---

## 🔁 Reproducibility

To reproduce the robot:

1. Follow hardware instructions (CAD + wiring)
2. Flash Ubuntu 22.04 and install dependencies
3. Run the test scripts (manual or ROS-based)
4. Use `decisions.md` to trace engineering rationale

All firmware-free and open hardware. See `LICENSE`.

---

## 👨‍🔬 Paper Citation

**Arun Sharma**, **Pau Domínguez Ruiz**, **Gerard Souto Eslava**, **Chengjie Peng Lin**,  
*“Bose: A Stair-Climbing Robot for Hybrid Terrain Navigation,”* Universitat Autònoma de Barcelona, 2025.

```bibtex
@misc{bose2025,
  author      = {Sharma, Arun and Domínguez Ruiz, Pau and Souto Eslava, Gerard and Peng Lin, Chengjie},
  title       = {Bose: A Stair-Climbing Robot for Hybrid Terrain Navigation},
  year        = {2025},
  institution = {Universitat Autònoma de Barcelona},
  note        = {Bachelor’s Course in Robotics, Language and Planning}
}
```

---

## 🤝 Acknowledgments

- Prof. Fernando L. Vilariño Freire  
- Prof. Carlos G. Calvo  
- Prof. Vernon S. Albayeros Duarte  
- Open Labs – Escola d’Enginyeria, UAB  
- Maxon Spain and Robot Electronics support teams

---

## 🪪 License

This repository is licensed under the MIT License. See `LICENSE` for details.

> 📬 For academic use, extension, or replication, please contact the authors or contribute via GitHub issues and pull requests.

---

_"To climb stairs is to rise beyond limits. Bose takes that literally."_
