# 🌱 Agricultural Hybrid Robot

### Autonomous Tomato Harvesting Platform with 4-DOF SCARA Robot

![Python](https://img.shields.io/badge/Python-3.x-blue?logo=python)
![STM32](https://img.shields.io/badge/STM32-Embedded-success)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Instance%20Segmentation-red)
![Jetson](https://img.shields.io/badge/NVIDIA-Jetson%20Orin%20Nano-green?logo=nvidia)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-orange?logo=opencv)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen)

**Senior Design Project**
**Department of Mechatronics Engineering**
**Kocaeli University**



# 📌 Project Overview

Agricultural Hybrid Robot is an autonomous harvesting platform developed for greenhouse applications by integrating a **4-DOF SCARA robotic manipulator** with a **mobile Mid-Rover**.

The system combines **computer vision**, **embedded control**, and **robot kinematics** to detect ripe tomatoes, estimate their three-dimensional position, generate smooth robot trajectories, and perform autonomous harvesting.

A stereo vision camera continuously observes the workspace while a YOLOv8 instance segmentation model detects tomatoes according to their ripeness level. The detected targets are converted into the robot coordinate frame, processed through inverse kinematics, and transmitted to an STM32-based embedded controller for real-time execution.

The software architecture has been designed with a modular approach, allowing independent development and maintenance of the communication, control, graphical interface, and computer vision components.



# 🎯 Key Features

* 🍅 Autonomous tomato detection and harvesting
* 👁️ YOLOv8 instance segmentation
* 📷 Intel RealSense stereo depth camera
* 📍 3D target localization
* 🤖 4-DOF SCARA robotic manipulator
* 🚙 Differential-drive Mid-Rover platform
* 🧮 Forward & Inverse Kinematics
* 📈 Quintic polynomial trajectory planning
* ⚡ STM32 embedded motor control
* 🔄 UART communication protocol
* 🖥️ PyQt6 graphical user interface
* 🧩 Modular software architecture

---

# ⚙️ System Workflow


Intel RealSense Camera
          │
          ▼
YOLOv8 Instance Segmentation
          │
          ▼
Depth Estimation
          │
          ▼
3D Coordinate Calculation
          │
          ▼
Coordinate Transformation
          │
          ▼
Inverse Kinematics
          │
          ▼
Trajectory Planning
          │
          ▼
UART Communication
          │
          ▼
STM32 Embedded Controller
          │
          ▼
SCARA Robot Motion
          │
          ▼
Autonomous Harvesting



# 🛠 Hardware

| Component           | Description                  |
| ------------------- | ---------------------------- |
| Mobile Platform     | Differential Drive Mid-Rover |
| Manipulator         | 4-DOF SCARA Robot            |
| End-Effector        | Custom 3-Finger Gripper      |
| Main Processor      | NVIDIA Jetson Orin Nano      |
| Embedded Controller | STM32F401RE                  |
| Vision Sensor       | Intel RealSense Depth Camera |
| Actuators           | NEMA17 Stepper Motors        |


# 💻 Software Architecture


Python
│
├── COMM
│
├── GENERAL_CONTROL
│   ├── Forward Kinematics
│   ├── Inverse Kinematics
│   └── Trajectory Planning
│
├── GUI
│
└── VISION
    ├── YOLOv8
    ├── Target Tracking
    ├── Coordinate Transformation
    └── Autonomous Harvesting
            │
            ▼
UART Communication
            │
            ▼
STM32 Embedded Software
│
├── Motor Control
├── Robot Modes
├── UART Driver
├── CRC16
└── Gripper Controller

# 📂 Repository Structure

AgriculturalHybridRobot
│
├── Python
│   ├── COMM
│   ├── GENERAL_CONTROL
│   ├── GUI
│   └── VISION
│
├── STM32
│
├── Images
│
└── README.md


# 🚀 Autonomous Harvesting Pipeline


Detect Tomato
      │
      ▼
Estimate 3D Position
      │
      ▼
Transform Coordinates
      │
      ▼
Inverse Kinematics
      │
      ▼
Trajectory Generation
      │
      ▼
Move Robot
      │
      ▼
Grip Tomato
      │
      ▼
Twist Harvest
      │
      ▼
Place into Basket

# 📊 Results

The developed system successfully integrates computer vision, robot kinematics, embedded control, and autonomous motion planning into a unified harvesting platform.

The complete harvesting cycle—including object detection, depth estimation, coordinate transformation, inverse kinematics, trajectory generation, embedded motor control, and autonomous manipulation—was successfully demonstrated on the developed prototype.

---

# 👥 Project Team

* **Tuna Karatekeli**
* **Okan Tıkır**
* **Emirhan Pektemek**
* **Yusuf Kaan Çoğalan**

### Supervisor

**R.A. Dr. Haluk Özakyol**

---

# 📄 License

This project has been developed for educational and academic purposes.

Feel free to explore the repository and use it as a reference for robotics, embedded systems, and autonomous harvesting applications.
