# 🤖 SCARA Robot Control System (STM32 + Python GUI)

This project implements a **modular control architecture** for a SCARA robot using an STM32 microcontroller and a Python-based GUI. The system supports real-time manual control as well as waypoint-based motion execution through a teach pendant interface.

---

## 🚀 Features

* 🔧 **Manual Control Mode**

  * Direct joint control from Python GUI
  * Continuous streaming of joint commands over UART

* 📍 **TeachPad Mode (Waypoint Recording)**

  * User-defined joint positions are stored as waypoints
  * Waypoints are managed on the Python side (GUI)

* ▶️ **Waypoint Execution**

  * Recorded waypoints are executed sequentially
  * Robot moves point-to-point with controlled timing

* 🔌 **UART Communication Protocol**

  * Custom binary frame (25 bytes)
  * CRC-16 (Modbus) for data integrity
  * Mode-based operation (MANUAL / TEACHPAD / AUTO-ready)

---

## 🧠 System Architecture

```
[ Python GUI ]
        ↓
[ Controller Logic ]
        ↓
[ UART Communication ]
        ↓
[ STM32 Firmware ]
        ↓
[ Motor Drivers ]
```

### Responsibilities

| Component           | Responsibility                  |
| ------------------- | ------------------------------- |
| Python GUI          | User input, waypoint management |
| Controller (Python) | Execution logic (send & wait)   |
| UART Layer          | Data transmission               |
| STM32               | Real-time motor control         |

---

## 📦 Communication Protocol

Each command is sent as a **25-byte frame**:

| Byte Index | Description                            |
| ---------- | -------------------------------------- |
| 0          | Start byte (0xFF)                      |
| 1          | Mode (1: MANUAL, 2: TEACHPAD, 3: AUTO) |
| 2–21       | Payload (5 × float32)                  |
| 22–23      | CRC-16 (Modbus)                        |
| 24         | Stop byte (0xFE)                       |

### Payload Structure

```
[theta1, dist2, theta3, theta4, gripper_cmd]
```

---

## 🎮 Operating Modes

### 🟢 MANUAL

* Direct control from GUI
* Robot follows incoming joint commands instantly

---

### 🟡 TEACHPAD

* GUI records joint configurations as waypoints
* STM32 receives data but does not store path internally
* Python maintains waypoint list

---

### 🔵 EXECUTION (Teach Run)

* Waypoints are replayed sequentially from Python
* Each waypoint is sent as a MANUAL command
* Fixed delay (~50 ms) between points (can be improved with feedback)

---

## 🧩 Waypoint System

Waypoints are stored in Python as:

```python
teach_waypoints = [
    (q1, d2, q3, q4, grip),
    ...
]
```

Execution logic:

```python
for wp in teach_waypoints:
    send_frame(...)
    time.sleep(0.05)
```

---

## ⚠️ Current Limitations

* ⏱ Uses fixed delay instead of feedback-based synchronization
* 🤖 No trajectory interpolation (point-to-point motion only)
* 🔁 STM32 does not yet execute paths autonomously (AUTO mode planned)

---

## 🔮 Future Improvements

* ✅ Feedback system (`target reached` signal from STM32)
* 🧠 Autonomous execution mode (AUTO)
* 📈 Smooth trajectory planning (interpolation)
* 🛑 Command system (START / STOP / PAUSE)
* 📡 Bidirectional communication (ACK / status feedback)

---

## 🛠 Technologies Used

* **Embedded:** STM32 (HAL Library, C)
* **Communication:** UART (Custom protocol + CRC16)
* **Desktop:** Python (GUI + Control Logic)

---

## 📌 Summary

This project demonstrates a **clean separation of concerns** between:

* User interface (Python GUI)
* Control logic (Python controller)
* Real-time execution (STM32)

The system has evolved from simple manual control into a **structured robotic control architecture** supporting waypoint-based motion.

---

## 👨‍💻 Author

Developed as part of an embedded robotics project focusing on real-time control, modular design, and scalable system architecture.
