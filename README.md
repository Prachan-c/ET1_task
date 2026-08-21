# Embedded Computing Lab – ET1 Robot Tasks

This repository contains the complete implementation, experiments, and documentation for the **ET1 Robot**
All tasks (Task 1 to Task 4), including source code, web interface, and experimental results, are included. A detailed **lab report** describing objectives, methodology, observations, and results is also provided.
---
## 📌 Overview

The project focuses on:
- Raspberry Pi–based robot control (ET1)
- Motor control using GPIO and PWM
- Rotary encoder feedback and PID control
- Ultrasonic distance sensing (SR04)
- Obstacle avoidance and autonomous movement
- Web-based control using Node.js and Socket.IO
- Multi-language integration (C, Python, JavaScript)

---
## 🧩 Task Summary

### **Task 1 – Environment Setup & Hardware Verification**
- Verified ET1 Raspberry Pi setup via VNC
- Tested availability of:
  - Python (Thonny)
  - C Development Environment
  - Node.js
  - Firefox
- Reviewed and validated provided test scripts:
  - LED blinking
  - Pushbutton input
  - Motor control
  - Distance and line sensors

---
### **Task 2 – Motor Control & Motion Accuracy**
Includes multiple sub-tasks:

- **2A–2C:**  
  - Button-controlled LED and motor motion  
  - Acceleration/deceleration (forward & backward)
- **2D:**  
  - Evaluation of motion accuracy and drift
- **2E:**  
  - Encoder-based motion correction  
  - Fixed-rotation strategy  
  - PID-based speed balancing
- **2F:**  
  - Obstacle avoidance using SR04  
  - State-machine-based navigation  
  - Encoder-based 90° turns
- **2G:**  
  - Proper GPIO and resource cleanup

---
### **Task 3 – Web-Controlled Robot Movement**
- Node.js web server with Socket.IO
- Web UI with checkboxes to:
  - Move forward until obstacle (~10 cm)
  - Move backward and turn randomly
  - Continuous random navigation mode
- Python scripts triggered from the browser
- Live display of:
  - Distance to obstacle
  - Left & right encoder ticks
- Sequential and conditional script execution

---
### **Task 4 – Multi-Process Sensor Fusion System**
Integration of **four programs**:

- **C (sr04_client.c):**
  - Reads SR04 ultrasonic distance
  - Sends data via TCP
- **Python Sensor Server (soc_server.py):**
  - Sensor fusion
  - Tick counting, speed & turn estimation
- **Python Motor Controller (motor_task4.py):**
  - PID-based motor control
  - Predefined motion sequence
- **Node.js Web Server:**
  - Starts/stops system from UI
  - Displays live fusion data (speed, turn ticks, delta distance)

---
## 🛠️ Technologies Used

- **Hardware**
  - Raspberry Pi (ET1 Robot)
  - DC Motors with H-Bridge
  - Rotary Encoders
  - SR04 Ultrasonic Sensor
  - Line Sensors

- **Software**
  - Python (GPIO, PID control, state machines)
  - C (low-level ultrasonic sensing)
  - Node.js + Socket.IO
  - HTML / JavaScript
  - Linux (Raspberry Pi OS)

