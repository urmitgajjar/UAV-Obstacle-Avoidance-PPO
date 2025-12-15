# UAV Obstacle Avoidance Using PPO and Ultrasonic Sensors

## Overview
This project presents a real-time obstacle avoidance framework for a hexacopter UAV using ultrasonic sensors and a Proximal Policy Optimization (PPO)–based reinforcement learning model. The system follows a companion-computer architecture in which a Raspberry Pi 4 performs sensing, data processing, and machine learning inference, while a Pixhawk flight controller manages low-level stabilization and motor control. Communication between the Raspberry Pi and Pixhawk is achieved using the MAVLink protocol.

The primary objective of this project is to enable safe and adaptive UAV navigation in cluttered and dynamic environments using low-cost sensors and lightweight onboard computation.

---

## System Architecture
- **Sensing Layer:** Five US-100 ultrasonic sensors mounted in the front, back, left, right, and top directions to provide obstacle distance measurements.  
- **Processing Layer:** Raspberry Pi 4 acquires sensor data, receives IMU feedback, and executes the PPO-based obstacle avoidance model.  
- **Control Layer:** Pixhawk 2.4.8 flight controller receives roll, pitch, and yaw correction commands via MAVLink and ensures flight stabilization.

---

## Hardware Components
- F550 Hexacopter Frame  
- Pixhawk 2.4.8 Flight Controller  
- Raspberry Pi 4 Model B (2 GB RAM)  
- US-100 Ultrasonic Sensors (x5)  
- DJI 2212 920KV Brushless DC Motors (x6)  
- BLHeli 30A DShot ESCs (x6)  
- 10-inch ABS Plastic Propellers (CW + CCW)  
- Pro-Range 4S 5200mAh Li-Po Battery  
- FlySky i6X Transmitter with iA10B Receiver  

---

## Software Stack
- Python 3  
- Stable-Baselines3 (PPO)  
- MAVLink (pymavlink)  
- Mission Planner  
- ArduPilot Firmware  
- Raspberry Pi OS  

---

## Repository Structure
