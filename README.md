# DIY ESP32 Flight Controller (C++)

A custom-built flight controller written in **C++ for ESP32**, designed and implemented from scratch and validated in **real flight**.

No PX4.  
No ArduPilot.  
No prebuilt flight stack.

**Flight test video:**  
https://drive.google.com/file/d/1yiodxVZ2e_W75nkZD7_hRxJ_GFKJbH9i/view

---

## Overview

This project is a ground-up implementation of a multirotor flight controller running on an **ESP32**, with full ownership of:

- Sensor acquisition
- Attitude estimation
- Control loops
- Flight modes
- Safety and arming logic
- Motor mixing and timing

All control logic is custom-written and runs in a fixed-time control loop.  
The system has been tested on a real airframe in flight.

---

## Flight Demo

The video linked above shows:

- Real flight footage (not simulation)
- ESP32 running custom firmware
- MPU6050 IMU-based attitude estimation
- Manual flight in **Acro** and **Angle** modes
- No external stabilization or autopilot software

If the aircraft is stable, it is due to the control code executing on the ESP32.

---

## Hardware

### Flight Controller
- **MCU:** ESP32
- **Language:** C++
- **Control loop rate:** 500 Hz (2 ms)
- **Motor output:** MCPWM (ESC PWM signals)

### Sensors
- **IMU:** MPU6050
  - Accelerometer
  - Gyroscope
- **Magnetometer:** External (used for yaw correction)

### Input
- **RC input:** PPM (interrupt-driven)

### Output
- **ESCs:** 4 motor outputs (quad configuration)

---

## Software Architecture

### Main Control Loop
- Fixed 2 ms loop using `esp_timer_get_time()`
- Deterministic timing enforcement
- CPU load monitoring via onboard LED

### Sensor Processing
- Raw IMU data acquisition over I2C
- Gyro and accelerometer calibration at startup
- Scaled physical units (deg/s, g)

### Attitude Estimation
- Roll and pitch:
  - 1D Kalman filter (gyro integration + accelerometer correction)
- Yaw:
  - Gyro integration with magnetometer-based drift correction
  - Declination compensation

---

## Flight Modes

### Acro Mode (Rate Mode)
- Direct angular rate control
- No self-leveling
- Pilot commands roll, pitch, and yaw rates
- Intended for tuning and aggressive control

### Angle Mode (Stabilized)
- Self-leveling roll and pitch
- Pilot commands desired angles
- Outer angle loop feeds inner rate loop
- Tilt-compensated throttle

Mode switching is locked out if the aircraft is excessively tilted to prevent unsafe transitions.

---

## Control System

### PID Structure
- **Inner loop:** Rate PID (roll, pitch, yaw)
- **Outer loop:** Angle P controller (Angle mode only)

### Features
- Trapezoidal integration for I-term
- Derivative based on filtered angular rate
- Output clamping to prevent actuator saturation
- Integral windup protection
- Tilt-compensated throttle

PID tuning guidance is included directly in the source code.

---

## Safety and Arming Logic

- Explicit **UNARMED / ARMED / SAFETY_TRIP** states
- Throttle-low requirement to arm
- Safety trip if extreme attitude is detected
- Manual reset required after safety trip
- Motors forced to minimum output when not armed

---

## Autonomous Flight (Work in Progress)

A **Raspberry Pi 3** is being integrated as a companion computer for autonomous operation.

Planned responsibilities of the Raspberry Pi:
- High-level navigation and mission logic
- Autonomous setpoint generation
- Sensor fusion beyond IMU (GPS, vision, etc.)

The ESP32 remains the **real-time, safety-critical flight controller**.  
The Raspberry Pi does not directly drive motors.

---

## Project Status

- Stable manual flight achieved
- Acro mode operational
- Angle mode operational
- Safety and arming logic implemented
- Autonomous flight integration in progress

This is an active development project.

---

## Disclaimer

This project is experimental and intended for learning and research purposes.

- Not flight-certified
- Not safety-approved
- No guarantees of stability or reliability

Use at your own risk. You are responsible for all hardware, property, and personal safety.

---

## Purpose

This repository exists to:

- Demonstrate a working custom flight controller
- Share low-level implementation details
- Provide a learning reference for flight control systems
- Encourage understanding beyond black-box autopilots

---

Built from scratch.  
Flown in the real world.
