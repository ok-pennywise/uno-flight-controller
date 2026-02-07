# DIY ESP32 Flight Controller (C++)

A custom flight controller written in **C++ for ESP32**, implemented from scratch and validated in **real flight**.

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

All control logic is custom-written and executed inside a deterministic, fixed-period control loop.  
The system has been tested on a real airframe in flight.

---

## Flight Demonstration

The linked video shows:

- Real flight footage (not simulation)
- ESP32 running custom firmware
- MPU6050-based attitude estimation
- Manual flight in **Acro** and **Angle** modes
- No external stabilization or autopilot software

Aircraft stability is achieved entirely through the onboard control algorithms.

---

## Hardware

### Flight Controller
- **MCU:** ESP32
- **Language:** C++
- **Control loop frequency:** 500 Hz (2 ms)
- **Motor output:** MCPWM-based ESC control

### Sensors
- **IMU:** MPU6050  
  - 3-axis gyroscope  
  - 3-axis accelerometer
- **Magnetometer:** External (used for yaw drift correction)

### Input
- **RC protocol:** PPM (interrupt-driven)

### Output
- **Motors:** 4 ESC outputs (quadcopter configuration)

---

## Software Architecture

### Main Control Loop
- Fixed 2 ms loop enforced using `esp_timer_get_time()`
- Deterministic timing via busy-wait synchronization
- CPU budget monitoring using onboard LED

### Sensor Processing
- Raw IMU acquisition over I2C
- Startup calibration for gyroscope and accelerometer
- Physical unit scaling (deg/s, g)

### Attitude Estimation
- **Roll and Pitch**
  - 1D Kalman filter
  - Gyroscope integration with accelerometer correction
- **Yaw**
  - Gyroscope integration
  - Magnetometer-based drift correction
  - Magnetic declination compensation

---

## Flight Modes

### Acro Mode (Rate Mode)
- Direct angular rate control
- No self-leveling
- Pilot commands roll, pitch, and yaw rates
- Used for tuning and aggressive maneuvering

### Angle Mode (Stabilized)
- Self-leveling roll and pitch
- Pilot commands desired angles
- Outer angle loop feeding inner rate loop
- Throttle tilt compensation applied

Mode switching is inhibited when the aircraft exceeds safe tilt limits.

---

## Control System

### PID Structure
- **Inner loop:** Rate PID (roll, pitch, yaw)
- **Outer loop:** Angle P controller (Angle mode only)

### Features
- Trapezoidal integration for I-term
- Derivative term based on filtered angular rate
- Output clamping to prevent actuator saturation
- Integral windup protection
- Throttle compensation for attitude tilt

Detailed PID tuning guidance is included directly in the source code.

---

## Safety and Arming Logic

- Explicit **UNARMED / ARMED / SAFETY_TRIP** state machine
- Throttle-low arming requirement
- Automatic safety trip on excessive roll or pitch
- Manual reset required after safety trip
- Motors forced to minimum output when disarmed

---

## Autonomous Flight (Work in Progress)

A **Raspberry Pi 3** is being integrated as a companion computer for autonomous flight.

Planned responsibilities of the companion computer:
- High-level navigation and mission logic
- Autonomous setpoint generation
- Extended sensor fusion (GPS, vision, etc.)

The ESP32 remains the **real-time, safety-critical flight controller**.  
The companion computer does not directly drive motors.

---

## Project Status

- Stable manual flight achieved
- Acro mode operational
- Angle mode operational
- Safety and arming logic implemented
- Autonomous flight integration in progress

This is an active development project.

---

## Credits

This project was developed through hands-on experimentation and self-study.

The following resources were instrumental in shaping the understanding behind this flight controller:

- **Carbon Aeronautics** — for teaching multirotor flight dynamics, how a quadcopter moves through 3D space, sensor fusion concepts, and practical filter implementation used for attitude estimation.

- **Joop Brooking** — for practical guidance on flight controller behavior, PID tuning methodology, control loop structure, and real-world flight testing considerations.

All code in this repository is original. These sources influenced the understanding and design approach, not the implementation itself.

---

## Disclaimer

This project is experimental and intended for learning and research purposes only.

- Not flight-certified
- Not safety-approved
- No guarantees of stability or reliability

Use at your own risk. You are responsible for all hardware, property, and personal safety.

---

Built from scratch.  
Flown in the real world.
