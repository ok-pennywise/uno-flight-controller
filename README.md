# DIY Open-Source Flight Controller (ESP32)

**A custom flight controller written in C++ for ESP32, built and flown from scratch.**  
No PX4. No ArduPilot. No prebuilt stacks.

👉 **[Watch the real flight test video](https://drive.google.com/file/d/1yiodxVZ2e_W75nkZD7_hRxJ_GFKJbH9i/view)**

---

## ✈️ Overview

This project is a **ground-up implementation of a flight controller**, designed to give full control over:

- Sensor fusion
- Attitude estimation
- Control loops
- Flight modes
- Hardware timing

The controller has been **tested in real flight**, not simulation.

---

## 🎥 Flight Demo

📹 **Real flight footage:**  
👉 [Watch on Google Drive](https://drive.google.com/file/d/1yiodxVZ2e_W75nkZD7_hRxJ_GFKJbH9i/view)

What the video shows:
- ESP32 running custom flight firmware
- MPU6050 IMU for attitude sensing
- Manual flight in **Acro** and **Angle** modes
- Real motors, ESCs, and airframe
- No external stabilization systems

If it flies, it’s because the control code works.

---

## 🧠 System Architecture

### Flight Controller (ESP32)
- **Language:** C++
- **MCU:** ESP32
- **IMU:** MPU6050
- **Loop:** High-rate control loop (sensor → estimate → control → output)
- **Modes:**
  - **Acro Mode** (rate control)
  - **Angle Mode** (self-leveling)

Responsibilities:
- Sensor sampling
- Attitude estimation
- PID control
- Motor mixing
- RC input handling

---

### Companion Computer (Autonomous Flight – WIP)
- **Platform:** Raspberry Pi 3
- **Purpose:** High-level autonomy
- **Planned responsibilities:**
  - Navigation logic
  - Mission planning
  - Sensor fusion beyond IMU (GPS / vision / etc.)
  - Commanding the ESP32 flight controller

The ESP32 remains the **real-time safety-critical controller**.  
The Raspberry Pi acts as a **high-level decision maker**, not a replacement FC.

---

## 🎮 Flight Modes

### 🟢 Acro Mode
- Direct angular rate control
- No self-leveling
- Pilot fully responsible for attitude
- Used for tuning and aggressive maneuvering

### 🔵 Angle Mode
- Self-leveling using IMU data
- Limited tilt angles
- Easier manual control and testing

---

## 🎯 Project Goals

- Implement a **real flight controller from first principles**
- Maintain full ownership of:
  - Timing
  - Control laws
  - Sensor handling
- Keep the code:
  - Minimal
  - Readable
  - Hackable
- Validate everything with **actual flight tests**

---

## 🚧 Current Status

- ✅ ESP32 flight controller operational
- ✅ MPU6050 integration
- ✅ Acro mode flying
- ✅ Angle mode flying
- 🔧 PID tuning refinement
- 🚧 Autonomous flight (Raspberry Pi 3) in development

This is an **active project**.

---

## ⚠️ Disclaimer

This project is **experimental** and intended for learning and research.

- ❌ Not flight-certified
- ❌ Not safety-approved
- ❌ No guarantees of stability or safety

You are fully responsible for any hardware, property, or personal risk.

---

## 🤝 Why This Exists

This repository exists to:
- Prove that a **custom flight controller can be built and flown**
- Share implementation ideas
- Serve as a learning reference for flight control systems
- Encourage understanding beyond black-box autopilots

If you want to know how flight controllers *actually* work — this repo is for you.

---

**Built from scratch. Tested in the air. Still evolving.**
