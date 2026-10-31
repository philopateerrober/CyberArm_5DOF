CyberArm— 5-DOF Wireless Teleoperated Robotic Arm

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Framework: Arduino / ESP-IDF](https://img.shields.io/badge/Framework-Arduino%20Core%20v3.x-teal.svg)](https://github.com/espressif/arduino-esp32)
[![Hardware: ESP32--C3](https://img.shields.io/badge/Glove-ESP32--C3%20SuperMini-darkblue.svg)](https://www.espressif.com/en/products/socs/esp32-c3)
[![Hardware: ESP32--WROOM](https://img.shields.io/badge/Arm-ESP32--WROOM--32-red.svg)](https://www.espressif.com/en/products/socs/esp32)

> **CyberArm** is a high-performance, 5-Degree-of-Freedom (5-DOF) wireless teleoperated robotic arm system. It features bidirectional dual-wireless communication via low-latency ESP-NOW and Wi-Fi WebSockets, a sensory smart glove with haptic force-feedback, an edge AI computer vision pipeline powered by Google MediaPipe, and a real-time Three.js 3D Digital Twin Ground Control Station (GCS).

---

## Table of Contents
1. [System Architecture & Data Flow](#system-architecture--data-flow)
2. [Key Engineering Features](#key-engineering-features)
3. [5-DOF Kinematic Specifications](#5-dof-kinematic-specifications)
4. [Bill of Materials (BOM)](#bill-of-materials-bom)
5. [Wiring Schematics & Pinout Tables](#wiring-schematics--pinout-tables)
6. [Mathematical Modeling & Sensor Calibration](#mathematical-modeling--sensor-calibration)
7. [Packet Structure & Safety Protocol](#packet-structure--safety-protocol)
8. [Firmware Setup & Flashing Guide](#firmware-setup--flashing-guide)
9. [Ground Control Station Operation](#ground-control-station-operation)
10. [Repository Structure](#repository-structure)
11. [License & Attribution](#license--attribution)

---

## System Architecture & Data Flow

The system employs a dual-wireless decoupled pipeline that isolates low-latency joint actuation from the web graphics telemetry stream:

```text
 +-------------------------------------------------------------------------+
 |                       SMART GLOVE (ESP32-C3)                            |
 |  [MPU6050 6-Axis IMU]   [ME-32671 Flex Sensor]   [TP4056 Battery ADC]   |
 |  [3x Tactile Buttons]   [Haptic Motor Engine]    [Li-Ion + 5V Boost]    |
 +-------------------------------------------------------------------------+
                                    |
                                    | ESP-NOW Protocol (2.4GHz Direct PHY)
                                    | Update Rate: 50 Hz (~20ms latency)
                                    | Packet: 22-byte packed binary with CRC-8
                                    v
 +-------------------------------------------------------------------------+
 |                     ROBOTIC ARM CONTROLLER (ESP32)                      |
 |  - Hardware LEDC PWM Servo Bank (5 Channels @ 50Hz, 14-bit resolution)  |
 |  - Safety Interlocks: Link Timeout (<500ms) / Hardware Limit Detection  |
 |  - Dual-Band Bridge: ESP-NOW Transceiver + Wi-Fi WebSocket Server       |
 +-------------------------------------------------------------------------+
                                    |
                                    | Wi-Fi WebSockets (Port 81) / JSON Stream
                                    | Update Rate: 20 Hz Telemetry & AI Stream
                                    v
 +-------------------------------------------------------------------------+
 |                  GROUND CONTROL STATION (index.html)                    |
 |  - Three.js 3D Kinematic Digital Twin (Deterministic Angle Mapping)     |
 |  - MediaPipe AI Computer Vision (Vision-driven 5-DOF hand tracking)     |
 |  - Web Audio Synthesizer + Web Speech API Arabic Voice Greeting         |
 +-------------------------------------------------------------------------+
```

---

## Key Engineering Features
* **Fully Cable-Free Wireless Operation:** Neither the glove nor the robotic arm requires physical tethering to the computer during run-time.
* **Non-Blocking PlayStation-Style Haptic Feedback:** A coin vibration motor (`DC1030`) driven by an NPN transistor (`BC547`) provides physical alerts.
* **Deterministic 3D Digital Twin:** The web dashboard uses `Three.js` to render the actual kinematic state of the physical arm.
* **Edge AI Computer Vision Tracking:** Operator control can be handed over to an integrated laptop webcam tracking system using `Google MediaPipe Hands`.
* **Integrated Arabic Voice Greeting:** An automated system check sound sequence concludes with an authentic Arabic voice announcement via the browser's native `Web Speech API`.

---

## Firmware Setup & Flashing Guide

### Prerequisites
1. Install **Arduino IDE** (v1.8.19 or v2.x).
2. Install the **ESP32 Arduino Core** (v3.0.0 or higher) via the Boards Manager.
3. Install the required external library via Library Manager: `WebSockets` by Markus Sattler.

### 1. Smart Glove Firmware (`Glove.ino`)
1. Open the sketch in `firmware/Glove/`.
2. Confirm that `config.h` is present in the same directory.
3. Select board: **ESP32C3 Dev Module**.
4. Set **USB CDC On Boot: "Enabled"** (crucial for serial output on ESP32-C3).
5. Connect the ESP32-C3 via USB-C and flash the firmware.

### 2. Robotic Arm Firmware (`RobotArm.ino`)
1. Open the sketch in `firmware/RobotArm/`.
2. **Wi-Fi Configuration:** 
   The Wi-Fi credentials for the robotic arm are securely embedded in the code.
   * **Network Name (SSID):** `[CyberArm_GCS]`
   * **Password:** `[alhawnash123]`
   *(Note: Ensure you are connected to this network to access the Ground Control Station via WebSockets).*
3. Select board: **DOIT ESP32 DEVKIT V1** (or **ESP32 Dev Module**).
4. Flash the board, open Serial Monitor at **115200 baud**, and note the assigned IP address.

---

## License & Attribution
This project is licensed under the **MIT License**.
