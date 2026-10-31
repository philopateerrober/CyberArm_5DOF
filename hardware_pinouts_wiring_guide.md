# Hardware Pinouts, Wiring Schematics & Control Logic Guide
**Project:** Al-Hanash Al-Ali (الحنش الآلي) — 5-DOF Teleoperated Robotic Arm  
**System Architecture:** ESP32-C3 Smart Glove & ESP32-WROOM Robotic Controller  

---

## 1. Smart Glove (ESP32-C3 SuperMini)

The glove acts as the primary sensory telemetry node. It samples inertial dynamics, finger flexion, and user input buttons, transmitting a serialized packet via ESP-NOW at 50 Hz.

### Complete ESP32-C3 Pin Allocation

| ESP32-C3 Pin | Peripheral / Target | Electrical Interface | Operational Role & Notes |
| :--- | :--- | :--- | :--- |
| **`GPIO 6`** | MPU6050 (SDA) | $I^2C$ Data Line | 400 kHz Fast-Mode $I^2C$. Direct line to MPU6050 SDA. |
| **`GPIO 7`** | MPU6050 (SCL) | $I^2C$ Clock Line | 400 kHz Fast-Mode $I^2C$. Direct line to MPU6050 SCL. |
| **`GPIO 0`** | Flex Sensor (ME-32671) | Analog (ADC1_CH0) | Center tap of voltage divider ($10\text{ k}\Omega$ pull-down to GND). |
| **`GPIO 1`** | Battery Voltage Monitor | Analog (ADC1_CH1) | Center tap of $1:1$ divider ($10\text{ k}\Omega + 10\text{ k}\Omega$) from Battery ($B+$). |
| **`GPIO 3`** | Tactile Switch 1 (BTN1) | Digital Input | Internal `PULLUP` enabled. Active LOW when pressed. |
| **`GPIO 4`** | Tactile Switch 2 (BTN2) | Digital Input | Internal `PULLUP` enabled. Active LOW when pressed. |
| **`GPIO 10`** | Tactile Switch 3 (BTN3) | Digital Input | Internal `PULLUP` enabled. Active LOW when pressed. |
| **`GPIO 5`** | Haptic Vibration Motor | Digital Output | Drives BC547 NPN transistor base through a $1\text{ k}\Omega$ resistor. |
| **`3.3V`** | Sensor Power Rail | Regulated Output | Powers MPU6050 and the top side of the Flex Sensor divider. |
| **`5V / VBUS`** | System Power Rail | Boost Converter Out | Connected to 5V Step-Up module output (powers ESP32 & motor). |
| **`GND`** | System Ground | Ground Reference | Common ground for logic, sensors, and power stages. |

> **CRITICAL VOLTAGE RULE:** Always power the **MPU6050 from the 3.3V pin**, never 5V. Connecting the IMU to 5V will burn its $I^2C$ transceivers.

---

## 2. Push-Button Functionality & Control States

The glove uses debounced time-threshold detection (`Short Press < 600ms` vs. `Long Press \ge 600ms`) to provide multiple operational commands across three tactile buttons.

```
       [Pin] ----+-----\ ---- [GND]
                 |
                 +--- (Internal Pull-Up Resistor Enabled)
```

### BTN1 (`GPIO 3`) — Trajectory Record & Playback Engine
* **Short Press ($< 600\text{ ms}$): Save Waypoint**
  * Captures the current physical angles of all 5 joints into internal memory.
  * Triggers a brief haptic pulse confirmation ($60\text{ ms}$).
* **Long Press ($\ge 600\text{ ms}$): Loop Playback**
  * Toggles automated trajectory playback.
  * The arm loops through all stored waypoints smoothly with sinusoidal easing until the button is held again.

### BTN2 (`GPIO 4`) — Emergency Stop (E-STOP) & RPS Game Mode
* **Short Press ($< 600\text{ ms}$): Instant E-STOP**
  * Immediately cuts PWM command generation and freezes all servos in their current coordinates.
  * Broadcasts `CMD_ESTOP` to the ground station dashboard and flags system warning LEDs.
* **Long Press ($\ge 600\text{ ms}$): "الحنش الآلي" (Rock-Paper-Scissors Mode)**
  * Activates the interactive game state machine.
  * The robotic arm performs 3 synchronized countdown shakes (elbow and wrist oscillation), then snaps randomly into a designated pose:
    * **Rock:** Gripper fully closed ($100\%$), wrist horizontal.
    * **Paper:** Gripper fully open ($0\%$), wrist elevated.
    * **Scissors:** Wrist tilted downward, gripper opened partially ($45\%$).
  * The result is transmitted via telemetry to the GCS dashboard for visual display.

### BTN3 (`GPIO 10`) — Kinematic DOF Pairing & Safe Home Pose
* **Short Press ($< 600\text{ ms}$): Cycle Active Joint Axis**
  * Switches teleoperation focus between predefined joint banks:
    * *Pair 0:* Pitch controls Shoulder / Roll controls Base Turntable.
    * *Pair 1:* Pitch controls Elbow Flexion.
    * *Pair 2:* Pitch controls Wrist Elevation.
* **Long Press ($\ge 600\text{ ms}$): Safe Home Pose**
  * Commands all joints to return smoothly to their default factory resting positions:
    * Base: $90^\circ$, Shoulder: $90^\circ$, Elbow: $90^\circ$, Wrist Pitch: $90^\circ$, Gripper: $50^\circ$ (Open).
  * Clears any active kinematic offsets.

---

## 3. Peripheral Sub-Circuits & Schematics (Glove)

### A. Flex Sensor Voltage Divider (SKU: ME-32671)
The carbon resistive strip varies between $10\text{ k}\Omega$ (flat) and $50\text{ k}\Omega$ ($90^\circ$ bend):

```
       +3.3V Rail
           |
           +-----------------------+
                                   |
                             [ Flex Sensor ] (10k - 50k ohm)
                                   |
   GPIO 0 (ADC1_CH0) <-------------+
                                   |
                             [ Fixed Resistor ] (10k ohm, 1%)
                                   |
           GND --------------------+
```

### B. Haptic Feedback Motor Driver (Coin Motor DC1030)
Because the ESP32 GPIO cannot source the $\approx 70\text{ mA}$ required by the motor, an NPN transistor circuit is implemented:

```
                      +5V (Boost Converter Output)
                         |
                         +------------+
                         |            |
                     [DC1030]     [1N4007] (Cathode to +5V, Anode to Collector)
                         |            |
                         +------------+
                         |
                    C (Collector)
                         |
  GPIO 5 ----[ 1k ]------+ B (Base)   BC547 NPN
                         |
                    E (Emitter)
                         |
                        GND
```

### C. Battery Voltage Divider (TP4056 Telemetry)
Scales single-cell Li-Ion voltage ($3.2\text{ V} \to 4.2\text{ V}$) down by half to protect the $3.3\text{ V}$ ADC:

```
       Battery (+) Terminal (3.2V - 4.2V)
           |
           +-----------------------+
                                   |
                             [ Resistor R1 ] (10k ohm)
                                   |
   GPIO 1 (ADC1_CH1) <-------------+  (Reads 1.6V to 2.1V)
                                   |
                             [ Resistor R2 ] (10k ohm)
                                   |
           GND --------------------+
```

---

## 4. Robotic Arm Controller (ESP32-WROOM-32)

The robotic arm controller receives ESP-NOW packets from the glove, converts target angles into 14-bit timer counts via hardware LEDC PWM, and exposes a Wi-Fi WebSocket bridge (Port 81) for the dashboard.

### 5-DOF Servo Channel Mapping

| Servo Joint | ESP32 GPIO | LEDC Channel | Frequency | Resolution | Mechanical Range | Safe Home |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **0: BASE** (Yaw) | **`GPIO 13`** | Channel 0 | 50 Hz | 14-bit | $10^\circ \to 170^\circ$ | $90^\circ$ |
| **1: SHOULDER** (Pitch) | **`GPIO 16`** | Channel 1 | 50 Hz | 14-bit | $30^\circ \to 150^\circ$ | $90^\circ$ |
| **2: ELBOW** (Pitch) | **`GPIO 17`** | Channel 2 | 50 Hz | 14-bit | $30^\circ \to 150^\circ$ | $90^\circ$ |
| **3: WRIST PITCH** | **`GPIO 18`** | Channel 3 | 50 Hz | 14-bit | $20^\circ \to 160^\circ$ | $90^\circ$ |
| **4: GRIPPER** (Claw) | **`GPIO 23`** | Channel 4 | 50 Hz | 14-bit | $40^\circ \to 120^\circ$ | $50^\circ$ (Open) |
| **STATUS LED** | **`GPIO 2`** | — | — | — | Onboard Blue LED | Heartbeat / Link |

---

## 5. High-Current Power Distribution Architecture

> **MANDATORY SAFETY NOTICE:** Servos MG996R / MG995 draw up to $1.5\text{ A} - 2.5\text{ A}$ under dynamic stall. A dedicated external **5V 10A regulated power supply (SMPS)** is strictly required. Attempting to power the servos from the ESP32 onboard regulator will cause instant brownout resets or permanent controller destruction.

### Power Wiring Diagram

```
 +-------------------------------------------------------------------+
 |             EXTERNAL 5V 10A SWITCHING POWER SUPPLY (SMPS)         |
 +-------------------------------------------------------------------+
         (+) Positive (5V)                      (-) Negative (GND)
                 |                                      |
                 |                                      |
                 |   +------------------------------+   |
                 |   |  1000uF / 16V Capacitor      |   |
                 +---+ (Buffer across main rails)   +---+
                 |   +------------------------------+   |
                 |                                      |
                 |                                      +--------------------+
                 |                                      |                    |
                 v                                      v                    v
  +-------------------------------+      +-------------------------------+   |
  |     SERVO POWER BUS (0 to 4)  |      |   ESP32-WROOM CONTROLLER      |   |
  |  - VCC Wires (All RED)        |      |   - GND Pin                   |<--+
  +-------------------------------+      +-------------------------------+
                                                        |
                                                        |
  +-----------------------------------------------------+--------------------+
  |  MANDATORY COMMON GROUND BUS:                                            |
  |  Every GND wire (PSU, ESP32 board, and Servos) MUST meet at a common node |
  +--------------------------------------------------------------------------+
```

### Signal Distribution:
* **Servo Orange / Yellow Wires (PWM Signal):** Run directly from ESP32 pins (`GPIO 13, 16, 17, 18, 23`) to the respective servo signal pins.
* **Servo Red Wires (VCC):** Connected strictly to the external 5V 10A power rail.
* **Servo Brown / Black Wires (GND):** Connected strictly to the common GND bus.

---

## 6. Wiring Verification & Diagnostic Checklist

Before applying main power:
1. **Continuity & Ground Check:** Measure continuity between the ESP32 GND and the power supply negative output. Resistance must be $< 0.5\ \Omega$.
2. **Voltage Rail Verification:** Using a multimeter, verify that the servo VCC rail reads between $5.0\text{ V}$ and $5.2\text{ V}$ before connecting servo signal headers.
3. **Polarity Check on Capacitors:** Ensure the electrolytic capacitor on the servo bus has its negative stripe connected to GND.
4. **Pull-Up Resistor Verification:** Confirm that buttons drop to $0.0\text{ V}$ when pressed and return to $3.3\text{ V}$ when released.