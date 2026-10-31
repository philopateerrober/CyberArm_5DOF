// =====================================================================
//  config.h  -  shared by Glove.ino (ESP32-C3 Super Mini)
//                     and RobotArm.ino (ESP32-WROOM-32 / DevKit V1)
//
//  AL-HAWNASH AL-ALI  (الحونش الآلي)  -  6-DOF teleoperated arm
//
//  Keep this file IDENTICAL in both sketch folders:
//      Glove/config.h     RobotArm/config.h
//
//  Each sketch defines BUILD_GLOVE or BUILD_ROBOT_ARM before including
//  this file, so a wrong board selection stops with a clear #error.
//
//  Required Arduino libraries (RobotArm only):
//      "WebSockets" by Markus Sattler (links2004)  -> WebSocketsServer.h
//  WebServer / ESPmDNS / WiFi / esp_now ship with the ESP32 Arduino core.
// =====================================================================
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#if defined(BUILD_GLOVE) && defined(BUILD_ROBOT_ARM)
#error "Define only one of BUILD_GLOVE / BUILD_ROBOT_ARM"
#endif
#if defined(BUILD_GLOVE) && !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Glove.ino must be built for ESP32-C3 (Arduino board: 'ESP32C3 Dev Module')"
#endif
#if defined(BUILD_ROBOT_ARM) && !defined(CONFIG_IDF_TARGET_ESP32)
#error "RobotArm.ino must be built for classic ESP32 (Arduino board: 'ESP32 Dev Module' / 'DOIT ESP32 DEVKIT V1')"
#endif

// =====================================================================
// 1. MAC ADDRESSES  (edit these two; all-FF = broadcast / accept any)
// =====================================================================
static const uint8_t ARM_MAC[6]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t GLOVE_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static inline bool macIsBroadcast(const uint8_t* m) {
  for (uint8_t i = 0; i < 6; i++) if (m[i] != 0xFF) return false;
  return true;
}
static inline bool macEqual(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

// =====================================================================
// 2. ESP-NOW
// =====================================================================
// ESP-NOW must share the Wi-Fi channel of the arm. When the arm joins your
// router, it inherits the router's channel, so the glove SCANS channels 1..13
// until the arm answers (the arm sends a feedback heartbeat to the glove).
// ESPNOW_CHANNEL is the glove's first guess and the channel of the arm's
// fallback access point.
#define ESPNOW_CHANNEL            1
#define GLOVE_SEND_INTERVAL_MS    20      // 50 packets / second
#define GLOVE_WIFI_TX_POWER       WIFI_POWER_8_5dBm
#define GLOVE_FB_LOST_MS          1200    // no arm feedback this long -> glove starts channel scan
#define GLOVE_HOP_DWELL_MS        300     // time spent on each channel while scanning
#define FB_HEARTBEAT_MS           100     // arm -> glove heartbeat period (10 Hz)
#define FB_EVENT_REPEAT_MS        400     // an event is re-sent inside heartbeats this long (ESP-NOW has no retry)

// =====================================================================
// 3. GLOVE HARDWARE  -- ESP32-C3 Super Mini (GPIO numbers)
// =====================================================================
// Usable, safe pins on the Super Mini: 0,1,3,4,5,6,7,10  (+20,21 = UART0)
#define GLOVE_PIN_I2C_SDA         6       // MPU6050 SDA
#define GLOVE_PIN_I2C_SCL         7       // MPU6050 SCL
#define GLOVE_I2C_HZ              400000
#define GLOVE_PIN_FLEX            0       // ADC1_CH0 (ADC1 keeps working with Wi-Fi on)
#define GLOVE_PIN_BATTERY         1       // ADC1_CH1: 2x 10k voltage divider from TP4056 B+
#define GLOVE_PIN_BTN1            3       // Record / Playback     (GPIO -> switch -> GND)
#define GLOVE_PIN_BTN2            4       // Stop / Game / Reset   (GPIO -> switch -> GND)
#define GLOVE_PIN_BTN3            10      // DOF select / Home     (GPIO -> switch -> GND)
#define GLOVE_PIN_HAPTIC          5       // Vibration motor via BC547 NPN transistor base (1k resistor)
#define GLOVE_PIN_LED             8       // on-board LED of the Super Mini (optional)
#define GLOVE_LED_ACTIVE_LOW      1       // Super Mini LED is active LOW
#define GLOVE_LED_ENABLE          1

// Power-robust start-up (fixes "Brownout detector was triggered" reboot loops)
#define GLOVE_BOOT_HAPTIC         0       // 1 = buzz the motor at power-up (needs a strong supply)
#define GLOVE_SETTLE_MS           400     // wait for the supply to settle before the Wi-Fi radio starts
#define GLOVE_WIFI_START_DELAY_MS 150     // pause between Wi-Fi mode switch and esp_now_init
#define GLOVE_BROWNOUT_DISABLE    0       // 1 = turn the brownout detector OFF (last resort / bench testing only)

// =====================================================================
// 4. GLOVE: MPU6050 / FLEX / BUTTONS / BATTERY / HAPTICS
// =====================================================================
#define MPU6050_ADDR             0x68     // 0x69 if AD0 is tied to 3V3
#define IMU_INTERVAL_MS          5        // 200 Hz sensor loop
#define COMP_FILTER_ALPHA        0.98f    // gyro weight of the complementary filter
#define ACCEL_TRUST_MIN_G        0.75f    // ignore accel correction outside this window
#define ACCEL_TRUST_MAX_G        1.25f
#define GYRO_CAL_SAMPLES         200      // ~1 s at 200 Hz (keep the glove still at power-up)
#define GYRO_CAL_MAX_RANGE_RAW   300      // max (max-min) raw gyro during cal
#define GYRO_BIAS_MAX_DPS        20.0f
#define IMU_FAIL_LIMIT           20       // consecutive I2C failures -> IMU fault (auto retry)

// Neutral pose (captured by the glove after gyro cal; re-capture = hold BTN3 long)
#define NEUTRAL_CAL_MS           1500
#define NEUTRAL_MIN_SAMPLES      100
#define NEUTRAL_MAX_MOVE_DEG     4.0f     // hand must stay within this while sampling
#define GLOVE_INVERT_ROLL        0        // 1 = flip roll direction
#define GLOVE_INVERT_PITCH       0        // 1 = flip pitch direction

// Tilt -> command (rate control: tilt from neutral = joint speed)
#define GLOVE_DEADZONE_DEG       5.0f     // no command inside this delta from neutral
#define GLOVE_FULL_DEFLECTION_DEG 35.0f   // delta at which the command reaches +-100 %
#define GLOVE_ANGLE_ALPHA        0.20f    // low-pass on relative pitch/roll (per IMU tick)

// Flex sensor
// ME-32671 flex sensor: 10 kOhm (flat) .. 50 kOhm (bent 90 deg).
// Divider: 3.3 V -> 10 kOhm fixed -> ADC node -> flex sensor -> GND
//   Vadc = 3.3 V * Rflex / (Rflex + 10k)
//   flat : 3.3 * 10/20 = 1.650 V -> 1.650/3.3 * 4095 = 2047.5 -> 2048
//   bent : 3.3 * 50/60 = 2.750 V -> 2.750/3.3 * 4095 = 3412.5 -> 3413
#define FLEX_MIN                 2048     // raw value with hand OPEN   -> gripper 0 %
#define FLEX_MAX                 3413     // raw value with fingers BENT -> gripper 100 %
#define FLEX_FAULT_LOW           20       // raw below this  = wire broken / shorted -> invalid
#define FLEX_FAULT_HIGH          4080     // raw above this  = wire broken / shorted -> invalid
#define GLOVE_FLEX_ALPHA         0.20f    // flex low-pass (per IMU tick)

// Battery monitoring (TP4056 Li-Ion 3.2V .. 4.2V via 10k/10k divider)
#define BATT_MIN_MV              3200     // 0 %
#define BATT_MAX_MV              4200     // 100 %
#define BATT_DIVIDER_RATIO       2.0f     // 2x 10k resistors
#define BATT_LOW_WARN_PCT        20       // trigger periodic heartbeat vibration below 20%
#define BATT_LOW_ALERT_PERIOD_MS 30000    // repeat low-battery haptic alert every 30s

// Buttons (all non-blocking, millis() based)
//   BTN1 short = RECORD toggle        BTN1 hold 0.8 s = PLAYBACK toggle
//   BTN2 press = EMERGENCY STOP       BTN2 hold 2 s   = start "Al-Hawnash Al-Ali" RPS game
//                                      (if the arm was ALREADY stopped when pressed: RESET instead)
//   BTN3 short = next DOF             BTN3 hold 1.2 s = HOME + re-capture neutral pose
#define BUTTON_DEBOUNCE_MS       25
#define BTN1_LONG_MS             800
#define BTN2_HOLD_MS             2000
#define BTN3_RECENTER_MS         1200
#define STOP_STRETCH_MS          250      // keep the STOP flag set this long after release
#define DOF_SWITCH_LOCK_MS       400      // after changing DOF, send zero motion for this long
#define ARM_MODE_FRESH_MS        600      // arm mode learned from feedback is trusted this long

#define GLOVE_DEBUG_INTERVAL_MS  1000

// =====================================================================
// 5. ROBOT ARM HARDWARE -- ESP32-WROOM-32 / DevKit V1 (GPIO numbers)
// =====================================================================
#define ARM_PIN_LED              2        // on-board LED of the DevKit V1 (optional)
#define ARM_LED_ENABLE           1

#define SERVO_PWM_HZ             50
#define SERVO_LEDC_BITS          14       // PWM resolution (50 Hz, 14 bit = 1.2 us steps)

// =====================================================================
// 5b. ARM NETWORK / GROUND CONTROL STATION (Wi-Fi + WebSocket)
// =====================================================================
// Router credentials. Leave WIFI_STA_SSID empty ("") to run ONLY the access point.
// If the router cannot be joined within WIFI_STA_TIMEOUT_MS the arm starts the
// access point WIFI_AP_SSID (dashboard address then is 192.168.4.1).
#define WIFI_STA_SSID            ""
#define WIFI_STA_PASS            ""
#define WIFI_STA_TIMEOUT_MS      12000
#define WIFI_AP_SSID             "CyberArm_GCS"
#define WIFI_AP_PASS             "alhawnash123"   // 8..63 chars
#define WIFI_HOSTNAME            "alhawnash"      // reachable as alhawnash.local (mDNS)
#define GCS_WS_PORT              81
#define GCS_HTTP_PORT            80
#define GCS_TELEMETRY_INTERVAL_MS 50      // 20 Hz
#define GCS_SERIAL_TELEMETRY     0        // 1 = also print telemetry JSON on USB serial
#define GCS_CMD_TIMEOUT_MS       2500     // CAMERA_CV with no dashboard command this long -> hold

// =====================================================================
// 6. SERVOS / DOF  (robot)
// =====================================================================
enum ServoId : uint8_t {
  SERVO_BASE = 0,
  SERVO_SHOULDER,
  SERVO_ELBOW,
  SERVO_WRIST_PITCH,
  SERVO_WRIST_ROLL,
  SERVO_GRIPPER,
  SERVO_COUNT               // must match the table below
};
#define SERVO_NONE           0xFF

struct ServoConfig {
  uint8_t  pin;             // ESP32 GPIO for the servo signal wire (unique)
  float    minAngle;        // software limit (deg)
  float    maxAngle;        // software limit (deg)
  float    homeAngle;       // HOME pose
  float    parkAngle;       // pose the arm physically rests in when powered off
  uint16_t minPulseUs;      // pulse at 0 deg
  uint16_t maxPulseUs;      // pulse at 180 deg
  float    maxSpeedDps;     // slew-rate limit (deg/s)
  bool     invert;          // mirror direction
};

static const ServoConfig SERVO_CONFIG[SERVO_COUNT] = {
  // pin  min   max  home  park  minUs maxUs  dps  invert
  { 13,   10,  170,   90,   90,   500, 2500, 150, false },   // BASE
  { 16,   30,  150,   90,   90,   500, 2500,  90, false },   // SHOULDER
  { 17,   30,  150,   90,   90,   500, 2500,  90, false },   // ELBOW
  { 18,   20,  160,   90,   90,   500, 2400, 180, false },   // WRIST PITCH
  { 19,   10,  170,   90,   90,   500, 2400, 180, false },   // WRIST ROLL
  { 23,   40,  120,   50,   50,   500, 2400, 200, false },   // GRIPPER
};

#define GRIPPER_OPEN_ANGLE       50.0f
#define GRIPPER_CLOSED_ANGLE     110.0f

struct DofMap {
  const char* name;
  uint8_t rollServo;   float rollGain;
  uint8_t pitchServo;  float pitchGain;
};
static const DofMap DOF_MAP[] = {
  { "BASE+SHOULDER", SERVO_BASE,       1.0f, SERVO_SHOULDER,    1.0f },
  { "ELBOW",         SERVO_NONE,       0.0f, SERVO_ELBOW,       1.0f },
  { "WRIST_PITCH",   SERVO_NONE,       0.0f, SERVO_WRIST_PITCH, 1.0f },
  { "WRIST_ROLL",    SERVO_WRIST_ROLL, 1.0f, SERVO_NONE,        0.0f },
};
#define DOF_COUNT  ((uint8_t)(sizeof(DOF_MAP) / sizeof(DOF_MAP[0])))

// =====================================================================
// 7. ROBOT: CONTROL / SAFETY / MOTION
// =====================================================================
#define LINK_TIMEOUT_MS          500      // no valid packet this long -> E-STOP (safe state)
#define GLOVE_STALE_MS           200      // data older than this is ignored (arm holds)
#define START_CONNECT_MS         1000     // glove must be connected this long before auto-start
#define SEQ_STALE_WINDOW         50       // seq this far behind = old/duplicate -> dropped
#define CONTROL_INTERVAL_MS      10       // servo update tick (100 Hz)
#define STATUS_PRINT_INTERVAL_MS 2000

#define MANUAL_MAX_RATE_DPS      110.0f   // joint speed at 100 % command (sensitivity)
#define SERVO_SMOOTH_RATE        25.0f    // 1/s exponential easing toward target
#define PLAYBACK_SMOOTH_RATE     35.0f
#define HOME_SPEED_SCALE         0.40f    // fraction of maxSpeedDps while homing / returning
#define PLAY_START_SPEED_SCALE   0.50f
#define AT_TARGET_TOL_DEG        0.6f
#define PLAY_START_TOL_DEG       1.5f
#define PLAY_START_TIMEOUT_MS    6000

#define RECORD_INTERVAL_MS       40       // 25 frames / s
#define MAX_FRAMES               1500     // 60 s (~24 KB RAM, 0.1 deg resolution)

// =====================================================================
// 7b. "AL-HAWNASH AL-ALI" ROCK-PAPER-SCISSORS GAME
// =====================================================================
// Sequence: READY pose -> 3 shakes (UP/DOWN, a haptic tick on each down-beat)
//           -> REVEAL (arm snaps into a random gesture) -> hold -> slow return HOME.
#define RPS_SHAKES               3
#define RPS_BEAT_HALF_MS         380      // one UP or one DOWN stroke
#define RPS_READY_TOL_DEG        2.5f
#define RPS_READY_TIMEOUT_MS     4000
#define RPS_REVEAL_HOLD_MS       2400
#define RPS_RETURN_TIMEOUT_MS    5000
#define RPS_SPEED_SCALE          1.0f
#define RPS_SNAP_SMOOTH_RATE     60.0f    // faster easing for the reveal "snap"
#define RPS_SNIP_START_MS        350      // scissors: start snipping the jaws this long after reveal
#define RPS_SNIP_PERIOD_MS       150
#define RPS_SNIP_A_DEG           66.0f
#define RPS_SNIP_B_DEG           92.0f

enum RpsPhase : uint8_t {
  RPS_PHASE_IDLE = 0,
  RPS_PHASE_READY,
  RPS_PHASE_SHAKE,
  RPS_PHASE_REVEAL,
  RPS_PHASE_RETURN,
  RPS_PHASE_COUNT
};
enum RpsGesture : uint8_t {
  RPS_G_NONE = 0,
  RPS_G_ROCK,
  RPS_G_PAPER,
  RPS_G_SCISSORS,
  RPS_G_COUNT
};

//                                     BASE SHLDR ELBOW WR-P  WR-R  GRIP
static const float RPS_POSE_READY[SERVO_COUNT]    = { 90,  82,   98,   90,   90,  114 };
static const float RPS_POSE_UP[SERVO_COUNT]       = { 90,  68,   80,   84,   90,  114 };
static const float RPS_POSE_DOWN[SERVO_COUNT]     = { 90,  90,  104,   96,   90,  114 };
static const float RPS_POSE_ROCK[SERVO_COUNT]     = { 90,  80,   96,   90,   90,  116 };  // fist: jaws closed
static const float RPS_POSE_PAPER[SERVO_COUNT]    = { 90,  80,   96,   76,   90,   46 };  // flat hand: jaws wide open, palm tilted
static const float RPS_POSE_SCISSORS[SERVO_COUNT] = { 90,  80,   96,   90,  138,   80 };  // wrist rolled + jaws snip

// =====================================================================
// 8. PACKETS  (Glove <-> Robot)
// =====================================================================
#define PACKET_MAGIC             0xA5
#define PACKET_VERSION           4        // v4: battery, haptics, RPS state in feedback

// Arm operating modes (shared: the glove reads the arm mode from feedback)
enum RobotMode : uint8_t {
  MODE_WAIT_GLOVE = 0,
  MODE_HOMING,
  MODE_MANUAL,
  MODE_RECORD,
  MODE_PLAYBACK,
  MODE_CAMERA_CV,   // driven by the dashboard (AI hand tracking or sliders)
  MODE_RPS_GAME,    // "Al-Hawnash Al-Ali" rock-paper-scissors routine
  MODE_ESTOP
};

enum GloveCmd : uint8_t {
  CMD_NONE = 0,
  CMD_ESTOP,             // emergency stop (also sent as a level flag, see FLAG_ESTOP_HELD)
  CMD_RESET,             // leave E-STOP and return HOME slowly
  CMD_RECORD_TOGGLE,     // start / stop recording
  CMD_PLAY_TOGGLE,       // start / stop playback
  CMD_HOME,              // stop record/playback/game, all joints return slowly to HOME
  CMD_RPS_GAME,          // start the rock-paper-scissors game (toggles: stops it if already running)
  CMD_COUNT
};

// Haptic event types triggered locally on the Glove or remotely by the Arm
enum HapticEvent : uint8_t {
  HAP_NONE = 0,
  HAP_SINGLE_PULSE,      // crisp tap (DOF switch / mode toggle / neutral ready)
  HAP_DOUBLE_RUMBLE,     // stutter (joint hit mechanical limit / reset / long-press accepted)
  HAP_HEAVY_ESTOP,       // heavy 1.2 s continuous buzz (E-STOP)
  HAP_LOW_BATT,          // heartbeat double-tap (battery < 20 %)
  HAP_RPS_BEAT,          // very short tick on every game shake
  HAP_RPS_REVEAL,        // triple pulse when the arm shows its gesture
  HAP_COUNT
};

// Button bits
#define BTN1_BIT                 0x01
#define BTN2_BIT                 0x02
#define BTN3_BIT                 0x04
#define BTN_MASK                 0x07

// Flag bits
#define FLAG_IMU_OK              0x01     // IMU running, pitch/roll valid
#define FLAG_CTRL_VALID          0x02     // neutral captured: rollCmd/pitchCmd are live
#define FLAG_FLEX_OK             0x04     // flex sensor reading plausible
#define FLAG_ESTOP_HELD          0x08     // STOP button down (+ stretch) - highest priority
#define FLAG_MASK                0x0F

// Glove -> Robot Packet (22 bytes, packed)
struct __attribute__((packed)) GlovePacket {
  uint8_t  magic;        // PACKET_MAGIC
  uint8_t  version;      // PACKET_VERSION
  uint16_t seq;          // +1 per packet
  uint32_t tsMs;         // glove millis()
  uint8_t  cmd;          // GloveCmd (sticky)
  uint8_t  cmdId;        // changes when a NEW command is issued
  uint8_t  selDof;       // index into DOF_MAP
  uint8_t  buttons;      // BTN*_BIT debounced states
  uint8_t  flags;        // FLAG_*
  uint8_t  gripperPct;   // 0..100 (flex, already scaled)
  int16_t  pitchX10;     // degrees*10, relative to neutral
  int16_t  rollX10;      // degrees*10, relative to neutral
  int8_t   pitchCmd;     // -100..+100 %
  int8_t   rollCmd;      // -100..+100 %
  uint8_t  batteryPct;   // 0..100 % Li-Ion battery level
  uint8_t  crc;          // CRC-8 of all previous bytes
};
static_assert(sizeof(GlovePacket) == 22, "GlovePacket must be 22 bytes");

// Robot -> Glove Feedback Packet (11 bytes, packed). Also used as 10 Hz heartbeat
// (hapticEvent = HAP_NONE) so the glove can detect the arm and lock its Wi-Fi channel.
struct __attribute__((packed)) ArmFeedbackPacket {
  uint8_t  magic;        // PACKET_MAGIC
  uint8_t  version;      // PACKET_VERSION
  uint16_t seq;          // +1 per feedback packet
  uint8_t  hapticEvent;  // HapticEvent enum
  uint8_t  eventId;      // increments on each distinct haptic event
  uint8_t  armMode;      // RobotMode
  uint8_t  limitMask;    // bitmask of servos currently at min/max limit
  uint8_t  rpsPhase;     // RpsPhase
  uint8_t  rpsGesture;   // RpsGesture shown by the arm (0 until revealed)
  uint8_t  crc;          // CRC-8 of all previous bytes
};
static_assert(sizeof(ArmFeedbackPacket) == 11, "ArmFeedbackPacket must be 11 bytes");

static inline uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}
static inline uint8_t gloveCrc(const GlovePacket& p) {
  return crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(GlovePacket) - 1);
}
static inline uint8_t feedbackCrc(const ArmFeedbackPacket& p) {
  return crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(ArmFeedbackPacket) - 1);
}
static inline float wrapDeg(float a) {
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a - 180.0f;
}

static inline bool packetValid(const GlovePacket& p) {
  if (p.magic != PACKET_MAGIC || p.version != PACKET_VERSION) return false;
  if (p.crc != gloveCrc(p)) return false;
  if (p.cmd >= CMD_COUNT) return false;
  if (p.selDof >= DOF_COUNT) return false;
  if (p.buttons & (uint8_t)~BTN_MASK) return false;
  if (p.flags & (uint8_t)~FLAG_MASK) return false;
  if (p.gripperPct > 100) return false;
  if (p.batteryPct > 100) return false;
  if (p.pitchX10 < -1805 || p.pitchX10 > 1805) return false;
  if (p.rollX10  < -1805 || p.rollX10  > 1805) return false;
  if (p.pitchCmd < -100 || p.pitchCmd > 100) return false;
  if (p.rollCmd  < -100 || p.rollCmd  > 100) return false;
  return true;
}

static inline bool feedbackPacketValid(const ArmFeedbackPacket& p) {
  if (p.magic != PACKET_MAGIC || p.version != PACKET_VERSION) return false;
  if (p.crc != feedbackCrc(p)) return false;
  if (p.hapticEvent >= HAP_COUNT) return false;
  if (p.armMode > MODE_ESTOP) return false;
  if (p.rpsPhase >= RPS_PHASE_COUNT) return false;
  if (p.rpsGesture >= RPS_G_COUNT) return false;
  return true;
}
