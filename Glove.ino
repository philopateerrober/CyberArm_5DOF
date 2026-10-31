/*
  Glove.ino  -  ESP32-C3 Super Mini glove controller for "Al-Hawnash Al-Ali"
                MPU6050 + flex sensor + 3 push buttons + haptic motor + battery ADC
                Bidirectional ESP-NOW <-> Robot Arm (with automatic Wi-Fi channel lock)

  Arduino IDE:  Board "ESP32C3 Dev Module", "USB CDC On Boot: Enabled" (for Serial).

  Buttons
    BTN1 short  = record toggle        BTN1 hold  = playback toggle
    BTN2 press  = EMERGENCY STOP       BTN2 hold 2 s = start RPS game
                                         (or RESET if the arm was already stopped / in a game when pressed)
    BTN3 short  = next DOF             BTN3 hold  = HOME + re-capture neutral pose
*/
#define BUILD_GLOVE
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#if GLOVE_BROWNOUT_DISABLE
  #include "soc/soc.h"
  #include "soc/rtc_cntl_reg.h"
#endif
#include "config.h"

// ---------------------------------------------------------------- MPU6050
#define MPU_REG_SMPLRT_DIV    0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75
#define GYRO_LSB_PER_DPS      65.5f      // +-500 dps
#define ACCEL_LSB_PER_G       16384.0f   // +-2 g
#define RAD2DEG_F             57.2957795f
#define PITCH_LIMIT_DEG       85.0f      // Euler gimbal-lock guard

enum ImuState     : uint8_t { IMU_FAULT, IMU_WARMUP, IMU_GYRO_CAL, IMU_RUN };
enum NeutralState : uint8_t { NEUTRAL_WAIT, NEUTRAL_SAMPLING, NEUTRAL_READY };

static ImuState imuState = IMU_FAULT;
static uint32_t imuLastTryMs = 0, imuWarmupStartMs = 0, lastImuMs = 0, lastImuUs = 0;
static uint8_t  imuFailCount = 0;
static int32_t  gSum[3];
static int16_t  gMin[3], gMax[3];
static uint16_t gCount = 0;
static float    gBias[3] = {0, 0, 0};
static bool     angleInit = false;
static float    pitchDeg = 0, rollDeg = 0;          // absolute (filtered)

// neutral pose + relative angles
static NeutralState neutralState = NEUTRAL_WAIT;
static uint32_t nStartMs = 0;
static uint16_t nCount = 0;
static float    nRefP = 0, nRefR = 0, nSumP = 0, nSumR = 0, nMinP = 0, nMaxP = 0, nMinR = 0, nMaxR = 0;
static float    neutralPitch = 0, neutralRoll = 0;
static float    relPitchF = 0, relRollF = 0;        // relative to neutral, low-passed (deg)

// flex
static float    flexF = 0;
static bool     flexInit = false, flexOk = false;

// battery ADC (TP4056)
static float    battMvF = 4000.0f;
static bool     battInit = false;
static uint8_t  battPct = 100;
static uint32_t lastBattReadMs = 0;
static uint32_t lastLowBattAlertMs = 0;

// ---------------------------------------------------------------- haptic engine (non-blocking step player)
struct HapticStep {
  uint16_t durationMs;
  bool     motorOn;
};

class HapticEngine {
public:
  void begin() {
    pinMode(GLOVE_PIN_HAPTIC, OUTPUT);
    digitalWrite(GLOVE_PIN_HAPTIC, LOW);
    _active = false;
    _currentEv = HAP_NONE;
  }

  void trigger(HapticEvent ev, uint32_t now) {
    if (ev == HAP_NONE) return;
    // The E-STOP buzz has top priority: it cannot be interrupted, and it is not restarted
    // when the arm's confirmation of the same E-STOP arrives.
    if (_active && _currentEv == HAP_HEAVY_ESTOP) return;

    switch (ev) {
      case HAP_SINGLE_PULSE:
        _steps[0] = { 65, true };
        _stepCount = 1;
        break;
      case HAP_DOUBLE_RUMBLE:
        _steps[0] = { 80, true };
        _steps[1] = { 55, false };
        _steps[2] = { 110, true };
        _stepCount = 3;
        break;
      case HAP_HEAVY_ESTOP:
        _steps[0] = { 1200, true };
        _stepCount = 1;
        break;
      case HAP_LOW_BATT:
        _steps[0] = { 50, true };
        _steps[1] = { 80, false };
        _steps[2] = { 50, true };
        _steps[3] = { 400, false };
        _steps[4] = { 60, true };
        _stepCount = 5;
        break;
      case HAP_RPS_BEAT:
        _steps[0] = { 35, true };
        _stepCount = 1;
        break;
      case HAP_RPS_REVEAL:
        _steps[0] = { 60, true };
        _steps[1] = { 50, false };
        _steps[2] = { 60, true };
        _steps[3] = { 50, false };
        _steps[4] = { 140, true };
        _stepCount = 5;
        break;
      default:
        return;
    }
    _currentEv = ev;
    _stepIdx = 0;
    _stepStartMs = now;
    _active = true;
    digitalWrite(GLOVE_PIN_HAPTIC, _steps[0].motorOn ? HIGH : LOW);
  }

  void update(uint32_t now) {
    if (!_active) return;
    if ((uint32_t)(now - _stepStartMs) >= _steps[_stepIdx].durationMs) {
      _stepIdx++;
      if (_stepIdx >= _stepCount) {
        digitalWrite(GLOVE_PIN_HAPTIC, LOW);
        _active = false;
        _currentEv = HAP_NONE;
      } else {
        _stepStartMs = now;
        digitalWrite(GLOVE_PIN_HAPTIC, _steps[_stepIdx].motorOn ? HIGH : LOW);
      }
    }
  }

private:
  HapticStep  _steps[6];
  uint8_t     _stepCount = 0;
  uint8_t     _stepIdx = 0;
  uint32_t    _stepStartMs = 0;
  bool        _active = false;
  HapticEvent _currentEv = HAP_NONE;
};

static HapticEngine haptics;

// ---------------------------------------------------------------- buttons
enum { BI_1 = 0, BI_2 = 1, BI_3 = 2, BTN_COUNT = 3 };
struct Btn {
  uint8_t  pin;
  bool     raw, stable, longFired;
  uint32_t changeMs, pressMs;
};
static Btn btn[BTN_COUNT] = {
  { GLOVE_PIN_BTN1, false, false, false, 0, 0 },
  { GLOVE_PIN_BTN2, false, false, false, 0, 0 },
  { GLOVE_PIN_BTN3, false, false, false, 0, 0 },
};
static uint32_t estopUntilMs = 0;
static uint32_t dofLockUntilMs = 0;
static uint8_t  selDof = 0;
static uint8_t  btn2ModeAtPress = 0xFF;   // arm mode known when BTN2 went down (0xFF = unknown)

// command (sticky until replaced)
static uint8_t  pendingCmd = CMD_NONE;
static uint8_t  cmdId = 0;

// ESP-NOW + channel lock
static uint16_t txSeq = 0;
static uint32_t txCount = 0, txFail = 0, lastSendMs = 0, lastDebugMs = 0;
static volatile uint32_t txDelivered = 0, txNotDelivered = 0;
static bool     espNowOk = false;
static uint8_t  espChannel = ESPNOW_CHANNEL;
static bool     channelLocked = false;
static uint32_t lastHopMs = 0;

// Arm -> Glove feedback
static portMUX_TYPE fbMux = portMUX_INITIALIZER_UNLOCKED;
static ArmFeedbackPacket rxFbPkt;
static bool     rxFbNew = false;
static bool     fbEverSeen = false;
static uint32_t lastFbMs = 0;
static uint8_t  lastFbEventId = 0xFF;
static uint8_t  armMode = 0xFF;
static uint32_t armModeMs = 0;
static uint8_t  armRpsPhase = RPS_PHASE_IDLE;
static uint8_t  armRpsGesture = RPS_G_NONE;

// ---------------------------------------------------------------- small helpers
static void sendCmd(uint8_t c) { pendingCmd = c; cmdId++; }

static bool estopActive(uint32_t now) {
  return btn[BI_2].stable || (int32_t)(estopUntilMs - now) > 0;
}

static bool armModeFresh(uint32_t now) {
  return fbEverSeen && (uint32_t)(now - armModeMs) < (uint32_t)ARM_MODE_FRESH_MS;
}

static const char* cmdName(uint8_t c) {
  switch (c) {
    case CMD_ESTOP:         return "ESTOP";
    case CMD_RESET:         return "RESET";
    case CMD_RECORD_TOGGLE: return "RECORD toggle";
    case CMD_PLAY_TOGGLE:   return "PLAYBACK toggle";
    case CMD_HOME:          return "HOME";
    case CMD_RPS_GAME:      return "RPS GAME";
    default:                return "NONE";
  }
}

// ---------------------------------------------------------------- MPU6050 I2C
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission((uint8_t)MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuReadBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission((uint8_t)MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)n, true) != n) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!Wire.available()) return false;
    buf[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool mpuInit() {
  uint8_t who = 0;
  if (!mpuReadBytes(MPU_REG_WHO_AM_I, &who, 1)) return false;
  if (who != 0x68 && who != 0x69) Serial.printf("[WARN] MPU WHO_AM_I=0x%02X (clone?) - continuing\n", who);
  bool ok = true;
  ok &= mpuWrite(MPU_REG_PWR_MGMT_1, 0x01);     // wake, gyro-X PLL clock
  ok &= mpuWrite(MPU_REG_SMPLRT_DIV, 0x04);
  ok &= mpuWrite(MPU_REG_CONFIG, 0x03);         // DLPF ~42 Hz
  ok &= mpuWrite(MPU_REG_GYRO_CONFIG, 0x08);    // +-500 dps
  ok &= mpuWrite(MPU_REG_ACCEL_CONFIG, 0x00);   // +-2 g
  return ok;
}

static bool mpuReadRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t b[14];
  if (!mpuReadBytes(MPU_REG_ACCEL_XOUT_H, b, 14)) return false;
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

// ---------------------------------------------------------------- flex & battery
static void updateFlex() {
  uint32_t s = 0;
  for (uint8_t i = 0; i < 4; i++) s += analogRead(GLOVE_PIN_FLEX);
  const float v = (float)(s / 4);
  flexOk = (v >= FLEX_FAULT_LOW && v <= FLEX_FAULT_HIGH);
  if (!flexOk) return;
  if (!flexInit) { flexF = v; flexInit = true; }
  else           { flexF += GLOVE_FLEX_ALPHA * (v - flexF); }
}

static uint8_t gripperPercent() {
  float n = (flexF - (float)FLEX_MIN) / (float)(FLEX_MAX - FLEX_MIN);
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
  return (uint8_t)lroundf(n * 100.0f);
}

static void updateBattery(uint32_t now) {
  if ((uint32_t)(now - lastBattReadMs) < 200) return;
  lastBattReadMs = now;

  uint32_t mvSum = 0;
  for (uint8_t i = 0; i < 4; i++) mvSum += analogReadMilliVolts(GLOVE_PIN_BATTERY);
  const float pinMv = (float)(mvSum / 4);
  const float estBattMv = pinMv * BATT_DIVIDER_RATIO;

  // Divider not wired yet (< 1.0 V): report 100 % and skip the low-battery alarm
  if (estBattMv < 1000.0f) {
    battPct = 100;
    return;
  }

  if (!battInit) { battMvF = estBattMv; battInit = true; }
  else           { battMvF += 0.10f * (estBattMv - battMvF); }

  float pct = (battMvF - (float)BATT_MIN_MV) / (float)(BATT_MAX_MV - BATT_MIN_MV) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  battPct = (uint8_t)lroundf(pct);

  if (battPct <= BATT_LOW_WARN_PCT && (uint32_t)(now - lastLowBattAlertMs) >= (uint32_t)BATT_LOW_ALERT_PERIOD_MS) {
    lastLowBattAlertMs = now;
    haptics.trigger(HAP_LOW_BATT, now);
  }
}

// ---------------------------------------------------------------- IMU
static void startImuWarmup(uint32_t now) {
  imuState = IMU_WARMUP;
  imuWarmupStartMs = now;
  imuFailCount = 0;
}

static void startGyroCal() {
  imuState = IMU_GYRO_CAL;
  gCount = 0;
  for (uint8_t i = 0; i < 3; i++) { gSum[i] = 0; gMin[i] = INT16_MAX; gMax[i] = INT16_MIN; }
  angleInit = false;
  Serial.println("[CAL] Gyro calibration - keep the glove still...");
}

static void updateImu(uint32_t now) {
  if (imuState == IMU_FAULT) {
    if ((uint32_t)(now - imuLastTryMs) >= 1000) {
      imuLastTryMs = now;
      if (mpuInit()) { Serial.println("[OK] MPU6050 connected"); startImuWarmup(now); }
    }
    return;
  }
  if (imuState == IMU_WARMUP) {
    if ((uint32_t)(now - imuWarmupStartMs) >= 100) startGyroCal();
    return;
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) {
    if (++imuFailCount >= IMU_FAIL_LIMIT) {
      imuState = IMU_FAULT;
      imuLastTryMs = now;
      angleInit = false;
      Serial.println("[ERROR] MPU6050 read failed - sending buttons/flex only, retrying");
    }
    return;
  }
  imuFailCount = 0;

  if (imuState == IMU_GYRO_CAL) {
    const int16_t g[3] = { gx, gy, gz };
    for (uint8_t i = 0; i < 3; i++) {
      gSum[i] += g[i];
      if (g[i] < gMin[i]) gMin[i] = g[i];
      if (g[i] > gMax[i]) gMax[i] = g[i];
    }
    if (++gCount >= GYRO_CAL_SAMPLES) {
      bool still = true;
      float bias[3];
      for (uint8_t i = 0; i < 3; i++) {
        bias[i] = (float)gSum[i] / (float)gCount;
        if ((gMax[i] - gMin[i]) > GYRO_CAL_MAX_RANGE_RAW) still = false;
        if (fabsf(bias[i]) / GYRO_LSB_PER_DPS > GYRO_BIAS_MAX_DPS) still = false;
      }
      if (!still) {
        Serial.println("[CAL] Glove moved during gyro cal, restarting");
        startGyroCal();
      } else {
        for (uint8_t i = 0; i < 3; i++) gBias[i] = bias[i];
        imuState = IMU_RUN;
        angleInit = false;
        lastImuUs = micros();
        Serial.println("[CAL] Gyro calibration complete");
      }
    }
    return;
  }

  // ---- IMU_RUN: complementary filter ---------------------------------
  const uint32_t nowUs = micros();
  float dt = (float)(nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (dt <= 0.0f || dt > 0.05f) dt = IMU_INTERVAL_MS * 1e-3f;

  const float fax = (float)ax / ACCEL_LSB_PER_G;
  const float fay = (float)ay / ACCEL_LSB_PER_G;
  const float faz = (float)az / ACCEL_LSB_PER_G;
  const float gx_dps = ((float)gx - gBias[0]) / GYRO_LSB_PER_DPS;
  const float gy_dps = ((float)gy - gBias[1]) / GYRO_LSB_PER_DPS;
  const float gz_dps = ((float)gz - gBias[2]) / GYRO_LSB_PER_DPS;

  const float accRoll  = atan2f(fay, faz) * RAD2DEG_F;
  const float accPitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * RAD2DEG_F;
  const float mag = sqrtf(fax * fax + fay * fay + faz * faz);
  const bool accelOk = (mag > ACCEL_TRUST_MIN_G && mag < ACCEL_TRUST_MAX_G);

  if (!angleInit) {
    if (accelOk) {
      rollDeg = accRoll;
      pitchDeg = fminf(fmaxf(accPitch, -PITCH_LIMIT_DEG), PITCH_LIMIT_DEG);
      angleInit = true;
    }
    return;
  }

  const float rollRad  = rollDeg / RAD2DEG_F;
  const float pitchRad = fminf(fmaxf(pitchDeg, -PITCH_LIMIT_DEG), PITCH_LIMIT_DEG) / RAD2DEG_F;
  const float sr = sinf(rollRad), cr = cosf(rollRad), tp = tanf(pitchRad);
  const float rollRate  = gx_dps + (sr * tp) * gy_dps + (cr * tp) * gz_dps;
  const float pitchRate = cr * gy_dps - sr * gz_dps;

  float r = wrapDeg(rollDeg + rollRate * dt);
  float p = pitchDeg + pitchRate * dt;
  if (accelOk) {
    r = wrapDeg(r + (1.0f - COMP_FILTER_ALPHA) * wrapDeg(accRoll - r));
    p = p + (1.0f - COMP_FILTER_ALPHA) * (accPitch - p);
  }
  if (p > PITCH_LIMIT_DEG) p = PITCH_LIMIT_DEG;
  if (p < -PITCH_LIMIT_DEG) p = -PITCH_LIMIT_DEG;
  rollDeg = r;
  pitchDeg = p;
}

// ---------------------------------------------------------------- neutral pose
static void startNeutral(uint32_t now) {
  neutralState = NEUTRAL_SAMPLING;
  nStartMs = now;
  nCount = 0;
  nSumP = nSumR = 0.0f;
  nMinP = nMaxP = nMinR = nMaxR = 0.0f;
  nRefP = pitchDeg;
  nRefR = rollDeg;
  Serial.println("[CAL] Hold your hand in the neutral pose...");
}

static void updateNeutral(uint32_t now) {
  const bool imuRun = (imuState == IMU_RUN) && angleInit;
  if (!imuRun) { neutralState = NEUTRAL_WAIT; return; }
  if (neutralState == NEUTRAL_WAIT) { startNeutral(now); return; }

  if (neutralState == NEUTRAL_READY) {
    const float sp = GLOVE_INVERT_PITCH ? -1.0f : 1.0f;
    const float sr = GLOVE_INVERT_ROLL  ? -1.0f : 1.0f;
    const float dp = sp * (pitchDeg - neutralPitch);
    const float dr = sr * wrapDeg(rollDeg - neutralRoll);
    relPitchF += GLOVE_ANGLE_ALPHA * (dp - relPitchF);
    relRollF  += GLOVE_ANGLE_ALPHA * (dr - relRollF);
    return;
  }

  // NEUTRAL_SAMPLING
  const float dp = pitchDeg - nRefP;
  const float dr = wrapDeg(rollDeg - nRefR);
  nSumP += dp;  nSumR += dr;  nCount++;
  if (dp < nMinP) nMinP = dp;
  if (dp > nMaxP) nMaxP = dp;
  if (dr < nMinR) nMinR = dr;
  if (dr > nMaxR) nMaxR = dr;

  if ((nMaxP - nMinP) > NEUTRAL_MAX_MOVE_DEG || (nMaxR - nMinR) > NEUTRAL_MAX_MOVE_DEG) {
    Serial.println("[CAL] Hand moved, restarting - hold still");
    startNeutral(now);
    return;
  }
  if ((uint32_t)(now - nStartMs) >= (uint32_t)NEUTRAL_CAL_MS && nCount >= NEUTRAL_MIN_SAMPLES) {
    neutralPitch = nRefP + nSumP / (float)nCount;
    neutralRoll  = wrapDeg(nRefR + nSumR / (float)nCount);
    relPitchF = relRollF = 0.0f;
    neutralState = NEUTRAL_READY;
    haptics.trigger(HAP_SINGLE_PULSE, now);
    Serial.printf("[CAL] Neutral captured (pitch=%.1f roll=%.1f) - controls LIVE\n", neutralPitch, neutralRoll);
  }
}

static int8_t axisCmd(float delta) {
  const float a = fabsf(delta);
  if (a <= GLOVE_DEADZONE_DEG) return 0;
  float r = (a - GLOVE_DEADZONE_DEG) / (GLOVE_FULL_DEFLECTION_DEG - GLOVE_DEADZONE_DEG);
  if (r > 1.0f) r = 1.0f;
  r = 0.5f * r + 0.5f * (r * r * r);
  const int v = (int)lroundf(r * 100.0f);
  return (int8_t)(delta < 0.0f ? -v : v);
}

// ---------------------------------------------------------------- buttons (state machine, no delay)
static void nextDof(uint32_t now) {
  selDof = (uint8_t)((selDof + 1) % DOF_COUNT);
  dofLockUntilMs = now + DOF_SWITCH_LOCK_MS;
  haptics.trigger(HAP_SINGLE_PULSE, now);
  Serial.printf("[DOF] Current DOF: %u/%u  %s\n", (unsigned)(selDof + 1), (unsigned)DOF_COUNT, DOF_MAP[selDof].name);
}

static void onPress(uint8_t i, uint32_t now) {
  btn[i].pressMs = now;
  btn[i].longFired = false;
  if (i == BI_2) {                                      // EMERGENCY STOP: immediate, on press
    btn2ModeAtPress = armModeFresh(now) ? armMode : 0xFF;   // remember what the arm was doing BEFORE this press
    estopUntilMs = now + STOP_STRETCH_MS;
    sendCmd(CMD_ESTOP);
    haptics.trigger(HAP_HEAVY_ESTOP, now);
    Serial.println("[STOP] EMERGENCY STOP sent");
  }
}

static void onHold(uint8_t i, uint32_t now) {
  if (i == BI_1 && (uint32_t)(now - btn[i].pressMs) >= (uint32_t)BTN1_LONG_MS) {
    btn[i].longFired = true;
    sendCmd(CMD_PLAY_TOGGLE);
    haptics.trigger(HAP_DOUBLE_RUMBLE, now);
    Serial.println("[BTN1] Playback toggle sent");
  } else if (i == BI_2 && (uint32_t)(now - btn[i].pressMs) >= (uint32_t)BTN2_HOLD_MS) {
    btn[i].longFired = true;
    const bool wasStopped = (btn2ModeAtPress == MODE_ESTOP || btn2ModeAtPress == MODE_RPS_GAME);
    if (wasStopped) {
      sendCmd(CMD_RESET);
      Serial.println("[BTN2] RESET sent (leave E-STOP, return HOME)");
    } else {
      sendCmd(CMD_RPS_GAME);
      Serial.println("[BTN2] RPS GAME start sent");
    }
    haptics.trigger(HAP_DOUBLE_RUMBLE, now);
  } else if (i == BI_3 && (uint32_t)(now - btn[i].pressMs) >= (uint32_t)BTN3_RECENTER_MS) {
    btn[i].longFired = true;
    sendCmd(CMD_HOME);
    neutralState = NEUTRAL_WAIT;
    haptics.trigger(HAP_DOUBLE_RUMBLE, now);
    Serial.println("[BTN3] HOME sent + re-capturing neutral pose (hold the hand still)");
  }
}

static void onRelease(uint8_t i, uint32_t now) {
  if (i == BI_2) estopUntilMs = now + STOP_STRETCH_MS;
  if (btn[i].longFired) return;
  if (i == BI_1) {
    sendCmd(CMD_RECORD_TOGGLE);
    haptics.trigger(HAP_SINGLE_PULSE, now);
    Serial.println("[BTN1] Record toggle sent");
  } else if (i == BI_3) {
    nextDof(now);
  }
}

static void updateButtons(uint32_t now) {
  for (uint8_t i = 0; i < BTN_COUNT; i++) {
    Btn& b = btn[i];
    const bool raw = (digitalRead(b.pin) == LOW);
    if (raw != b.raw) { b.raw = raw; b.changeMs = now; }
    if (b.raw != b.stable && (uint32_t)(now - b.changeMs) >= (uint32_t)BUTTON_DEBOUNCE_MS) {
      b.stable = b.raw;
      if (b.stable) onPress(i, now); else onRelease(i, now);
    }
    if (b.stable && !b.longFired) onHold(i, now);
  }
}

// ---------------------------------------------------------------- ESP-NOW (bidirectional)
#if defined(ESP_ARDUINO_VERSION_VAL)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0)
    #define SEND_CB_USES_TX_INFO 1
  #endif
#endif
#ifdef SEND_CB_USES_TX_INFO
static void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
#else
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
#endif
  if (status == ESP_NOW_SEND_SUCCESS) txDelivered++; else txNotDelivered++;
}

static void handleFeedbackRx(const uint8_t* src, const uint8_t* data, int len) {
  if (!data || !src || len != (int)sizeof(ArmFeedbackPacket)) return;
  if (!macIsBroadcast(ARM_MAC) && !macEqual(src, ARM_MAC)) return;
  ArmFeedbackPacket fb;
  memcpy(&fb, data, sizeof(fb));
  if (!feedbackPacketValid(fb)) return;
  portENTER_CRITICAL(&fbMux);
  rxFbPkt = fb;
  rxFbNew = true;
  portEXIT_CRITICAL(&fbMux);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  handleFeedbackRx(info ? info->src_addr : nullptr, data, len);
}
#else
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  handleFeedbackRx(mac, data, len);
}
#endif

static void processArmFeedback(uint32_t now) {
  ArmFeedbackPacket fb;
  bool fresh = false;
  portENTER_CRITICAL(&fbMux);
  if (rxFbNew) {
    fb = rxFbPkt;
    rxFbNew = false;
    fresh = true;
  }
  portEXIT_CRITICAL(&fbMux);
  if (!fresh) return;

  fbEverSeen = true;
  lastFbMs = now;
  armMode = fb.armMode;
  armModeMs = now;
  armRpsPhase = fb.rpsPhase;
  armRpsGesture = fb.rpsGesture;

  // The arm repeats an event inside its heartbeats for a short time; eventId de-duplicates.
  if (fb.hapticEvent != HAP_NONE && fb.eventId != lastFbEventId) {
    lastFbEventId = fb.eventId;
    haptics.trigger((HapticEvent)fb.hapticEvent, now);
    Serial.printf("[HAPTIC] Feedback event=%u limitMask=0x%02X\n", (unsigned)fb.hapticEvent, (unsigned)fb.limitMask);
  }
}

// Channel supervision: lock when the arm answers, scan 1..13 when it does not.
static void superviseChannel(uint32_t now) {
  const bool fresh = fbEverSeen && (uint32_t)(now - lastFbMs) < (uint32_t)GLOVE_FB_LOST_MS;
  if (fresh) {
    if (!channelLocked) {
      channelLocked = true;
      Serial.printf("[LINK] Arm locked on Wi-Fi channel %u\n", (unsigned)espChannel);
    }
    return;
  }
  if (channelLocked) {
    channelLocked = false;
    lastFbEventId = 0xFF;
    lastHopMs = now;
    Serial.println("[LINK] Arm feedback lost - scanning channels");
  }
  if (!espNowOk) return;
  if ((uint32_t)(now - lastHopMs) >= (uint32_t)GLOVE_HOP_DWELL_MS) {
    lastHopMs = now;
    espChannel = (uint8_t)((espChannel % 13) + 1);
    esp_wifi_set_channel(espChannel, WIFI_SECOND_CHAN_NONE);
  }
}

static bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
#ifdef GLOVE_WIFI_TX_POWER
  WiFi.setTxPower(GLOVE_WIFI_TX_POWER);
#endif
  delay(GLOVE_WIFI_START_DELAY_MS);                     // spread the radio's start-up current peak
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(espChannel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, ARM_MAC, 6);
  peer.channel = 0;                  // 0 = "current channel" so scanning keeps working
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

static void sendPacket(uint32_t now) {
  const bool imuRun = (imuState == IMU_RUN) && angleInit;
  const bool ctrl   = imuRun && (neutralState == NEUTRAL_READY);
  const bool estop  = estopActive(now);
  const bool locked = (int32_t)(dofLockUntilMs - now) > 0;

  GlovePacket p;
  memset(&p, 0, sizeof(p));
  p.magic      = PACKET_MAGIC;
  p.version    = PACKET_VERSION;
  p.seq        = txSeq++;
  p.tsMs       = now;
  p.cmd        = pendingCmd;
  p.cmdId      = cmdId;
  p.selDof     = selDof;
  p.buttons    = (uint8_t)((btn[BI_1].stable ? BTN1_BIT : 0) | (btn[BI_2].stable ? BTN2_BIT : 0) |
                           (btn[BI_3].stable ? BTN3_BIT : 0));
  p.flags      = (uint8_t)((imuRun ? FLAG_IMU_OK : 0) | (ctrl ? FLAG_CTRL_VALID : 0) |
                           (flexOk && flexInit ? FLAG_FLEX_OK : 0) | (estop ? FLAG_ESTOP_HELD : 0));
  p.gripperPct = gripperPercent();
  p.batteryPct = battPct;

  if (ctrl) {
    long pr = lroundf(relPitchF * 10.0f), rr = lroundf(relRollF * 10.0f);
    if (pr < -1800) pr = -1800;
    if (pr >  1800) pr =  1800;
    if (rr < -1800) rr = -1800;
    if (rr >  1800) rr =  1800;
    p.pitchX10 = (int16_t)pr;
    p.rollX10  = (int16_t)rr;
    if (!estop && !locked) {
      p.pitchCmd = axisCmd(relPitchF);
      p.rollCmd  = axisCmd(relRollF);
    }
  }
  p.crc = gloveCrc(p);

  txCount++;
  if (esp_now_send(ARM_MAC, reinterpret_cast<const uint8_t*>(&p), sizeof(p)) != ESP_OK) txFail++;
}

// ---------------------------------------------------------------- LED / debug
static void updateLed(uint32_t now) {
#if GLOVE_LED_ENABLE
  bool on;
  if (estopActive(now))                      on = ((now / 100) & 1);   // fast blink: STOP
  else if (!channelLocked)                   on = ((now / 60) & 1);    // very fast flicker: searching for the arm
  else if (neutralState == NEUTRAL_READY)    on = true;                // solid: ready
  else                                       on = ((now / 250) & 1);   // slow blink: calibrating
  digitalWrite(GLOVE_PIN_LED, (on != (bool)GLOVE_LED_ACTIVE_LOW) ? HIGH : LOW);
#else
  (void)now;
#endif
}

static void printStatus(uint32_t now) {
  if ((uint32_t)(now - lastDebugMs) < (uint32_t)GLOVE_DEBUG_INTERVAL_MS) return;
  lastDebugMs = now;
  Serial.printf("[GLOVE] imu=%s neutral=%s dof=%s pitch=%.1f roll=%.1f flex=%.0f(%s) grip=%u%% batt=%u%% btn=%u%u%u cmd=%s ch=%u%s arm=%u rps=%u/%u tx=%lu fail=%lu delivered=%lu lost=%lu\n",
                imuState == IMU_RUN ? "RUN" : (imuState == IMU_FAULT ? "FAULT" : "CAL"),
                neutralState == NEUTRAL_READY ? "OK" : "WAIT",
                DOF_MAP[selDof].name, relPitchF, relRollF, flexF, flexOk ? "ok" : "BAD",
                (unsigned)gripperPercent(), (unsigned)battPct,
                btn[BI_1].stable ? 1 : 0, btn[BI_2].stable ? 1 : 0, btn[BI_3].stable ? 1 : 0,
                cmdName(pendingCmd), (unsigned)espChannel, channelLocked ? "(locked)" : "(scan)",
                (unsigned)armMode, (unsigned)armRpsPhase, (unsigned)armRpsGesture,
                (unsigned long)txCount, (unsigned long)txFail,
                (unsigned long)txDelivered, (unsigned long)txNotDelivered);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
#if GLOVE_BROWNOUT_DISABLE
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);            // WARNING: hides a real power problem
#endif
  Serial.begin(115200);
  delay(300);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3 && defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  Serial.println("[BOOT] Glove starting (ESP32-C3 + Haptics + Battery Telemetry)");

  haptics.begin();
#if GLOVE_BOOT_HAPTIC
  haptics.trigger(HAP_SINGLE_PULSE, millis());          // optional boot pulse (current spike!)
#endif
  delay(GLOVE_SETTLE_MS);                               // let the battery / USB rail settle before the radio starts

  for (uint8_t i = 0; i < BTN_COUNT; i++) pinMode(btn[i].pin, INPUT_PULLUP);
  delay(5);
  for (uint8_t i = 0; i < BTN_COUNT; i++) {
    btn[i].raw = btn[i].stable = (digitalRead(btn[i].pin) == LOW);
    btn[i].changeMs = millis();
    btn[i].longFired = btn[i].stable;                   // a button held at power-up does nothing
  }
#if GLOVE_LED_ENABLE
  pinMode(GLOVE_PIN_LED, OUTPUT);
#endif
  analogReadResolution(12);
  analogSetPinAttenuation(GLOVE_PIN_FLEX, ADC_11db);
  analogSetPinAttenuation(GLOVE_PIN_BATTERY, ADC_11db);

  Wire.begin(GLOVE_PIN_I2C_SDA, GLOVE_PIN_I2C_SCL);
  Wire.setClock(GLOVE_I2C_HZ);
  Wire.setTimeOut(3);

  espNowOk = initEspNow();
  Serial.println(espNowOk ? "[OK] ESP-NOW initialized" : "[ERROR] ESP-NOW init failed");
  Serial.print("[INFO] My MAC: ");
  Serial.println(WiFi.macAddress());
  if (macIsBroadcast(ARM_MAC)) Serial.println("[WARN] ARM_MAC not set in config.h -> broadcasting");

  randomSeed(micros() ^ (uint32_t)ESP.getEfuseMac() ^ (uint32_t)analogRead(GLOVE_PIN_FLEX));
  txSeq = (uint16_t)random(0, 65536);
  cmdId = (uint8_t)random(0, 256);

  if (mpuInit()) {
    Serial.println("[OK] MPU6050 connected");
    startImuWarmup(millis());
  } else {
    Serial.println("[ERROR] MPU6050 not found - will retry");
    imuState = IMU_FAULT;
    imuLastTryMs = millis();
  }
  lastHopMs = millis();
  lastImuMs = millis();
  Serial.printf("[DOF] Current DOF: %u/%u  %s\n", (unsigned)(selDof + 1), (unsigned)DOF_COUNT, DOF_MAP[selDof].name);
}

void loop() {
  const uint32_t now = millis();

  updateButtons(now);
  processArmFeedback(now);
  superviseChannel(now);
  haptics.update(now);
  updateBattery(now);

  if ((uint32_t)(now - lastImuMs) >= (uint32_t)IMU_INTERVAL_MS) {
    lastImuMs += IMU_INTERVAL_MS;
    if ((uint32_t)(now - lastImuMs) >= (uint32_t)IMU_INTERVAL_MS) lastImuMs = now;

    updateFlex();
    updateImu(now);
    updateNeutral(now);
  }

  if (espNowOk && (uint32_t)(now - lastSendMs) >= (uint32_t)GLOVE_SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendPacket(now);
  }

  updateLed(now);
  printStatus(now);
}
