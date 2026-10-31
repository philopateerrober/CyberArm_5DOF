/*
  RobotArm.ino  -  ESP32-WROOM-32 / ESP32 DevKit V1 + 6 servos driven directly
                   "Al-Hawnash Al-Ali" (الحونش الآلي)

  * Bidirectional ESP-NOW with the glove (feedback heartbeat lets the glove lock the Wi-Fi channel)
  * Wi-Fi: joins your router (config.h) or falls back to the access point "CyberArm_GCS"
  * Embedded HTTP server (port 80) + WebSocket server (port 81) for the Ground Control Station
      ws://<arm-ip>:81/   telemetry out (20 Hz JSON), commands in (JSON)
  * Rock-Paper-Scissors game state machine (glove BTN2 hold or dashboard)

  Libraries: "WebSockets" by Markus Sattler (links2004). Everything else ships with the ESP32 core.

  Dashboard -> arm JSON commands (one JSON object per WebSocket text frame):
    {"cmd":"PING"}                         keep-alive (sent every 500 ms)
    {"cmd":"ESTOP"}   {"cmd":"RESET"}      emergency stop / leave E-STOP and go HOME
    {"cmd":"HOME"}                         all joints to HOME
    {"cmd":"MODE","val":"MANUAL|CAMERA_CV|RECORD|PLAYBACK|RPS_GAME"}
    {"cmd":"RPS_START"}  {"cmd":"RPS_STOP"}
    {"cmd":"SERVO","id":0,"angle":95.0}
    {"cmd":"ANGLES","val":[90,90,90,90,90,50]}
*/
#define BUILD_ROBOT_ARM
#include <Arduino.h>
#include <stdarg.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include "config.h"

// =====================================================================
// Custom types used in function signatures.
// Keep these ABOVE every function: the Arduino IDE auto-generates prototypes
// right after the includes, so a type declared later would be "not declared".
// =====================================================================
enum EstopReason : uint8_t { ESTOP_BUTTON, ESTOP_TIMEOUT, ESTOP_GCS };
enum PlayPhase   : uint8_t { PLAY_MOVE_TO_START, PLAY_RUNNING };

// =====================================================================
// Servo bank: direct LEDC PWM + limits + smoothing + slew-rate cap
// =====================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define SERVO_PWM_ATTACH(i, pin)       ledcAttach((pin), SERVO_PWM_HZ, SERVO_LEDC_BITS)
  #define SERVO_PWM_WRITE(i, pin, duty)  ledcWrite((pin), (duty))
#else
  #define SERVO_PWM_ATTACH(i, pin)       do { ledcSetup((i), SERVO_PWM_HZ, SERVO_LEDC_BITS); ledcAttachPin((pin), (i)); } while (0)
  #define SERVO_PWM_WRITE(i, pin, duty)  ledcWrite((i), (duty))
#endif

class ServoBank {
public:
  bool begin() {
    _enabled = false;
    if (!validateConfig()) return false;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      _current[i] = _target[i] = clampAngle(i, SERVO_CONFIG[i].parkAngle);
      _lastUs[i] = 0;
      pinMode(SERVO_CONFIG[i].pin, OUTPUT);
      digitalWrite(SERVO_CONFIG[i].pin, LOW);
    }
    return true;
  }

  // Outputs stay LOW (no pulses, servos limp) until the first real command arrives.
  void enableOutputs() {
    if (_enabled) return;
    _enabled = true;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      _lastUs[i] = 0;
      SERVO_PWM_ATTACH(i, SERVO_CONFIG[i].pin);
      writeServo(i);
    }
  }
  bool     outputsEnabled() const { return _enabled; }

  void     setTarget(uint8_t i, float a) { if (i < SERVO_COUNT && !isnan(a)) _target[i] = clampAngle(i, a); }
  void     setAllTargets(const float* a) { if (a) for (uint8_t i = 0; i < SERVO_COUNT; i++) setTarget(i, a[i]); }
  float    target(uint8_t i)  const { return i < SERVO_COUNT ? _target[i]  : 0.0f; }
  float    current(uint8_t i) const { return i < SERVO_COUNT ? _current[i] : 0.0f; }
  void     goHome() { for (uint8_t i = 0; i < SERVO_COUNT; i++) _target[i] = clampAngle(i, SERVO_CONFIG[i].homeAngle); }
  void     hold()   { for (uint8_t i = 0; i < SERVO_COUNT; i++) _target[i] = _current[i]; }
  bool     atTarget(float tol) const {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) if (fabsf(_target[i] - _current[i]) > tol) return false;
    return true;
  }
  void     setSpeedScale(float s) { _speedScale = (s < 0.05f) ? 0.05f : (s > 1.0f ? 1.0f : s); }
  void     setSmoothRate(float r) { _smoothRate = (r < 1.0f) ? 1.0f : r; }

  float clampAngle(uint8_t i, float a) const {
    if (i >= SERVO_COUNT || isnan(a)) return 90.0f;
    const ServoConfig& c = SERVO_CONFIG[i];
    if (a < c.minAngle) return c.minAngle;
    if (a > c.maxAngle) return c.maxAngle;
    return a;
  }

  // Bitmask of servos whose TARGET touches the mechanical min/max limit
  uint8_t getLimitMask() const {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      const ServoConfig& c = SERVO_CONFIG[i];
      if (_target[i] <= c.minAngle + 0.2f || _target[i] >= c.maxAngle - 0.2f) mask |= (1U << i);
    }
    return mask;
  }

  void update(float dt) {
    if (!_enabled || dt <= 0.0f) return;
    if (dt > 0.1f) dt = 0.1f;
    float blend = _smoothRate * dt;
    if (blend > 1.0f) blend = 1.0f;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      const float diff = _target[i] - _current[i];
      if (fabsf(diff) < 0.05f) {
        _current[i] = _target[i];
      } else {
        float step = diff * blend;
        const float maxStep = SERVO_CONFIG[i].maxSpeedDps * _speedScale * dt;
        if (step > maxStep) step = maxStep;
        else if (step < -maxStep) step = -maxStep;
        _current[i] += step;
      }
      _current[i] = clampAngle(i, _current[i]);
      writeServo(i);
    }
  }

private:
  bool validateConfig() const {
    uint64_t used = 0;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      const ServoConfig& c = SERVO_CONFIG[i];
      const bool pinOk = c.pin <= 33 && !(c.pin >= 6 && c.pin <= 11) && c.pin != 0 && c.pin != 2 &&
                         c.pin != 5 && c.pin != 12 && c.pin != 15 && c.pin != 1 && c.pin != 3;
      const bool ok = pinOk && !(used & (1ULL << c.pin)) &&
                      c.minAngle >= 0.0f && c.maxAngle <= 180.0f && c.minAngle < c.maxAngle &&
                      c.homeAngle >= c.minAngle && c.homeAngle <= c.maxAngle &&
                      c.parkAngle >= c.minAngle && c.parkAngle <= c.maxAngle &&
                      c.minPulseUs < c.maxPulseUs && c.minPulseUs >= 400 && c.maxPulseUs <= 2600 &&
                      c.maxSpeedDps > 0.0f;
      if (!ok) { Serial.printf("[ERROR] Servo %u config invalid (pin/limits/pulse)\n", (unsigned)i); return false; }
      used |= (1ULL << c.pin);
    }
    return true;
  }
  uint16_t angleToUs(uint8_t i, float angle) const {
    const ServoConfig& c = SERVO_CONFIG[i];
    float a = clampAngle(i, angle);
    if (c.invert) a = 180.0f - a;
    return (uint16_t)((float)c.minPulseUs + (a / 180.0f) * (float)(c.maxPulseUs - c.minPulseUs) + 0.5f);
  }
  void writeServo(uint8_t i) {
    const uint16_t us = angleToUs(i, _current[i]);
    if (us == _lastUs[i]) return;
    const float period_us = 1000000.0f / (float)SERVO_PWM_HZ;
    const uint32_t maxDuty = (1UL << SERVO_LEDC_BITS) - 1;
    uint32_t duty = (uint32_t)((float)us / period_us * (float)(1UL << SERVO_LEDC_BITS) + 0.5f);
    if (duty > maxDuty) duty = maxDuty;
    SERVO_PWM_WRITE(i, SERVO_CONFIG[i].pin, duty);
    _lastUs[i] = us;
  }

  float    _current[SERVO_COUNT];
  float    _target[SERVO_COUNT];
  uint16_t _lastUs[SERVO_COUNT];
  float    _speedScale = 1.0f;
  float    _smoothRate = SERVO_SMOOTH_RATE;
  bool     _enabled    = false;
};

// =====================================================================
// Motion recorder (teach & repeat)
// =====================================================================
struct MotionFrame {
  uint32_t t;
  uint16_t aX10[SERVO_COUNT];
};

class MotionRecorder {
public:
  void     startRecording(uint32_t now) { _count = 0; _idx = 0; _playing = false; _recording = true; _recStart = now; }
  void     stopRecording() { _recording = false; }
  bool     isRecording() const { return _recording; }
  bool     isPlaying() const { return _playing; }
  uint16_t frameCount() const { return _count; }
  uint32_t durationMs() const { return _count ? _f[_count - 1].t : 0; }
  bool     hasRecording() const { return !_recording && _count >= 2; }

  bool addFrame(uint32_t now, const uint16_t* angX10) {
    if (!_recording || !angX10 || _count >= MAX_FRAMES) return false;
    MotionFrame& f = _f[_count];
    f.t = now - _recStart;
    if (_count > 0 && f.t < _f[_count - 1].t) f.t = _f[_count - 1].t;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) f.aX10[i] = angX10[i];
    _count++;
    return true;
  }
  bool firstFrame(float* out) const {
    if (!out || _count == 0) return false;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) out[i] = (float)_f[0].aX10[i] * 0.1f;
    return true;
  }
  bool startPlayback(uint32_t now) {
    if (!hasRecording()) return false;
    _idx = 0; _playStart = now; _playing = true;
    return true;
  }
  void stopPlayback() { _playing = false; }
  bool sample(uint32_t now, float* out) {
    if (!_playing || !out || _count < 2) return false;
    const uint32_t t = now - _playStart;
    const uint16_t last = _count - 1;
    if (t >= _f[last].t) {
      for (uint8_t i = 0; i < SERVO_COUNT; i++) out[i] = (float)_f[last].aX10[i] * 0.1f;
      _playing = false;
      return false;
    }
    while ((uint16_t)(_idx + 1) < last && _f[_idx + 1].t <= t) _idx++;
    const MotionFrame& a = _f[_idx];
    const MotionFrame& b = _f[_idx + 1];
    const uint32_t span = b.t - a.t;
    float frac = 1.0f;
    if (span > 0) {
      frac = (float)(t - a.t) / (float)span;
      if (frac < 0.0f) frac = 0.0f;
      if (frac > 1.0f) frac = 1.0f;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; i++) out[i] = ((float)a.aX10[i] + ((float)b.aX10[i] - (float)a.aX10[i]) * frac) * 0.1f;
    return true;
  }
private:
  MotionFrame _f[MAX_FRAMES];
  uint16_t _count = 0, _idx = 0;
  bool     _recording = false, _playing = false;
  uint32_t _recStart = 0, _playStart = 0;
};

// =====================================================================
// Network objects + logging (Serial + WebSocket)
// =====================================================================
static WebServer        httpServer(GCS_HTTP_PORT);
static WebSocketsServer wsServer(GCS_WS_PORT);
static int     wsClientCount = 0;
static bool    netIsAp = false;
static char    netIp[20] = "0.0.0.0";

// printf-style log line to USB serial and to every connected dashboard
static void gcsLog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void gcsLog(const char* fmt, ...) {
  char msg[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  Serial.println(msg);
  if (wsClientCount <= 0) return;

  char out[240];
  size_t o = 0;
  const char* head = "{\"type\":\"log\",\"msg\":\"";
  for (const char* h = head; *h; h++) out[o++] = *h;
  for (const char* c = msg; *c && o < sizeof(out) - 6; c++) {
    if ((uint8_t)*c < 0x20) continue;
    if (*c == '"' || *c == '\\') out[o++] = '\\';
    out[o++] = *c;
  }
  out[o++] = '"';
  out[o++] = '}';
  out[o] = '\0';
  wsServer.broadcastTXT(out, o);
}

// =====================================================================
// State
// =====================================================================
static ServoBank       servos;
static MotionRecorder  recorder;
static RobotMode       mode = MODE_WAIT_GLOVE;
static EstopReason     estopReason = ESTOP_BUTTON;
static bool servosOk = false, nowOk = false;

// Shared with the ESP-NOW callback: guarded by rxMux
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
static GlovePacket  rxPkt;
static uint8_t      rxSrcMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static int8_t       rxRssi = -50;
static bool         rxNew = false;
static uint32_t     rxGood = 0, rxBad = 0, rxForeign = 0;

// Main-loop state
static GlovePacket glove;
static uint8_t  activeGloveMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static int8_t   lastRssi = -55;
static bool     linkHasPacket = false, linkWasUp = false, needPrime = true;
static uint32_t linkLastMs = 0, linkSinceMs = 0, rxRejected = 0;
static uint16_t lastSeq = 0;
static uint32_t lastTs = 0;
static uint8_t  lastCmdId = 0;
static bool     estopPrev = false;
static uint8_t  curDof = 0;

// Feedback (Arm -> Glove)
static uint8_t  glovePeerMac[6];
static bool     glovePeerAdded = false;
static uint16_t fbSeq = 0;
static uint8_t  fbEventId = 0;
static uint8_t  fbLastEvent = HAP_NONE;
static uint32_t fbLastEventMs = 0;
static uint8_t  prevLimitMask = 0;
static uint32_t lastLimitBuzzMs = 0;
static uint32_t lastHeartbeatMs = 0;

// Timing, GCS
static uint32_t lastCtrlMs = 0, lastRecordMs = 0, playPhaseStartMs = 0, lastStatusMs = 0, lastGcsMs = 0;
static uint32_t lastGcsCmdMs = 0;
static PlayPhase playPhase = PLAY_MOVE_TO_START;
static char     serialBuf[256];
static uint16_t serialIdx = 0;
static char     wsCmdBuf[256];

// Al-Hawnash Al-Ali game
static uint8_t  rpsPhase = RPS_PHASE_IDLE;
static uint8_t  rpsBeat = 0;
static uint8_t  rpsGesture = RPS_G_NONE;
static uint16_t rpsRound = 0;
static uint32_t rpsPhaseStartMs = 0;
static int16_t  rpsHalf = -1;
static uint32_t rpsLastSnipMs = 0;
static bool     rpsSnipOpen = false;

// =====================================================================
// Helpers & feedback sender
// =====================================================================
static const char* modeName(RobotMode m) {
  switch (m) {
    case MODE_WAIT_GLOVE: return "WAIT_GLOVE";
    case MODE_HOMING:     return "HOMING";
    case MODE_MANUAL:     return "MANUAL";
    case MODE_RECORD:     return "RECORD";
    case MODE_PLAYBACK:   return "PLAYBACK";
    case MODE_CAMERA_CV:  return "CAMERA_CV";
    case MODE_RPS_GAME:   return "RPS_GAME";
    case MODE_ESTOP:      return "ESTOP";
  }
  return "?";
}

static const char* rpsGestureName(uint8_t g) {
  switch (g) {
    case RPS_G_ROCK:     return "ROCK";
    case RPS_G_PAPER:    return "PAPER";
    case RPS_G_SCISSORS: return "SCISSORS";
    default:             return "-";
  }
}

static bool ensureGlovePeer(const uint8_t* mac) {
  if (glovePeerAdded && macEqual(glovePeerMac, mac)) return true;
  if (glovePeerAdded) { esp_now_del_peer(glovePeerMac); glovePeerAdded = false; }
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;                 // follow the current Wi-Fi channel (router or AP)
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  memcpy(glovePeerMac, mac, 6);
  glovePeerAdded = true;
  return true;
}

// Sends one feedback packet. ev == HAP_NONE is a heartbeat; it re-carries the last real
// event for FB_EVENT_REPEAT_MS because ESP-NOW broadcast/unicast frames have no retry.
static void sendFeedbackToGlove(HapticEvent ev) {
  if (!nowOk) return;
  const uint32_t now = millis();
  const uint8_t* destMac = !macIsBroadcast(GLOVE_MAC) ? GLOVE_MAC : activeGloveMac;
  if (!ensureGlovePeer(destMac)) return;

  uint8_t outEv = (uint8_t)ev;
  if (ev != HAP_NONE) {
    fbEventId++;
    fbLastEvent = (uint8_t)ev;
    fbLastEventMs = now;
  } else if (fbLastEvent != HAP_NONE && (uint32_t)(now - fbLastEventMs) < (uint32_t)FB_EVENT_REPEAT_MS) {
    outEv = fbLastEvent;
  }

  ArmFeedbackPacket fb;
  fb.magic       = PACKET_MAGIC;
  fb.version     = PACKET_VERSION;
  fb.seq         = fbSeq++;
  fb.hapticEvent = outEv;
  fb.eventId     = fbEventId;
  fb.armMode     = (uint8_t)mode;
  fb.limitMask   = servos.getLimitMask();
  fb.rpsPhase    = rpsPhase;
  fb.rpsGesture  = rpsGesture;
  fb.crc         = feedbackCrc(fb);
  esp_now_send(destMac, reinterpret_cast<const uint8_t*>(&fb), sizeof(fb));
}

static void sendHeartbeat(uint32_t now) {
  if (!nowOk || !linkHasPacket) return;
  if (!(linkHasPacket && (uint32_t)(now - linkLastMs) <= (uint32_t)LINK_TIMEOUT_MS)) return;
  if ((uint32_t)(now - lastHeartbeatMs) < (uint32_t)FB_HEARTBEAT_MS) return;
  lastHeartbeatMs = now;
  sendFeedbackToGlove(HAP_NONE);
}

static void setMode(RobotMode m) {
  if (m == mode) return;
  mode = m;
  if (m != MODE_RPS_GAME) rpsPhase = RPS_PHASE_IDLE;     // leaving the game by ANY route ends it
  gcsLog("[MODE] %s", modeName(m));
}

static void restoreNormalMotion() {
  servos.setSpeedScale(1.0f);
  servos.setSmoothRate(SERVO_SMOOTH_RATE);
}

static bool linkAlive(uint32_t now) {
  return linkHasPacket && (uint32_t)(now - linkLastMs) <= (uint32_t)LINK_TIMEOUT_MS;
}

// =====================================================================
// ESP-NOW receive
// =====================================================================
static void handleRx(const uint8_t* src, const uint8_t* data, int len, int8_t rssi) {
  if (!data || !src) return;
  if (!macIsBroadcast(GLOVE_MAC) && !macEqual(src, GLOVE_MAC)) { rxForeign++; return; }
  GlovePacket p;
  if (len != (int)sizeof(GlovePacket)) { rxBad++; return; }
  memcpy(&p, data, sizeof(p));
  if (!packetValid(p)) { rxBad++; return; }
  portENTER_CRITICAL(&rxMux);
  rxPkt = p;
  memcpy(rxSrcMac, src, 6);
  rxRssi = rssi;
  rxNew = true;
  rxGood++;
  portEXIT_CRITICAL(&rxMux);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int8_t rssi = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : -50;
  handleRx(info ? info->src_addr : nullptr, data, len, rssi);
}
#else
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  handleRx(mac, data, len, -50);
}
#endif

// Wi-Fi must already be up (startNetwork) so ESP-NOW uses the same radio channel.
static bool initEspNow() {
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (esp_now_init() != ESP_OK) return false;
  return esp_now_register_recv_cb(onDataRecv) == ESP_OK;
}

static bool takeRx(GlovePacket& out) {
  bool fresh;
  portENTER_CRITICAL(&rxMux);
  fresh = rxNew;
  if (fresh) {
    out = rxPkt;
    memcpy(activeGloveMac, rxSrcMac, 6);
    lastRssi = rxRssi;
    rxNew = false;
  }
  portEXIT_CRITICAL(&rxMux);
  return fresh;
}

static bool acceptPacket(const GlovePacket& p, uint32_t now) {
  if (linkHasPacket && !needPrime) {
    const int16_t d = (int16_t)(p.seq - lastSeq);
    if (d == 0) return false;
    if (d < 0 && d > -(int16_t)SEQ_STALE_WINDOW) return false;
    if ((int32_t)(p.tsMs - lastTs) < -2000) {
      needPrime = true;
      gcsLog("[LINK] Glove restart detected - resync");
    }
  }
  lastSeq = p.seq;
  lastTs = p.tsMs;
  linkHasPacket = true;
  linkLastMs = now;
  return true;
}

// =====================================================================
// Emergency stop / reset / record / playback
// =====================================================================
static void enterEstop(EstopReason r) {
  if (mode == MODE_ESTOP) return;
  recorder.stopRecording();
  recorder.stopPlayback();
  servos.hold();
  restoreNormalMotion();
  estopReason = r;
  setMode(MODE_ESTOP);
  sendFeedbackToGlove(HAP_HEAVY_ESTOP);
  gcsLog(r == ESTOP_BUTTON ? "[EMERGENCY] STOP (glove button)" :
        (r == ESTOP_GCS    ? "[EMERGENCY] STOP (GCS dashboard)" : "[EMERGENCY] STOP (glove link timeout)"));
}

static void startHoming() {
  if (!servosOk) { gcsLog("[HOME] Servo config invalid"); return; }
  if (!servos.outputsEnabled()) servos.enableOutputs();
  servos.setSpeedScale(HOME_SPEED_SCALE);
  servos.setSmoothRate(SERVO_SMOOTH_RATE);
  servos.goHome();
  setMode(MODE_HOMING);
  gcsLog("[HOME] Moving to HOME slowly");
}

// Leave E-STOP (or anything else) and return to the safe HOME pose
static void doReset() {
  if (!servosOk) { gcsLog("[RESET] Servo config invalid"); return; }
  recorder.stopRecording();
  recorder.stopPlayback();
  gcsLog("[RESET] Leaving E-STOP, returning to safe HOME pose");
  sendFeedbackToGlove(HAP_DOUBLE_RUMBLE);
  startHoming();
}

static void recordFrame(uint32_t now, bool force) {
  if (!recorder.isRecording()) return;
  if (!force && (uint32_t)(now - lastRecordMs) < (uint32_t)RECORD_INTERVAL_MS) return;
  lastRecordMs = now;
  uint16_t a[SERVO_COUNT];
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    long v = lroundf(servos.current(i) * 10.0f);
    if (v < 0) v = 0;
    if (v > 1800) v = 1800;
    a[i] = (uint16_t)v;
  }
  if (!recorder.addFrame(now, a)) {
    gcsLog("[REC] Memory full - recording stopped");
    recorder.stopRecording();
    setMode(MODE_MANUAL);
  }
}

static void startRecording(uint32_t now) {
  recorder.startRecording(now);
  setMode(MODE_RECORD);
  gcsLog("[REC] Recording started");
  recordFrame(now, true);
}

static void stopRecording() {
  recorder.stopRecording();
  gcsLog("[REC] Recording stopped (%u frames, %.1f s)", (unsigned)recorder.frameCount(), recorder.durationMs() / 1000.0f);
  setMode(MODE_MANUAL);
}

static void startPlayback(uint32_t now) {
  if (!recorder.hasRecording()) { gcsLog("[PLAY] Nothing recorded"); return; }
  if (!servos.outputsEnabled()) servos.enableOutputs();
  float first[SERVO_COUNT];
  recorder.firstFrame(first);
  servos.setSpeedScale(PLAY_START_SPEED_SCALE);
  servos.setSmoothRate(SERVO_SMOOTH_RATE);
  servos.setAllTargets(first);
  playPhase = PLAY_MOVE_TO_START;
  playPhaseStartMs = now;
  setMode(MODE_PLAYBACK);
  gcsLog("[PLAY] Playback started (moving to first frame)");
}

static void stopPlayback(bool finished) {
  recorder.stopPlayback();
  if (!finished) servos.hold();
  restoreNormalMotion();
  gcsLog(finished ? "[PLAY] Playback finished" : "[PLAY] Playback stopped");
  setMode(MODE_MANUAL);
}

static void updatePlayback(uint32_t now) {
  if (playPhase == PLAY_MOVE_TO_START) {
    if (servos.atTarget(PLAY_START_TOL_DEG) || (uint32_t)(now - playPhaseStartMs) > (uint32_t)PLAY_START_TIMEOUT_MS) {
      servos.setSpeedScale(1.0f);
      servos.setSmoothRate(PLAYBACK_SMOOTH_RATE);
      recorder.startPlayback(now);
      playPhase = PLAY_RUNNING;
    }
    return;
  }
  float pose[SERVO_COUNT];
  const bool playing = recorder.sample(now, pose);
  servos.setAllTargets(pose);
  if (!playing) stopPlayback(true);
}

// =====================================================================
// "Al-Hawnash Al-Ali" - rock / paper / scissors
//   READY pose -> 3 shakes (UP,DOWN,UP,DOWN,UP,DOWN; tick on every DOWN) ->
//   REVEAL random gesture -> hold -> slow RETURN to HOME -> MANUAL
// =====================================================================
static const float* rpsPoseFor(uint8_t g) {
  switch (g) {
    case RPS_G_ROCK:  return RPS_POSE_ROCK;
    case RPS_G_PAPER: return RPS_POSE_PAPER;
    default:          return RPS_POSE_SCISSORS;
  }
}

static void startRpsGame(uint32_t now) {
  if (!servosOk) { gcsLog("[RPS] Servo config invalid"); return; }
  recorder.stopRecording();
  recorder.stopPlayback();
  if (!servos.outputsEnabled()) servos.enableOutputs();
  servos.setSpeedScale(RPS_SPEED_SCALE);
  servos.setSmoothRate(SERVO_SMOOTH_RATE);
  servos.setAllTargets(RPS_POSE_READY);
  rpsPhase = RPS_PHASE_READY;
  rpsBeat = 0;
  rpsGesture = RPS_G_NONE;
  rpsHalf = -1;
  rpsRound++;
  rpsPhaseStartMs = now;
  setMode(MODE_RPS_GAME);
  gcsLog("[RPS] Round %u - Al-Hawnash is getting ready", (unsigned)rpsRound);
  sendFeedbackToGlove(HAP_SINGLE_PULSE);
}

static void finishRpsGame() {
  restoreNormalMotion();
  gcsLog("[RPS] Round %u finished", (unsigned)rpsRound);
  setMode(MODE_MANUAL);            // also clears rpsPhase
}

// Abort the running game (mid-motion). Either return home slowly or just freeze.
static void stopRpsGame(bool goHome) {
  gcsLog("[RPS] Game stopped");
  if (goHome) {
    startHoming();
  } else {
    servos.hold();
    restoreNormalMotion();
    setMode(MODE_MANUAL);
  }
}

static void updateRpsGame(uint32_t now) {
  const uint32_t t = (uint32_t)(now - rpsPhaseStartMs);
  switch (rpsPhase) {
    case RPS_PHASE_READY:
      if (servos.atTarget(RPS_READY_TOL_DEG) || t > (uint32_t)RPS_READY_TIMEOUT_MS) {
        rpsPhase = RPS_PHASE_SHAKE;
        rpsPhaseStartMs = now;
        rpsHalf = -1;
      }
      break;

    case RPS_PHASE_SHAKE: {
      const int16_t h = (int16_t)(t / (uint32_t)RPS_BEAT_HALF_MS);
      if (h == rpsHalf) break;
      rpsHalf = h;
      if (h >= (int16_t)(2 * RPS_SHAKES)) {
        // ---- REVEAL: snap into a random gesture ----
        rpsGesture = (uint8_t)(1 + (esp_random() % 3));
        servos.setSmoothRate(RPS_SNAP_SMOOTH_RATE);
        servos.setAllTargets(rpsPoseFor(rpsGesture));
        rpsPhase = RPS_PHASE_REVEAL;
        rpsPhaseStartMs = now;
        rpsLastSnipMs = now;
        rpsSnipOpen = false;
        sendFeedbackToGlove(HAP_RPS_REVEAL);
        gcsLog("[RPS] SHOOT! Al-Hawnash plays %s", rpsGestureName(rpsGesture));
      } else if ((h & 1) == 0) {
        servos.setAllTargets(RPS_POSE_UP);
      } else {
        servos.setAllTargets(RPS_POSE_DOWN);
        rpsBeat = (uint8_t)((h + 1) / 2);
        sendFeedbackToGlove(HAP_RPS_BEAT);
        gcsLog("[RPS] %u...", (unsigned)rpsBeat);
      }
      break;
    }

    case RPS_PHASE_REVEAL:
      if (rpsGesture == RPS_G_SCISSORS && t >= (uint32_t)RPS_SNIP_START_MS &&
          (uint32_t)(now - rpsLastSnipMs) >= (uint32_t)RPS_SNIP_PERIOD_MS) {
        rpsLastSnipMs = now;
        rpsSnipOpen = !rpsSnipOpen;
        servos.setTarget(SERVO_GRIPPER, rpsSnipOpen ? RPS_SNIP_B_DEG : RPS_SNIP_A_DEG);
      }
      if (t >= (uint32_t)RPS_REVEAL_HOLD_MS) {
        servos.setSpeedScale(HOME_SPEED_SCALE);
        servos.setSmoothRate(SERVO_SMOOTH_RATE);
        servos.goHome();
        rpsPhase = RPS_PHASE_RETURN;
        rpsPhaseStartMs = now;
      }
      break;

    case RPS_PHASE_RETURN:
      if (servos.atTarget(AT_TARGET_TOL_DEG) || t > (uint32_t)RPS_RETURN_TIMEOUT_MS) finishRpsGame();
      break;

    default:
      break;
  }
}

// =====================================================================
// Commands from the glove
// =====================================================================
static void executeCommand(uint8_t cmd, uint32_t now) {
  if (cmd == CMD_RESET) { doReset(); return; }
  if (cmd == CMD_RPS_GAME) {
    // Allowed from E-STOP: the glove only sends it after a deliberate 2 s hold of BTN2,
    // and decides RESET vs GAME from what the arm was doing before the press.
    if (mode == MODE_RPS_GAME) stopRpsGame(true);
    else                       startRpsGame(now);
    return;
  }
  if (mode == MODE_ESTOP || mode == MODE_WAIT_GLOVE || mode == MODE_HOMING) return;

  switch (cmd) {
    case CMD_RECORD_TOGGLE:
      if (mode == MODE_MANUAL)      startRecording(now);
      else if (mode == MODE_RECORD) stopRecording();
      break;
    case CMD_HOME:
      recorder.stopRecording();
      recorder.stopPlayback();
      gcsLog("[HOME] Command from glove");
      startHoming();
      break;
    case CMD_PLAY_TOGGLE:
      if (mode == MODE_MANUAL)        startPlayback(now);
      else if (mode == MODE_RECORD)   { stopRecording(); startPlayback(now); }
      else if (mode == MODE_PLAYBACK) stopPlayback(false);
      break;
    default:
      break;
  }
}

static void handlePacket(const GlovePacket& p, uint32_t now) {
  glove = p;

  if (needPrime) {
    lastCmdId = p.cmdId;
    estopPrev = false;
    needPrime = false;
  }

  const bool estopFlag = (p.flags & FLAG_ESTOP_HELD) != 0;
  const bool estopEdge = estopFlag && !estopPrev;
  estopPrev = estopFlag;
  const bool cmdNew = (p.cmdId != lastCmdId);
  lastCmdId = p.cmdId;
  if (estopEdge || (cmdNew && p.cmd == CMD_ESTOP)) {
    enterEstop(ESTOP_BUTTON);
    return;
  }

  if (cmdNew) executeCommand(p.cmd, now);

  if (p.selDof != curDof) {
    curDof = p.selDof;
    gcsLog("[DOF] Current DOF: %u/%u  %s", (unsigned)(curDof + 1), (unsigned)DOF_COUNT, DOF_MAP[curDof].name);
  }
}

// =====================================================================
// Link supervision & limit haptics
// =====================================================================
static void superviseLink(uint32_t now) {
  const bool alive = linkAlive(now);
  if (alive && !linkWasUp) {
    linkWasUp = true;
    linkSinceMs = now;
    gcsLog("[OK] Glove connected");
  } else if (!alive && linkWasUp) {
    linkWasUp = false;
    needPrime = true;
    gcsLog("[ERROR] Glove timeout");
    // Dashboard-driven CAMERA_CV does not depend on the glove
    if (mode == MODE_HOMING || mode == MODE_MANUAL || mode == MODE_RECORD ||
        mode == MODE_PLAYBACK || mode == MODE_RPS_GAME) {
      enterEstop(ESTOP_TIMEOUT);
    }
  }
}

// CAMERA_CV needs a live dashboard: with no command/PING the arm freezes in place.
static void superviseGcs(uint32_t now) {
  if (mode == MODE_CAMERA_CV && (uint32_t)(now - lastGcsCmdMs) > (uint32_t)GCS_CMD_TIMEOUT_MS) {
    servos.hold();
    gcsLog("[GCS] Dashboard silent - holding position (MANUAL)");
    setMode(MODE_MANUAL);
  }
}

static void checkServoLimitsAndBuzz(uint32_t now) {
  const uint8_t mask = servos.getLimitMask();
  const uint8_t newlyHit = mask & (uint8_t)(~prevLimitMask);
  prevLimitMask = mask;
  if (newlyHit != 0 && (uint32_t)(now - lastLimitBuzzMs) >= 400) {
    lastLimitBuzzMs = now;
    sendFeedbackToGlove(HAP_DOUBLE_RUMBLE);
  }
}

// =====================================================================
// Manual control & control tick
// =====================================================================
static void manualControl(uint32_t now, float dt) {
  if ((uint32_t)(now - linkLastMs) > (uint32_t)GLOVE_STALE_MS) return;
  if (glove.flags & FLAG_ESTOP_HELD) return;

  if ((glove.flags & FLAG_CTRL_VALID) && glove.selDof < DOF_COUNT) {
    const DofMap& m = DOF_MAP[glove.selDof];
    if (m.rollServo < SERVO_COUNT && glove.rollCmd != 0) {
      servos.setTarget(m.rollServo, servos.target(m.rollServo) +
                       m.rollGain * ((float)glove.rollCmd / 100.0f) * MANUAL_MAX_RATE_DPS * dt);
    }
    if (m.pitchServo < SERVO_COUNT && glove.pitchCmd != 0) {
      servos.setTarget(m.pitchServo, servos.target(m.pitchServo) +
                       m.pitchGain * ((float)glove.pitchCmd / 100.0f) * MANUAL_MAX_RATE_DPS * dt);
    }
  }
  if (glove.flags & FLAG_FLEX_OK) {
    servos.setTarget(SERVO_GRIPPER, GRIPPER_OPEN_ANGLE +
                     ((float)glove.gripperPct / 100.0f) * (GRIPPER_CLOSED_ANGLE - GRIPPER_OPEN_ANGLE));
  }
}

static void runControlTick(uint32_t now) {
  const uint32_t elapsed = (uint32_t)(now - lastCtrlMs);
  if (elapsed < (uint32_t)CONTROL_INTERVAL_MS) return;
  lastCtrlMs = now;
  float dt = (float)elapsed * 0.001f;
  if (dt > 0.1f) dt = 0.1f;

  switch (mode) {
    case MODE_HOMING:                                   break;
    case MODE_MANUAL:
    case MODE_RECORD:    manualControl(now, dt);        break;
    case MODE_PLAYBACK:  updatePlayback(now);           break;
    case MODE_CAMERA_CV:                                break;   // targets are set by dashboard commands
    case MODE_RPS_GAME:  updateRpsGame(now);            break;
    default:             return;                                  // WAIT_GLOVE / ESTOP: outputs hold
  }

  servos.update(dt);
  checkServoLimitsAndBuzz(now);

  if (mode == MODE_HOMING && servos.atTarget(AT_TARGET_TOL_DEG)) {
    restoreNormalMotion();
    gcsLog("[HOME] Home reached");
    setMode(MODE_MANUAL);
  } else if (mode == MODE_RECORD) {
    recordFrame(now, false);
  }
}

// =====================================================================
// Ground Control Station protocol (WebSocket + optional USB serial)
// =====================================================================
static bool jsonStr(const char* s, const char* key, char* out, size_t n) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(s, pat);
  if (!p) return false;
  p += strlen(pat);
  while (*p == ' ' || *p == ':') p++;
  if (*p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i < n - 1) out[i++] = *p++;
  out[i] = '\0';
  return true;
}

static bool jsonNum(const char* s, const char* key, float& v) {
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(s, pat);
  if (!p) return false;
  p += strlen(pat);
  while (*p == ' ' || *p == ':') p++;
  if (!((*p >= '0' && *p <= '9') || *p == '-' || *p == '+' || *p == '.')) return false;
  v = (float)atof(p);
  return true;
}

// Switch to a dashboard-controlled mode (MANUAL / CAMERA_CV) without a position jump
static void gcsEnterMode(RobotMode m) {
  if (mode == MODE_ESTOP) { gcsLog("[GCS] Mode change refused - E-STOP active (send RESET)"); return; }
  if (!servosOk) { gcsLog("[GCS] Servo config invalid"); return; }
  recorder.stopRecording();
  recorder.stopPlayback();
  if (!servos.outputsEnabled()) servos.enableOutputs();
  servos.hold();
  restoreNormalMotion();
  setMode(m);
  sendFeedbackToGlove(HAP_SINGLE_PULSE);
}

static void handleGcsLine(const char* line, uint32_t now) {
  if (!line || line[0] != '{') return;
  char cmd[16];
  if (!jsonStr(line, "cmd", cmd, sizeof(cmd))) return;
  lastGcsCmdMs = now;

  if (!strcmp(cmd, "PING")) return;
  if (!strcmp(cmd, "ESTOP"))  { enterEstop(ESTOP_GCS); return; }
  if (!strcmp(cmd, "RESET"))  { doReset(); return; }
  if (!strcmp(cmd, "HOME"))   {
    if (mode == MODE_ESTOP) { gcsLog("[GCS] HOME refused - E-STOP active (send RESET)"); return; }
    recorder.stopRecording();
    recorder.stopPlayback();
    startHoming();
    return;
  }

  if (!strcmp(cmd, "RPS_START")) {
    if (mode == MODE_ESTOP) { gcsLog("[RPS] Refused - E-STOP active (send RESET first)"); return; }
    startRpsGame(now);
    return;
  }
  if (!strcmp(cmd, "RPS_STOP")) {
    if (mode == MODE_RPS_GAME) stopRpsGame(true);
    return;
  }

  if (!strcmp(cmd, "MODE")) {
    char val[16];
    if (!jsonStr(line, "val", val, sizeof(val))) return;
    if (!strcmp(val, "CAMERA_CV"))      gcsEnterMode(MODE_CAMERA_CV);
    else if (!strcmp(val, "MANUAL"))    gcsEnterMode(MODE_MANUAL);
    else if (!strcmp(val, "RPS_GAME")) {
      if (mode == MODE_ESTOP) gcsLog("[RPS] Refused - E-STOP active (send RESET first)");
      else                    startRpsGame(now);
    }
    else if (!strcmp(val, "RECORD")) {
      if (mode == MODE_MANUAL || mode == MODE_CAMERA_CV) startRecording(now);
      else if (mode == MODE_RECORD)                      stopRecording();
    }
    else if (!strcmp(val, "PLAYBACK")) {
      if (mode == MODE_PLAYBACK)       stopPlayback(false);
      else if (mode != MODE_ESTOP)     startPlayback(now);
    }
    return;
  }

  if (!strcmp(cmd, "SERVO")) {
    if (mode == MODE_ESTOP || mode == MODE_RPS_GAME) return;
    float id = -1.0f, ang = 0.0f;
    if (!jsonNum(line, "id", id) || !jsonNum(line, "angle", ang)) return;
    if (id < 0.0f || id >= (float)SERVO_COUNT || !servosOk) return;
    if (!servos.outputsEnabled()) servos.enableOutputs();
    if (mode == MODE_WAIT_GLOVE) { restoreNormalMotion(); setMode(MODE_CAMERA_CV); }
    servos.setTarget((uint8_t)id, ang);
    return;
  }

  if (!strcmp(cmd, "ANGLES")) {
    if (mode == MODE_ESTOP || mode == MODE_RPS_GAME || !servosOk) return;
    const char* val = strstr(line, "\"val\"");
    const char* arr = val ? strchr(val, '[') : nullptr;
    if (!arr) return;
    if (!servos.outputsEnabled()) servos.enableOutputs();
    if (mode != MODE_CAMERA_CV && mode != MODE_RECORD) {
      restoreNormalMotion();
      setMode(MODE_CAMERA_CV);
    }
    arr++;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      servos.setTarget(i, (float)atof(arr));
      const char* comma = strchr(arr, ',');
      if (!comma) break;
      arr = comma + 1;
    }
    return;
  }
}

static void processSerialCommands(uint32_t now) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        handleGcsLine(serialBuf, now);
        serialIdx = 0;
      }
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

static void sendGcsTelemetry(uint32_t now) {
  if ((uint32_t)(now - lastGcsMs) < (uint32_t)GCS_TELEMETRY_INTERVAL_MS) return;
  lastGcsMs = now;
  wsClientCount = wsServer.connectedClients();
  if (wsClientCount <= 0 && !GCS_SERIAL_TELEMETRY) return;

  const bool alive = linkAlive(now);
  const uint8_t dofIdx = (curDof < DOF_COUNT) ? curDof : 0;

  char buf[720];
  const int n = snprintf(buf, sizeof(buf),
      "{\"type\":\"telemetry\",\"mode\":\"%s\",\"link\":%s,\"rssi\":%d,\"batt\":%u,\"dof\":%u,\"dofName\":\"%s\","
      "\"angles\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
      "\"targets\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
      "\"limits\":%u,\"pitch\":%.1f,\"roll\":%.1f,\"grip\":%u,\"recFrames\":%u,"
      "\"rpsPhase\":%u,\"rpsBeat\":%u,\"rpsGesture\":%u,\"rpsRound\":%u,"
      "\"ip\":\"%s\",\"ap\":%s}",
      modeName(mode),
      alive ? "true" : "false",
      alive ? (int)lastRssi : -100,
      (unsigned)glove.batteryPct,
      (unsigned)dofIdx,
      DOF_MAP[dofIdx].name,
      servos.current(0), servos.current(1), servos.current(2),
      servos.current(3), servos.current(4), servos.current(5),
      servos.target(0), servos.target(1), servos.target(2),
      servos.target(3), servos.target(4), servos.target(5),
      (unsigned)servos.getLimitMask(),
      (float)glove.pitchX10 * 0.1f,
      (float)glove.rollX10 * 0.1f,
      (unsigned)glove.gripperPct,
      (unsigned)recorder.frameCount(),
      (unsigned)rpsPhase, (unsigned)rpsBeat, (unsigned)rpsGesture, (unsigned)rpsRound,
      netIp, netIsAp ? "true" : "false");
  if (n <= 0 || n >= (int)sizeof(buf)) return;

  if (wsClientCount > 0) wsServer.broadcastTXT(buf, (size_t)n);
#if GCS_SERIAL_TELEMETRY
  Serial.println(buf);
#endif
}

// =====================================================================
// Wi-Fi + HTTP + WebSocket
// =====================================================================
static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsClientCount = wsServer.connectedClients();
      lastGcsCmdMs = millis();
      gcsLog("[GCS] Dashboard connected (%d client(s))", wsClientCount);
      break;
    case WStype_DISCONNECTED:
      wsClientCount = wsServer.connectedClients();
      gcsLog("[GCS] Dashboard disconnected (%d client(s))", wsClientCount);
      break;
    case WStype_TEXT:
      if (payload && length > 0 && length < sizeof(wsCmdBuf)) {
        memcpy(wsCmdBuf, payload, length);
        wsCmdBuf[length] = '\0';
        handleGcsLine(wsCmdBuf, millis());
      }
      break;
    default:
      break;
  }
  (void)num;
}

static void handleHttpRoot() {
  char page[640];
  snprintf(page, sizeof(page),
    "<!doctype html><html><head><meta charset=\"utf-8\"><title>Al-Hawnash Al-Ali</title></head>"
    "<body style=\"font-family:monospace;background:#04070d;color:#00f3ff;padding:24px\">"
    "<h2>الحونش الآلي // AL-HAWNASH AL-ALI</h2>"
    "<p>Arm controller online. Mode: %s</p>"
    "<p>Open <b>index.html</b> (Ground Control Station) on your laptop and connect to:</p>"
    "<p><b>ws://%s:%d/</b> &nbsp; (or %s.local)</p>"
    "<p>Network: %s</p></body></html>",
    modeName(mode), netIp, GCS_WS_PORT, WIFI_HOSTNAME, netIsAp ? "access point " WIFI_AP_SSID : "router (STA)");
  httpServer.send(200, "text/html; charset=utf-8", page);
}

static void handleHttpStatus() {
  char js[200];
  snprintf(js, sizeof(js), "{\"mode\":\"%s\",\"ip\":\"%s\",\"ap\":%s,\"wsClients\":%d,\"link\":%s}",
           modeName(mode), netIp, netIsAp ? "true" : "false", wsClientCount,
           linkAlive(millis()) ? "true" : "false");
  httpServer.send(200, "application/json", js);
}

// One-time, boot-time network bring-up (blocking is fine here: servos are still disabled).
static void startNetwork() {
  WiFi.persistent(false);
  WiFi.setHostname(WIFI_HOSTNAME);

  bool staOk = false;
  if (strlen(WIFI_STA_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    gcsLog("[NET] Joining router \"%s\" ...", WIFI_STA_SSID);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - t0) < (uint32_t)WIFI_STA_TIMEOUT_MS) delay(100);
    staOk = (WiFi.status() == WL_CONNECTED);
  }

  if (staOk) {
    netIsAp = false;
    WiFi.localIP().toString().toCharArray(netIp, sizeof(netIp));
    gcsLog("[NET] Router joined: IP %s, channel %d", netIp, (int)WiFi.channel());
  } else {
    if (strlen(WIFI_STA_SSID) > 0) gcsLog("[NET] Router not reachable - starting access point");
    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_AP_STA);
    delay(200);                                   // let the radio settle (power spike)
    bool apOk = false;
    for (int i = 0; i < 3 && !apOk; i++) {
      apOk = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, ESPNOW_CHANNEL);
      if (!apOk) { gcsLog("[NET] softAP failed, retry %d", i + 1); delay(300); }
    }
    netIsAp = true;
    WiFi.softAPIP().toString().toCharArray(netIp, sizeof(netIp));
    gcsLog("[NET] Access point \"%s\" %s: IP %s, channel %d", WIFI_AP_SSID, apOk ? "ready" : "FAILED", netIp, ESPNOW_CHANNEL);
  }

  if (MDNS.begin(WIFI_HOSTNAME)) {
    MDNS.addService("http", "tcp", GCS_HTTP_PORT);
    MDNS.addService("ws", "tcp", GCS_WS_PORT);
    gcsLog("[NET] mDNS: %s.local", WIFI_HOSTNAME);
  }

  httpServer.on("/", handleHttpRoot);
  httpServer.on("/status", handleHttpStatus);
  httpServer.onNotFound([]() { httpServer.send(404, "text/plain", "Not found"); });
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  wsServer.enableHeartbeat(15000, 3000, 2);       // drop dead dashboards
  gcsLog("[NET] WebSocket server: ws://%s:%d/", netIp, GCS_WS_PORT);
}

// =====================================================================
// Boot / wait for glove / LED / status
// =====================================================================
static void waitForGlove(uint32_t now) {
  if (!servosOk || !linkAlive(now)) return;
  if ((uint32_t)(now - linkSinceMs) < (uint32_t)START_CONNECT_MS) return;
  if (!(glove.flags & FLAG_CTRL_VALID)) {
    static uint32_t lastMsg = 0;
    if ((uint32_t)(now - lastMsg) >= 2000) {
      lastMsg = now;
      gcsLog((glove.flags & FLAG_IMU_OK)
        ? "[WAIT] Glove linked, waiting for its neutral pose (hold the hand still ~1.5 s)"
        : "[WAIT] Glove linked, but its MPU6050 is NOT ready (check MPU wiring / keep glove still)");
    }
    return;
  }
  gcsLog("[START] Glove ready - starting arm");
  startHoming();
}

static void updateLed(uint32_t now) {
#if ARM_LED_ENABLE
  bool on;
  switch (mode) {
    case MODE_WAIT_GLOVE: on = ((now / 500) & 1); break;
    case MODE_ESTOP:      on = ((now / 100) & 1); break;
    case MODE_RECORD:     on = ((now / 250) & 1); break;
    case MODE_PLAYBACK:   on = ((now % 1000) < 800); break;
    case MODE_RPS_GAME:   on = ((now / 150) & 1); break;
    default:              on = true; break;
  }
  digitalWrite(ARM_PIN_LED, on ? HIGH : LOW);
#else
  (void)now;
#endif
}

static void printStatus(uint32_t now) {
  if ((uint32_t)(now - lastStatusMs) < (uint32_t)STATUS_PRINT_INTERVAL_MS) return;
  lastStatusMs = now;
  Serial.printf("[STAT] %s link=%s age=%ums dof=%s batt=%u%% good=%lu bad=%lu drop=%lu frames=%u ws=%d ip=%s\n",
                modeName(mode), linkAlive(now) ? "OK" : "LOST",
                linkHasPacket ? (unsigned)(now - linkLastMs) : 0u,
                curDof < DOF_COUNT ? DOF_MAP[curDof].name : "-",
                (unsigned)glove.batteryPct,
                (unsigned long)rxGood, (unsigned long)rxBad,
                (unsigned long)rxRejected, (unsigned)recorder.frameCount(),
                wsClientCount, netIp);
}

// =====================================================================
// setup / loop
// =====================================================================
void setup() {
#if ARM_LED_ENABLE
  pinMode(ARM_PIN_LED, OUTPUT);                  // boot indicator: 3 quick blinks = the code is running
  for (uint8_t i = 0; i < 3; i++) { digitalWrite(ARM_PIN_LED, HIGH); delay(120); digitalWrite(ARM_PIN_LED, LOW); delay(120); }
#endif
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] Al-Hawnash Al-Ali arm starting (ESP32-WROOM-32 + Wi-Fi/WebSocket GCS bridge)");
#if ARM_LED_ENABLE
  pinMode(ARM_PIN_LED, OUTPUT);
#endif
  servosOk = servos.begin();
  Serial.println(servosOk ? "[OK] Servo outputs ready (disabled until glove or GCS connects)"
                          : "[ERROR] Servo config invalid - arm will not start");

  startNetwork();                        // Wi-Fi first: ESP-NOW then shares its radio channel

  nowOk = initEspNow();
  Serial.println(nowOk ? "[OK] ESP-NOW initialized" : "[ERROR] ESP-NOW init failed");
  Serial.print("[INFO] My MAC: ");
  Serial.println(WiFi.macAddress());
  if (macIsBroadcast(GLOVE_MAC)) Serial.println("[WARN] GLOVE_MAC not set in config.h -> accepting any sender");

  lastCtrlMs = millis();
  Serial.println("[WAIT] Waiting for glove or GCS command...");
}

void loop() {
  const uint32_t now = millis();

  GlovePacket p;
  if (takeRx(p)) {
    if (acceptPacket(p, now)) handlePacket(p, now);
    else rxRejected++;
  }
  superviseLink(now);
  processSerialCommands(now);
  superviseGcs(now);

  if (mode == MODE_WAIT_GLOVE) waitForGlove(now);
  runControlTick(now);
  sendHeartbeat(now);
  updateLed(now);
  sendGcsTelemetry(now);

  httpServer.handleClient();
  wsServer.loop();
  printStatus(now);

  if (!nowOk) {
    static uint32_t lastTry = 0;
    if ((uint32_t)(now - lastTry) >= 2000) {
      lastTry = now;
      nowOk = initEspNow();
      if (nowOk) gcsLog("[OK] ESP-NOW initialized");
    }
  }
}
