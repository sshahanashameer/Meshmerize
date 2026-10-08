#include "control/drive.h"
#include <math.h>
#include "utils/logger.h"

static const char* TAG = "drive";
static const float RAD2DEG = 57.2957795f;

static float wrapDeg180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

const char* eventName(Event e) {
  switch (e) {
    case Event::NONE: return "NONE";
    case Event::JUNCTION: return "JUNCTION";
    case Event::DEAD_END: return "DEAD_END";
    case Event::FINISH: return "FINISH";
    case Event::DISTANCE: return "DISTANCE";
    case Event::TIMEOUT: return "TIMEOUT";
    case Event::ABORT: return "ABORT";
  }
  return "?";
}

Drive::Drive(QTR8RC& q, Encoders& e, BNO055& i, MotorDriver& m, EKF& k)
    : _qtr(q), _enc(e), _imu(i), _motors(m), _ekf(k) {
  _linePid.setGains(cfg::LINE_KP, cfg::LINE_KI, cfg::LINE_KD);
  _linePid.setOutputLimits(-cfg::LINE_MAX_CORR_MMPS, cfg::LINE_MAX_CORR_MMPS);
  _linePid.setDerivativeFilter(0.5f);
  _headPid.setGains(cfg::HEAD_KP, cfg::HEAD_KI, cfg::HEAD_KD);
  _headPid.setOutputLimits(-cfg::HEAD_MAX_CORR_MMPS, cfg::HEAD_MAX_CORR_MMPS);
  _turnPid.setGains(cfg::TURN_KP, cfg::TURN_KI, cfg::TURN_KD);
  _turnPid.setOutputLimits(-cfg::TURN_MAX_SPEED_MMPS, cfg::TURN_MAX_SPEED_MMPS);
  _spdL.setGains(cfg::SPEED_KP, cfg::SPEED_KI, cfg::SPEED_KD);
  _spdR.setGains(cfg::SPEED_KP, cfg::SPEED_KI, cfg::SPEED_KD);
  _spdL.setOutputLimits(-0.5f, 0.5f);
  _spdR.setOutputLimits(-0.5f, 0.5f);
  _spdL.setIntegralLimit(0.4f);
  _spdR.setIntegralLimit(0.4f);
}

// ============================ SENSOR TASK (single-core compatible) ============================
bool Drive::start(bool imuOk) {
  _imuOk = imuOk;
  _sampleSem = xSemaphoreCreateBinary();
  if (_sampleSem == nullptr) return false;
  const BaseType_t ok = xTaskCreate(&Drive::sensorTaskThunk, "sensors", 4096, this,
                                    cfg::SENSOR_TASK_PRIO, &_sensorTask);
  if (ok != pdPASS) { vSemaphoreDelete(_sampleSem); _sampleSem = nullptr; return false; }
  return true;
}

void Drive::sensorTaskThunk(void* arg) { static_cast<Drive*>(arg)->sensorLoop(); }

void Drive::sensorLoop() {
  const TickType_t period = pdMS_TO_TICKS(cfg::SENSOR_PERIOD_MS);
  float headingOffset = 0;
  if (_imuOk) {
    _imu.update();
    headingOffset = _imu.headingRad();
  }
  _ekf.reset(0, 0, 0);
  uint32_t lastUs = micros();
  uint32_t count = 0;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);
    const uint32_t now = micros();
    float dt = (now - lastUs) * 1e-6f;
    lastUs = now;
    if (dt < 1e-4f) dt = cfg::SENSOR_PERIOD_MS * 1e-3f;

    _qtr.read();
    _enc.update(dt);
    float dL, dR;
    _enc.takeDeltaMm(dL, dR);

    bool imuNew = false;
    if (_imuOk && (count % cfg::IMU_DIVIDER) == 0) {
      imuNew = _imu.update();
      if (!imuNew) _imuOk = false; // fail closed: no stale fused heading after I2C failure
    }
    count++;

    const uint8_t cmd = _poseCmd;
    if (cmd) {
      if (cmd & CMD_ZERO_HEADING) {
        headingOffset = _imuOk ? _imu.headingRad() : 0.0f;
        _ekf.reset(0, 0, 0);
      }
      if (cmd & CMD_SET_POS) _ekf.setPosition(_cmdX, _cmdY);
      _poseCmd = 0;
    }

    // Prediction: encoder distance + heading change (gyro, or encoders as fallback).
    // Gyro Z is CCW-positive, our heading is CW-positive -> minus sign.
    const float ds = 0.5f * (dL + dR);
    const float dth = _imuOk ? -_imu.gyroZRadPerSec() * dt : (dL - dR) / cfg::WHEEL_BASE_MM;
    _ekf.predict(ds, dth);
    if (imuNew) _ekf.updateHeading(EKF::wrapPi(_imu.headingRad() - headingOffset));

    RobotState s;
    s.seq = count;
    s.tUs = now;
    const uint16_t* v = _qtr.values();
    for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) s.sensor[i] = v[i];
    s.linePos = _qtr.position();
    s.darkMask = _qtr.darkMask();
    s.darkCount = _qtr.darkCount();
    s.lineFound = _qtr.lineDetected();
    s.distL = _enc.distanceL();
    s.distR = _enc.distanceR();
    s.speedL = _enc.speedL();
    s.speedR = _enc.speedR();
    s.yawRate = _imuOk ? -_imu.gyroZRadPerSec() : (_enc.speedL() - _enc.speedR()) / cfg::WHEEL_BASE_MM;
    s.x = _ekf.x();
    s.y = _ekf.y();
    s.theta = _ekf.theta();
    s.imuCal = _imuOk ? _imu.calibStatus() : 0;
    s.imuOk = _imuOk;

    portENTER_CRITICAL(&_mux);
    _state = s;
    portEXIT_CRITICAL(&_mux);
    xSemaphoreGive(_sampleSem);
  }
}

RobotState Drive::state() const {
  RobotState s;
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  s = _state;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  return s;
}

// ============================ MISSION-SIDE HELPERS ============================
bool Drive::tick(RobotState& s, float& dt) {
  if (_abort) { _motors.brake(); return false; }
  if (xSemaphoreTake(_sampleSem, pdMS_TO_TICKS(50)) != pdTRUE) {
    LOGE(TAG, "sensor task stalled");
    return false;
  }
  s = state();
  dt = (s.tUs - _lastTUs) * 1e-6f;
  _lastTUs = s.tUs;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  return true;
}

void Drive::waitPoseCmd() {
  const uint32_t t0 = millis();
  while (_poseCmd && millis() - t0 < 200) vTaskDelay(1);
  RobotState s;
  float dt;
  tick(s, dt);   // two fresh samples so state() reflects the new pose
  tick(s, dt);
}

void Drive::zeroPose() {
  _cmdX = 0;
  _cmdY = 0;
  _poseCmd = CMD_ZERO_HEADING | CMD_SET_POS;
  waitPoseCmd();
}

void Drive::setPosition(float x, float y) {
  _cmdX = x;
  _cmdY = y;
  _poseCmd = CMD_SET_POS;
  waitPoseCmd();
}

float Drive::approach(float cur, float target, float dt) const {
  if (target > cur) return fminf(target, cur + cfg::ACCEL_MMPS2 * dt);
  return fmaxf(target, cur - cfg::DECEL_MMPS2 * dt);
}

float Drive::headingCorrection(const RobotState& s, float targetDeg, float dt) {
  const float err = wrapDeg180(targetDeg - s.theta * RAD2DEG);
  return _headPid.updateWithRate(err, -s.yawRate * RAD2DEG, dt);
}

// Inner wheel-speed loop: feed-forward + PI on encoder speed.
void Drive::commandWheels(float vL, float vR, const RobotState& s, float dt) {
  if (_abort) { _motors.brake(); return; }
  const float uL = vL * cfg::SPEED_FF + _spdL.update(vL - s.speedL, dt);
  const float uR = vR * cfg::SPEED_FF + _spdR.update(vR - s.speedR, dt);
  _motors.setSpeeds(uL, uR);
}

void Drive::stop() {
  _motors.brake();
  _vBase = 0;
  _spdL.reset();
  _spdR.reset();
}

bool Drive::waitStopped() {
  RobotState s;
  float dt;
  _vBase = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < cfg::STOP_TIMEOUT_MS) {
    if (!tick(s, dt)) return false;
    commandWheels(0, 0, s, dt);
    if (fabsf(s.speedL) < 20.0f && fabsf(s.speedR) < 20.0f) break;
  }
  return true;
}

// ================================ TURNING =====================================
bool Drive::turnToHeading(float targetDeg) {
  RobotState s;
  float dt;
  if (!waitStopped()) {
    stop();
    return false;
  }
  _turnPid.reset();
  _spdL.reset();
  _spdR.reset();
  const uint32_t t0 = millis();
  float settledMs = 0;
  for (;;) {
    if (!tick(s, dt)) {
      stop();
      return false;
    }
    const float err = wrapDeg180(targetDeg - s.theta * RAD2DEG);   // + = turn clockwise
    const float rate = s.yawRate * RAD2DEG;
    float u = _turnPid.updateWithRate(err, -rate, dt);
    if (fabsf(err) > cfg::TURN_TOL_DEG && fabsf(u) < cfg::TURN_MIN_SPEED_MMPS)
      u = copysignf(cfg::TURN_MIN_SPEED_MMPS, err);
    commandWheels(u, -u, s, dt);

    if (fabsf(err) < cfg::TURN_TOL_DEG && fabsf(rate) < 20.0f) settledMs += dt * 1000.0f;
    else settledMs = 0;
    if (settledMs >= cfg::TURN_SETTLE_MS) {
      stop();
      return true;
    }
    if (millis() - t0 > cfg::TURN_TIMEOUT_MS) {
      LOGW(TAG, "turn timeout, err=%.1f deg", err);
      stop();
      return false;
    }
  }
}

// ============================ LINE FOLLOWING ==================================
static bool isJunctionMask(uint8_t m) {
  const bool center = (m & 0x18) != 0;            // sensors 3,4
  const bool left = (m & 0x01) != 0;              // sensor 0
  const bool right = (m & 0x80) != 0;             // sensor 7
  return (center && (left || right)) || __builtin_popcount(m) >= 7;
}

Event Drive::followToEvent(const FollowParams& p, JunctionInfo& info) {
  info = JunctionInfo();
  RobotState s;
  float dt;
  if (!tick(s, dt)) return _abort ? Event::ABORT : Event::TIMEOUT;
  const float d0 = travel(s);
  float lostStart = -1.0f;
  const uint32_t t0 = millis();
  _linePid.reset();
  _headPid.reset();

  for (;;) {
    if (!tick(s, dt)) {
      stop();
      return _abort ? Event::ABORT : Event::TIMEOUT;
    }
    const float trav = travel(s) - d0;
    if (millis() - t0 > cfg::EDGE_TIMEOUT_MS) {
      stop();
      return Event::TIMEOUT;
    }

    float rem = 1e9f;
    if (p.expectedDist > 0) {
      rem = p.expectedDist - trav;
      if (p.stopAtDistance && rem <= 0) {
        stop();
        return Event::DISTANCE;
      }
      if (!p.stopAtDistance && rem < -cfg::EDGE_OVERSHOOT_MM) {
        stop();
        return Event::TIMEOUT;
      }
    }

    if (p.detectEvents && isJunctionMask(s.darkMask)) return scanJunction(p, s, info);

    // Speed profile: brake so that we arrive at expectedDist with slowSpeed.
    float v = p.speed;
    if (rem < 1e8f) {
      const float vmax = sqrtf(p.slowSpeed * p.slowSpeed + 2.0f * cfg::DECEL_MMPS2 * fmaxf(rem, 0.0f));
      v = fminf(v, vmax);
    }
    v = fmaxf(v, fminf(p.speed, cfg::MIN_CRAWL_MMPS));
    _vBase = approach(_vBase, v, dt);

    float corr;
    if (s.lineFound) {
      lostStart = -1.0f;
      corr = _linePid.update((float)s.linePos - 3500.0f, dt);   // + = line on the right
    } else {
      corr = headingCorrection(s, p.headingDeg, dt);            // coast straight on the IMU
      if (p.detectEvents) {
        if (lostStart < 0) lostStart = trav;
        else if (trav - lostStart >= cfg::LOST_LINE_MM) {
          stop();
          return Event::DEAD_END;
        }
      }
    }
    commandWheels(_vBase + corr, _vBase - corr, s, dt);
  }
}

// Drive straight (IMU heading hold) until the wheel axle is over the junction centre,
// recording which exits the sensor array crossed on the way.
Event Drive::scanJunction(const FollowParams& p, const RobotState& s0, JunctionInfo& info) {
  RobotState s;
  float dt;
  const float start = travel(s0);
  bool L = (s0.darkMask & 0x03) != 0, R = (s0.darkMask & 0xC0) != 0;
  bool straight = false;
  float darkStart = (s0.darkCount >= 7) ? 0.0f : -1.0f, maxDark = 0.0f, adv = 0.0f;
  const float target = cfg::SENSOR_TO_AXLE_MM + cfg::LINE_HALF_WIDTH_MM;
  _headPid.reset();

  for (;;) {
    if (!tick(s, dt)) {
      stop();
      return _abort ? Event::ABORT : Event::TIMEOUT;
    }
    adv = travel(s) - start;
    _vBase = approach(_vBase, p.scanSpeed, dt);
    const float corr = headingCorrection(s, p.headingDeg, dt);
    commandWheels(_vBase + corr, _vBase - corr, s, dt);

    if (s.darkMask & 0x03) L = true;
    if (s.darkMask & 0xC0) R = true;
    const bool allDark = s.darkCount >= 7;
    if (allDark) {
      if (darkStart < 0) darkStart = adv;
      maxDark = fmaxf(maxDark, adv - darkStart);
    } else {
      darkStart = -1.0f;
    }
    straight = (s.darkMask & 0x18) != 0;

    if (adv >= target && !allDark) break;
    if (adv >= target + cfg::FINISH_MAX_MM) break;
  }
  info.left = L;
  info.right = R;
  info.straight = straight;
  info.advance = adv;
  info.finish = maxDark >= cfg::FINISH_ALLDARK_MM;
  if (info.finish) {
    stop();
    return Event::FINISH;
  }
  return Event::JUNCTION;
}

// =============================== CALIBRATION ==================================
bool Drive::calibrateSensors() {
  RobotState s;
  float dt;
  if (!tick(s, dt)) return false;
  const float h0 = s.theta * RAD2DEG;
  _qtr.resetCalibration();
  _qtr.startCalibration();
  const bool ok = turnToHeading(h0 - cfg::CAL_SWEEP_DEG) && turnToHeading(h0 + cfg::CAL_SWEEP_DEG) &&
                  turnToHeading(h0);
  _qtr.stopCalibration();
  stop();
  if (!_qtr.isCalibrated()) {
    LOGW(TAG, "calibration incomplete: place the array over the line and retry");
    return false;
  }
  _qtr.saveCalibration();
  LOGI(TAG, "QTR calibrated and stored");
  return ok;
}
