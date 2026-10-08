#include "sensors/qtr8rc.h"
#include <Arduino.h>
#include <Preferences.h>

void QTR8RC::begin() {
  for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
    _min[i] = cfg::QTR_TIMEOUT_US;
    _max[i] = 0;
    pinMode(cfg::QTR_PINS[i], INPUT);
  }
  if (cfg::QTR_EMITTER_PIN >= 0) {
    pinMode(cfg::QTR_EMITTER_PIN, OUTPUT);
    digitalWrite(cfg::QTR_EMITTER_PIN, HIGH);
  }
}

void QTR8RC::readRaw(uint16_t* out) {
  // 1) charge every sensor capacitor
  for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
    out[i] = cfg::QTR_TIMEOUT_US;
    pinMode(cfg::QTR_PINS[i], OUTPUT);
    digitalWrite(cfg::QTR_PINS[i], HIGH);
  }
  delayMicroseconds(10);
  // 2) release the lines and time the decay (dark surface = slow decay = large value)
  for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) pinMode(cfg::QTR_PINS[i], INPUT);
  uint8_t pending = (1u << cfg::QTR_COUNT) - 1u;
  const uint32_t start = micros();
  while (pending) {
    const uint32_t t = micros() - start;
    if (t >= cfg::QTR_TIMEOUT_US) break;
    for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
      if ((pending & (1u << i)) && !digitalRead(cfg::QTR_PINS[i])) {
        out[i] = (uint16_t)t;
        pending &= ~(1u << i);
      }
    }
  }
}

void QTR8RC::resetCalibration() { _resetReq = true; }

void QTR8RC::read() {
  readRaw(_raw);
  if (_resetReq) {
    for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
      _min[i] = cfg::QTR_TIMEOUT_US;
      _max[i] = 0;
    }
    _resetReq = false;
  }
  if (_calibrating) {
    for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
      if (_raw[i] < _min[i]) _min[i] = _raw[i];
      if (_raw[i] > _max[i]) _max[i] = _raw[i];
    }
  }

  uint32_t sum = 0, wsum = 0;
  _mask = 0;
  _darkCount = 0;
  for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
    int32_t lo = _min[i], hi = _max[i];
    if (hi - lo < (int32_t)cfg::QTR_MIN_RANGE_US) {   // uncalibrated: use the full range
      lo = 0;
      hi = cfg::QTR_TIMEOUT_US;
    }
    int32_t v = ((int32_t)_raw[i] - lo) * 1000 / (hi - lo);
    if (v < 0) v = 0;
    if (v > 1000) v = 1000;
    if (!cfg::QTR_LINE_IS_BLACK) v = 1000 - v;
    _val[i] = (uint16_t)v;
    if (v > (int32_t)cfg::QTR_DARK_THRESHOLD) {
      _mask |= (1u << i);
      _darkCount++;
    }
    if (v > 50) {                                     // ignore noise floor in the average
      sum += (uint32_t)v;
      wsum += (uint32_t)v * i * 1000u;
    }
  }
  _lineFound = sum > cfg::QTR_LINE_MIN_SUM;
  if (_lineFound) {
    _pos = (uint16_t)(wsum / sum);
  } else {
    _pos = (_pos < 3500) ? 0 : 7000;                  // saturate towards where the line was lost
  }
}

bool QTR8RC::isCalibrated() const {
  for (uint8_t i = 0; i < cfg::QTR_COUNT; i++)
    if ((int32_t)_max[i] - (int32_t)_min[i] < (int32_t)cfg::QTR_MIN_RANGE_US) return false;
  return true;
}

bool QTR8RC::saveCalibration() {
  Preferences p;
  if (!p.begin("qtr", false)) return false;
  size_t a = p.putBytes("min", _min, sizeof(_min));
  size_t b = p.putBytes("max", _max, sizeof(_max));
  p.end();
  return a == sizeof(_min) && b == sizeof(_max);
}

bool QTR8RC::loadCalibration() {
  Preferences p;
  if (!p.begin("qtr", true)) return false;
  uint16_t mn[cfg::QTR_COUNT], mx[cfg::QTR_COUNT];
  bool ok = p.getBytes("min", mn, sizeof(mn)) == sizeof(mn) &&
            p.getBytes("max", mx, sizeof(mx)) == sizeof(mx);
  p.end();
  if (ok) {
    for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
      _min[i] = mn[i];
      _max[i] = mx[i];
    }
  }
  return ok && isCalibrated();
}
