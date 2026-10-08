#include "control/pid.h"

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void PID::reset() {
  _i = 0;
  _prevErr = 0;
  _dFilt = 0;
  _out = 0;
  _first = true;
}

float PID::update(float error, float dt) {
  if (dt <= 0.0f) return _out;
  const float d = _first ? 0.0f : (error - _prevErr) / dt;
  _prevErr = error;
  _first = false;
  _dFilt += _dAlpha * (d - _dFilt);
  return compute(error, _dFilt, dt);
}

float PID::updateWithRate(float error, float errorRate, float dt) {
  if (dt <= 0.0f) return _out;
  _prevErr = error;
  _first = false;
  return compute(error, errorRate, dt);
}

float PID::compute(float e, float d, float dt) {
  const float p = _kp * e;
  const float dTerm = _kd * d;
  const float iTent = clampf(_i + _ki * e * dt, -_iLimit, _iLimit);
  float out = p + iTent + dTerm;
  if (out > _outMax) {
    out = _outMax;
    if (e < 0) _i = iTent;          // integrate only when it helps leave saturation
  } else if (out < _outMin) {
    out = _outMin;
    if (e > 0) _i = iTent;
  } else {
    _i = iTent;
  }
  _out = out;
  return out;
}
