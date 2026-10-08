#pragma once

// Generic PID with output clamp, conditional-integration anti-windup and an optional
// first-order filter on the derivative. Pure C++ (no Arduino dependency) so it can be
// unit-tested on the host.
class PID {
 public:
  PID(float kp = 0, float ki = 0, float kd = 0) : _kp(kp), _ki(ki), _kd(kd) {}

  void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
  void setOutputLimits(float lo, float hi) { _outMin = lo; _outMax = hi; }
  void setIntegralLimit(float lim) { _iLimit = lim; }          // |I term| <= lim
  void setDerivativeFilter(float alpha) { _dAlpha = alpha; }   // 1 = no filter, ->0 = heavy

  // Derivative computed from the error signal.
  float update(float error, float dt);
  // Derivative supplied by the caller (e.g. gyro rate) - avoids noise/kick.
  float updateWithRate(float error, float errorRate, float dt);

  void reset();
  float output() const { return _out; }
  float integral() const { return _i; }

 private:
  float compute(float e, float d, float dt);

  float _kp, _ki, _kd;
  float _outMin = -1e9f, _outMax = 1e9f, _iLimit = 1e9f, _dAlpha = 1.0f;
  float _i = 0, _prevErr = 0, _dFilt = 0, _out = 0;
  bool _first = true;
};
