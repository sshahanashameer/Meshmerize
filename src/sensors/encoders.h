#pragma once
#include <stdint.h>
#include "config.h"

// Quadrature encoders, x4 decoding by interrupt on both channels.
// Forward motion counts positive (use cfg::ENC_*_INVERT to fix polarity).
class Encoders {
 public:
  void begin();
  void update(float dt);                       // refresh filtered wheel speeds
  void takeDeltaMm(float& dL, float& dR);      // distance since previous call [mm]
  void reset();

  int32_t ticksL() const { return _l.ticks; }
  int32_t ticksR() const { return _r.ticks; }
  float distanceL() const { return _l.ticks * cfg::MM_PER_TICK; }
  float distanceR() const { return _r.ticks * cfg::MM_PER_TICK; }
  float speedL() const { return _vL; }         // mm/s, low-pass filtered
  float speedR() const { return _vR; }

 private:
  struct Channel {
    uint8_t a, b;
    int8_t sign;
    volatile int32_t ticks;
    volatile uint8_t last;
  };
  static void isr(void* arg);
  static void attach(Channel& c, uint8_t a, uint8_t b, bool invert);

  Channel _l{}, _r{};
  int32_t _spdPrevL = 0, _spdPrevR = 0, _dltPrevL = 0, _dltPrevR = 0;
  float _vL = 0, _vR = 0;
};
