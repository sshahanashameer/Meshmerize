#include "sensors/encoders.h"
#include <Arduino.h>
#include <driver/gpio.h>

// State-transition table, index = (previous<<2)|current, state = (A<<1)|B.
// Sequence 00->01->11->10 is +1. Must live in DRAM: it is read from an ISR.
static DRAM_ATTR const int8_t QEM[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// ESP-IDF GPIO API is SoC-portable; never read original-ESP32 GPIO structs on C6.
static inline int IRAM_ATTR fastRead(uint8_t pin) {
  return gpio_get_level((gpio_num_t)pin);
}

void IRAM_ATTR Encoders::isr(void* arg) {
  Channel* c = static_cast<Channel*>(arg);
  const uint8_t cur = (uint8_t)((fastRead(c->a) << 1) | fastRead(c->b));
  c->ticks = c->ticks + c->sign * QEM[(c->last << 2) | cur];
  c->last = cur;
}

void Encoders::attach(Channel& c, uint8_t a, uint8_t b, bool invert) {
  c.a = a;
  c.b = b;
  c.sign = invert ? -1 : 1;
  c.ticks = 0;
  pinMode(a, INPUT);
  pinMode(b, INPUT);
  c.last = (uint8_t)((digitalRead(a) << 1) | digitalRead(b));
  attachInterruptArg(digitalPinToInterrupt(a), &Encoders::isr, &c, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(b), &Encoders::isr, &c, CHANGE);
}

void Encoders::begin() {
  attach(_l, cfg::PIN_ENC_L_A, cfg::PIN_ENC_L_B, cfg::ENC_L_INVERT);
  attach(_r, cfg::PIN_ENC_R_A, cfg::PIN_ENC_R_B, cfg::ENC_R_INVERT);
  reset();
}

void Encoders::reset() {
  _l.ticks = 0;
  _r.ticks = 0;
  _spdPrevL = _spdPrevR = _dltPrevL = _dltPrevR = 0;
  _vL = _vR = 0;
}

void Encoders::update(float dt) {
  if (dt <= 0) return;
  noInterrupts();
  const int32_t tl = _l.ticks, tr = _r.ticks;
  interrupts();
  const float iL = (tl - _spdPrevL) * cfg::MM_PER_TICK / dt;
  const float iR = (tr - _spdPrevR) * cfg::MM_PER_TICK / dt;
  _spdPrevL = tl;
  _spdPrevR = tr;
  const float a = 0.4f;   // low-pass: 1 tick = ~0.09 mm is coarse at 200 Hz
  _vL += a * (iL - _vL);
  _vR += a * (iR - _vR);
}

void Encoders::takeDeltaMm(float& dL, float& dR) {
  noInterrupts();
  const int32_t tl = _l.ticks, tr = _r.ticks;
  interrupts();
  dL = (tl - _dltPrevL) * cfg::MM_PER_TICK;
  dR = (tr - _dltPrevR) * cfg::MM_PER_TICK;
  _dltPrevL = tl;
  _dltPrevR = tr;
}
