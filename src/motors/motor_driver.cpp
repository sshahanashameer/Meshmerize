#include "motors/motor_driver.h"
#include <Arduino.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define MD_LEDC_V3 1
#endif

static constexpr uint32_t kMaxDuty = (1u << cfg::MOTOR_PWM_BITS) - 1u;

void MotorDriver::write(const Ch& c, uint32_t duty) {
#ifdef MD_LEDC_V3
  ledcWrite(c.pwm, duty);
#else
  ledcWrite(c.ledc, duty);
#endif
}

void MotorDriver::begin() {
  const Ch* chans[2] = {&_l, &_r};
  for (int i = 0; i < 2; i++) {
    const Ch& c = *chans[i];
    pinMode(c.in1, OUTPUT);
    pinMode(c.in2, OUTPUT);
    digitalWrite(c.in1, LOW);
    digitalWrite(c.in2, LOW);
#ifdef MD_LEDC_V3
    ledcAttachChannel(c.pwm, cfg::MOTOR_PWM_FREQ_HZ, cfg::MOTOR_PWM_BITS, c.ledc);
#else
    ledcSetup(c.ledc, cfg::MOTOR_PWM_FREQ_HZ, cfg::MOTOR_PWM_BITS);
    ledcAttachPin(c.pwm, c.ledc);
#endif
    write(c, 0);
  }
  if (cfg::PIN_MOTOR_STBY >= 0) pinMode(cfg::PIN_MOTOR_STBY, OUTPUT);
  enable(true);
}

void MotorDriver::enable(bool on) {
  if (cfg::PIN_MOTOR_STBY >= 0) digitalWrite(cfg::PIN_MOTOR_STBY, on ? HIGH : LOW);
}

void MotorDriver::drive(const Ch& c, float u) {
  if (u > 1.0f) u = 1.0f;
  if (u < -1.0f) u = -1.0f;
  if (c.invert) u = -u;
  const uint32_t duty = (uint32_t)((u < 0 ? -u : u) * kMaxDuty + 0.5f);
  if (u > 0) {
    digitalWrite(c.in1, HIGH);
    digitalWrite(c.in2, LOW);
  } else if (u < 0) {
    digitalWrite(c.in1, LOW);
    digitalWrite(c.in2, HIGH);
  } else {
    digitalWrite(c.in1, LOW);
    digitalWrite(c.in2, LOW);
  }
  write(c, duty);
}

void MotorDriver::setSpeeds(float left, float right) {
  drive(_l, left);
  drive(_r, right);
}

void MotorDriver::brake() {
  const Ch* chans[2] = {&_l, &_r};
  for (int i = 0; i < 2; i++) {
    digitalWrite(chans[i]->in1, HIGH);
    digitalWrite(chans[i]->in2, HIGH);
    write(*chans[i], kMaxDuty);
  }
}

void MotorDriver::coast() {
  const Ch* chans[2] = {&_l, &_r};
  for (int i = 0; i < 2; i++) {
    digitalWrite(chans[i]->in1, LOW);
    digitalWrite(chans[i]->in2, LOW);
    write(*chans[i], 0);
  }
}
