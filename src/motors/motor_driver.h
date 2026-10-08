#pragma once
#include <stdint.h>
#include "config.h"

// TB6612FNG driver on LEDC PWM. Commands are -1.0 .. +1.0 (positive = forward).
class MotorDriver {
 public:
  void begin();
  void setSpeeds(float left, float right);
  void brake();                // both H-bridge low sides on: fast stop
  void coast();                // outputs floating
  void enable(bool on);        // STBY pin

 private:
  struct Ch {
    uint8_t pwm, in1, in2, ledc;
    bool invert;
  };
  void drive(const Ch& c, float u);
  void write(const Ch& c, uint32_t duty);
  Ch _l{cfg::PIN_ML_PWM, cfg::PIN_ML_IN1, cfg::PIN_ML_IN2, 0, cfg::MOTOR_L_INVERT};
  Ch _r{cfg::PIN_MR_PWM, cfg::PIN_MR_IN1, cfg::PIN_MR_IN2, 1, cfg::MOTOR_R_INVERT};
};
