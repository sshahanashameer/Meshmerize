#pragma once
#include <stdint.h>
#include <Wire.h>
#include "config.h"

// Minimal BNO055 driver (I2C). One burst read gives gyro, Euler and linear accel.
// Conventions exposed here:  heading = clockwise-positive (compass style, 0..360)
//                            gyroZ   = counter-clockwise-positive (right-hand rule)
// Mount the board flat (Z up). Other orientations need the axis-remap registers.
class BNO055 {
 public:
  bool begin(TwoWire& wire = Wire, uint8_t addr = cfg::BNO_I2C_ADDR, uint8_t mode = cfg::BNO_MODE);
  bool update();                                  // returns true when new data was read

  bool present() const { return _ok; }
  float headingDeg() const { return _heading; }
  float headingRad() const { return _heading * 0.0174532925f; }
  float rollDeg() const { return _roll; }
  float pitchDeg() const { return _pitch; }
  float gyroZDps() const { return _gyroZ; }
  float gyroZRadPerSec() const { return _gyroZ * 0.0174532925f; }
  float linAccelX() const { return _lx; }         // m/s^2, body frame
  float linAccelY() const { return _ly; }
  uint8_t calibStatus() const { return _calib; }  // sys<<6 | gyr<<4 | acc<<2 | mag
  bool isFullyCalibrated() const { return ((_calib >> 6) & 3) == 3 && ((_calib >> 4) & 3) == 3; }

 private:
  bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len);
  bool readReg(uint8_t reg, uint8_t& v) { return readBytes(reg, &v, 1); }
  bool writeReg(uint8_t reg, uint8_t v);

  TwoWire* _wire = nullptr;
  uint8_t _addr = 0x28;
  bool _ok = false;
  uint8_t _calibDiv = 0;
  float _heading = 0, _roll = 0, _pitch = 0, _gyroZ = 0, _lx = 0, _ly = 0;
  uint8_t _calib = 0;
};
