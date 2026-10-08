#include "sensors/bno055.h"
#include <Arduino.h>

namespace {
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_GYR_DATA = 0x14;   // gyro X/Y/Z, Euler H/R/P, quaternion, linear accel (26 bytes)
constexpr uint8_t REG_CALIB_STAT = 0x35;
constexpr uint8_t REG_UNIT_SEL = 0x3B;
constexpr uint8_t REG_OPR_MODE = 0x3D;
constexpr uint8_t REG_PWR_MODE = 0x3E;
constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
constexpr uint8_t REG_PAGE_ID = 0x07;
constexpr uint8_t CHIP_ID = 0xA0;
constexpr uint8_t MODE_CONFIG = 0x00;
}  // namespace

bool BNO055::writeReg(uint8_t reg, uint8_t v) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(v);
  return _wire->endTransmission() == 0;
}

bool BNO055::readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom(_addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
  return true;
}

bool BNO055::begin(TwoWire& wire, uint8_t addr, uint8_t mode) {
  _wire = &wire;
  _addr = addr;
  _ok = false;
  _wire->begin(cfg::PIN_I2C_SDA, cfg::PIN_I2C_SCL);
  _wire->setClock(400000);
  _wire->setTimeOut(3); // bound bus stalls; physical deadline still requires measurement

  uint8_t id = 0;
  for (int i = 0; i < 40 && id != CHIP_ID; i++) {   // chip needs up to ~650 ms after power-up
    if (!readReg(REG_CHIP_ID, id)) id = 0;
    if (id != CHIP_ID) delay(50);
  }
  if (id != CHIP_ID) return false;

  if (!writeReg(REG_OPR_MODE, MODE_CONFIG)) return false;
  delay(25);
  if (!writeReg(REG_SYS_TRIGGER, 0x20)) return false; // reset
  delay(50);
  id = 0;
  for (int i = 0; i < 40 && id != CHIP_ID; i++) {
    if (!readReg(REG_CHIP_ID, id)) id = 0;
    if (id != CHIP_ID) delay(30);
  }
  if (id != CHIP_ID) return false;
  delay(50);

  if (!writeReg(REG_PWR_MODE, 0x00)) return false;  // normal power
  delay(10);
  if (!writeReg(REG_PAGE_ID, 0x00) || !writeReg(REG_UNIT_SEL, 0x00)) return false;
  // Do not force the external crystal: many breakout boards do not populate one.
  if (!writeReg(REG_SYS_TRIGGER, 0x00)) return false;
  delay(10);
  if (!writeReg(REG_OPR_MODE, mode)) return false;
  delay(30);

  _ok = true;
  return update();
}

bool BNO055::update() {
  if (!_ok) return false;
  uint8_t b[26];
  if (!readBytes(REG_GYR_DATA, b, sizeof(b))) { _ok = false; return false; }
  auto s16 = [&b](int i) -> int16_t { return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8)); };
  _gyroZ = s16(4) / 16.0f;        // deg/s, CCW positive
  _heading = s16(6) / 16.0f;      // deg, CW positive, 0..360
  _roll = s16(8) / 16.0f;
  _pitch = s16(10) / 16.0f;
  _lx = s16(20) / 100.0f;
  _ly = s16(22) / 100.0f;
  if (++_calibDiv >= 50) {        // calibration status about once per half second
    _calibDiv = 0;
    readReg(REG_CALIB_STAT, _calib);
  }
  return true;
}
