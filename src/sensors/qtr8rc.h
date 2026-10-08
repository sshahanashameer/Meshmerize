#pragma once
#include <stdint.h>
#include "config.h"

// Pololu QTR-8RC reflectance array (RC timing). read() is called from ONE task only
// (the sensor task); the other methods are safe to call from the mission task.
class QTR8RC {
 public:
  void begin();
  void readRaw(uint16_t* raw);          // blocking RC measurement, <= QTR_TIMEOUT_US
  void read();                          // readRaw + calibration + normalise + position

  void resetCalibration();              // takes effect on the next read()
  void startCalibration() { _calibrating = true; }
  void stopCalibration() { _calibrating = false; }
  bool isCalibrating() const { return _calibrating; }
  bool isCalibrated() const;
  bool saveCalibration();               // NVS
  bool loadCalibration();               // NVS

  const uint16_t* raw() const { return _raw; }
  const uint16_t* values() const { return _val; }   // 0..1000, 1000 = on line
  uint16_t position() const { return _pos; }        // 0..7000, 3500 = centred
  bool lineDetected() const { return _lineFound; }
  uint8_t darkMask() const { return _mask; }        // bit i = sensor i on line
  uint8_t darkCount() const { return _darkCount; }

 private:
  uint16_t _raw[cfg::QTR_COUNT] = {0};
  uint16_t _val[cfg::QTR_COUNT] = {0};
  uint16_t _min[cfg::QTR_COUNT];
  uint16_t _max[cfg::QTR_COUNT];
  uint16_t _pos = 3500;
  uint8_t _mask = 0;
  uint8_t _darkCount = 0;
  bool _lineFound = false;
  volatile bool _calibrating = false;
  volatile bool _resetReq = false;
};
