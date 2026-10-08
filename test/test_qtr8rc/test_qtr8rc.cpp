// On-target. Place the sensor array over the dark floor and the white line when prompted.
#include <Arduino.h>
#include <unity.h>
#include "../../src/sensors/qtr8rc.cpp"

static QTR8RC qtr;
void setUp() {}
void tearDown() {}

void test_raw_values_in_range() {
  qtr.begin();
  uint16_t raw[cfg::QTR_COUNT];
  qtr.readRaw(raw);
  for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
    char msg[32];
    snprintf(msg, sizeof(msg), "sensor %u raw=%u", i, raw[i]);
    TEST_ASSERT_TRUE_MESSAGE(raw[i] <= cfg::QTR_TIMEOUT_US, msg);
  }
}

void test_read_time_bounded() {
  uint16_t raw[cfg::QTR_COUNT];
  const uint32_t t0 = micros();
  qtr.readRaw(raw);
  TEST_ASSERT_TRUE(micros() - t0 < cfg::QTR_TIMEOUT_US + 500);
}

void test_calibrate_and_position() {
  TEST_MESSAGE("Slide the array back and forth across the line for 4 s...");
  qtr.resetCalibration();
  qtr.startCalibration();
  const uint32_t t0 = millis();
  while (millis() - t0 < 4000) { qtr.read(); delay(5); }
  qtr.stopCalibration();
  TEST_ASSERT_TRUE_MESSAGE(qtr.isCalibrated(), "need both the white line AND dark floor under each sensor while calibrating");
  TEST_MESSAGE("Centre the line under the array; sampling in 3 s");
  delay(3000);
  qtr.read();
  for (uint8_t i = 0; i < cfg::QTR_COUNT; i++) {
    char m[40];
    snprintf(m, sizeof(m), "S%u=%u", i, qtr.values()[i]);
    TEST_MESSAGE(m);
  }
  TEST_ASSERT_TRUE(qtr.lineDetected());
  TEST_ASSERT_INT_WITHIN(1200, 3500, qtr.position());
}

void test_calibration_persists() {
  TEST_ASSERT_TRUE(qtr.saveCalibration());
  TEST_ASSERT_TRUE(qtr.loadCalibration());
}

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_raw_values_in_range);
  RUN_TEST(test_read_time_bounded);
  RUN_TEST(test_calibrate_and_position);
  RUN_TEST(test_calibration_persists);
  UNITY_END();
}
void loop() {}
