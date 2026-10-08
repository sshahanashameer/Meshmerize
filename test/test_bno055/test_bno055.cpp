// On-target. Keep the robot still for the first tests; rotate it by hand for the last one.
#include <Arduino.h>
#include <unity.h>
#include "../../src/sensors/bno055.cpp"

static BNO055 imu;
void setUp() {}
void tearDown() {}

void test_init() { TEST_ASSERT_TRUE_MESSAGE(imu.begin(), "BNO055 not found (check wiring / address)"); }

void test_heading_valid_and_stable() {
  TEST_ASSERT_TRUE(imu.update());
  const float h0 = imu.headingDeg();
  TEST_ASSERT_TRUE(h0 >= 0.0f && h0 <= 360.0f);
  delay(1000);
  for (int i = 0; i < 100; i++) { imu.update(); delay(10); }
  float d = fabsf(imu.headingDeg() - h0);
  if (d > 180) d = 360 - d;
  TEST_ASSERT_TRUE_MESSAGE(d < 3.0f, "heading drifted > 3 deg in 2 s while stationary");
}

void test_gyro_quiet_when_still() {
  float sum = 0;
  for (int i = 0; i < 100; i++) { imu.update(); sum += fabsf(imu.gyroZDps()); delay(10); }
  TEST_ASSERT_TRUE(sum / 100 < 1.5f);
}

void test_heading_is_clockwise_positive() {
  TEST_MESSAGE("Rotate the robot ~90 deg CLOCKWISE (seen from above) within 6 s");
  imu.update();
  const float h0 = imu.headingDeg();
  float best = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 6000) {
    imu.update();
    float d = imu.headingDeg() - h0;
    if (d < -180) d += 360;
    if (d > 180) d -= 360;
    if (d > best) best = d;
    delay(10);
  }
  TEST_ASSERT_TRUE_MESSAGE(best > 45.0f, "heading did not increase for clockwise rotation");
}

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_init);
  RUN_TEST(test_heading_valid_and_stable);
  RUN_TEST(test_gyro_quiet_when_still);
  RUN_TEST(test_heading_is_clockwise_positive);
  UNITY_END();
}
void loop() {}
