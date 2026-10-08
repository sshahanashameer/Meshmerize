// On-target. Lift the robot; turn each wheel BY HAND one full revolution FORWARD when prompted.
#include <Arduino.h>
#include <unity.h>
#include "../../src/sensors/encoders.cpp"

static Encoders enc;
void setUp() {}
void tearDown() {}

static void waitForTurn(bool left) {
  enc.reset();
  Serial.printf(">> turn the %s wheel one revolution FORWARD within 8 s\n", left ? "LEFT" : "RIGHT");
  const uint32_t t0 = millis();
  while (millis() - t0 < 8000) {
    const float d = left ? enc.distanceL() : enc.distanceR();
    if (d > 0.9f * 3.14159f * cfg::WHEEL_DIAMETER_MM) break;
    delay(10);
  }
}

void test_left_counts_forward() {
  waitForTurn(true);
  const float expected = 3.14159f * cfg::WHEEL_DIAMETER_MM;
  TEST_ASSERT_TRUE_MESSAGE(enc.distanceL() > 0, "negative: flip cfg::ENC_L_INVERT");
  TEST_ASSERT_FLOAT_WITHIN(0.15f * expected, expected, enc.distanceL());
}

void test_right_counts_forward() {
  waitForTurn(false);
  const float expected = 3.14159f * cfg::WHEEL_DIAMETER_MM;
  TEST_ASSERT_TRUE_MESSAGE(enc.distanceR() > 0, "negative: flip cfg::ENC_R_INVERT");
  TEST_ASSERT_FLOAT_WITHIN(0.15f * expected, expected, enc.distanceR());
}

void test_delta_matches_total() {
  enc.reset();
  float dl, dr;
  enc.takeDeltaMm(dl, dr);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, dl);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, dr);
}

void setup() {
  delay(2000);
  enc.begin();
  UNITY_BEGIN();
  RUN_TEST(test_left_counts_forward);
  RUN_TEST(test_right_counts_forward);
  RUN_TEST(test_delta_matches_total);
  UNITY_END();
}
void loop() {}
