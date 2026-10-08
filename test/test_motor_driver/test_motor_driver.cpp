// On-target. PUT THE ROBOT ON A STAND (wheels free). Watch the wheels and confirm each prompt.
#include <Arduino.h>
#include <unity.h>
#include "../../src/motors/motor_driver.cpp"

static MotorDriver md;
void setUp() {}
void tearDown() { md.brake(); }

void test_left_forward_then_back() {
  TEST_MESSAGE("LEFT wheel forward 1 s then backward 1 s. Forward must be robot-forward.");
  md.setSpeeds(0.4f, 0); delay(1000);
  md.setSpeeds(-0.4f, 0); delay(1000);
  md.brake();
  TEST_PASS();
}

void test_right_forward_then_back() {
  TEST_MESSAGE("RIGHT wheel forward 1 s then backward 1 s.");
  md.setSpeeds(0, 0.4f); delay(1000);
  md.setSpeeds(0, -0.4f); delay(1000);
  md.brake();
  TEST_PASS();
}

void test_both_ramp() {
  TEST_MESSAGE("Both wheels ramp 0 -> 60 % -> 0, robot should drive straight");
  for (int i = 0; i <= 60; i++) { md.setSpeeds(i / 100.0f, i / 100.0f); delay(20); }
  for (int i = 60; i >= 0; i--) { md.setSpeeds(i / 100.0f, i / 100.0f); delay(20); }
  md.brake();
  TEST_PASS();
}

void test_values_are_clamped() {
  md.setSpeeds(5.0f, -5.0f);   // must not crash or overflow the duty register
  delay(100);
  md.brake();
  TEST_PASS();
}

void setup() {
  delay(3000);
  md.begin();
  UNITY_BEGIN();
  RUN_TEST(test_left_forward_then_back);
  RUN_TEST(test_right_forward_then_back);
  RUN_TEST(test_both_ramp);
  RUN_TEST(test_values_are_clamped);
  UNITY_END();
}
void loop() {}
