// Host-runnable (pio test -e native) and on-target (pio test -e esp32-c6-devkitc-1).
#include <unity.h>
#include <math.h>
#include "../../src/localization/ekf.cpp"

void setUp() {}
void tearDown() {}

void test_straight_north() {
  EKF f;
  f.reset(0, 0, 0);
  for (int i = 0; i < 100; i++) f.predict(10.0f, 0.0f);   // 1000 mm facing North
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, f.y());
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, f.x());
}

void test_straight_east() {
  EKF f;
  f.reset(0, 0, 1.5707963f);                               // clockwise 90 deg = East
  for (int i = 0; i < 100; i++) f.predict(10.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, f.x());
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, f.y());
}

void test_turn_right_updates_heading() {
  EKF f;
  f.reset(0, 0, 0);
  for (int i = 0; i < 90; i++) f.predict(0.0f, 0.0174532925f);   // 90 x 1 deg, clockwise
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5707963f, f.theta());
}

void test_square_loop_closes() {
  EKF f;
  f.reset(0, 0, 0);
  for (int side = 0; side < 4; side++) {
    for (int i = 0; i < 50; i++) f.predict(10.0f, 0.0f);          // 500 mm
    for (int i = 0; i < 90; i++) f.predict(0.0f, 0.0174532925f);  // turn right 90 deg
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, f.x());
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, f.y());
}

void test_heading_update_pulls_estimate() {
  EKF f;
  f.reset(0, 0, 0);
  for (int i = 0; i < 500; i++) { f.predict(0.0f, 0.0f); f.updateHeading(0.5f); }
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, f.theta());
}

void test_heading_update_handles_wrap() {
  EKF f;
  f.reset(0, 0, 3.10f);
  float innov = f.updateHeading(-3.10f);                   // only 0.083 rad apart across +-pi
  TEST_ASSERT_TRUE(fabsf(innov) < 0.1f);
  TEST_ASSERT_TRUE(fabsf(f.theta()) > 3.0f);
}

void test_covariance_grows_then_shrinks() {
  EKF f;
  f.reset(0, 0, 0);
  for (int i = 0; i < 200; i++) f.predict(5.0f, 0.0f);
  const float pBefore = f.P(2, 2);
  f.updateHeading(0.0f);
  TEST_ASSERT_TRUE(f.P(2, 2) < pBefore);
  TEST_ASSERT_TRUE(f.P(0, 0) > 1.0f);
}

void test_set_position() {
  EKF f;
  f.reset(10, 20, 0.3f);
  f.setPosition(100, 200);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, f.x());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 200.0f, f.y());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, f.theta());
}

static void runAll() {
  UNITY_BEGIN();
  RUN_TEST(test_straight_north);
  RUN_TEST(test_straight_east);
  RUN_TEST(test_turn_right_updates_heading);
  RUN_TEST(test_square_loop_closes);
  RUN_TEST(test_heading_update_pulls_estimate);
  RUN_TEST(test_heading_update_handles_wrap);
  RUN_TEST(test_covariance_grows_then_shrinks);
  RUN_TEST(test_set_position);
  UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() { delay(2000); runAll(); }
void loop() {}
#else
int main(int, char**) { runAll(); return 0; }
#endif
