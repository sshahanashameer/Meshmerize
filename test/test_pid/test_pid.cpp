// Host-runnable (pio test -e native) and on-target (pio test -e esp32-c6-devkitc-1).
#include <unity.h>
#include <math.h>
#include "../../src/control/pid.cpp"

void setUp() {}
void tearDown() {}

void test_proportional_only() {
  PID pid(2.0f, 0, 0);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, pid.update(1.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -3.0f, pid.update(-1.5f, 0.01f));
}

void test_output_clamp() {
  PID pid(10.0f, 0, 0);
  pid.setOutputLimits(-1.0f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, pid.update(5.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, pid.update(-5.0f, 0.01f));
}

void test_integral_accumulates() {
  PID pid(0, 1.0f, 0);
  float out = 0;
  for (int i = 0; i < 100; i++) out = pid.update(1.0f, 0.01f);   // integral of 1 over 1 s
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, out);
}

void test_anti_windup_recovers_quickly() {
  PID pid(0, 5.0f, 0);
  pid.setOutputLimits(-1.0f, 1.0f);
  for (int i = 0; i < 1000; i++) pid.update(10.0f, 0.01f);   // saturated for 10 s
  TEST_ASSERT_TRUE(pid.integral() <= 1.01f);                  // did not wind up to 500
  int steps = 0;
  while (pid.update(-1.0f, 0.01f) > 0.0f && steps < 1000) steps++;
  TEST_ASSERT_TRUE(steps < 60);
}

void test_derivative_with_external_rate() {
  PID pid(0, 0, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 5.0f, pid.updateWithRate(0.0f, 10.0f, 0.01f));
}

void test_closed_loop_converges() {
  PID pid(8.0f, 4.0f, 0.2f);
  float x = 0, dt = 0.005f;                                   // plant: x' = u
  for (int i = 0; i < 2000; i++) x += pid.update(1.0f - x, dt) * dt;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, x);
}

void test_reset() {
  PID pid(1, 1, 1);
  pid.update(5, 0.01f);
  pid.reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.output());
}

static void runAll() {
  UNITY_BEGIN();
  RUN_TEST(test_proportional_only);
  RUN_TEST(test_output_clamp);
  RUN_TEST(test_integral_accumulates);
  RUN_TEST(test_anti_windup_recovers_quickly);
  RUN_TEST(test_derivative_with_external_rate);
  RUN_TEST(test_closed_loop_converges);
  RUN_TEST(test_reset);
  UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() { delay(2000); runAll(); }
void loop() {}
#else
int main(int, char**) { runAll(); return 0; }
#endif
