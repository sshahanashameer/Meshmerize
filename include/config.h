#pragma once
// =============================================================================
//  config.h  -  every pin, geometry value, gain and threshold in one place.
// Target: ESP32-C6 WROOM-1. The exact carrier-board pinout was NOT supplied.
// The legacy GPIO values below are retained only as placeholders and MUST NOT be
// used until replaced with the verified carrier wiring. Firmware movement is
// interlocked by HARDWARE_PINMAP_VERIFIED; do not flip it on by guesswork.
// =============================================================================
#include <stdint.h>

namespace cfg {

// Set true only after replacing every legacy pin below using the actual carrier schematic.
constexpr bool HARDWARE_PINMAP_VERIFIED = false;

// ------------------------------- PINS ----------------------------------------
// QTR-8RC, index 0 = left-most sensor. Powered from 3.3 V so outputs are 3.3 V safe.
constexpr uint8_t QTR_COUNT = 8;
constexpr uint8_t QTR_PINS[QTR_COUNT] = {32, 33, 25, 26, 27, 14, 13, 4};
constexpr int8_t  QTR_EMITTER_PIN = -1;      // LEDON pin, -1 = left floating (always on)

// TB6612FNG dual motor driver
constexpr uint8_t PIN_ML_PWM = 18, PIN_ML_IN1 = 19, PIN_ML_IN2 = 23;
constexpr uint8_t PIN_MR_PWM = 17, PIN_MR_IN1 = 16, PIN_MR_IN2 = 5;
constexpr int8_t  PIN_MOTOR_STBY = 15;       // -1 if STBY is tied to 3V3
constexpr bool    MOTOR_L_INVERT = false;    // flip if wheel spins backwards for +command
constexpr bool    MOTOR_R_INVERT = false;
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;
constexpr uint8_t  MOTOR_PWM_BITS = 10;

// Quadrature encoders (input-only pins; use 3V3 hall encoders, no pull-ups on these pins)
constexpr uint8_t PIN_ENC_L_A = 34, PIN_ENC_L_B = 35;
constexpr uint8_t PIN_ENC_R_A = 36, PIN_ENC_R_B = 39;
constexpr bool    ENC_L_INVERT = false;      // flip so that FORWARD motion counts positive
constexpr bool    ENC_R_INVERT = true;       // mirrored motor -> usually inverted

// BNO055 (I2C)
constexpr uint8_t PIN_I2C_SDA = 21, PIN_I2C_SCL = 22;
constexpr uint8_t BNO_I2C_ADDR = 0x28;       // 0x29 if COM3/ADR pulled high
constexpr uint8_t BNO_MODE = 0x08;           // 0x08 IMUPLUS (gyro+accel, no magnetometer), 0x0C NDOF

constexpr uint8_t PIN_BUTTON = 0;            // BOOT button, active low
constexpr uint8_t PIN_LED    = 2;

// ------------------------------ GEOMETRY -------------------------------------
constexpr float WHEEL_DIAMETER_MM = 24.0f;
constexpr float WHEEL_BASE_MM     = 85.0f;   // centre-to-centre distance between wheels
constexpr float ENC_TICKS_PER_REV = 840.0f;  // wheel shaft ticks at x4 decoding (e.g. 7 PPR*4*30:1)
constexpr float MM_PER_TICK       = 3.14159265f * WHEEL_DIAMETER_MM / ENC_TICKS_PER_REV;
constexpr float MAX_WHEEL_SPEED_MMPS = 700.0f;  // free-running speed at 100 % duty (feed-forward)
constexpr float SENSOR_TO_AXLE_MM = 55.0f;   // wheel axle -> QTR array
constexpr float LINE_HALF_WIDTH_MM = 9.0f;   // half of the track line width

// ------------------------------- TIMING --------------------------------------
constexpr uint32_t SENSOR_PERIOD_MS = 5;     // 200 Hz sensor/estimation task
constexpr uint8_t  IMU_DIVIDER = 2;          // read BNO055 every 2nd cycle (100 Hz = its output rate)
constexpr int      SENSOR_TASK_PRIO = 5;
constexpr int      MISSION_TASK_PRIO = 3;

// ------------------------------- QTR -----------------------------------------
constexpr uint16_t QTR_TIMEOUT_US = 2500;
constexpr uint16_t QTR_MIN_RANGE_US = 150;   // below this the sensor is considered uncalibrated
constexpr bool     QTR_LINE_IS_BLACK = false; // white line on dark floor; normalized white = high
constexpr uint16_t QTR_DARK_THRESHOLD = 500; // normalised 0..1000, > = "on line"
constexpr uint16_t QTR_LINE_MIN_SUM = 300;   // sum of normalised values needed to say "line seen"
constexpr float    CAL_SWEEP_DEG = 35.0f;    // calibration sweep half-angle

// ------------------------------- CONTROL -------------------------------------
// Line follower: error = position-3500 (+-3500), output = wheel speed difference [mm/s]
constexpr float LINE_KP = 0.060f, LINE_KI = 0.0f, LINE_KD = 0.0020f;
constexpr float LINE_MAX_CORR_MMPS = 300.0f;
// Heading hold (straight driving through junctions): error in deg -> mm/s
constexpr float HEAD_KP = 6.0f, HEAD_KI = 0.0f, HEAD_KD = 0.15f;
constexpr float HEAD_MAX_CORR_MMPS = 150.0f;
// In-place turns
constexpr float TURN_KP = 3.0f, TURN_KI = 0.0f, TURN_KD = 0.15f;
constexpr float TURN_MAX_SPEED_MMPS = 220.0f;
constexpr float TURN_MIN_SPEED_MMPS = 50.0f;   // overcome static friction
constexpr float TURN_TOL_DEG = 1.5f;
constexpr float TURN_SETTLE_MS = 80.0f;
constexpr uint32_t TURN_TIMEOUT_MS = 3000;
// Wheel speed loops (output is motor command -1..1 on top of feed-forward)
constexpr float SPEED_KP = 0.0010f, SPEED_KI = 0.0080f, SPEED_KD = 0.0f;
constexpr float SPEED_FF = 1.0f / MAX_WHEEL_SPEED_MMPS;

constexpr float ACCEL_MMPS2 = 1500.0f;
constexpr float DECEL_MMPS2 = 2500.0f;
constexpr float MIN_CRAWL_MMPS = 80.0f;
constexpr uint32_t STOP_TIMEOUT_MS = 400;
constexpr uint32_t EDGE_TIMEOUT_MS = 30000;
constexpr float EDGE_OVERSHOOT_MM = 150.0f;    // known-length edge: give up this far past expected

// ----------------------------- RUN PROFILES ----------------------------------
constexpr float EXPLORE_SPEED_MMPS = 250.0f;
constexpr float EXPLORE_SCAN_SPEED_MMPS = 180.0f;
constexpr float RUN_CRUISE_MMPS = 500.0f;
constexpr float RUN_NODE_SPEED_MMPS = 250.0f;
constexpr float TURN_PENALTY_MM = 120.0f;      // path cost of one 90 deg turn (in "mm of travel")
constexpr bool  FULL_EXPLORATION = false;      // competition run is Start -> End; stop at confirmed End
constexpr uint16_t MAX_EXPLORE_STEPS = 300;
constexpr uint32_t START_DELAY_MS = 1500;

// ------------------------------ JUNCTIONS ------------------------------------
constexpr float LOST_LINE_MM = 15.0f;          // line absent this long -> dead end
constexpr float FINISH_ALLDARK_MM = 30.0f;     // all sensors dark for this travel -> finish box
constexpr float FINISH_MAX_MM = 80.0f;         // give up waiting for the box to end
constexpr float NODE_MATCH_TOL_MM = 70.0f;     // two nodes closer than this are the same node

// ------------------------------- EKF -----------------------------------------
constexpr float EKF_SIGMA_DIST_FRAC = 0.03f;   // encoder distance noise (fraction of travel)
constexpr float EKF_SIGMA_DIST_MIN_MM = 0.05f;
constexpr float EKF_SIGMA_YAW_STEP = 0.0005f;  // gyro integration noise per step [rad]
constexpr float EKF_R_HEADING = 0.0007f;       // BNO055 heading variance [rad^2]

}  // namespace cfg
