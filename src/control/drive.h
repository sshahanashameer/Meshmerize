#pragma once
// =============================================================================
//  Drive: shared motion layer used by both runs.
//    * owns the sensor task (nominal 200 Hz): QTR + encoders + BNO055 -> EKF -> RobotState
//    * offers blocking motion primitives for the mission task:
//        followToEvent(), turnToHeading(), stop(), calibrateSensors(), pose helpers
//  Primitives are virtual so the navigation logic can be simulated on a PC.
// =============================================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "sdkconfig.h"
#include "config.h"
#include "sensors/qtr8rc.h"
#include "sensors/encoders.h"
#include "sensors/bno055.h"
#include "motors/motor_driver.h"
#include "localization/ekf.h"
#include "control/pid.h"

#if CONFIG_FREERTOS_UNICORE
#define ROBOT_CORE_SENSORS tskNO_AFFINITY
#define ROBOT_CORE_MISSION tskNO_AFFINITY
#else
#define ROBOT_CORE_SENSORS 0
#define ROBOT_CORE_MISSION 1
#endif

// Snapshot published by the sensor task (copied under a spinlock).
struct RobotState {
  uint32_t seq = 0;
  uint32_t tUs = 0;
  uint16_t sensor[cfg::QTR_COUNT] = {0};   // normalised 0..1000
  uint16_t linePos = 3500;
  uint8_t darkMask = 0;
  uint8_t darkCount = 0;
  bool lineFound = false;
  float distL = 0, distR = 0;              // mm since boot
  float speedL = 0, speedR = 0;            // mm/s
  float yawRate = 0;                       // rad/s, clockwise positive
  float x = 0, y = 0, theta = 0;           // EKF pose (mm, mm, rad cw from North)
  uint8_t imuCal = 0;
  bool imuOk = false;
};

enum class Event : uint8_t { NONE, JUNCTION, DEAD_END, FINISH, DISTANCE, TIMEOUT, ABORT };
const char* eventName(Event e);

struct JunctionInfo {                      // exits seen relative to the travel direction
  bool left = false, straight = false, right = false, finish = false;
  float advance = 0;
};

struct FollowParams {
  float speed = cfg::EXPLORE_SPEED_MMPS;       // cruise [mm/s]
  float headingDeg = 0;                        // reference heading (compass, cw)
  float expectedDist = 0;                      // known edge length [mm], 0 = unknown
  float slowSpeed = cfg::EXPLORE_SPEED_MMPS;   // speed wanted at expectedDist
  float scanSpeed = cfg::EXPLORE_SCAN_SPEED_MMPS;  // speed while crossing a junction
  bool detectEvents = true;                    // false: pure line following by distance
  bool stopAtDistance = false;                 // true: return DISTANCE at expectedDist
};

class Drive {
 public:
  Drive(QTR8RC& qtr, Encoders& enc, BNO055& imu, MotorDriver& motors, EKF& ekf);
  virtual ~Drive() {}

  bool start(bool imuOk);                      // create the sensor task

  // ---- state ----
  virtual RobotState state() const;
  static float travel(const RobotState& s) { return 0.5f * (s.distL + s.distR); }
  virtual void zeroPose();                     // north = current heading, position (0,0)
  virtual void setPosition(float x, float y);  // loop-closure correction

  // ---- motion primitives (call from the mission task only) ----
  virtual Event followToEvent(const FollowParams& p, JunctionInfo& info);
  virtual bool turnToHeading(float targetDeg);
  virtual void stop();
  virtual bool calibrateSensors();

  void abort() { _abort = true; _motors.brake(); }
  void clearAbort() { _abort = false; }
  virtual bool aborted() const { return _abort; }

 protected:
  bool tick(RobotState& s, float& dt);         // wait for the next sensor sample
  Event scanJunction(const FollowParams& p, const RobotState& s0, JunctionInfo& info);
  float headingCorrection(const RobotState& s, float targetDeg, float dt);
  void commandWheels(float vL, float vR, const RobotState& s, float dt);
  float approach(float cur, float target, float dt) const;
  bool waitStopped();
  void waitPoseCmd();

  static void sensorTaskThunk(void* arg);
  void sensorLoop();

  QTR8RC& _qtr;
  Encoders& _enc;
  BNO055& _imu;
  MotorDriver& _motors;
  EKF& _ekf;

  PID _linePid, _headPid, _turnPid, _spdL, _spdR;
  float _vBase = 0;
  uint32_t _lastTUs = 0;
  volatile bool _abort = false;
  bool _imuOk = false;

  TaskHandle_t _sensorTask = nullptr;
  SemaphoreHandle_t _sampleSem = nullptr;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  RobotState _state;

  enum : uint8_t { CMD_ZERO_HEADING = 1, CMD_SET_POS = 2 };
  volatile uint8_t _poseCmd = 0;
  volatile float _cmdX = 0, _cmdY = 0;
};
