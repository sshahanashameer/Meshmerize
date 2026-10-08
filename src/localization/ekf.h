#pragma once
#include <math.h>

// 3-state Extended Kalman Filter, state = [x, y, theta].
//   World frame: x = East [mm], y = North [mm], theta = heading, CLOCKWISE from North [rad]
//   (same convention as the BNO055 heading).
//   predict():       dead reckoning from encoder distance (ds) + gyro heading change (dtheta)
//   updateHeading(): absolute heading measurement from the BNO055 fusion output
// Pure C++ (host-testable).
class EKF {
 public:
  EKF() { reset(0, 0, 0); }

  void reset(float x, float y, float theta);
  void setPosition(float x, float y);                 // loop-closure style position reset
  void predict(float ds, float dtheta);
  float updateHeading(float measuredTheta);           // returns the innovation [rad]

  float x() const { return _x[0]; }
  float y() const { return _x[1]; }
  float theta() const { return _x[2]; }
  float P(int i, int j) const { return _P[i][j]; }

  static float wrapPi(float a) {
    while (a > 3.14159265f) a -= 6.28318531f;
    while (a <= -3.14159265f) a += 6.28318531f;
    return a;
  }

 private:
  float _x[3];
  float _P[3][3];
};
