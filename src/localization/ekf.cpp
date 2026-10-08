#include "localization/ekf.h"
#include "config.h"

void EKF::reset(float x, float y, float theta) {
  _x[0] = x;
  _x[1] = y;
  _x[2] = wrapPi(theta);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) _P[i][j] = 0.0f;
  _P[0][0] = 1.0f;       // mm^2
  _P[1][1] = 1.0f;
  _P[2][2] = 1e-4f;      // rad^2
}

void EKF::setPosition(float x, float y) {
  _x[0] = x;
  _x[1] = y;
  _P[0][0] = 1.0f;
  _P[1][1] = 1.0f;
  _P[0][1] = _P[1][0] = 0.0f;
  _P[0][2] = _P[2][0] = 0.0f;
  _P[1][2] = _P[2][1] = 0.0f;
}

void EKF::predict(float ds, float dtheta) {
  const float thm = _x[2] + 0.5f * dtheta;
  const float s = sinf(thm), c = cosf(thm);

  _x[0] += ds * s;
  _x[1] += ds * c;
  _x[2] = wrapPi(_x[2] + dtheta);

  // Jacobian of the motion model w.r.t. the state
  const float F[3][3] = {{1, 0, ds * c}, {0, 1, -ds * s}, {0, 0, 1}};
  // Jacobian w.r.t. the inputs (ds, dtheta)
  const float G[3][2] = {{s, 0.5f * ds * c}, {c, -0.5f * ds * s}, {0, 1}};
  const float sd = cfg::EKF_SIGMA_DIST_FRAC * fabsf(ds) + cfg::EKF_SIGMA_DIST_MIN_MM;
  const float st = cfg::EKF_SIGMA_YAW_STEP;
  const float qd = sd * sd, qt = st * st;

  float FP[3][3], Pn[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      FP[i][j] = 0;
      for (int k = 0; k < 3; k++) FP[i][j] += F[i][k] * _P[k][j];
    }
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      float v = 0;
      for (int k = 0; k < 3; k++) v += FP[i][k] * F[j][k];          // (F P) F^T
      v += G[i][0] * qd * G[j][0] + G[i][1] * qt * G[j][1];         // G Q G^T
      Pn[i][j] = v;
    }
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) _P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);   // keep symmetric
}

float EKF::updateHeading(float z) {
  const float innov = wrapPi(z - _x[2]);
  const float S = _P[2][2] + cfg::EKF_R_HEADING;
  float K[3];
  for (int i = 0; i < 3; i++) K[i] = _P[i][2] / S;

  for (int i = 0; i < 3; i++) _x[i] += K[i] * innov;
  _x[2] = wrapPi(_x[2]);

  float Pn[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) Pn[i][j] = _P[i][j] - K[i] * _P[2][j];   // (I - K H) P
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) _P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
  return innov;
}
