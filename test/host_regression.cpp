#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "control/pid.h"
#include "localization/ekf.h"
#include "mapping/maze.h"
#include "navigation/path_solver.h"
#include "utils/logger.h"

unsigned long millis() { return 0; }
void Logger::begin(unsigned long) {}
void Logger::setLevel(LogLevel) {}
void Logger::log(LogLevel, const char*, const char*, ...) {}

int main() {
  // PID output stays bounded under persistent large error.
  PID pid;
  pid.setGains(10.0f, 1.0f, 0.0f);
  pid.setOutputLimits(-5.0f, 5.0f);
  for (int i = 0; i < 100; ++i) assert(fabsf(pid.update(100.0f, 0.01f)) <= 5.001f);

  // EKF cardinal convention: theta=0 moves North; +pi/2 moves East.
  EKF ekf;
  ekf.reset(0, 0, 0);
  ekf.predict(100.0f, 0.0f);
  assert(fabsf(ekf.x()) < 0.01f && fabsf(ekf.y() - 100.0f) < 0.1f);
  ekf.reset(0, 0, 1.57079632679f);
  ekf.predict(100.0f, 0.0f);
  assert(fabsf(ekf.x() - 100.0f) < 0.1f && fabsf(ekf.y()) < 0.01f);
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) assert(isfinite(ekf.P(i,j)));

  // Maze reciprocal links, map validity, and heading-aware path reconstruction.
  Maze m;
  int8_t s = m.addNode(0, 0, NODE_START);
  int8_t a = m.addNode(0, 100, 0);
  int8_t b = m.addNode(140, 0, 0);
  int8_t g = m.addNode(140, 100, NODE_FINISH);
  m.startId = s; m.finishId = g;
  assert(m.connect(s, NORTH, a, 100));
  assert(m.connect(a, EAST, g, 140));
  assert(m.connect(s, EAST, b, 140));
  assert(m.connect(b, NORTH, g, 100));
  assert(m.validate(true));
  Path path;
  assert(PathSolver::shortestPath(m, s, g, NORTH, 0.0f, path));
  assert(path.count == 3);
  assert(fabsf(path.lengthMm - 240.0f) < 0.1f);
  assert(!m.connect(s, NORTH, b, 10)); // conflicting link must not overwrite graph
  assert(m.validate(true));
  m.node((uint8_t)a).neighbor[EAST] = -1;
  assert(!m.validate(true)); // reject non-reciprocal graph state
  Maze full;
  for (int i = 0; i < Maze::MAX_NODES; ++i) assert(full.addNode(i * 10, 0, 0) >= 0);
  assert(full.addNode(9999, 0, 0) < 0); // bounded node table
  puts("host_regression: PASS (PID bounds, EKF cardinal motion/covariance, graph integrity, route planning)");
  return 0;
}
