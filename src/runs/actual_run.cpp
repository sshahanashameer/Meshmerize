#include "runs/actual_run.h"
#include "navigation/path_solver.h"
#include "utils/logger.h"

static const char* TAG = "run";

bool ActualRun::run() {
  if (!_m.load()) {
    LOGE(TAG, "no stored map: do a dry run first");
    return false;
  }
  if (!_m.validate(true) || _m.startId < 0 || _m.finishId < 0) {
    LOGE(TAG, "map has no start/finish");
    return false;
  }
  Path path;
  if (!PathSolver::shortestPath(_m, _m.startId, _m.finishId, NORTH, cfg::TURN_PENALTY_MM, path)) {
    LOGE(TAG, "no route from start to finish");
    return false;
  }
  LOGI(TAG, "route: %u nodes, %.0f mm, %u turns", path.count, path.lengthMm, path.turns);

  _d.zeroPose();
  _d.setPosition(_m.node(_m.startId).x, _m.node(_m.startId).y);
  Dir heading = NORTH;
  const uint32_t t0 = millis();

  for (uint8_t i = 0; i + 1 < path.count; i++) {
    const Node& from = _m.node(path.nodes[i]);
    const Node& to = _m.node(path.nodes[i + 1]);
    const Dir d = path.dirs[i];
    const bool last = (i + 2 == path.count);
    const bool turnNext = !last && (path.dirs[i + 1] != d);

    if (d != heading) {
      if (!_d.turnToHeading(dirToDeg(d))) {
        _d.stop();
        return false;
      }
      heading = d;
    }

    FollowParams p;
    p.speed = cfg::RUN_CRUISE_MMPS;
    p.headingDeg = dirToDeg(d);
    p.expectedDist = from.edgeLen[d];
    p.detectEvents = true;
    // Brake into nodes where we must turn or stop; roll straight through the others.
    p.slowSpeed = (last || turnNext) ? cfg::RUN_NODE_SPEED_MMPS : cfg::RUN_CRUISE_MMPS;
    p.scanSpeed = p.slowSpeed;

    JunctionInfo info;
    const Event ev = _d.followToEvent(p, info);
    const Event expected = last ? Event::FINISH : Event::JUNCTION;
    if (ev != expected) {
      _d.stop();
      LOGE(TAG, "edge %u/%u: expected %s, got %s", i + 1, path.count - 1, eventName(expected), eventName(ev));
      return false;
    }
    if (!last) _d.setPosition(to.x, to.y);     // remove accumulated drift at every node
  }
  _d.stop();
  LOGI(TAG, "FINISH reached in %lu ms", (unsigned long)(millis() - t0));
  return true;
}
