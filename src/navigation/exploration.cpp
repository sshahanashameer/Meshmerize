#include "navigation/exploration.h"
#include <math.h>
#include "utils/logger.h"

static const char* TAG = "explore";

bool Exploration::begin() {
  _d.zeroPose();
  _m.clear();
  const int8_t id = _m.addNode(0, 0, NODE_START);
  if (id < 0) return false;
  _m.startId = id;
  _cur = id;
  _dir = NORTH;
  Node& n = _m.node(id);
  for (int d = 0; d < 4; d++) n.exitState[d] = (d == NORTH) ? EXIT_OPEN : EXIT_NONE;
  return true;
}

bool Exploration::chooseNext(Dir& out) {
  const Node& n = _m.node(_cur);
  const Dir order[4] = {dirLeft(_dir), _dir, dirRight(_dir), dirBack(_dir)};
  for (int i = 0; i < 4; i++) {
    const Dir d = order[i];
    if (n.exitState[d] == EXIT_OPEN && n.neighbor[d] < 0) {
      out = d;
      return true;
    }
  }
  Path p;
  if (PathSolver::toNearestUnexplored(_m, _cur, _dir, 0.0f, p) && p.count >= 2) {
    out = p.dirs[0];
    return true;
  }
  return false;
}

Exploration::Status Exploration::step() {
  if (_d.aborted()) return Status::ABORTED;
  if (!cfg::FULL_EXPLORATION && _m.finishId >= 0) return Status::COMPLETE;
  Dir next;
  if (!chooseNext(next)) return Status::COMPLETE;
  return traverse(next, false);
}

Exploration::Status Exploration::traverse(Dir dir, bool intoStart) {
  if (dir != _dir) {
    if (!_d.turnToHeading(dirToDeg(dir))) return _d.aborted() ? Status::ABORTED : Status::ERROR;
    _dir = dir;
  }
  const Node& from = _m.node(_cur);
  FollowParams p;
  p.speed = cfg::EXPLORE_SPEED_MMPS;
  p.slowSpeed = cfg::EXPLORE_SPEED_MMPS;
  p.scanSpeed = cfg::EXPLORE_SCAN_SPEED_MMPS;
  p.headingDeg = dirToDeg(dir);
  p.detectEvents = true;
  if (intoStart) {                      // the start box has no junction marking: stop by distance
    p.detectEvents = false;
    p.expectedDist = from.edgeLen[dir];
    p.stopAtDistance = true;
    p.slowSpeed = 0;
  }

  JunctionInfo info;
  const Event ev = _d.followToEvent(p, info);
  if (ev == Event::ABORT) return Status::ABORTED;
  if (ev == Event::TIMEOUT || ev == Event::NONE) {
    LOGE(TAG, "edge from node %d (%s) failed: %s", _cur, dirName(dir), eventName(ev));
    return Status::ERROR;
  }
  _d.stop();   // exploration halts at every node: keeps timing of the bookkeeping safe

  if (intoStart) {
    _cur = _m.startId;
    _d.setPosition(_m.node(_cur).x, _m.node(_cur).y);
    return Status::CONTINUE;
  }
  const int8_t id = arrive(ev, info, dir, _cur);
  if (id < 0) return Status::ERROR;
  _cur = id;
  return Status::CONTINUE;
}

int8_t Exploration::arrive(Event ev, const JunctionInfo& info, Dir dir, int8_t from) {
  const RobotState s = _d.state();
  const Node& fn = _m.node(from);

  // The maze is axis aligned: the coordinate across the travel direction cannot change,
  // so take it from the node we left. Only the coordinate along the edge comes from the EKF.
  float x = s.x, y = s.y;
  if (dir == NORTH || dir == SOUTH) x = fn.x;
  else y = fn.y;

  int8_t id = _m.findNode(x, y, cfg::NODE_MATCH_TOL_MM);
  const bool isNew = (id < 0);
  if (isNew) {
    uint8_t flags = 0;
    if (ev == Event::FINISH) flags |= NODE_FINISH;
    if (ev == Event::DEAD_END) flags |= NODE_DEAD_END;
    id = _m.addNode(x, y, flags);
    if (id < 0) {
      LOGE(TAG, "node table full");
      return -1;
    }
  }
  Node& n = _m.node(id);
  if (ev == Event::FINISH) {
    n.flags |= NODE_FINISH;
    _m.finishId = id;
  }

  // Exits relative to the direction we arrived in. Never overwrite what is already known.
  auto setIfUnknown = [&n](Dir d, ExitState st) {
    if (n.exitState[d] == EXIT_UNKNOWN) n.exitState[d] = st;
  };
  const bool terminal = (ev == Event::DEAD_END || ev == Event::FINISH);
  setIfUnknown(dirLeft(dir), (!terminal && info.left) ? EXIT_OPEN : EXIT_NONE);
  setIfUnknown(dir, (!terminal && info.straight) ? EXIT_OPEN : EXIT_NONE);
  setIfUnknown(dirRight(dir), (!terminal && info.right) ? EXIT_OPEN : EXIT_NONE);

  const Dir back = dirBack(dir);
  if (n.neighbor[back] >= 0 && n.neighbor[back] != from)
    LOGW(TAG, "map inconsistency at node %d (%s side already linked to %d)", id, dirName(back),
         n.neighbor[back]);

  const float len = fabsf((dir == NORTH || dir == SOUTH) ? (n.y - fn.y) : (n.x - fn.x));
  if (!_m.connect(from, dir, id, len)) {
    LOGE(TAG, "conflicting reciprocal edge %d --%s--> %d", from, dirName(dir), id);
    return -1;
  }

  // Loop closure: snap the estimator onto the stored node position.
  _d.setPosition(n.x, n.y);
  LOGI(TAG, "%s node %d at (%d,%d) via %s len=%.0f  exits L%d S%d R%d%s", isNew ? "new" : "known", id,
       n.x, n.y, eventName(ev), len, info.left, info.straight, info.right,
       (n.flags & NODE_FINISH) ? " [FINISH]" : "");
  return id;
}

bool Exploration::returnToStart() {
  for (int guard = 0; guard < 100; guard++) {
    if (_d.aborted()) return false;
    if (_cur == _m.startId) break;
    Path p;
    if (!PathSolver::shortestPath(_m, _cur, _m.startId, _dir, 0.0f, p) || p.count < 2) return false;
    const bool last = (p.count == 2);          // final edge ends in the start box
    const Status st = traverse(p.dirs[0], last);
    if (st == Status::ABORTED || st == Status::ERROR) return false;
  }
  if (_cur != _m.startId) return false;
  if (!_d.turnToHeading(dirToDeg(NORTH))) return false;
  _dir = NORTH;
  return true;
}
