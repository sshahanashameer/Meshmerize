#pragma once
#include "control/drive.h"
#include "mapping/maze.h"
#include "navigation/path_solver.h"

// Graph-based depth-first exploration of a line maze.
// Each step() traverses exactly ONE edge: it picks an unexplored exit at the current node
// (priority left, straight, right, back) or, if none is left, the first edge of the route
// to the nearest node that still has one. It therefore handles loops and dead ends.
class Exploration {
 public:
  enum class Status : uint8_t { CONTINUE, COMPLETE, ERROR, ABORTED };

  Exploration(Drive& drive, Maze& maze) : _d(drive), _m(maze) {}

  bool begin();                  // zero pose, create start node (robot faces NORTH on the line)
  Status step();
  bool returnToStart();          // shortest known route back, finishes facing NORTH

  int8_t current() const { return _cur; }
  Dir heading() const { return _dir; }

 private:
  bool chooseNext(Dir& out);
  Status traverse(Dir dir, bool intoStart);
  int8_t arrive(Event ev, const JunctionInfo& info, Dir dir, int8_t from);

  Drive& _d;
  Maze& _m;
  int8_t _cur = -1;
  Dir _dir = NORTH;
};
