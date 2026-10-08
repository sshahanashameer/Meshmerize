#pragma once
#include "control/drive.h"
#include "mapping/maze.h"

// Fast run along the shortest/fastest route computed from the stored map.
class ActualRun {
 public:
  ActualRun(Drive& drive, Maze& maze) : _d(drive), _m(maze) {}
  bool run();

 private:
  Drive& _d;
  Maze& _m;
};
