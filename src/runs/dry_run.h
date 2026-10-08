#pragma once
#include "control/drive.h"
#include "mapping/maze.h"

// Start-to-End exploration. Stops at confirmed End; never returns to Start.
class DryRun {
 public:
  DryRun(Drive& drive, Maze& maze) : _d(drive), _m(maze) {}
  bool run();

 private:
  Drive& _d;
  Maze& _m;
};
