#pragma once
#include <stdint.h>
#include "mapping/maze.h"

// Route through the maze graph: nodes[0] = origin, dirs[i] = direction to leave nodes[i].
struct Path {
  int8_t nodes[Maze::MAX_NODES + 1];
  Dir dirs[Maze::MAX_NODES + 1];
  uint8_t count = 0;        // number of nodes in the path (edges = count-1)
  float lengthMm = 0;
  uint8_t turns = 0;
};

// Dijkstra over (node, arrival heading) states so that turns can be penalised:
// the fastest path is not always the shortest one.
class PathSolver {
 public:
  static bool shortestPath(const Maze& m, int8_t from, int8_t to, Dir startHeading,
                           float turnPenaltyMm, Path& out);
  // Nearest node (other than `from`) that still has an unexplored open exit.
  static bool toNearestUnexplored(const Maze& m, int8_t from, Dir startHeading,
                                  float turnPenaltyMm, Path& out);

 private:
  static bool search(const Maze& m, int8_t from, int8_t to, Dir h, float pen, Path& out);
};
