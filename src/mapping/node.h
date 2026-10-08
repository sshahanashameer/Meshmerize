#pragma once
#include <stdint.h>

// Absolute compass directions. The robot starts facing NORTH (+y).
enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Dir dirLeft(Dir d) { return (Dir)((d + 3) & 3); }
inline Dir dirRight(Dir d) { return (Dir)((d + 1) & 3); }
inline Dir dirBack(Dir d) { return (Dir)((d + 2) & 3); }
inline float dirToDeg(Dir d) { return 90.0f * (float)d; }
const char* dirName(Dir d);

enum ExitState : uint8_t { EXIT_UNKNOWN = 0, EXIT_NONE = 1, EXIT_OPEN = 2 };
enum NodeFlags : uint8_t { NODE_START = 0x01, NODE_FINISH = 0x02, NODE_DEAD_END = 0x04 };

// One junction / corner / dead end / start / finish. POD: stored verbatim in NVS.
struct Node {
  int16_t x, y;             // mm, start node = (0,0), +y = North, +x = East
  int8_t neighbor[4];       // node id reached through each exit, -1 = none/unexplored
  uint16_t edgeLen[4];      // mm
  uint8_t exitState[4];     // ExitState per absolute direction
  uint8_t flags;

  void init(int16_t px, int16_t py, uint8_t f);
  bool hasUnexplored() const;   // an OPEN exit that leads nowhere known yet
  uint8_t degree() const;       // number of connected neighbours
};
