#include "mapping/node.h"

const char* dirName(Dir d) {
  static const char* const names[4] = {"N", "E", "S", "W"};
  return names[d & 3];
}

void Node::init(int16_t px, int16_t py, uint8_t f) {
  x = px;
  y = py;
  flags = f;
  for (int i = 0; i < 4; i++) {
    neighbor[i] = -1;
    edgeLen[i] = 0;
    exitState[i] = EXIT_UNKNOWN;
  }
}

bool Node::hasUnexplored() const {
  for (int i = 0; i < 4; i++)
    if (exitState[i] == EXIT_OPEN && neighbor[i] < 0) return true;
  return false;
}

uint8_t Node::degree() const {
  uint8_t n = 0;
  for (int i = 0; i < 4; i++)
    if (neighbor[i] >= 0) n++;
  return n;
}
