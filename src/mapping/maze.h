#pragma once
#include <stdint.h>
#include "mapping/node.h"

// Topological map of the line maze: a graph of Nodes joined by straight, axis-aligned edges.
class Maze {
 public:
  enum { MAX_NODES = 64 };

  Maze() { clear(); }
  void clear();

  int8_t addNode(float x, float y, uint8_t flags);              // -1 if full
  int8_t findNode(float x, float y, float tolMm) const;         // nearest within tol, else -1
  bool connect(int8_t a, Dir dirFromA, int8_t b, float lenMm);  // symmetric link

  uint8_t size() const { return _count; }
  Node& node(uint8_t id) { return _nodes[id]; }
  const Node& node(uint8_t id) const { return _nodes[id]; }

  bool save() const;     // persist to NVS (survives power cycle)
  bool load();
  bool validate(bool requireFinish = false) const;
  void print() const;    // dump through the logger

  int8_t startId = -1;
  int8_t finishId = -1;

 private:
  Node _nodes[MAX_NODES];
  uint8_t _count = 0;
};
