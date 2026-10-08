#include "mapping/maze.h"
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include "utils/logger.h"

static const char* TAG = "maze";
static const uint32_t kMagic = 0x4D415A45;   // "MAZE"
static const uint8_t kVersion = 1;

struct MazeHeader {
  uint32_t magic;
  uint8_t version;
  uint8_t count;
  int8_t startId;
  int8_t finishId;
};

void Maze::clear() {
  _count = 0;
  startId = -1;
  finishId = -1;
  for (int i = 0; i < MAX_NODES; i++) _nodes[i].init(0, 0, 0);
}

int8_t Maze::addNode(float x, float y, uint8_t flags) {
  if (_count >= MAX_NODES) return -1;
  _nodes[_count].init((int16_t)lroundf(x), (int16_t)lroundf(y), flags);
  return (int8_t)_count++;
}

int8_t Maze::findNode(float x, float y, float tolMm) const {
  int8_t best = -1;
  float bestD = tolMm;
  for (uint8_t i = 0; i < _count; i++) {
    const float d = hypotf(_nodes[i].x - x, _nodes[i].y - y);
    if (d <= bestD) {
      bestD = d;
      best = (int8_t)i;
    }
  }
  return best;
}

bool Maze::connect(int8_t a, Dir da, int8_t b, float lenMm) {
  if (a < 0 || b < 0 || a >= (int8_t)_count || b >= (int8_t)_count) return false;
  long l = lroundf(lenMm);
  if (l < 1) l = 1;
  if (l > 65535) l = 65535;
  const Dir db = dirBack(da);
  if ((_nodes[a].neighbor[da] >= 0 && _nodes[a].neighbor[da] != b) ||
      (_nodes[b].neighbor[db] >= 0 && _nodes[b].neighbor[db] != a)) return false;
  if (a == b) return false;
  _nodes[a].neighbor[da] = b;
  _nodes[a].edgeLen[da] = (uint16_t)l;
  _nodes[a].exitState[da] = EXIT_OPEN;
  _nodes[b].neighbor[db] = a;
  _nodes[b].edgeLen[db] = (uint16_t)l;
  _nodes[b].exitState[db] = EXIT_OPEN;
  return true;
}

bool Maze::save() const {
  if (!validate(true)) return false;
  uint8_t buf[sizeof(MazeHeader) + sizeof(_nodes)];
  MazeHeader h;
  h.magic = kMagic;
  h.version = kVersion;
  h.count = _count;
  h.startId = startId;
  h.finishId = finishId;
  memcpy(buf, &h, sizeof(h));
  const size_t nbytes = (size_t)_count * sizeof(Node);
  memcpy(buf + sizeof(h), _nodes, nbytes);
  Preferences p;
  if (!p.begin("maze", false)) return false;
  const size_t want = sizeof(h) + nbytes;
  const size_t wrote = p.putBytes("data", buf, want);
  p.end();
  if (wrote != want) LOGE(TAG, "NVS save failed (%u/%u)", (unsigned)wrote, (unsigned)want);
  return wrote == want;
}

bool Maze::load() {
  uint8_t buf[sizeof(MazeHeader) + sizeof(_nodes)];
  Preferences p;
  if (!p.begin("maze", true)) return false;
  const size_t got = p.getBytes("data", buf, sizeof(buf));
  p.end();
  if (got < sizeof(MazeHeader)) return false;
  MazeHeader h;
  memcpy(&h, buf, sizeof(h));
  if (h.magic != kMagic || h.version != kVersion || h.count > MAX_NODES) return false;
  if (got != sizeof(h) + (size_t)h.count * sizeof(Node)) return false;
  clear();
  memcpy(_nodes, buf + sizeof(h), (size_t)h.count * sizeof(Node));
  _count = h.count;
  startId = h.startId;
  finishId = h.finishId;
  if (!validate(true)) { clear(); return false; }
  return true;
}

bool Maze::validate(bool requireFinish) const {
  if (_count == 0 || _count > MAX_NODES) return false;
  if (startId < 0 || startId >= (int8_t)_count) return false;
  if (!( _nodes[(uint8_t)startId].flags & NODE_START)) return false;
  if (requireFinish && (finishId < 0 || finishId >= (int8_t)_count)) return false;
  if (finishId >= 0 && (finishId >= (int8_t)_count || !(_nodes[(uint8_t)finishId].flags & NODE_FINISH))) return false;
  for (uint8_t i = 0; i < _count; ++i) {
    const Node& n = _nodes[i];
    for (int d = 0; d < 4; ++d) {
      if (n.exitState[d] > EXIT_OPEN) return false;
      const int8_t nb = n.neighbor[d];
      if (nb < -1 || nb >= (int8_t)_count) return false;
      if (nb < 0) { if (n.edgeLen[d] != 0) return false; continue; }
      if (nb == (int8_t)i || n.edgeLen[d] == 0) return false;
      const Node& other = _nodes[(uint8_t)nb];
      const int back = (d + 2) & 3;
      if (other.neighbor[back] != (int8_t)i || other.edgeLen[back] != n.edgeLen[d] || other.exitState[back] != EXIT_OPEN) return false;
      if (n.exitState[d] != EXIT_OPEN) return false;
    }
  }
  return true;
}

void Maze::print() const {
  LOGI(TAG, "maze: %u nodes, start=%d finish=%d", _count, startId, finishId);
  for (uint8_t i = 0; i < _count; i++) {
    const Node& n = _nodes[i];
    LOGI(TAG, " #%02u (%5d,%5d) %s%s%s  N:%d(%u) E:%d(%u) S:%d(%u) W:%d(%u)", i, n.x, n.y,
         (n.flags & NODE_START) ? "START " : "", (n.flags & NODE_FINISH) ? "FINISH " : "",
         (n.flags & NODE_DEAD_END) ? "DEADEND " : "", n.neighbor[0], n.edgeLen[0], n.neighbor[1],
         n.edgeLen[1], n.neighbor[2], n.edgeLen[2], n.neighbor[3], n.edgeLen[3]);
  }
}
