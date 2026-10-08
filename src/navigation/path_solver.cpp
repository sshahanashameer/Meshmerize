#include "navigation/path_solver.h"

bool PathSolver::shortestPath(const Maze& m, int8_t from, int8_t to, Dir h, float pen, Path& out) {
  if (from < 0 || to < 0) return false;
  return search(m, from, to, h, pen, out);
}

bool PathSolver::toNearestUnexplored(const Maze& m, int8_t from, Dir h, float pen, Path& out) {
  if (from < 0) return false;
  return search(m, from, -1, h, pen, out);
}

bool PathSolver::search(const Maze& m, int8_t from, int8_t to, Dir h, float pen, Path& out) {
  const int N = m.size();
  if (from >= N) return false;
  const int S = N * 4;
  static const float INF = 1e30f;
  float dist[Maze::MAX_NODES * 4];
  int16_t parent[Maze::MAX_NODES * 4];
  bool done[Maze::MAX_NODES * 4];
  for (int i = 0; i < S; i++) {
    dist[i] = INF;
    parent[i] = -1;
    done[i] = false;
  }
  const int startState = from * 4 + (int)h;
  dist[startState] = 0;
  int goal = -1;

  for (;;) {
    int s = -1;
    float best = INF;
    for (int i = 0; i < S; i++)
      if (!done[i] && dist[i] < best) {
        best = dist[i];
        s = i;
      }
    if (s < 0) break;
    done[s] = true;
    const int n = s >> 2, d = s & 3;
    const bool isGoal = (to >= 0) ? (n == to) : (n != from && m.node((uint8_t)n).hasUnexplored());
    if (isGoal) {
      goal = s;
      break;
    }
    const Node& nd = m.node((uint8_t)n);
    for (int nxt = 0; nxt < 4; nxt++) {
      const int nb = nd.neighbor[nxt];
      if (nb < 0 || nb >= N) continue;
      const int diff = (nxt - d + 4) & 3;                  // 0 straight, 1 right, 2 U-turn, 3 left
      const float turnCost = diff == 0 ? 0.0f : (diff == 2 ? 2.0f * pen : pen);
      const float len = nd.edgeLen[nxt] < 1 ? 1.0f : (float)nd.edgeLen[nxt];
      const float nc = dist[s] + len + turnCost;
      const int ns = nb * 4 + nxt;
      if (nc < dist[ns]) {
        dist[ns] = nc;
        parent[ns] = (int16_t)s;
      }
    }
  }
  if (goal < 0) return false;

  // Reconstruct (goal -> start), then reverse.
  int16_t chain[Maze::MAX_NODES * 4];
  int len = 0;
  for (int s = goal; s >= 0 && len < Maze::MAX_NODES * 4; s = parent[s]) chain[len++] = (int16_t)s;
  out.count = 0;
  out.lengthMm = 0;
  out.turns = 0;
  for (int i = len - 1; i >= 0; i--) {
    if (out.count >= Maze::MAX_NODES + 1) return false;
    const int node = chain[i] >> 2;
    out.nodes[out.count] = (int8_t)node;
    if (i > 0) {
      const Dir leave = (Dir)(chain[i - 1] & 3);       // heading used to reach the next state
      out.dirs[out.count] = leave;
      out.lengthMm += m.node((uint8_t)node).edgeLen[leave];
      const Dir arrived = (Dir)(chain[i] & 3);
      if (leave != arrived) out.turns++;
    } else {
      out.dirs[out.count] = (Dir)(chain[i] & 3);
    }
    out.count++;
  }
  return true;
}
