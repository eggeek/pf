#include "rect_jump_point_locator.h"
#include "constants.h"
#include <algorithm>
#include <cstdint>
#include <queue>
#include <vector>

typedef warthog::rectscan::rect_jump_point_locator rectlocator;
using namespace std;

bool rectlocator::_find_jpt(Rect* cur_rect, eposition cure, 
    int curx, int cury, int dx, int dy, int& node_id) {
  int x, y;
  bool res = false;
  vector<int> *jpts;
  vector<int>::iterator it;
  cost_t cost = INF;
  node_id = INF;

  int curid = map_->to_id(curx, cury);

  auto find = [&](eposition e) {
    if (dx + dy > 0) {
      // find min jpt in jptf > node id
      jpts = &(cur_rect->jptf[e]);
      // jpts in jptf stores in ascend order
      it = std::upper_bound(jpts->begin(), jpts->end(), curid);
    }
    else {
      // find max jpt in jptr < node id
      jpts = &(cur_rect->jptr[e]);
      // jpts in jptr stores in descend order
      it = std::upper_bound(jpts->begin(), jpts->end(), curid, greater<int>());
    }
  };

  find(cure);
  if (it != jpts->end()) {
    node_id = *it;
    map_->to_xy(node_id, x, y);
    cost = octile_dist(curx, cury, x, y);
    res = true;
  }
  // if the rect is a line (or dot)
  if ((dx?cur_rect->h: cur_rect->w) == 1) {
    // we also need to check another side
    find(eposition(cure^2));
    if (it != jpts->end()) {
      map_->to_xy(*it, x, y);
      cost_t new_cost = octile_dist(curx, cury, x, y);
      if (new_cost < cost) {
        node_id = *it;
        cost = new_cost;
        res = true;
      }
    }
  }
  // move to adjacent rect
  if (!res) {
    int d2F = cur_rect->disF(dx, dy, curx, cury);
    int cur_mask, nxt_mask;
    curx += dx * d2F;
    cury += dy * d2F;
    if (map_->get_rid(curx+dx, cury+dy) != -1) {
      if (dx) {
        cur_mask = map_->get_maskw(curx, cury);
        nxt_mask = map_->get_maskw(curx+dx, cury+dy);
      }
      else {
        cur_mask = map_->get_maskh(curx, cury);
        nxt_mask = map_->get_maskh(curx+dx, cury+dy);
      }
      if (map_->isjptr[cur_mask][nxt_mask]) {
        node_id = map_->to_id(curx+dx, cury+dy);
        res = true;
      }
    }
  } 
  return res;
}

inline bool rectlocator::_scanLR(Rect* r, int curx, int cury, int dx, int dy) {
    // int node_id = INF;
    bool onL = r->onLR(rdirect::L, dx, dy, curx, cury);
    bool onR = r->onLR(rdirect::R, dx, dy, curx, cury);
    if ( onL || onR) {
      if (r->disF(dx, dy, curx, cury) > minstep) {
        int node_id;
        if (_find_jpt(r, R2E(dx, dy, onL?rdirect::L: rdirect::R), curx, cury, dx, dy, node_id)) {
          jpts_.push_back((uint32_t)node_id);
          return true;
        }
        else return false;
      }
      else {
        size_t sidx = jpts_.size();
        jpl->jump(jps::v2d(dx, dy), map_->gmap->to_padded_id(curx, cury), padded_goal_id,
          jpts_, costs_);
        for (size_t i=sidx; i<jpts_.size(); i++) {
          jpts_[i] = map_->gmap->to_unpadded_id(jpts_[i]);
          int tx, ty;
          map_->to_xy(jpts_[i], tx, ty);
          //      << "at (" << tx << ", " << ty << ") " << jpts_[i] << endl;
        }
        return true;
      }
    }
    return false;
  }

// Precondition:
// interval [lb, ub] is on F border of curr
// push [lb, ub] in forward direction,
// push adjacent intervals to FIFO
// nxtr is the next rect ptr in (dx, dy)
// if nxtr is not nullptr, 
// the `adjacent` interval in nxtr wouldn't be pushed to FIFO
// instead, it will update lb and ub, 
// so that [lb, ub] can be projected in the next diagonal move
//
// Postcondition: [lb, ub] is in the nxtr if exist, 
// otherwise set to INF.
//
inline void rectlocator::_pushIntervalF(
    queue<Interval>& intervals, Rect* curr, Rect* nxtr, 
    int &lb, int &ub, int dx, int dy) {

  // cerr << "push interval forward, dx: " << dx << ", dy: " << dy
  //      << "[" << lb << ", " << ub << "]" << endl;
  // interval is on F border now
  eposition cure = R2E(dx, dy, rdirect::F);
  eposition nxte = R2E(dx, dy, rdirect::B);
  int newLb=INF, newUb=INF, nxtL=INF, nxtU=INF;
  Rect* nxtRect = nullptr;

  auto bs = [&]() {
    int s=0, t=curr->adj[cure].size()-1, l=0, r=0;
    int best=t+1;

    while (s<=t) {
      int m = (s+t)>>1;
      map_->rects[curr->adj[cure][m]].get_range(nxte, l, r);
      if (r < lb) s=m+1;
      else {
        best = m;
        t=m-1;
      }
    }
    return best;
  };

  int sidx = bs();
  for (int i=sidx; i<(int)curr->adj[cure].size(); i++) {
    int rid = curr->adj[cure][i];
    // the adjacent rect contains the goal
    Rect* r = &(map_->rects[rid]);

    int rL=0, rR=0, hori, vertL, vertR;
    r->get_range(cure, rL, rR);
    if (rL > ub) break;
    rL = max(rL, lb); 
    rR = min(rR, ub);

    if (rid == _goal_rid) {
      rL = max(rL, lb);
      rR = min(rR, ub);
      int ax = r->axis(nxte);
      int x, y;
      if (dx == 0) {
        y = ax;
        if (rL <= _goalx && _goalx <= rR) x=_goalx;
        else if (rL > _goalx) x=rL;
        else x=rR;
      }
      else {
        x = ax;
        if (rL <= _goaly && _goaly <= rR) y=_goaly;
        else if (rL > _goaly) y=rL;
        else y=rR;
      }
      jpts_.push_back(map_->to_id(x, y));
      // a shorter path may pass another interval,
      // so we should continue instead of break
      continue;
    }

    hori = r->axis(nxte);
    vertL = r->axis(dx?eposition::N: eposition::W);
    if (lb <= vertL && vertL <= ub) {
      if (_scanLR(r, dx?hori: vertL, dx?vertL: hori, dx, dy))
        rL++;
    }
    vertR = r->axis(dx?eposition::S: eposition::E);
    if (vertL != vertR && lb <= vertR && vertR <= ub) {
      if (_scanLR(r, dx?hori:vertR, dx?vertR:hori, dx, dy))
        rR--;
    }
    if (rL <= rR) {
      if (nxtr != nullptr && r == nxtr) {
        assert(newLb == INF);
        assert(newUb == INF);
        newLb = rL;
        newUb = rR;
      }
      else {
        intervals.push({rL, rR, r});
      }
    }
  }
  lb = newLb, ub = newUb;
}

// interval [lb, ub] in rectangle at B border
void rectlocator::_pushInterval(queue<Interval>& intervals, int dx, int dy) {
  int ax, lb, ub;
  eposition cure = R2E(dx, dy, rdirect::B);

  while (!intervals.empty()) {
    Interval c = intervals.front(); intervals.pop();
    ax = c.r->axis(cure);
    lb = c.lb, ub = c.ub;
    jps::scan_cnt++;
    // if it is short, we will use normal block based scanning
    int d2F = c.r->disF(dx, dy, dx?ax: lb, dx?lb: ax);
    if (d2F*(ub-lb+1) <= minarea) {
      for (int i=lb; i<=ub; i++) {
        _block_scan(dx?ax: i, dx?i: ax, dx, dy);
      }
      continue;
    }
    // the coordinate of interval [lb, ub]
    // dy=0 then ax is y-axis otherwise it is x-axis
    // is lb on L/R border
    if (_scanLR(c.r, dx?ax: lb, dx?lb: ax, dx, dy)) 
      lb++;
    if (c.lb != c.ub && _scanLR(c.r, dx?ax: ub, dx?ub: ax, dx, dy))
      ub--;
    if (lb <= ub) {
      _pushIntervalF(intervals, c.r, nullptr, lb, ub, dx, dy);
    }
  }
}
