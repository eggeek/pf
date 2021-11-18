#pragma once

// rect_jump_point_locator.h
//
// @author: shizhe
// @created: 14/09/2021
//

#include "constants.h"
#include "jps.h"
#include "rectmap.h"
#include "online_jump_point_locator2.h"
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>
#include <algorithm>

namespace warthog {
namespace rectscan {
using namespace std;

inline double octile_dist(int x0, int y0, int x1, int y1) {
  int dx = abs(x0 - x1);
  int dy = abs(y0 - y1);
  int diag = min(dx, dy);
  return diag * warthog::DBL_ROOT_TWO + (double)(dx + dy - (diag<<1));
}

const int INF = numeric_limits<int>::max();

class rect_jump_point_locator
{
	public: 

		rect_jump_point_locator(rectscan::RectMap* map): map_(map) {
      cur_node_id_ = INF;
      cur_goal_id_ = INF;
      _curx = _cury = _goal_rid = INF;
      jpts_.reserve(1<<7);
      costs_.reserve(1<<7);
      jpl = new jps::online_jump_point_locator2(map_->gmap);
    };
    ~rect_jump_point_locator() {
      delete jpl;
    }
    int scan_cnt = 0;

		void
		jump(jps::direction d, int node_id, int goal_id, Rect* rect) {

      assert(intervals_h.empty());
      assert(intervals_v.empty());
      cur_node_id_ = node_id;
      map_->to_xy(cur_node_id_, _curx, _cury);
      if (cur_goal_id_ != goal_id) {
        cur_goal_id_ = goal_id;
        map_->to_xy(cur_goal_id_, _goalx, _goaly);
        _goal_rid = map_->get_rid(_goalx, _goaly);
        padded_goal_id = map_->gmap->to_padded_id(_goalx, _goaly);
      }
      if (rect->rid == _goal_rid) {
        jpts_.push_back(cur_goal_id_);
      }
      else {
        switch(d) {
          case jps::NORTH:
            _scan<0, -1>(node_id, rect);
            break;
          case jps::SOUTH:
            _scan<0, 1>(node_id, rect);
            break;
          case jps::EAST:
            _scan<1, 0>(node_id, rect);
            break;
          case jps::WEST:
            _scan<-1, 0>(node_id, rect);
            break;
          case jps::NORTHEAST:
            _scanDiag<1, -1>(node_id, rect);
            break;
          case jps::NORTHWEST:
            _scanDiag<-1, -1>(node_id, rect);
            break;
          case jps::SOUTHEAST:
            _scanDiag<1, 1>(node_id, rect);
            break;
          case jps::SOUTHWEST:
            _scanDiag<-1, 1>(node_id, rect);
            break;
          default:
            break;
        }
      }
      costs_.resize(jpts_.size());
      for (int i=0; i<(int)jpts_.size(); i++) {
        int x, y;
        map_->to_xy(jpts_[i], x, y);
        costs_[i] = octile_dist(x, y, _curx, _cury);
      }
      assert(intervals_h.empty());
      assert(intervals_v.empty());
    }

		int mem() { return sizeof(this); }

    template<int dx, int dy>
    void scanInterval(int lb, int ub, Rect* cur_rect) {

      queue<Interval> &intervals = dx? intervals_v: intervals_h;
      intervals.push({lb, ub, cur_rect});
      _pushInterval<dx, dy>(intervals);
    }

    vector<uint32_t>& get_jpts() { return jpts_; }
    void set_jpts(vector<uint32_t> vi) { jpts_ = vector<uint32_t>(vi.begin(), vi.end()); }

    vector<cost_t>& get_costs() { return costs_; }
    void set_costs(vector<cost_t> vc) { costs_ = vector<cost_t>(vc.begin(), vc.end());}

    inline void reset() {
      jpts_.clear();
      costs_.clear();
    }

    void set_minarea(int v) {minarea = v;}
    void set_minstep(int v) {minstep = v;}

    template<int dx, int dy>
    uint32_t inline neis_const() {
      switch (dy) {
        case 1:
          switch (dx) {
            // SOUTHEAST <1, 1>
            case 1: return 394752;
            // SOUTHWEST <1, -1>
            default: return 197376;
          }
        default:
          switch (dx) {
            // NORTHEAST <1, -1> 
            case 1: return 1542;
            // NORTHWEST <-1, -1>
            default: return 771;
          }
      }
    }

  private:
    int minarea = 128;
    int minstep = 8;
		int cur_goal_id_;
    uint32_t padded_goal_id;
		int cur_node_id_;
    int _curx, _cury, _goalx, _goaly, _goal_rid;
    RectMap* map_;
    jps::online_jump_point_locator2* jpl;
    vector<uint32_t> jpts_;
    vector<cost_t> costs_;

    struct Interval {
      int lb, ub;
      Rect* r;
    };
    queue<Interval> intervals_h, intervals_v;

  bool _find_jpt(Rect* cur_rect, eposition cure, 
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

  template<int dx, int dy>
  void _scanDiag(int node_id, Rect* rect) {
    int curx, cury, vertD, horiD, d, xlb, xub, ylb, yub;
    uint32_t neis;
    auto move_diag = [&]() {
      uint32_t padded_id = map_->gmap->to_padded_id(curx, cury);
      map_->gmap->get_neighbours(padded_id, (uint8_t*)&neis);
      // if cannot make the first diagonal move
      if ((neis & neis_const<dx, dy>()) != neis_const<dx, dy>())
        return (rectscan::Rect*)nullptr;
      else {
        curx += dx, cury += dy;
        node_id += map_->mapw*dy + dx;
        return map_->get_rect(curx, cury);
      }
    };


    assert(intervals_h.size() == 0);
    assert(intervals_v.size() == 0);

    map_->to_xy(node_id, curx, cury);
    rect = move_diag();
    xlb = xub = curx;
    ylb = yub = cury;
    while (rect != nullptr) {
      jps::scan_cnt++;

      // if reach the rect that contains the goal
      if (rect->rid == _goal_rid) {
        jpts_.push_back(map_->to_id(curx, cury));
        break;
      }

      vertD = rect->disF(0, dy, curx, cury);
      horiD = rect->disF(dx, 0, curx, cury);
      d  = min(vertD, horiD);
      if (d+1 <= minstep) {
        for (int i=0; i<=d; i++) {
          _block_scan<dx, 0>(curx+i*dx, cury+i*dy);
          _block_scan<0, dy>(curx+i*dx, cury+i*dy);
        }
        curx += dx*d;
        cury += dy*d;
        node_id += map_->mapw *d*dx + d*dy;
        rect = move_diag();
        xlb = xub = curx;
        ylb = yub = cury;
      }
      else {
        // Check jump points before make diagonal move:
        // try to move interval (xlb, xub) to the front, 
        // find jpt if the path is on L/R border
        if (_scanLR<0, dy>(rect, dx>0?xlb: xub, cury))
          dx>0?xlb++: xub--;
        // try to move interval (ylb, yub) to the front
        // find jpt if the path is on L/R border
        if (_scanLR<dx, 0>(rect, curx, dy>0?ylb: yub))
          dy>0?ylb++: yub--;

        dx > 0? xub += d: xlb -= d;
        dy > 0? yub += d: ylb -= d;

        curx += dx*d;
        cury += dy*d;
        node_id += map_->mapw * d*dy + d*dx;

        // Check jump points after make diagonal move in the rectangle
        if (xlb <= xub && _scanLR<0, dy>(rect, curx, cury))
          dx>0?xub--: xlb++;
        if (ylb <= yub && _scanLR<dx, 0>(rect, curx, cury))
          dy>0?yub--: ylb++;
      
        // Move out the rectangle
        Rect* nxt_rect = move_diag();

        if (xlb <= xub)
          _pushIntervalF<0, dy>(intervals_h, rect, nullptr, xlb, xub);
        if (ylb <= yub) 
          _pushIntervalF<dx, 0>(intervals_v, rect, nullptr, ylb, yub);
        // reset [xlb, xub] and [ylb, yub] if there are invalid
        // otherwise extend ub/lb to curx/cury
        if (xlb > xub || xlb == INF) xlb = xub = curx;
        else dx > 0? xub=curx: xlb=curx;
        if (ylb > yub || ylb == INF) ylb = yub = cury;
        else dy > 0? yub=cury: ylb=cury;

        rect = nxt_rect;
      }
    }
    _pushInterval<0, dy>(intervals_h);
    _pushInterval<dx, 0>(intervals_v);
  }

  template<int dx, int dy>
  inline bool _scanLR(Rect* r, int curx, int cury) {
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
  template<int dx, int dy>
  inline void _pushIntervalF(
      queue<Interval>& intervals, Rect* curr, Rect* nxtr, 
      int &lb, int &ub) {

    // cerr << "push interval forward, dx: " << dx << ", dy: " << dy
    //      << "[" << lb << ", " << ub << "]" << endl;
    // interval is on F border now
    eposition cure = R2E(dx, dy, rdirect::F);
    eposition nxte = R2E(dx, dy, rdirect::B);
    int newLb=INF, newUb=INF;
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
        if (_scanLR<dx, dy>(r, dx?hori: vertL, dx?vertL: hori))
          rL++;
      }
      vertR = r->axis(dx?eposition::S: eposition::E);
      if (vertL != vertR && lb <= vertR && vertR <= ub) {
        if (_scanLR<dx, dy>(r, dx?hori:vertR, dx?vertR:hori))
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
  template<int dx, int dy>
  void _pushInterval(queue<Interval>& intervals) {
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
          _block_scan<dx, dy>(dx?ax: i, dx?i: ax);
        }
        continue;
      }
      // the coordinate of interval [lb, ub]
      // dy=0 then ax is y-axis otherwise it is x-axis
      // is lb on L/R border
      if (_scanLR<dx, dy>(c.r, dx?ax: lb, dx?lb: ax)) 
        lb++;
      if (c.lb != c.ub && _scanLR<dx, dy>(c.r, dx?ax: ub, dx?ub: ax))
        ub--;
      if (lb <= ub) {
        _pushIntervalF<dx, dy>(intervals, c.r, nullptr, lb, ub);
      }
    }
  }

  template<int dx, int dy>
  void _scan(int node_id, Rect* cur_rect) {

    rdirect curp;
    eposition cure;
    int curx, cury;
    map_->to_xy(node_id, curx, cury);
    cure = cur_rect->pos(curx, cury);

    int jpid, d2F;
    bool onL = false, onR = false;

    d2F = cur_rect->disF(dx, dy, curx, cury);
    if (d2F < minstep) {
      _block_scan<dx, dy>(curx, cury);
      return;
    }

    onL = cur_rect->disLR(rdirect::L, dx, dy, curx, cury) == 0;
    onR = cur_rect->disLR(rdirect::R, dx, dy, curx, cury) == 0;

    auto move_fwd = [&]() {
      cure = R2E(dx, dy, rdirect::F);
      d2F = cur_rect->disF(dx, dy, curx, cury);
      switch (dx) {
        case 0:
          cury += dy * d2F;
          break;
        default:
          curx += dx * d2F;
          break;
      }
    };
    
    // inside, then move to the forward edge
    if (cure == eposition::I) {
      move_fwd();
    }

    // we need to explicitly check jump points if on border L/R
    if (onL)
      cure = R2E(dx, dy, rdirect::L);
    else if (onR)
      cure = R2E(dx, dy, rdirect::R);

    curp = E2R(dx, dy, cure);

    while (true) {
      // when the cur rect contains the goal
      if (cur_rect->rid == _goal_rid) {
        jpts_.push_back(map_->to_id(curx, cury));
        break;
      }
      jps::scan_cnt++;
      switch (curp) {
        // base case
        // on verticle border
        case rdirect::L:
        case rdirect::R:
        {
          if (cur_rect->disF(dx, dy, curx, cury) > minstep) {
            bool res = _find_jpt(cur_rect, cure, curx, cury, dx, dy, jpid);
            if (res) {
              jpts_.push_back((uint32_t)jpid);
              return;
            }
          }
          else {
            if (_block_scan<dx, dy>(curx, cury))
              return;
          }
        }
        // cross the rect
        case rdirect::B:
        {
          // move to the end of the border in this direction
          // and going to move to adjacent rect
          move_fwd();
          curp = rdirect::F;
        }
        // move to adjacent rect
        case rdirect::F:
        {
          int rid = map_->get_rid(curx+dx, cury+dy);
          if (rid == -1)  // no adjacent, dead end
            return;

          // move to adjacent rect in (dx, dy)
          curx += dx, cury += dy;
          cure = R2E(dx, dy, rdirect::B);
          cur_rect = &(map_->rects[rid]);
          onL = cur_rect->disLR(rdirect::L, dx, dy, curx, cury) == 0;
          onR = cur_rect->disLR(rdirect::R, dx, dy, curx, cury) == 0;

          // we need to explicitly check jump points if on border L/R
          if (onL)
            cure = R2E(dx, dy, rdirect::L);
          else if (onR)
            cure = R2E(dx, dy, rdirect::R);
          curp = E2R(dx, dy, cure);
        } break;
      }
    }
  }

  template<int dx, int dy>
  inline bool _block_scan(int curx, int cury) {
    jps::direction d = jps::v2d(dx, dy);
    size_t sidx = jpts_.size();
    jpl->jump(d, map_->gmap->to_padded_id(curx, cury), padded_goal_id,
        jpts_, costs_);
    for (size_t i=sidx; i<jpts_.size(); i++)
      jpts_[i] = map_->gmap->to_unpadded_id(jpts_[i]);
    return sidx < jpts_.size();
  }
};

}}

