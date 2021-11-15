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
      jpl = new jps::online_jump_point_locator2(&map_->gmap);
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
        padded_goal_id = map_->gmap.to_padded_id(_goalx, _goaly);
      }
      if (rect->h * rect->w <= minstep) {
        int sidx = jpts_.size();
        jpl->jump(d, map_->gmap.to_padded_id(_curx, _cury), padded_goal_id, 
            jpts_, costs_);
        for (int i=sidx; i<(int)jpts_.size(); i++) {
          jpts_[i] = map_->gmap.to_unpadded_id(jpts_[i]);
        }
        return;
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

    void scanInterval(int lb, int ub, Rect* cur_rect, int dx, int dy) {

      queue<Interval> &intervals = dx? intervals_v: intervals_h;
      cur_rect->set_mark(lb, r2e.at({dx, dy, rdirect::B}), ub);
      cur_rect->set_mark(ub, r2e.at({dx, dy, rdirect::B}), lb);
      intervals.push({lb, ub, cur_rect});
      _pushInterval(intervals, dx, dy);
    }

    vector<uint32_t>& get_jpts() { return jpts_; }
    void set_jpts(vector<uint32_t> vi) { jpts_ = vector<uint32_t>(vi.begin(), vi.end()); }

    vector<cost_t>& get_costs() { return costs_; }
    void set_costs(vector<cost_t> vc) { costs_ = vector<cost_t>(vc.begin(), vc.end());}

    inline void reset() {
      jpts_.clear();
      costs_.clear();
    }

	private:
    const int minstep=1<<9;
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

    bool _find_jpt(Rect* cur_rect, eposition cure, int curx, int cury, 
        int dx, int dy, int& node_id);

    template<int dx, int dy>
    void _scanDiag(int node_id, Rect* rect) {
      int curx, cury, vertD, horiD, d, xlb, xub, ylb, yub;
      auto move_diag = [&]() {
        if (map_->get_rid(curx+dx, cury) != -1 &&
            map_->get_rid(curx, cury+dy) != -1 &&
            map_->get_rid(curx+dx, cury+dy) != -1) {
          curx += dx, cury += dy;
          node_id += map_->mapw * dy + dx;
          return map_->get_rect(curx, cury);
        }
        else return (rectscan::Rect*)nullptr;
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

        // try to move interval (xlb, xub) to the front, 
        // find jpt if the path is on L/R border
        if (_scanLR(rect, dx>0?xlb: xub, cury, 0, dy))
          dx>0?xlb++: xub--;
        // try to move interval (ylb, yub) to the front
        // find jpt if the path is on L/R border
        if (_scanLR(rect, curx, dy>0?ylb: yub, dx, 0))
          dy>0?ylb++: yub--;

        vertD = rect->disF(0, dy, curx, cury);
        horiD = rect->disF(dx, 0, curx, cury);
        d  = min(vertD, horiD);
        dx > 0? xub += d: xlb -= d;
        dy > 0? yub += d: ylb -= d;

        curx += dx*d;
        cury += dy*d;
        node_id += map_->mapw * d*dy + d*dx;

        // scan on border
        if (d > 0) {
          if (vertD >= horiD) {
            int node_id = INF;
            if (_find_jpt(rect, dx>0?eposition::E: eposition::W, 
                          curx, cury, 0, dy, node_id)) {
              jpts_.push_back((uint32_t)node_id);
              dx>0?xub--: xlb++;
            }
          }
          if (horiD >= vertD) {
            int node_id = INF;
            if (_find_jpt(rect, dy>0?eposition::S: eposition::N, 
                          curx, cury, dx, 0, node_id)) {
              jpts_.push_back((uint32_t)node_id);
              dy>0?yub--: ylb++;
            }
          }
        }
        else {
          if (xlb <= xub && _scanLR(rect, curx, cury, 0, dy))
            dx>0?xub--: xlb++;
          if (ylb <= yub && _scanLR(rect, curx, cury, dx, 0))
            dy>0?yub--: ylb++;
        }
        
        Rect* nxt_rect = move_diag();

        if (xlb <= xub)
          _pushIntervalF(intervals_h, rect, nxt_rect, xlb, xub, 0, dy);
        if (ylb <= yub) 
          _pushIntervalF(intervals_v, rect, nxt_rect, ylb, yub, dx, 0);
        // reset [xlb, xub] and [ylb, yub] if there are invalid
        // otherwise extend ub/lb to curx/cury
        if (xlb > xub || xlb == INF) xlb = xub = curx;
        else dx > 0? xub=curx: xlb=curx;
        if (ylb > yub || ylb == INF) ylb = yub = cury;
        else dy > 0? yub=cury: ylb=cury;

        rect = nxt_rect;
      }
      _pushInterval(intervals_h, 0, dy);
      _pushInterval(intervals_v, dx, 0);
    }

    bool _scanLR(Rect* r, int curx, int cury, int dx, int dy);

    void _pushIntervalF(queue<Interval>& intervals, Rect* r, Rect* nxt, int& lb, int& ub, int dx, int dy);
    void _pushInterval(queue<Interval>& intervals, int dx, int dy);

    template<int dx, int dy>
    void _scan(int node_id, Rect* cur_rect) {

      rdirect curp;
      eposition cure;
      int curx, cury;
      map_->to_xy(node_id, curx, cury);
      cure = cur_rect->pos(curx, cury);

      int jpid, d2F;
      bool onL = false, onR = false;

      // d2F = cur_rect->disF(dx, dy, curx, cury);
      // if (d2F < minstep) {
      //   _block_scan(curx, cury, dx, dy);
      //   return;
      // }

      onL = cur_rect->disLR(rdirect::L, dx, dy, curx, cury) == 0;
      onR = cur_rect->disLR(rdirect::R, dx, dy, curx, cury) == 0;

      auto move_fwd = [&]() {
        cure = r2e.at({dx, dy, rdirect::F});
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

      // if (d2F < minstep) {
      //   _block_scan(curx, cury, dx, dy);
      //   return;
      // }
      
      // inside, then move to the forward edge
      if (cure == eposition::I) {
        move_fwd();
      }

      // we need to explicitly check jump points if on border L/R
      if (onL)
        cure = r2e.at({dx, dy, rdirect::L});
      else if (onR)
        cure = r2e.at({dx, dy, rdirect::R});

      assert(e2r.find({dx, dy, cure}) != e2r.end());
      curp = e2r.at({dx, dy, cure});

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
            bool res = _find_jpt(cur_rect, cure, curx, cury, dx, dy, jpid);
            if (res) {
              jpts_.push_back((uint32_t)jpid);
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
            cure = r2e.at({dx, dy, rdirect::B});
            cur_rect = &(map_->rects[rid]);
            onL = cur_rect->disLR(rdirect::L, dx, dy, curx, cury) == 0;
            onR = cur_rect->disLR(rdirect::R, dx, dy, curx, cury) == 0;

            // we need to explicitly check jump points if on border L/R
            if (onL)
              cure = r2e.at({dx, dy, rdirect::L});
            else if (onR)
              cure = r2e.at({dx, dy, rdirect::R});
            curp = e2r.at({dx, dy, cure});
          } break;
        }
      }
    }

    inline void _block_scan(int curx, int cury, int dx, int dy) {
      jps::direction d = jps::v2d(dx, dy);
      int sidx = jpts_.size();
      jpl->jump(d, map_->gmap.to_padded_id(curx, cury), padded_goal_id,
          jpts_, costs_);
      for (int i=sidx; i<(int)jpts_.size(); i++)
        jpts_[i] = map_->gmap.to_unpadded_id(jpts_[i]);
    }
};

}}

