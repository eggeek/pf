#pragma once

// convrect_jump_point_locator.h
//
// @author: shizhe
// @created: 4/12/2021
//

#include "constants.h"
#include "jps.h"
#include "convex_rectmap.h"
#include "online_jump_point_locator2.h"
#include "rect_jump_point_locator.h"

namespace warthog {
namespace rectscan {
using namespace convrectscan;

class convrect_jump_point_locator 
{
  public:
    convrect_jump_point_locator(ConvRectMap* map): map_(map) {
      jpts_.reserve(1<<7);
      costs_.reserve(1<<7);
      _jptc.reserve(1);
      _cstc.reserve(1);
      jpl = new jps::online_jump_point_locator2(map_->gmap);
    }
    ~convrect_jump_point_locator() { delete jpl; }
    int scan_cnt = 0;

    void jump(jps::direction d, int cid, int gid, ConvRect* rp) {
      int cx, cy;
      map_->to_xy(cid, cx, cy);
      if (gid_ != gid) {
        gid_ = gid;
        map_->to_xy(gid_, _gx, _gy);
        _goal_rid = map_->get_rid(_gx, _gy);
        padded_gid_ = map_->gmap->to_padded_id(_gx, _gy);
      }
      if (_goal_rid == rp->rid) {
        _internalJump(d, rp, 0, cx, cy);
        return;
      }
      switch(d) {
        case jps::NORTH:
          _scan<0, -1>(rp, cx, cy, 0);
          break;
        case jps::SOUTH:
          _scan<0, 1>(rp, cx, cy, 0);
          break;
        case jps::EAST:
          _scan<1, 0>(rp, cx, cy, 0);
          break;
        case jps::WEST:
          _scan<-1, 0>(rp, cx, cy, 0);
          break;
        case jps::NORTHEAST:
          _scanDiag<1, -1>(cx, cy, 0, rp);
          break;
        case jps::NORTHWEST:
          _scanDiag<-1, -1>(cx, cy, 0, rp);
          break;
        case jps::SOUTHEAST:
          _scanDiag<1, 1>(cx, cy, 0, rp);
          break;
        case jps::SOUTHWEST:
          _scanDiag<-1, 1>(cx, cy, 0, rp);
          break;
        default:
          break;
      }
    }

    void _internalJump(jps::direction d, ConvRect* rp, cost_t c, int cx, int cy) {
      switch(d) {
        case jps::NORTH: internalCardinalScanY<-1>(rp, cx, cy, c); break;
        case jps::SOUTH: internalCardinalScanY< 1>(rp, cx, cy, c); break;
        case jps::EAST:  internalCardinalScanX< 1>(rp, cx, cy, c); break;
        case jps::WEST:  internalCardinalScanX<-1>(rp, cx, cy, c); break;
        case jps::NORTHEAST: internalDiagScan<1,  -1>(rp, cx, cy, c); break;
        case jps::NORTHWEST: internalDiagScan<-1, -1>(rp, cx, cy, c); break;
        case jps::SOUTHEAST: internalDiagScan< 1,  1>(rp, cx, cy, c); break;
        case jps::SOUTHWEST: internalDiagScan<-1,  1>(rp, cx, cy, c); break;
        default:
          break;
      }
    }

    template<int dx, int dy>
    inline bool internalDiagScan(ConvRect* rp, int cx, int cy, cost_t cur_cost) {
      // current node and goal are in same convex rectangle
      // <dx, dy> is a diagonal move, this move may blocked by
      // the inside obstacles, we will follow obstacles by cardinal moves
      // until hit the jump point, e.g. assume in northeast:
      //
      // case 0: ..  all empty, keep moving in <dx, dy>
      //         x.
      // 
      // case 1: .@  deadend, no further jump point
      //         x.
      //
      // case 2: @? move dx until jump point then diagonal
      //         x.
      //
      // case 3: .? move dy until jump point then diagonal
      //         x@
      //
      // case 4: @? deadend, no further move 
      //         x@
      // return whether find the target

      {
        // due to the "convex" property,
        // we wouldn't reach a jump point that has a different diagonal move.
        // we would never scan any tile with -dx/-dy inside the "convex" rect
        // thus can stop the scanning in this case
        int dgx = _gx - cx, dgy = _gy - cy;
        if (dgx * dx < 0 || dgy * dy < 0) return false;
        // they cannot both be 0, which implies current node is goal
        assert( dgx || dgy);
      }
      int bX, bY, bD; // is dx,dy,<dx,dy> blocked
      bool flag = true;
      while (flag) {
        // base case: reach goal in cardinal direction
        // convex property gaurantees that goal is reachable now
        if (cx == _gx || cy == _gy) {
          jpts_.push_back(gid_);
          costs_.push_back((double)((_gx-cx)*dx+(_gy-cy)*dy)+cur_cost);
          return true;
        }
        bX = map_->get_label(cx+dx, cy);      // is (cx+dx, cy) empty
        bY = map_->get_label(cx, cy+dy);      // is (cx, cy+dy) empty
        bD = map_->get_label(cx+dx, cy+dy);   // is (cx+dx, cy+dy) empty 
        while (bX && bY) { // case 0
          cx += dx;
          cy += dy;
          cur_cost += warthog::DBL_ROOT_TWO;

          // base case: reach goal in cardinal direction
          if (cx == _gx || cy == _gy) {
            jpts_.push_back(gid_);
            costs_.push_back((double)((_gx-cx)*dx+(_gy-cy)*dy)+cur_cost);
            return true;
          }
          if (cx>rp->xu() || cx<rp->xl() || cy>rp->yu() || cy<rp->yl()) {
            flag = false;
            break;
          }
          bX = map_->get_label(cx+dx, cy);      // is (cx+dx, cy) empty
          bY = map_->get_label(cx, cy+dy);      // is (cx, cy+dy) empty
          bD = map_->get_label(cx+dx, cy+dy);   // is (cx+dx, cy+dy) empty 
        }
        // continue only in case 2 or case 3
        if ((bX || bY) && flag) {
          // run cardinal move to next jump point
          if (!_cardinal_block_scan(dx*bX, dy*bY, cx, cy, rp))
            break; // no jump point found in this direction, implies a deadend
          cx += dx*bX*(int)_cstc.back();
          cy += dy*bY*(int)_cstc.back();
          cur_cost += _cstc.back();

          if (cx>rp->xu() || cx<rp->xl() || cy>rp->yu() || cy<rp->yl())
            break;
        } else break;
      }
      return false;
    }

    template<int dy>
    inline bool internalCardinalScanY(ConvRect* rp, int cx, int cy, cost_t cur_cost) {
      // current node and goal are in same convex rectangle
      // <dx, dy> is a cardinal move
      // this may call internalDiagScan if the current node is adjacent to an obstacle,
      // e.g. assume move in north:
      // case 1:  .x.  not adjacent to any obstacle, keep moving in north
      // case 2:  @x?  adjacent to lft obstacle, call <northwest>internalDiagScan instead
      // case 3:  ?x@  adjacent to rht obstacle, call <northeast>internalDiagScan instead
      // return whether find the target

      int lL = map_->get_label(cx-1, cy), lR = map_->get_label(cx+1, cy);

      // case 1: ".x."
      if (lL == 1 && lR == 1) { 
        // we can find the target only if it is on the way
        int cost = (_gy-cy)*dy;
        if (cost > 0 && _gx == cx) {
          jpts_.push_back(gid_);
          costs_.push_back((double)cost + cur_cost);
        }
        return true;
      }
      // case 2: "@x?"
      if (lL == 0) {
        if (internalDiagScan<-1, dy>(rp, cx, cy, cur_cost)) return true;
      }
      // case 3: "?x@"
      if (lR == 0) {
        // restore the current cost
        if (internalDiagScan<1, dy>(rp, cx, cy, cur_cost)) return true;
      }
      return false;
    }

    template<int dx>
    inline bool internalCardinalScanX(ConvRect* rp, int cx, int cy, cost_t cur_cost) {
      // similar to internalCardinalScanY
      int lU = map_->get_label(cx, cy-1), lB = map_->get_label(cx, cy+1);
      if (lU == 1 && lB == 1) {
        int cost = (_gx-cx)*dx;
        if (cost > 0 && _gy == cy) {
          jpts_.push_back(gid_);
          costs_.push_back((double)cost);
        }
        return true;
      }
      if (lU == 0) {
        if (internalDiagScan<dx, -1>(rp, cx, cy, cur_cost)) return true;
      }
      if (lB == 0) {
        if (internalDiagScan<dx, 1>(rp, cx, cy, cur_cost)) return true;
      }
      return false;
    }
    
    template<int dx, int dy>
    inline bool blockedLR(int cx, int cy, ConvRect* rp) {
      constexpr int lx = lrdx<dx, dy, rdir::L>();
      constexpr int ly = lrdy<dx, dy, rdir::L>();
      if (rp->inrect(cx+lx, cy+ly) && !map_->get_label(cx+lx, cy+ly))
        return true;
      constexpr int rx = lrdx<dx, dy, rdir::R>();
      constexpr int ry = lrdy<dx, dy, rdir::R>();
      if (rp->inrect(cx+rx, cy+ry) && !map_->get_label(cx+rx, cy+ry))
        return true;
      return false;
    }

    template<int dx, int dy>
    inline void _scan(ConvRect* rp, int cx, int cy, cost_t cur_cost) {
      /* precondtion: <dx, dy> is a cardinal move
       * assume scan in NORTH
       * case 0: .x.  -> move forward
       * case 1: @x.  -> blocked left, call _scanOnBound
       * case 2: .x@  -> blocked right, call _scanOnBound
       * case 3: x??  -> on left border, call _scanLR
       * case 4: ??x  -> on right border, call _scanLR
       */
      int d2F;
      bool onL, onR;

      auto moveF = [&]() {
        d2F = rp->disF(dx, dy, cx, cy);
        cx += dx*d2F;
        cy += dy*d2F;
        cur_cost += d2F;
      };

      while (true) {
        if (rp->rid == _goal_rid) {
          jpts_.push_back(map_->to_id(cx, cy));
          costs_.push_back(cur_cost);
          break;
        }
        // case 1 or case 2
        if (blockedLR<dx, dy>(cx, cy, rp)) {
          // try to find an internal jpoint
          if (_cardinal_block_scan(dx, dy, cx, cy, rp)) {
            cost_t c = _cstc.back();
            cx += dx * (int)c;
            cy += dy * (int)c;
            jpts_.push_back(map_->to_id(cx, cy));
            costs_.push_back(cur_cost + c);
            return;
          }
        }
        else { // case 3 or case 4
          onL = rp->onLR(rdir::L, dx, dy, cx, cy);
          onR = rp->onLR(rdir::R, dx, dy, cx, cy);
          if ((onL || onR) &&
              _scanLR<dx, dy>(cx, cy, cur_cost, onL?rdir::L:rdir::R, rp)) return;
        }
        // no jpoint found and cannot move out rect
        if (rp->is_blocked<dx, dy>(cx, cy)) return;
        moveF();
        int rid = map_->get_rid(cx+dx, cy+dy);
        if (rid == -1) // no adjacent rect, dead end
          return;
        assert(rid > 0); // convex rect rid > 0
        // check whether (cx+dx, cy+dy) is a jump point
        if (map_->is_nxtmove_jp<dx, dy>(cx, cy)) {
          jpts_.push_back(map_->to_id(cx+dx, cy+dy));
          costs_.push_back(cur_cost+1);
          return;
        }
        cx += dx, cy += dy;
        cur_cost ++;
        rp = &(map_->rects[rid-1]);
      }
    }

    template<int dx, int dy>
    inline bool _scanLR(int cx, int cy, cost_t cur_cost, rdir rd, ConvRect* rp) {
      // (cx, cy) is on the L/R border of convex rect
      vector<int> *jpts;
      vector<int>::iterator it;
      epos cure = R2E(dx, dy, rd);
      int cid = map_->to_id(cx, cy);
      if (dx + dy > 0) {
        // find min jpt in jptf > cid
        jpts = &(rp->jptf[cure]);
        // jptf stores in ascend order
        it = std::upper_bound(jpts->begin(), jpts->end(), cid);
      }
      else {
        // find max jpt in jptr < cid
        jpts = &(rp->jptr[cure]);
        // jptr stores in descend order
        it = std::upper_bound(jpts->begin(), jpts->end(), cid, greater<int>());
      }
      if (it != jpts->end()) {
        int nx, ny;
        map_->to_xy(*it, nx, ny);
        cost_t cost = abs(nx-cx) + abs(ny-cy);
        jpts_.push_back(*it);
        costs_.push_back(cur_cost + cost);
        return true;
      }
      return false;
    }

    template<int dx, int dy>
    inline void _scanDiag(int cx, int cy, cost_t cur_cost, ConvRect* rp) {
      assert(false);
    }

    inline void reset() {
      jpts_.clear();
      costs_.clear();
    }

    inline vector<uint32_t>& get_jpts() { return jpts_; }
    inline vector<cost_t>& get_costs() { return costs_; }

  private:
    int gid_;
    uint32_t padded_gid_;
    int _gx, _gy, _goal_rid; // unpadded goal id, x and y
    jps::online_jump_point_locator2* jpl;
    ConvRectMap* map_;
    vector<uint32_t> jpts_;
    vector<cost_t> costs_;
    // vector for `jpl` to run cardinal block based scan
    // the size of those vectors should <= 1
    vector<uint32_t> _jptc;
    vector<cost_t> _cstc;
    struct Interval {
      int lb, ub;
      cost_t pcost; // gvalue of parent
      ConvRect* r;
    };
    queue<Interval> invlH, invlV;

    inline bool _cardinal_block_scan(int dx, int dy, int curx, int cury, ConvRect* rp) {
      // must not be a diagonal move
      assert(dx == 0 || dy == 0);
      jps::direction d = jps::v2d(dx, dy);
      int bx=curx, by=cury;
      switch(d) {
        case jps::NORTH: by=rp->yl()-1; break;
        case jps::EAST:  bx=rp->xu()+1; break;
        case jps::SOUTH: by=rp->yu()+1; break;
        case jps::WEST:  bx=rp->xl()-1; break;
        default:
          assert(false);
          break;
      }
      // set temp obstacle to avoid scan outside rect
      bool tlabel = map_->get_label(bx, by);
      map_->set_label(bx, by, false);
      _jptc.clear();
      _cstc.clear();
      jpl->jump(d, map_->gmap->to_padded_id(curx, cury), padded_gid_,
          _jptc, _cstc);
      if (tlabel)
        map_->set_label(bx, by, tlabel);
      return !_cstc.empty();
    }
}; 

}}
