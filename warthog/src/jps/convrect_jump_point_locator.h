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
      cid_ = gid_ = padded_gid_ = INF;
      _cx = _cy = _goal_rid = INF;
      jpts_.reserve(1<<7);
      costs_.reserve(1<<7);
      _jptc.reserve(1);
      _cstc.reserve(1);
      jpl = new jps::online_jump_point_locator2(map_->gmap);
    }
    ~convrect_jump_point_locator() { delete jpl; }
    int scan_cnt = 0;

    void jump(jps::direction d, int cid, int gid, ConvRect* rp) {
      cid_ = cid;
      map_->to_xy(cid_, _cx, _cy);
      if (gid_ != gid) {
        gid_ = gid;
        map_->to_xy(gid_, _gx, _gy);
        _goal_rid = map_->get_rid(_gx, _gy);
        padded_gid_ = map_->gmap->to_padded_id(_gx, _gy);
      }
      if (_goal_rid == rp->rid) {
        _internalJump(d, rp);
        return;
      }
      switch(d) {
        case jps::NORTH:
          _scan<0, -1>(cid, rp);
          break;
        case jps::SOUTH:
          _scan<0, 1>(cid, rp);
          break;
        case jps::EAST:
          _scan<1, 0>(cid, rp);
          break;
        case jps::WEST:
          _scan<-1, 0>(cid, rp);
          break;
        case jps::NORTHEAST:
          _scanDiag<1, -1>(cid, rp);
          break;
        case jps::NORTHWEST:
          _scanDiag<-1, -1>(cid, rp);
          break;
        case jps::SOUTHEAST:
          _scanDiag<1, 1>(cid, rp);
          break;
        case jps::SOUTHWEST:
          _scanDiag<-1, 1>(cid, rp);
          break;
        default:
          break;
      }
    }

    void _internalJump(jps::direction d, ConvRect* rp) {
      switch(d) {
        case jps::NORTH: internalCardinalScanY<-1>(rp); break;
        case jps::SOUTH: internalCardinalScanY< 1>(rp); break;
        case jps::EAST:  internalCardinalScanX< 1>(rp); break;
        case jps::WEST:  internalCardinalScanX<-1>(rp); break;
        case jps::NORTHEAST: internalDiagScan<1,  -1>(rp); break;
        case jps::NORTHWEST: internalDiagScan<-1, -1>(rp); break;
        case jps::SOUTHEAST: internalDiagScan< 1,  1>(rp); break;
        case jps::SOUTHWEST: internalDiagScan<-1,  1>(rp); break;
        default:
          break;
      }
    }

    template<int dx, int dy>
    inline bool internalDiagScan(ConvRect* rp) {
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
        int dgx = _gx - _cx, dgy = _gy - _cy;
        if (dgx * dx < 0 || dgy * dy < 0) return false;
        // they cannot both be 0, which implies current node is goal
        assert( dgx || dgy);
      }
      int cx = _cx, cy = _cy;
      int bX, bY, bD; // is dx,dy,<dx,dy> blocked
      double cost = 0;
      bool flag = true;
      while (flag) {
        // base case: reach goal in cardinal direction
        // convex property gaurantees that goal is reachable now
        if (cx == _gx || cy == _gy) {
          jpts_.push_back(gid_);
          costs_.push_back((double)((_gx-cx)*dx+(_gy-cy)*dy)+cost);
          return true;
        }
        bX = map_->get_label(cx+dx, cy);      // is (cx+dx, cy) empty
        bY = map_->get_label(cx, cy+dy);      // is (cx, cy+dy) empty
        bD = map_->get_label(cx+dx, cy+dy);   // is (cx+dx, cy+dy) empty 
        while (bX && bY) { // case 0
          cx += dx;
          cy += dy;

          // base case: reach goal in cardinal direction
          if (cx == _gx || cy == _gy) {
            jpts_.push_back(gid_);
            costs_.push_back((double)((_gx-cx)*dx+(_gy-cy)*dy)+cost);
            return true;
          }
          if (cx>rp->xu() || cx<rp->xl() || cy>rp->yu() || cy<rp->yl()) {
            flag = false;
            break;
          }
          cost += warthog::DBL_ROOT_TWO;
          bX = map_->get_label(cx+dx, cy);      // is (cx+dx, cy) empty
          bY = map_->get_label(cx, cy+dy);      // is (cx, cy+dy) empty
          bD = map_->get_label(cx+dx, cy+dy);   // is (cx+dx, cy+dy) empty 
        }
        // continue only in case 2 or case 3
        if ((bX || bY) && flag) {
          // run cardinal move to next jump point
          _cardinal_block_scan(dx*bX, dy*bY, cx, cy);
          if (_cstc.empty()) // no jump point found in this direction, implies a deadend
            break;
          cx += dx*bX*(int)_cstc.back();
          cy += dy*bY*(int)_cstc.back();
          cost += _cstc.back();

          if (cx>rp->xu() || cx<rp->xl() || cy>rp->yu() || cy<rp->yl())
            break;
        } else break;
      }
      return false;
    }

    template<int dy>
    inline bool internalCardinalScanY(ConvRect* rp) {
      // current node and goal are in same convex rectangle
      // <dx, dy> is a cardinal move
      // this may call internalDiagScan if the current node is adjacent to an obstacle,
      // e.g. assume move in north:
      // case 1:  .x.  not adjacent to any obstacle, keep moving in north
      // case 2:  @x?  adjacent to lft obstacle, call <northwest>internalDiagScan instead
      // case 3:  ?x@  adjacent to rht obstacle, call <northeast>internalDiagScan instead
      // return whether find the target

      int lL = map_->get_label(_cx-1, _cy), lR = map_->get_label(_cx+1, _cy);

      // case 1: ".x."
      if (lL == 1 && lR == 1) { 
        // we can find the target only if it is on the way
        int cost = (_gy-_cy)*dy;
        if (cost > 0 && _gx == _cx) {
          jpts_.push_back(gid_);
          costs_.push_back((double)cost);
        }
        return true;
      }
      // case 2: "@x?"
      if (lL == 0) {
        if (internalDiagScan<-1, dy>(rp)) return true;
      }
      // case 3: "?x@"
      if (lR == 0) {
        if (internalDiagScan<1, dy>(rp)) return true;
      }
      return false;
    }

    template<int dx>
    inline bool internalCardinalScanX(ConvRect* rp) {
      // similar to internalCardinalScanY
      int lU = map_->get_label(_cx, _cy-1), lB = map_->get_label(_cx, _cy+1);
      if (lU == 1 && lB == 1) {
        int cost = (_gx-_cx)*dx;
        if (cost > 0 && _gy == _cy) {
          jpts_.push_back(gid_);
          costs_.push_back((double)cost);
        }
        return true;
      }
      if (lU == 0) {
        if (internalDiagScan<dx, -1>(rp)) return true;
      }
      if (lB == 0) {
        if (internalDiagScan<dx, 1>(rp)) return true;
      }
      return false;
    }

    template<int dx, int dy>
    inline void _scan(int cid, ConvRect* rp) {
      assert(false);
    }

    template<int dx, int dy>
    void _scanDiag(int cid, ConvRect* rp) {
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
    int cid_, _cx, _cy,      // current unpadded node id, x and y
        _gx, _gy, _goal_rid; // unpadded goal id, x and y
    jps::online_jump_point_locator2* jpl;
    ConvRectMap* map_;
    vector<uint32_t> jpts_;
    vector<cost_t> costs_;
    // vector for `jpl` to run cardinal block based scan
    // the size of those vectors should <= 1
    vector<uint32_t> _jptc;
    vector<cost_t> _cstc;

    inline void _cardinal_block_scan(int dx, int dy, int curx, int cury) {
      // must not be a diagonal move
      assert(dx == 0 || dy == 0);
      jps::direction d = jps::v2d(dx, dy);
      _jptc.clear();
      _cstc.clear();
      jpl->jump(d, map_->gmap->to_padded_id(curx, cury), padded_gid_,
          _jptc, _cstc);
    }
}; 

}}
