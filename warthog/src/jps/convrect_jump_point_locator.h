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
      dinfo.resize(map_->mapw*map_->maph);
      for (int i=0; i<(int)dinfo.size(); i++)
        dinfo[i] = {jps::NONE, 0, INF};
      jpl = new jps::online_jump_point_locator2(map_->gmap);
    }
    ~convrect_jump_point_locator() { delete jpl; }
		int mem() { return sizeof(this); }

    jps::direction get_pdir(int cid) {
      if (dinfo[cid].snumber != search_number) return jps::NONE;
      return dinfo[cid].fromd;
    }

    void jump(jps::direction d, int cid, int gid, ConvRect* rp) {
      int cx, cy;
      map_->to_xy(cid, cx, cy);
      debug_start_jump(cx, cy, d);
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
      }
      // base case: reach goal in cardinal direction
      // convex property gaurantees that goal is reachable now
      if (cx == _gx || cy == _gy) {
        debug_found_jpoint_in_goal_rect(_gx, _gy, 
            (double)((_gx-cx)*dx+(_gy-cy)*dy)+cur_cost);
        jpts_.push_back(gid_);
        costs_.push_back((double)((_gx-cx)*dx+(_gy-cy)*dy)+cur_cost);
        return true;
      }
      int bX, bY, bD; // is dx,dy,<dx,dy> blocked
      while (rp->inside(cx+dx, cy+dy)) {
        bX = map_->get_label(cx+dx, cy);      // is (cx+dx, cy) empty
        bY = map_->get_label(cx, cy+dy);      // is (cx, cy+dy) empty
        bD = map_->get_label(cx+dx, cy+dy);   // is (cx+dx, cy+dy) empty 
        if (bX && bX && bY) { // case 0: can make diagonal move
          cx += dx, cy += dy;
          cur_cost += warthog::DBL_ROOT_TWO;
        }
        else if (bX ^ bY) { // case 1: bX and bY are different
          if (!_cardinal_block_scan(dx*bX, dy*bY, cx, cy, rp))
            break;
          cx += dx*bX*(int)_cstc.back();
          cy += dy*bY*(int)_cstc.back();
          cur_cost += _cstc.back();
        }
        else { // case 2 or 3: dead-end
          assert((!bD && (bX && bY)) || (!bX && !bY));
          break;
        }
        if (cx == _gx || cy == _gy) {
          debug_found_jpoint_in_goal_rect(_gx, _gy, 
            (double)((_gx-cx)*dx+(_gy-cy)*dy)+cur_cost);
          jpts_.push_back(gid_);
          costs_.push_back((double)((_gx-cx)*dx+(_gy-cy)*dy)+cur_cost);
          return true;
        }
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
        if (cost >= 0 && _gx == cx) {

          debug_found_jpoint_in_goal_rect(_gx, _gy, (double)cost + cur_cost);
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
        if (cost >= 0 && _gy == cy) {
          debug_found_jpoint_in_goal_rect(_gx, _gy, (double)cost + cur_cost);
          jpts_.push_back(gid_);
          costs_.push_back((double)cost+cur_cost);
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
      if (rp->inside(cx+lx, cy+ly) && !map_->get_label(cx+lx, cy+ly))
        return true;
      constexpr int rx = lrdx<dx, dy, rdir::R>();
      constexpr int ry = lrdy<dx, dy, rdir::R>();
      if (rp->inside(cx+rx, cy+ry) && !map_->get_label(cx+rx, cy+ry))
        return true;
      return false;
    }

    template<int dx, int dy>
    inline void _scan(ConvRect* rp, int cx, int cy, cost_t cur_cost) {
      while (true) {
        if (_scanCardinalInRect<dx, dy, true>(rp, cx, cy, cur_cost))
          break;
        cx += dx, cy += dy;
        cur_cost ++;
        rp = map_->get_rect(cx, cy);
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
        add_jump_point(nx, ny, cur_cost + cost, jps::v2d(dx, dy));
        return true;
      }
      return false;
    }

    template<int dy>
    inline void updateBoundX(int& lb, int& ub, int& lby, int& uby, 
        cost_t& xlb_g, cost_t& xub_g, int cx, int cy, cost_t cur_cost, ConvRect* rp) {
      if (cx >= rp->get_lb<0, dy>() && cx <= rp->get_ub<0, dy>()) {
        if (cx <= lb) { lb = cx, lby = cy; xlb_g = cur_cost;}
        if (cx >= ub) { ub = cx, uby = cy; xub_g = cur_cost;}
      }
    }

    template<int dx>
    inline void updateBoundY(int& lb, int& ub, int& lbx, int& ubx, 
        cost_t& ylb_g, cost_t& yub_g, int cx, int cy, cost_t cur_cost, ConvRect* rp) {
      if (cy >= rp->get_lb<dx, 0>() && cy <= rp->get_ub<dx, 0>()) {
        if (cy <= lb) { lb = cy, lbx = cx; ylb_g = cur_cost;}
        if (cy >= ub) { ub = cy, ubx = cx; yub_g = cur_cost;}
      }
    }

    template<int dx, int dy>
    inline bool _scanThroughDiag(int& cx, int& cy, cost_t& cur_cost,
        ConvRect* rp, int& bX, int& bY, int& bD) {
      /*
       *  e.g. dy = -1
       *
       *    xlb        xub
       * yl  +---------+
       *     ^         ^
       *     ^         ^       when xlb/xub is on border of the rect
       *     ^         +       we need to check jpoints on border
       *     ^         xub_y   [xlb_y, yl] / [xub_y, yl]
       *     +                 similar in ylb/yub
       *    xlb_y
       *
       */
      int xlb = INF, xub = -1, ylb = INF, yub = -1;
      // the corresponding y/x value for xlb/xub/ylb/yub
      int xlb_y = INF, xub_y = INF, ylb_x = INF, yub_x = INF;
      cost_t xlb_g=0, xub_g=0, ylb_g=0, yub_g=0;
      bool res = true;
      if ((cy != rp->yl() && cy != rp->yu()) || 
          !_scanCardinalInRect<dx, 0>(rp, cx, cy, cur_cost))
        updateBoundY<dx>(ylb, yub, ylb_x, yub_x, ylb_g, yub_g, cx, cy, cur_cost, rp);
      if ((cx != rp->xl() && cx != rp->xu()) ||
          !_scanCardinalInRect<0, dy>(rp, cx, cy, cur_cost))
        updateBoundX<dy>(xlb, xub, xlb_y, xub_y, xlb_g, xub_g, cx, cy, cur_cost, rp);
      // <cx, cy> is inside the rect rp, the gvalue is cur_cost
      // return true if the diagonal move can reach the border of the rect
      while (rp->inside(cx+dx, cy+dy)) {
        bX = map_->get_label(cx+dx, cy);
        bY = map_->get_label(cx, cy+dy);
        bD = map_->get_label(cx+dx, cy+dy);
        if (bD && bX && bY) { // case 0: can make diagonal move
          cx += dx, cy += dy;
          cur_cost += warthog::DBL_ROOT_TWO;

          if ((cy != rp->yl() && cy != rp->yu()) ||
              !_scanCardinalInRect<dx, 0>(rp, cx, cy, cur_cost))
            updateBoundY<dx>(ylb, yub, ylb_x, yub_x, ylb_g, yub_g, cx, cy, cur_cost, rp);

          if ((cx != rp->xl() && cx != rp->xu()) ||
              !_scanCardinalInRect<0, dy>(rp, cx, cy, cur_cost))
            updateBoundX<dy>(xlb, xub, xlb_y, xub_y, xlb_g, xub_g, cx, cy, cur_cost, rp);
        }
        else if (bX ^ bY) { // case 1: bX and bY are different
          if (// in following cases, we have already scanned before the while-loop
              (!bY && (cy == rp->yl() || cy == rp->yu())) || // x blocked at y border
              (!bX && (cx == rp->xl() || cx == rp->xu())) || // y blocked at x border
              !_cardinal_block_scan(dx*bX, dy*bY, cx, cy, rp)
             ) {
            res = false;
            break;
          }
          cx += bX*dx*(int)_cstc.back();
          cy += bY*dy*(int)_cstc.back();
          cur_cost += _cstc.back();

          if ((cy != rp->yl() && cy != rp->yu()) ||
              !_scanCardinalInRect<dx, 0>(rp, cx, cy, cur_cost))
            updateBoundY<dx>(ylb, yub, ylb_x, yub_x, ylb_g, yub_g, cx, cy, cur_cost, rp);

          if ((cx != rp->xl() && cx != rp->xu()) ||
              !_scanCardinalInRect<0, dy>(rp, cx, cy, cur_cost))
            updateBoundX<dy>(xlb, xub, xlb_y, xub_y, xlb_g, xub_g, cx, cy, cur_cost, rp);
        }
        else { // case 2 or 3: dead end
          assert((!bD && (bX && bY)) || (!bX && !bY));
          res = false;
          break;
        }
      }
      if (xlb <= xub && _scanCardinalInRect<0, dy>(rp, xlb, xlb_y, xlb_g)) {
        xlb++;  // move 1 step forward (dx>0) or backward (dx<0) 
        xlb_y += dx*dy;  // update the y accordingly
        xlb_g += dx*warthog::DBL_ROOT_TWO; // update the gvalue accordingly
      }
      if (xlb <= xub && _scanCardinalInRect<0, dy>(rp, xub, xub_y, xub_g)) {
        xub--;
        xub_y -= dx*dy;
        xub_g -= dx*warthog::DBL_ROOT_TWO;
      }
      if (ylb <= yub && _scanCardinalInRect<dx, 0>(rp, ylb_x, ylb, ylb_g)) {
        ylb++;
        ylb_x += dx*dy;
        ylb_g += dy*warthog::DBL_ROOT_TWO;
      }
      if (ylb <= yub && _scanCardinalInRect<dx, 0>(rp, yub_x, yub, yub_g)) {
        yub--;
        yub_x -= dx*dy;
        yub_g -= dy*warthog::DBL_ROOT_TWO;
      }
      
      if (xlb <= xub) _pushIntervalAdj<0, dy>(invlH, rp, xlb, xub, dx>0?xlb: xub, dx>0?xlb_y: xub_y, min(xlb_g, xub_g));
      if (ylb <= yub) _pushIntervalAdj<dx, 0>(invlV, rp, ylb, yub, dy>0?ylb_x: yub_x, dy>0?ylb: yub, min(ylb_g, yub_g));
      return res;
    }

    template<int dx, int dy>
    inline void _scanDiag(int cx, int cy, cost_t cur_cost, ConvRect* rp) {
      { // if cannot make the first diagonal move
        uint32_t neis;
        map_->gmap->get_neighbours(map_->gmap->to_padded_id(cx, cy), (uint8_t*)&neis);
        if ((neis & jps::neis_const<dx, dy>()) != jps::neis_const<dx, dy>())
          return;
        // make the first diagonal move
        cx += dx, cy += dy;
        cur_cost += warthog::DBL_ROOT_TWO;
        rp = map_->get_rect(cx, cy);
      }

      // cases are same as in internalDiagScan
      int bX, bY, bD;
      while (rp != nullptr) {
        if (rp->rid == _goal_rid) {
          _internalJump(jps::v2d(dx, dy), rp, cur_cost, cx, cy);
          break;
        }
        if (!_scanThroughDiag<dx, dy>(cx, cy, cur_cost, rp, bX, bY, bD)) break;
        bX = map_->get_label(cx+dx, cy);
        bY = map_->get_label(cx, cy+dy);
        bD = map_->get_label(cx+dx, cy+dy);
        if (bD && bX && bY) {
          cx += dx;
          cy += dy;
          cur_cost += warthog::DBL_ROOT_TWO;
          rp = map_->get_rid(cx, cy) != -1? map_->get_rect(cx, cy): nullptr;
        } else break;
      }
      _pushInvB2F_Y<dy>(invlH);
      _pushInvB2F_X<dx>(invlV);
    }

    inline void reset() {
      jpts_.clear();
      costs_.clear();
    }

    inline vector<uint32_t>& get_jpts() { return jpts_; }
    inline vector<cost_t>& get_costs() { return costs_; }
    bool verbose = false;
    inline void set_search_number(uint32_t snumber) {
      search_number = snumber;
    }
    inline void set_parent_gvalue(cost_t g) { parent_g = g; } 

  private:
    int gid_ = -1;
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
    uint32_t search_number=0;
    cost_t parent_g=0;
    struct Interval {
      int lb, ub, px, py;
      cost_t pcost; // gvalue of parent
      ConvRect* r;
      void print(ostream& out) {
        out << " lb: " << lb << " ub: " << ub
            << " parent (" << px << ", " << py << ")"
            << " pcost: " << pcost
            << " rid: " << r->rid << "(" << gen_id_label(r->rid) << ")";
      }
    };
    queue<Interval> invlH, invlV;

    struct DirInfo {
      jps::direction fromd;
      cost_t gval;
      uint32_t snumber; // search id
    };
    vector<DirInfo> dinfo;

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
      bool tlabel = jpl->get_label(bx, by);
      jpl->set_label(bx, by, false);
      _jptc.clear();
      _cstc.clear();
      jpl->jump(d, map_->gmap->to_padded_id(curx, cury), padded_gid_,
          _jptc, _cstc);
      if (tlabel)
        jpl->set_label(bx, by, tlabel);
      return !_cstc.empty();
    }

    template<epos cure, epos nxte>
    inline int bsearch_adj_rect(ConvRect* curr, int lb, int ub) {
      int s = 0, t = curr->adj[cure].size()-1, l=0, r=0;
      int best = t+1;
      while (s<=t) {
        int m = (s+t)>>1;
        map_->rects[(curr->adj[cure][m])-1].get_range(nxte, l, r);
        if (r < lb) s=m+1;
        else {
          best = m;
          t = m-1;
        }
      }
      return best;
    }

    // Precondition:
    // interval [lb, ub] is on F border of curr
    // push [lb, ub] in forward direction,
    // and add adjacent intervals to FIFO
    //
    // Postcondition: [lb, ub] is in the nxtr if exist,
    template<int dx, int dy>
    inline void _pushIntervalAdj(queue<Interval>& invs, ConvRect* curr, 
        int lb, int ub, int px, int py, cost_t cur_cost) {
      constexpr epos cure = convrectscan::R2E<dx, dy, rdir::F>();
      constexpr epos nxte = convrectscan::R2E<dx, dy, rdir::B>();

      debug_push_intervalAdj(lb, ub, px, py, cur_cost, curr);
      // int nxtx, nxty;
      int sidx = bsearch_adj_rect<cure, nxte>(curr, lb, ub);
      for (int i=sidx; i<(int)curr->adj[cure].size(); i++) {
        int rid = curr->adj[cure][i];
        ConvRect* r = &(map_->rects[rid-1]);

        int rL, rU; 
        // int nxtL, nxtU, nxtAx;
        rL = r->get_lb(nxte);
        rU = r->get_ub(nxte);
        if (rL > ub) break;
        rL = max(rL, lb); 
        rU = min(rU, ub);

        if (rid == _goal_rid) {
          int ax = r->axis(nxte);
          int x, y;
          if (dx == 0) {
            y = ax;
            if (rL <= _gx && _gx <= rU) x=_gx;
            else if (rL > _gx) x=rL;
            else x=rU;
          }
          else {
            x = ax;
            if (rL <= _gy && _gy <= rU) y=_gy;
            else if (rL > _gy) y=rL;
            else y=rU;
          }
          _internalJump(jps::v2d(dx, dy), r, cur_cost + octile_dist(px, py, x, y), x, y);
          // jpts_.push_back(map_->to_id(x, y));
          // costs_.push_back(cur_cost + octile_dist(px ,py, x, y));
          // a shorter path may pass another interval,
          // so we should continue instead of break
          continue;
        }
        if (rL <= rU) {
          debug_push_interval(rL, rU, px, py, cur_cost, r);
          invs.push({rL, rU, px, py, cur_cost, r});
        }
      }
    }

    template<int dx>
    inline void _pushInvB2F_X(queue<Interval>& invs) {
      constexpr epos cure = convrectscan::R2E<dx, 0, rdir::B>();
      constexpr epos nxte = convrectscan::R2E<dx, 0, rdir::F>();
      int cx;
      cost_t curcost;
      /*
       *    ???@<<<<<<<<<<  c.lb
       *    @            <
       *    + open lb    <
       *    |            <
       *    |            <
       *    + open ub    <
       *    @            <
       *    @<<<<<<<<<<<<<  c.ub
       *
       *
       */
      while (!invs.empty()) {
        Interval c = invs.front(); invs.pop();
        debug_pop_interval(c);
        jps::scan_cnt++;
        cx = c.r->axis(cure);
        if (c.lb <= c.ub ) { // when lb==ylb, we need to explicitly check jump poin
          int &cy = c.lb;
          curcost = c.pcost + octile_dist(c.px, c.py, cx, cy);
          if (cy == c.r->get_lb(cure) && _scanCardinalInRect<dx, 0>(c.r, cx, cy, curcost))
            cy++;
        }
        if (c.lb <= c.ub) { // when ub==ylu
          int& cy = c.ub;
          curcost = c.pcost + octile_dist(c.px, c.py, cx, cy);
          if (cy == c.r->get_ub(cure) && _scanCardinalInRect<dx, 0>(c.r, cx, cy, curcost))
            cy--;
        }
        // lb must be in open interval of the "forward" border
        c.lb = max(c.lb, c.r->get_lb(nxte));
        // ub must be in open interval of the "forward" border
        c.ub = min(c.ub, c.r->get_ub(nxte));

        cx = c.r->axis(nxte);
        if (c.lb <= c.ub && map_->is_nxtmove_jp<dx, 0>(cx, c.lb)) {
          curcost = c.pcost + octile_dist(c.px, c.py, cx, c.lb);
          add_jump_point(cx+dx, c.lb, curcost+1, jps::v2d(dx, 0));
          c.lb++;
        }
        if (c.lb <= c.ub && map_->is_nxtmove_jp<dx, 0>(cx, c.ub)) {
          curcost = c.pcost + octile_dist(c.px, c.py, cx, c.ub);
          add_jump_point(cx+dx, c.ub, curcost+1, jps::v2d(dx, 0));
          c.ub--;
        }
        if (c.lb <= c.ub)
          _pushIntervalAdj<dx, 0>(invs, c.r, c.lb, c.ub, c.px, c.py, c.pcost);
      }
    }

    template<int dy>
    inline void _pushInvB2F_Y(queue<Interval>& invs) {
      // similar to _pushInvB2F_X
      constexpr epos cure = convrectscan::R2E<0, dy, rdir::B>();
      constexpr epos nxte = convrectscan::R2E<0, dy, rdir::F>();
      int cy;
      cost_t curcost;
      while (!invs.empty()) {
        Interval c = invs.front(); invs.pop();
        debug_pop_interval(c);
        jps::scan_cnt++;
        cy = c.r->axis(cure);
        if (c.lb <= c.ub) { // when lb==yl, we need to explicitly check jump poin
          int &cx = c.lb;
          curcost = c.pcost + octile_dist(c.px, c.py, cx, cy);
          if (cx == c.r->get_lb(cure) && _scanCardinalInRect<0, dy>(c.r, cx, cy, curcost))
            cx++;
        }
        if (c.lb <= c.ub) { // when ub==yu
          int& cx = c.ub;
          curcost = c.pcost + octile_dist(c.px, c.py, cx, cy);
          if (cx == c.r->get_ub(cure) && _scanCardinalInRect<0, dy>(c.r, cx, cy, curcost))
            cx--;
        }
        // lb must be in open interval of the "forward" border
        c.lb = max(c.lb, c.r->get_lb(nxte));
        // ub must be in open interval of the "forward" border
        c.ub = min(c.ub, c.r->get_ub(nxte));

        cy = c.r->axis(nxte);
        if (c.lb <= c.ub && map_->is_nxtmove_jp<0, dy>(c.lb, cy)) {
          curcost = c.pcost + octile_dist(c.px, c.py, c.lb, cy);
          add_jump_point(c.lb, cy+dy, curcost+1, jps::v2d(0, dy));
          c.lb++;
        }
        if (c.lb <= c.ub && map_->is_nxtmove_jp<0, dy>(c.ub, cy)) {
          curcost = c.pcost + octile_dist(c.px, c.py, c.ub, cy);
          add_jump_point(c.ub, cy+dy, curcost+1, jps::v2d(0, dy));
          c.ub--;
        }
        if (c.lb <= c.ub)
          _pushIntervalAdj<0, dy>(invs, c.r, c.lb, c.ub, c.px, c.py, c.pcost);
      }
    }

    template<int dx, int dy, bool inplace=false>
    inline bool _scanCardinalInRect(
        ConvRect* rp, int& cx, int& cy, cost_t& cur_cost) {
      /* precondtion: <dx, dy> is a cardinal move
       * e.g. when scan in NORTH
       * case 0: .x.  -> move forward
       * case 1: @x.  -> blocked left, call _scanOnBound
       * case 2: .x@  -> blocked right, call _scanOnBound
       * case 3: x??  -> on left border, call _scanLR
       * case 4: ??x  -> on right border, call _scanLR
       * return true if found jump point or dead end
       */

      int d2F;
      bool onL, onR;
      int nx, ny;
      if (rp->rid == _goal_rid) {
        _internalJump(jps::v2d(dx, dy), rp, cur_cost, cx, cy);
        return true;
      }
      // case 1 or case 2
      if (blockedLR<dx, dy>(cx, cy, rp)) {
        // try to find an internal jpoint
        if (_cardinal_block_scan(dx, dy, cx, cy, rp)) {
          cost_t c = _cstc.back();
          nx = cx + dx * (int)c;
          ny = cy + dy * (int)c;
          add_jump_point(nx, ny, cur_cost+c, jps::v2d(dx, dy));
          if (inplace) { cx = nx, cy = ny; }
          return true;
        }
      }
      else { // case 3 or case 4
        onL = rp->onLR(rdir::L, dx, dy, cx, cy);
        onR = rp->onLR(rdir::R, dx, dy, cx, cy);
        if ((onL || onR) &&
            _scanLR<dx, dy>(cx, cy, cur_cost, onL?rdir::L:rdir::R, rp))
          return true;
      }
      // no jpoint found and cannot move out rect
      if (rp->is_blocked<dx, dy>(cx, cy)) return true;
      // move forward
      d2F = rp->disF(dx, dy, cx, cy);
      nx = cx + dx*d2F, ny = cy + dy*d2F;
      cost_t nxt_cost = cur_cost + d2F;
      int rid = map_->get_rid(nx+dx, ny+dy);
      if (rid == -1) // no adjacent rect, dead end
        return true;
      assert(rid > 0); // convex rect rid > 0
      // check whether (cx+dx, cy+dy) is a jump point
      if (map_->is_nxtmove_jp<dx, dy>(nx, ny)) {
        add_jump_point(nx+dx, ny+dy, nxt_cost+1, jps::v2d(dx, dy));
        return true;
      }
      if (inplace) {
        cx = nx, cy = ny, cur_cost = nxt_cost;
      }
      return false;
    }

    inline void add_jump_point(int x, int y, cost_t cost, jps::direction d) {
      int cid = map_->to_id(x, y);
      // there is a better jump point exists
      if (dinfo[cid].snumber == this->search_number && 
          dinfo[cid].gval <= cost + parent_g)
        return;
      dinfo[cid] = {d, cost+parent_g, search_number};
      debug_found_jpoint(x, y, cost, d);
      jpts_.push_back(cid);
      costs_.push_back(cost);
    }

    inline void debug_start_jump(int cx, int cy, jps::direction d) {
      if (verbose) {
        cerr << "Start Jump at (" << cx << ", " << cy <<") " << jps::d2s(d) << endl;
      }
    }

    inline void debug_found_jpoint_in_goal_rect(int x, int y, cost_t gval) {
      if (verbose) {
        cerr << "Found goal jump point at (" << x << ", " << y << ")"
             << " id: " << map_->to_id(x, y) << " gval:" << gval
             << " rid: " << map_->get_rid(x, y) << endl;
      }
    }

    inline void debug_found_jpoint(int x, int y, cost_t gval, jps::direction d) {
      if (verbose) {
        cerr << "Found jump point at (" << x << ", " << y << ")"
             << " id: " << map_->to_id(x, y) 
             << " from dir: " << d2s(d)
             << " cost:" << gval+parent_g
             << " parent_g: " << parent_g
             << " rid: " << map_->get_rid(x, y)
             << "(" << gen_id_label(map_->get_rid(x, y)) << ")"
             << endl;
      }
    }

    inline void debug_found_jpoint(int id, cost_t gval, jps::direction d) {
      if (verbose) {
        int x, y;
        map_->to_xy(id, x, y);
        debug_found_jpoint(x, y, gval, d);
      }
    }

    inline void debug_pop_interval(Interval inv) {
      if (verbose) {
        cerr << "Pop interval: ";
        inv.print(cerr);
        cerr << endl;
      }
    }

    inline void debug_push_intervalAdj(int lb, int ub, int px, int py, cost_t c, ConvRect* r) {
      if (verbose) {
        cerr << "Push interval to adjacent: ";
        Interval t{lb, ub, px, py, c, r};
        t.print(cerr);
        cerr << endl;
      }
    }

    inline void debug_push_interval(int lb, int ub, int px, int py, cost_t c, ConvRect* r) {
      if (verbose) {
        cerr << "Push interval to queue: ";
        Interval t{lb, ub, px, py, c, r};
        t.print(cerr);
        cerr << endl;
      }
    }
}; 

}}
