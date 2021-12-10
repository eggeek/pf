#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <cassert>
#include <iomanip>
#include <queue>
#include <algorithm>
#include "online_jump_point_locator2.h"


namespace convrectgen {
using namespace std;
using namespace warthog;
using namespace warthog::jps;

extern vector<int> _idmap;
extern int mapw, maph, cntr;
extern bool verbose;

inline void init(int w, int h, bool v) {
  mapw = w, maph = h, verbose = v;
}

inline long long get_heuristic(int width, int height)
{
    long long out = min(width, height);
    out *= width;
    out *= height;
    return out;
}

struct Point {
  int x, y;
  bool operator<(const Point& rhs) const {
    if (rhs.x == x) return y<rhs.y;
    return x<rhs.x;
  }
  bool operator==(const Point& rhs) const {
    return rhs.x == x && rhs.y == y;
  }
  bool operator!=(const Point& rhs) const {
    return !(*this == rhs);
  }
};

struct SearchNode
{
    // bot-right corner of rect
    int x, y, w, h;

    SearchNode(int _x, int _y, int _w, int _h): x(_x), y(_y), w(_w), h(_h) { }

    long long hvalue() const {
      return get_heuristic(h, w);
    }

    // Comparison.
    // Always take the one with highest h.
    bool operator<(const SearchNode& other) const
    {
        return hvalue() < other.hvalue();
    }

    bool operator>(const SearchNode& other) const
    {
        return hvalue() > other.hvalue();
    }
};

struct FinalConvexRect {
  int xl, yl, xu, yu;
  int sx, sy, rid;
  int lb[4], ub[4];

  FinalConvexRect() {};
  FinalConvexRect(SearchNode s, int _rid) {
    xl = s.x-s.w+1, xu = s.x,
    yl = s.y-s.h+1, yu = s.y;
    sx = s.x, sy = s.y;
    lb[0] = lb[2] = xl;
    lb[1] = lb[3] = yl;
    ub[0] = ub[2] = xu;
    ub[1] = ub[3] = yu;
    rid = _rid;
  }
  FinalConvexRect(int x, int y, int h, int w, int _rid) {
    xl = x-w+1, xu = x,
    yl = y-h+1, yu = y;
    sx = x, sy = y;
    lb[0] = lb[2] = xl;
    lb[1] = lb[3] = yl;
    ub[0] = ub[2] = xu;
    ub[1] = ub[3] = yu;
    rid = _rid;
  }
  void calc_bound(vector<int>& idmap, int mapw) {
    lb[0] = lb[2] = xl, lb[1] = lb[3] = yl;
    ub[0] = ub[2] = xu, ub[1] = ub[3] = yu;
    while (lb[0]<=ub[0] && idmap[yl*mapw+lb[0]] != rid) lb[0]++;
    while (lb[1]<=ub[1] && idmap[lb[1]*mapw+xu] != rid) lb[1]++;
    while (lb[2]<=ub[2] && idmap[yu*mapw+lb[2]] != rid) lb[2]++;
    while (lb[3]<=ub[3] && idmap[lb[3]*mapw+xl] != rid) lb[3]++;

    while (ub[0]>=lb[0] && idmap[yl*mapw+ub[0]] != rid) ub[0]--;
    while (ub[1]>=lb[1] && idmap[ub[1]*mapw+xu] != rid) ub[1]--;
    while (ub[2]>=lb[2] && idmap[yu*mapw+ub[2]] != rid) ub[2]--;
    while (ub[3]>=lb[3] && idmap[ub[3]*mapw+xl] != rid) ub[3]--;
  }
};

extern vector<FinalConvexRect> finals;

inline void print_cell(int x, int y, vector<int>& idmap, warthog::gridmap* gmap, string cs) {
  if (x<0 || x>=(int)gmap->header_width() || y<0 || y>=(int)gmap->header_height()) {
    printf("%c", cs[1]);
    return;
  }
  int id = y * gmap->header_width() + x;
  assert(id >= 0);
  assert(id < (int)idmap.size());
  if (idmap[id] == -1) printf("%c", cs[1]);
  else {
    if (idmap[id]) printf("%d", idmap[id] % 10);
    else printf("%c", cs[0]);
  }
}

inline void print_rect_map(int xl, int yl, int xu, int yu, int rid, vector<int>& idmap, warthog::gridmap* gmap) {
  int pad = 2;
  printf("xl: %d yl: %d xu: %d yu: %d rid: %d pad: %d\n", xl, yl, xu, yu, rid, pad);
  // print above pad row
  for (int y=yl-pad; y<yl; y++) {
    for (int x=xl-pad; x<=xu+pad; x++) {
      print_cell(x, y, idmap, gmap, ".@");
    }
    printf("\n");
  }
  // print NORTH border
  // print left pad columns
  for (int x=xl-pad; x<xl; x++) {
    print_cell(x, yl, idmap, gmap, ".@");
  }
  for (int x=xl; x<=xu; x++) {
    print_cell(x, yl, idmap, gmap, "-*");
  }
  // print right pad columns
  for (int x=xu+1; x<=xu+pad; x++) {
    print_cell(x, yl, idmap, gmap, ".@");
  }

  printf("\n");
  // print Mid
  for (int y=yl+1; y<yu; y++) {
    
    // print left pad column
    for (int x=xl-pad; x<xl; x++) {
      print_cell(x, y, idmap, gmap, ".@");
    }
    print_cell(xl, y, idmap, gmap, "|*");
    for (int x=xl+1; x<xu; x++) {
      print_cell(x, y, idmap, gmap, ".@");
    }
    print_cell(xu, y, idmap, gmap, "|*");
    // print right pad column
    for (int x=xu+1; x<=xu+pad; x++) {
      print_cell(x, y, idmap, gmap, ".@");
    }
    printf("\n");
  }
  // print SOUTH border
  // print left pad columns
  for (int x=xl-pad; x<xl && yu>yl; x++) {
    print_cell(x, yu, idmap, gmap, ".@");
  }
  for (int x=xl; x<=xu && yu>yl; x++) {
    print_cell(x, yu, idmap, gmap, "-*");
  }
  // print right pad columns
  for (int x=xu+1; x<=xu+pad && yu>yl; x++) {
    print_cell(x, yu, idmap, gmap, ".@");
  }
  if (yu > yl) printf("\n");

  // print below pad row
  for (int y=yu+1; y<=yu+pad; y++) {
    for (int x=xl-pad; x<=xu+pad; x++) {
      print_cell(x, y, idmap, gmap, ".@");
    }
    printf("\n");
  }
}

inline int expand_border_x(int x, int yl, int yu, int& bL, int& bU, int rid,
    direction d, vector<int>& idmap, online_jump_point_locator2* jpl) {
  int i=bL, j=bU;
  int dx, dy;
  d2v(d, dx, dy);
  // hit the border
  if (x+dx>=mapw || x+dx<0) return 0;
  // find the blocked segments on (x+dx, [bL, bU])
  // [bL, i-1] and [j+1, bU] must be blocked
  // all cells in [i, j] must be empty and not taken by other rects
  while (i<=bU) {
    int id = i*mapw+(x+dx);
    // check the tile@(x+dx, i)
    // the tile is empty, we found the start of the empty segment
    if (idmap[id] == 0) {
      // but the tile above@(x+dx, i-1) must be blocked
      // otherwise it is not convex
      if (i>yl && idmap[(i-1)*mapw+(x+dx)] != -1) return 0;
      break;
    }
    // cannot expand, the tile is taken
    else if (idmap[id] > 0) return 0;
    else {
      assert(idmap[id] == -1);
      i++;
    }
  }
  while (j>=bL) {
    int id = j*mapw+(x+dx);
    if (idmap[id] == 0) {
      if (j<yu && idmap[(j+1)*mapw+(x+dx)] != -1) return 0;
      break;
    }
    else if (idmap[id] > 0) return 0;
    else {
      assert(idmap[id] == -1);
      j--;
    }
  }
  // all tiles in [i, j] must be 0 
  for (int k=i; k<=j; k++) 
  if (idmap[k*mapw+(x+dx)] != 0) return 0;
  // mark by rid
  for (int k=i; k<=j; k++) {
    idmap[k*mapw+(x+dx)] = rid;
    jpl->set_label(x+dx, k, false);
  }
  // update open border
  // bL > bU means the border is fully closed
  bL = i, bU = j;
  // postcondition: if bL<=bU, (x+dx, [bL, bU]) are traversable 
  return 1;
}

inline int expand_border_y(int y, int xl, int xu, int& bL, int& bU, int rid,
    direction d, vector<int>& idmap, online_jump_point_locator2* jpl) {
  int i=bL, j=bU;
  int dx, dy;
  d2v(d, dx, dy);
  // hit the border
  if (y+dy<0 || y+dy>=maph) return 0;
  // find the blocked segments on ([bL, bU], y+dy);
  // all cells in [i, j] must be empty and not taken by other rects
  while (i<=bU) {
    int id = (y+dy)*mapw + i;
    // check the tile@(i, y+dy)
    // the tile is empty, we found the start of empty segment
    if (idmap[id] == 0) {
      // but the left tile@(y+dy, i-1) must be blocked
      if (i>xl && idmap[(y+dy)*mapw + (i-1)] != -1) return 0;
      break;
    }
    // cannot expand, the tile is taken
    else if (idmap[id] > 0) return 0;
    else {
      assert(idmap[id] == -1);
      i++;
    }
  }
  while (j>=bL) {
    int id = (y+dy)*mapw + j;
    if (idmap[id] == 0) {
      if (j<xu && idmap[(y+dy)*mapw + (j+1)] != -1) return 0;
      break;
    }
    else if (idmap[id] > 0) return 0;
    else {
      assert(idmap[id] == -1);
      j--;
    }
  }
  // all tiles in [i, j] must be 0;
  for (int k=i; k<=j; k++) 
  if (idmap[(y+dy)*mapw + k] != 0) return 0;
  // mark by rid, and update label
  for (int k=i; k<=j; k++) {
    idmap[(y+dy)*mapw + k] = rid;
    jpl->set_label(k, y+dy, false);
  }
  // update open border
  // bL > bU means the border is fully closed
  bL = i, bU = j;
  // postcondition: if bL<=bU, ([bL, bU], y+dy) are traversable
  return 1;
}

inline int check_border_x(int xl, int yl, int xu, int yu, Point pa, Point pb, int dx, int dy, int rid, int mapw, vector<int>& idmap) {
  // check border from in dx direction
  // return the last non-decreasing position
  int curx = pa.x, cury = pa.y;
  while (curx != pb.x+dx) {
    int cid = cury * mapw + curx;
    // (l, cury) is an obstacle means a decreasing position, stop
    if (idmap[cid] == -1) break;
    // (l, cury) is an traversable tile, then must belongs to rid
    // assert(idmap[cid] == rid);
    if (idmap[cid] != rid) return -2;
    int dh = 0;
    // keep moving in dx@(l, cury);
    while ((cury+(dh+1)*dy)>=yl &&  // must be in the rectangle
           (cury+(dh+1)*dy)<=yu && 
           idmap[(cury+(dh+1)*dy)*mapw+curx]==rid) {
      dh++; // we can move on step
      // then the (l-dx, cury+dh*dy) position must be an obstacle
      if (xl<=curx-dx && curx-dx<=xu) {
        if (idmap[(cury+dh*dy)*mapw + (curx-dx)] != -1) return -2;
      }
    }
    cury += dh*dy;
    curx += dx;
  }
  // postcondition: 
  // * l==pb.x+dx if the entire border is non-decreasing
  // * otherwise (l, cury) must be an obstacle
  return curx;
}

inline int check_border_y(int xl, int yl, int xu, int yu, Point pa, Point pb, int dx, int dy, int rid, int mapw, vector<int>& idmap) {
  // similar to check_border_x
  int cury = pa.y, curx = pa.x;
  while (cury != pb.y+dy) {
    int cid = cury * mapw + curx;
    if (idmap[cid] == -1) break;
    if (idmap[cid] != rid) return -2;
    int dw = 0;
    while ((curx+(dw+1)*dx)>=xl &&
           (curx+(dw+1)*dx)<=xu &&
           idmap[cury*mapw+curx+(dw+1)*dx]==rid) {
      dw++;
      if (yl<=cury-dy && cury-dy<=yu) {
        if (idmap[(cury-dy)*mapw+(curx+dw*dx)] != -1) return -2;
      }
    }
    cury += dy;
    curx += dw*dx;
  }
  return cury;
}

inline bool try_expand_naive(int xl, int yl, int xu, int yu, int sx, int sy, 
    int rid, set<Point>& newp, 
    vector<int>& idmap, online_jump_point_locator2* jpl) {
  queue<Point> q;
  set<Point> vis;
  Point tl, tr, bl, br, rt, rb, lt, lb;
  int w = jpl->get_map()->header_width();
  tl = tr = bl = br = rt = rb = lt = lb = Point{sx, sy};
  q.push({sx, sy});
  vis.insert({sx, sy});
  while (!q.empty()) {
    Point c = q.front(); q.pop();
    if ((c.x == lt.x && c.y < lt.y) || c.x < lt.x) lt = c;
    if ((c.x == lb.x && c.y > lb.y) || c.x < lb.x) lb = c;
    if ((c.x == rt.x && c.y < rt.y) || c.x > rt.x) rt = c;
    if ((c.x == rb.x && c.y > rb.y) || c.x > rb.x) rb = c;

    if ((c.y == tl.y && c.x < tl.x) || c.y < tl.y) tl = c;
    if ((c.y == tr.y && c.x > tr.x) || c.y < tr.y) tr = c;
    if ((c.y == bl.y && c.x < bl.x) || c.y > bl.y) bl = c;
    if ((c.y == br.y && c.x > br.x) || c.y > br.y) br = c;

    for (int i=0; i<4; i++) {
      Point nxt;
      nxt.x = c.x + warthog::dx[i];
      nxt.y = c.y + warthog::dy[i];
      // not in the rectangle
      if (nxt.x < xl || nxt.x > xu) continue;
      if (nxt.y < yl || nxt.y > yu) continue;
      // not traversable
      if (idmap[nxt.y*w+nxt.x] == -1) continue;
      // has been visited
      if (vis.find(nxt) != vis.end()) continue;
      // (x, y) belongs to other rect
      if (idmap[nxt.y*w+nxt.x] > 0 && idmap[nxt.y*w+nxt.x] != rid) return false;
      // it is illegal if (x, y) is marked by other rectangle
      vis.insert(nxt);
      q.push(nxt);
      if (idmap[nxt.y*w+nxt.x] == 0) {
        idmap[nxt.y*w+nxt.x] = rid;
        newp.insert(nxt);
      }
    }
  }
  if (newp.size() == 0) return false;
  int l, r;
  // check top border from left
  l = check_border_x(xl, yl, xu, yu, lt, rt, 1, -1, rid, w, idmap);
  // check top border from right
  r = check_border_x(xl, yl, xu, yu, rt, lt, -1, -1, rid, w, idmap);
  if (l==-2 || r==-2 || l <= r) return false;

  // check bot border from left
  l = check_border_x(xl, yl, xu, yu, lb, rb, 1, 1, rid, w, idmap);
  // check bot border from right
  r = check_border_x(xl, yl, xu, yu, rb, lb, -1, 1, rid, w, idmap);
  if (l==-2 || r==-2 || l <= r) return false;

  // check left border from top 
  l = check_border_y(xl, yl, xu, yu, tl, bl, -1, 1, rid, w, idmap);
  // check left border from bot 
  r = check_border_y(xl, yl, xu, yu, bl, tl, -1, -1, rid, w, idmap);
  if (l==-2 || r==-2 || l <= r) return false;
  
  // check right border from top
  l = check_border_y(xl, yl, xu, yu, tr, br, 1, 1, rid, w, idmap);
  // check right border from bot
  r = check_border_y(xl, yl, xu, yu, br, tr, 1, -1, rid, w, idmap);
  if (l==-2 || r==-2 || l <= r) return false;
  return true;
}

inline bool expand_rect_naive(int& xl, int& yl, int& xu, int& yu, int rid,
   int lb[4], int ub[4], vector<int>& idmap, online_jump_point_locator2* jpl) {
  /*
   * precondition: [xl, yl, xu, yu] is a purely empty rectangle
   * idmap and jpl are marked by the input rectangle
   */
  if (verbose) {
    printf("before expand:\n");
    print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
  }
  mapw = jpl->get_map()->header_width();
  maph = jpl->get_map()->header_height();
  int sx = xu, sy = yu;
  bool res = false;
  int mask = (1<<4)-1;
  set<Point> newp;
  auto mark = [&](int i) {
    for (auto& p: newp) {
      assert(idmap[p.y*mapw+p.x]==rid);
      jpl->set_label(p.x, p.y, false);
    }
    mask |= 1<<i;
    res = true;
    newp.clear();
  };
  auto unmark = [&](int i) {
    for (auto& p: newp) {
      assert(idmap[p.y*mapw+p.x]==rid);
      idmap[p.y*mapw+p.x] = 0;
    }
    mask &= ~(1<<i);
    newp.clear();
  };
  while (mask) {
    // N
    if (yl-1>=0 && try_expand_naive(xl, yl-1, xu, yu, sx, sy, rid, newp, idmap, jpl)) {
      mark(0);
      yl--;
      if (verbose) {
        printf("after expand north:\n");
        print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
      }
    }
    else unmark(0);
    // E
    if (xu+1<mapw && try_expand_naive(xl, yl, xu+1, yu, sx, sy, rid, newp, idmap, jpl)) {
      mark(1);
      xu++;
      if (verbose) {
        printf("after expand east:\n");
        print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
      }
    }
    else unmark(1);
    // S
    if (yu+1<maph && try_expand_naive(xl, yl, xu, yu+1, sx, sy, rid, newp, idmap, jpl)) {
      mark(2);
      yu++;
      if (verbose) {
        printf("after expand south:\n");
        print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
      }
    }
    else unmark(2);
    // W
    if (xl-1>=0 && try_expand_naive(xl-1, yl, xu, yu, sx, sy, rid, newp, idmap, jpl)) {
      mark(3);
      xl--;
      if (verbose) {
        printf("after expand west:\n");
        print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
      }
    }
    else unmark(3);
  }
  // update lb, ub
  lb[0] = lb[2] = xl, lb[1] = lb[3] = yl;
  ub[0] = ub[2] = xu, ub[1] = ub[3] = yu;
  while (lb[0]<=ub[0] && idmap[yl*mapw+lb[0]] != rid) lb[0]++;
  while (lb[1]<=ub[1] && idmap[lb[1]*mapw+xu] != rid) lb[1]++;
  while (lb[2]<=ub[2] && idmap[yu*mapw+lb[2]] != rid) lb[2]++;
  while (lb[3]<=ub[3] && idmap[lb[3]*mapw+xl] != rid) lb[3]++;

  while (ub[0]>=lb[0] && idmap[yl*mapw+ub[0]] != rid) ub[0]--;
  while (ub[1]>=lb[1] && idmap[ub[1]*mapw+xu] != rid) ub[1]--;
  while (ub[2]>=lb[2] && idmap[yu*mapw+ub[2]] != rid) ub[2]--;
  while (ub[3]>=lb[3] && idmap[ub[3]*mapw+xl] != rid) ub[3]--;
  return res;
}

inline bool expand_rect(int& xl, int& yl, int& xu, int& yu, int rid,
    int lb[4], int ub[4], vector<int>& idmap, online_jump_point_locator2* jpl) {

  /*      
   *   xl l0  r0  xu
   * yl +-*---*--+
   * l3 *        * l1
   *    |        |
   * r3 *        * r1
   * yu +-*---*--+
   *     l2   r2
   *
   *  where l[i]/r[i] is the first/last traversable tile on the border
   *  when l[i] == xl/yl or r[i] == xu/yu, changing xl/yl, xu/yu must also update
   *  l[i]/r[i]
   */

  if (verbose) {
    printf("before expand:\n");
    print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
  }
  int d, mask = (1<<4)-1;
  bool res = false;
  while (mask) {
    d = 0;
    // expand N, only if last time expansion success
    if (lb[0] <= ub[0] && (mask & (1<<0))) {
      d = expand_border_y(yl, xl, xu, lb[0], ub[0], rid, NORTH, idmap, jpl);
      res |= d;
      if (d) {
        if (lb[1]==yl && idmap[(yl-d)*mapw+xu] == rid) lb[1] -= d;
        if (lb[3]==yl && idmap[(yl-d)*mapw+xl] == rid) lb[3] -= d;
        yl -= d;
        if (verbose) {
          printf("after expand north:\n");
          print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
        }
      }
    }
    if (!d) mask &= ~(1 << 0); // clear 0 bit if North expansion failed

    // expand E, only if last time expansion success
    d = 0;
    if (lb[1] <= ub[1] && (mask & (1<<1))) {
      d = expand_border_x(xu, yl, yu, lb[1], ub[1], rid, EAST, idmap, jpl);
      res |= d;
      if (d) {
        if (ub[0]==xu && idmap[yl*mapw+(xu+d)]==rid) ub[0] += d;
        if (ub[2]==xu && idmap[yu*mapw+(xu+d)]==rid) ub[2] += d;
        xu += d;
        if (verbose) {
          printf("after expand east:\n");
          print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
        }
      }
    }
    if (!d) mask &= ~(1 << 1); // clear 0 bit if East expansion failed

    // expand S, only if last time expansion success
    d = 0;
    if (lb[2] <= ub[2] && (mask & (1<<2))) {
      d = expand_border_y(yu, xl, xu, lb[2], ub[2], rid, SOUTH, idmap, jpl);
      res |= d;
      if (d) {
        if (ub[1]==yu && idmap[(yu+d)*mapw+xu]==rid) ub[1] += d;
        if (ub[3]==yu && idmap[(yu+d)*mapw+xl]==rid) ub[3] += d;
        yu += d;
        if (verbose) {
          printf("after expand south:\n");
          print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
        }
      }
    }
    if (!d) mask &= ~(1 << 2); // clear 2 bit if South expansion failed

    // expand W, only if last time expansion success
    d = 0;
    if (lb[3] <= ub[3] && (mask & (1<<3))) {
      d = expand_border_x(xl, yl, yu, lb[3], ub[3], rid, WEST, idmap, jpl);
      res |= d;
      if (d) {
        if (lb[0]==xl && idmap[yl*mapw+(xl-d)]==rid) lb[0] -= d;
        if (lb[2]==xl && idmap[yu*mapw+(xl-d)]==rid) lb[2] -= d;
        xl -= d;
        if (verbose) {
          printf("after expand west:\n");
          print_rect_map(xl, yl, xu, yu, rid, idmap, jpl->get_map());
        }
      }
    }
    if (!d) mask &= ~(1 << 3); // clear 3 bit if West expansion failed
  }
  return res;
}

inline SearchNode get_best_rect(SearchNode cur, vector<int>& idmap, online_jump_point_locator2* jpl) {
  SearchNode res = {0, 0, 0, 0};
  // (x, y) is -1 or marked
  if (idmap[cur.y*mapw+cur.x] != 0) return res;
  int h = jpl->rayscan(NORTH, cur.x, cur.y) + 1;
  int w = jpl->rayscan(WEST, cur.x, cur.y) + 1;
  int curh = h, curw = w;
  for (int hi=1; hi<=h; hi++) {
    curw = min(curw, jpl->rayscan(WEST, cur.x, cur.y-hi+1)+1);
    long long hvalue = get_heuristic(curw, hi);
    if (hvalue > res.hvalue())
      res = {cur.x, cur.y, curw, hi};
  }
  for (int wi=1; wi<=w; wi++) {
    curh = min(curh, jpl->rayscan(NORTH, cur.x-wi+1, cur.y)+1);
    long long hvalue = get_heuristic(wi, curh);
    if (hvalue > res.hvalue())
      res = {cur.x, cur.y, wi, curh};
  }
  return res;
}


inline void make_rectangles(online_jump_point_locator2* jpl, vector<int>& idmap)
{
  priority_queue<SearchNode> pq;
  gridmap* gmap = jpl->get_map();
  mapw = gmap->header_width(), maph = gmap->header_height();
  cntr = 0;
  idmap.resize(mapw*maph);
  fill(idmap.begin(), idmap.end(), 0);
  finals.clear();
  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++) {
    if (!jpl->get_label(x, y)) idmap[y*mapw+x] = -1;
    else {
      int h = jpl->rayscan(NORTH, gmap->to_padded_id(x, y))+1;
      int w = jpl->rayscan(WEST, gmap->to_padded_id(x, y))+1;
      SearchNode r = get_best_rect({x, y, w, h}, idmap, jpl);
      pq.push(r);
    }
  } 
  while (!pq.empty()) {
    SearchNode node = pq.top(); pq.pop();
    if (verbose) {
      printf("when pop out:\n");
      print_rect_map(node.x-node.w+1, node.y-node.h+1, node.x, node.y, 0, idmap, jpl->get_map());
    }
    SearchNode r = get_best_rect(node, idmap, jpl);
    if (verbose) {
      printf("best rect from current:\n");
      print_rect_map(r.x-r.w+1, r.y-r.h+1, r.x, r.y, 0, idmap, jpl->get_map());
    }
    if (r.hvalue() != node.hvalue()) {
      if (r.hvalue() != 0) pq.push(r);
      continue;
    }
    // use node
    ++cntr;
    for (int y=r.y; y>=r.y-r.h+1; y--)
    for (int x=r.x; x>=r.x-r.w+1; x--) {
      idmap[y*mapw+x] = cntr;
      jpl->set_label(x, y, false);
    }
    // int xl = r.x-r.w+1, yl = r.y-r.h+1, xu=r.x, yu=r.y;
    // int openl[4], openr[4];
    FinalConvexRect fr(r, cntr);
    expand_rect_naive(fr.xl, fr.yl, fr.xu, fr.yu, fr.rid, fr.lb, fr.ub, idmap, jpl);
    finals.push_back(fr);
  }
}
}
