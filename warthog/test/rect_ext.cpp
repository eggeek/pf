#include <cstdio>
#define CATCH_CONFIG_RUNNER
#include <string>
#include <iostream>
#include <fstream>

#include "catch.hpp"
#include "rectmap.h"
#include "constants.h"
#include "rect_jump_point_locator.h"

using namespace warthog::rectscan;
using namespace warthog::jps;
using namespace std;
string infile, outfile;
bool verbose = false;
const vector<string> desc = {
  "NORTH", "SOUTH", "EAST", "WEST",
  "NORTHEAST", "NORTHWEST", "SOUTHEAST", "SOUTHWEST"
};

struct Interval {
  int lb, ub;
};

struct Point {
  int x, y;
};

TEST_CASE("rayscan") {
  vector<string> maps = {
    "../maps/dao/arena.map",
    "../maps/bgmaps/AR0042SR.map",
    "../maps/starcraft/CatwalkAlley.map",
    "../maps/starcraft/GreenerPastures.map",
    "../maps/iron/scene_sp_endmaps.map"
  };
  for (auto& m: maps) {
    warthog::gridmap* gmap = new warthog::gridmap(m.c_str());
    online_jump_point_locator2* jpl = new online_jump_point_locator2(gmap);
    cerr << "Map: " << m << endl;

    for (int d=0; d<8; d++)
    for (int x=0; x<(int)gmap->header_width(); x++)
    for (int y=0; y<(int)gmap->header_height(); y++) {

      uint32_t id = gmap->to_padded_id(x, y);
      if (!gmap->get_label(id)) continue;
      // cerr << "x: " << x << ", y: " << y << ", d: " << desc[d] << " " << d << endl;
      direction dir = (direction)(1<<d);
      int s0 = jpl->rayscan(dir, id);
      int s1 = jpl->naive_rayscan(dir, id);
      REQUIRE(s0 == s1);
    }

    delete jpl;
    delete gmap;
  }
}

int convex_delta(const vector<Interval>& h) {
  int resL = h[0].lb;
  int i=0, j=(int)h.size()-1;
  while (i+1<(int)h.size() && h[i+1].lb>=h[i].lb) {
    resL = min(h[i+1].lb, h[i].ub);
    if (h[i+1].lb > h[i].ub) {
      i++;
      break;
    }
    i++;
  }
  int resR = h.back().lb;
  while (j-1>=0 && h[j-1].lb>=h[j].lb) {
    resR = min(h[j-1].lb, h[j].ub);
    if (h[j-1].lb > h[j].ub) {
      j--;
      break;
    }
    j--;
  }
  int res = min(resL, resR);
  for (int k=i+1; k<=j-1; k++) {
    res = min(res, h[k].lb-1);
  }
  return res;
}

int _expand_rect_y(
    int y, int& xl, int& xu,
    int bL, int bU,
    warthog::jps::direction d,
    online_jump_point_locator2* jpl) {
  // number of horizontal obstacle at xl-1 and xu+1
  // the expand distance cannot be larger this
  int dL = INF, dU = INF, dx, dy;
  int maph = jpl->get_map()->header_height();
  d2v(d, dx, dy);
  // dont expand if on border of map
  if (y+dy < 0 || y+dy>=maph) return 0;
  if (xl>bL) {
    dL = jpl->tilecnt(d, jpl->get_map()->to_padded_id(xl-1, y))-1;
  }
  if (xu<bU) {
    dU = jpl->tilecnt(d, jpl->get_map()->to_padded_id(xu+1, y))-1;
  }
  // precondition:
  // [xl, xu] are still open -> no obstacle
  // rayscan returns number of moves to the nearest obstacle
  // h stores distance to first obstacle -> h=rayscan+1;
  vector<Interval> h;
  warthog::gridmap* map = jpl->get_map();
  for (int i=xl; i<=xu; i++) {
    Interval inv = {0, 0};
    inv.lb = jpl->tilecnt(d, map->to_padded_id(i, y));
    inv.ub = inv.lb;
    if (0<=y+inv.lb*dy&&y+inv.lb*dy<maph)
      inv.ub += jpl->tilecnt(d, map->to_padded_id(i, y+inv.lb*dy))-1;

    h.push_back(inv);
    REQUIRE(h.back().lb > 0);
  }
  int _xl = xl;
  int res = convex_delta(h);
  res = min(min(dL, dU), res);
  // update open intervals
  while (xl<=xu && h[xl-_xl].lb <= res) xl++;
  while (xu>=xl && h[xu-_xl].lb <= res) xu--;
  return res;
}

int _expand_rect_x(
    int x, int& yl, int& yu,
    int bL, int bU,
    warthog::jps::direction d,
    online_jump_point_locator2* jpl) {
  // number of vertical obstacle at yl-1 and yu+1
  // the expand distance cannot be larger than this
  int dL = INF, dU = INF, dx, dy;
  int mapw = jpl->get_map()->header_width();
  warthog::jps::d2v(d, dx, dy);
  // dont expand if on border of map
  if (x+dx < 0 || x+dx>=mapw) return 0;
  if (yl>bL) {
    dL = jpl->tilecnt(d, jpl->get_map()->to_padded_id(x, yl-1))-1;
  }
  if (yu<bU) {
    dU = jpl->tilecnt(d, jpl->get_map()->to_padded_id(x, yu+1))-1;
  }
  // same in _expand_rect_y
  vector<Interval> h;
  warthog::gridmap* map = jpl->get_map();
  for (int i=yl; i<=yu; i++) {
    Interval inv;
    inv.lb = jpl->tilecnt(d, map->to_padded_id(x, i));
    inv.ub = inv.lb;
    if (0<=x+dx*inv.lb && x+dx*inv.lb < mapw)
      inv.ub += jpl->tilecnt(d, map->to_padded_id(x+dx*inv.lb, i))-1;
    h.push_back(inv);
    REQUIRE(h.back().lb > 0);
  }
  int _yl = yl;
  int res = convex_delta(h);
  res = min(min(dL, dU), res);
  // update open intervals
  while (yl<=yu && h[yl-_yl].lb <= res) yl++;
  while (yu>=yl && h[yu-_yl].lb <= res) yu--;
  return res;
}

bool is_convex_obs_seq(vector<int> nums) {
  // nums must be either
  // * non-increasing;
  // * non-decrasing;
  // * non-increasing then non-decrasing
  int i=0;
  // remove non-increasing part
  while (i+1<(int)nums.size() && nums[i+1]<=nums[i]) i++;
  // check if rest of part is non-decrasing
  while (i+1<(int)nums.size()) {
    if (nums[i+1] < nums[i]) return false;
    i++;
  }
  return true;
}

bool is_convex(int xl, int yl, int xu, int yu, int sx, int sy, vector<bool>& f, online_jump_point_locator2* jpl) {
  // expanded rectangle: top-left (xl, yl) bottom-right (xu, yu)
  // seed position (sx, sy) a traversable tile of the original empty rectangle
  // the function check whether the traversable component is "convex" in the expanded rectangle by:
  // 0. flood fill from (sx, sy) to mark the current traversable component
  // 1. get the top border of component, is it a "convex sequence";
  // 2. get the bottom, is it a "convex sequence";
  // 3. get the left outline, ....
  // 4. get the right outline, ...
  warthog::gridmap* gmap = jpl->get_map();
  int h = gmap->header_height(), w = gmap->header_width();
  f = vector<bool>(h*w, false);
  queue<Point> q;
  Point tl, tr, bl, br;
  tl = tr = bl = br = Point{sx, sy};
  // flood fill
  q.push({sx, sy});
  while (!q.empty()) {
    Point c = q.front(); q.pop();
    f[ c.y * w + c.x] = true;
    if ((c.x == tl.x && c.y < tl.y) || c.x < tl.x) tl = c;
    if ((c.x == tr.x && c.y < tr.y) || c.x > tr.x) tr = c;
    if ((c.x == bl.x && c.y > bl.y) || c.x < bl.x) bl = c;
    if ((c.x == br.x && c.y > br.y) || c.x > br.x) br = c;
    // printf("pop node (%d, %d)\n", c.x, c.y);
    // printf("tl: (%d, %d), tr: (%d, %d), bl: (%d, %d), br: (%d, %d)\n",
        // tl.x, tl.y, tr.x, tr.y, bl.x, bl.y, br.x, br.y);
    for (int i=0; i<4; i++) {
      Point nxt;
      nxt.x = c.x + warthog::dx[i];
      nxt.y = c.y + warthog::dy[i];
      // not in the rectangle
      if (nxt.x < xl || nxt.x > xu) continue;
      if (nxt.y < yl || nxt.y > yu) continue;
      // has been marked
      if (f[nxt.y*w+nxt.x]) continue;
      // not traversable
      if (!gmap->get_label(gmap->to_padded_id(nxt.x, nxt.y))) continue;
      f[nxt.y*w+nxt.x] = true;
      q.push(nxt);
    }
  }
  int l, r, py;
  // go through top border from left
  l = tl.x, r = tr.x;
  py = tl.y;
  while (l<tr.x) {
    // non-decrasing sequence stop
    if (!gmap->get_label(gmap->to_padded_id(l+1, py))) break;
    int dh = jpl->rayscan(warthog::jps::NORTH, gmap->to_padded_id(l+1, py));
    dh = min(dh, py-yl);
    for (int y=1; y<=dh; y++) {
      int step = jpl->rayscan(warthog::jps::WEST, gmap->to_padded_id(l+1, py-y));
      REQUIRE(step == 0);
    }
    py -= dh;
    l++;
  }
  // go through top border from right
  py = tr.y;
  while (r>tl.x) {
    // non-decrasing sequence stop
    if (!gmap->get_label(gmap->to_padded_id(r-1, py))) break;
    int dh = jpl->rayscan(warthog::jps::NORTH, gmap->to_padded_id(r-1, py));
    dh = min(dh, py-yl);
    for (int y=1; y<=dh; y++) {
      int step = jpl->rayscan(warthog::jps::EAST, gmap->to_padded_id(r-1, py-y));
      REQUIRE(step == 0);
    }
    py -= dh;
    r--;
  }
  REQUIRE(r <= l);

  // go through bot border from left
  l=bl.x, r=br.x;
  py = bl.y;
  while (l<br.x) {
    // non-decrasing sequence stop
    if (!gmap->get_label(gmap->to_padded_id(l+1, py))) break;
    int dh = jpl->rayscan(warthog::jps::SOUTH, gmap->to_padded_id(l+1, py));
    dh = min(dh, yu-py);
    for (int y=1; y<=dh; y++) {
      int step = jpl->rayscan(warthog::jps::WEST, gmap->to_padded_id(l+1, py+y));
      REQUIRE(step == 0);
    }
    py += dh;
    l++;
  }
  // go through bot border from right
  py = br.y;
  while (r>bl.x) {
    // non-decrasing sequence stop
    if (!gmap->get_label(gmap->to_padded_id(r-1, py))) break;
    int dh = jpl->rayscan(warthog::jps::SOUTH, gmap->to_padded_id(r-1, py));
    dh = min(dh, yu-py);
    for (int y=1; y<=dh; y++) {
      int step = jpl->rayscan(warthog::jps::EAST, gmap->to_padded_id(r-1, py+y));
      REQUIRE(step == 0);
    }
    py += dh;
    r--;
  }
  REQUIRE(r <= l);

  return true;
}

void print_cell(uint32_t cid, warthog::gridmap* gmap, string cs) {
  if (gmap->get_label(cid)) printf("%c", cs[0]);
  else printf("%c", cs[1]);
}

void print_rect_map(int xl, int yl, int xu, int yu, warthog::gridmap* gmap) {
  int pad = 1;
  printf("xl: %d yl: %d xu: %d yu: %d pad: %d\n", xl, yl, xu, yu, pad);
  // print above pad row
  for (int y=yl-pad; y<yl; y++) {
    for (int x=xl-pad; x<=xu+pad; x++) {
      print_cell(gmap->to_padded_id(x, y), gmap, ".@");
    }
    printf("\n");
  }
  // print NORTH border
  // print left pad columns
  for (int x=xl-pad; x<xl; x++) {
    print_cell(gmap->to_padded_id(x, yl), gmap, ".@");
  }
  for (int x=xl; x<=xu; x++) {
    print_cell(gmap->to_padded_id(x, yl), gmap, "-*");
  }
  // print right pad columns
  for (int x=xu+1; x<=xu+pad; x++) {
    print_cell(gmap->to_padded_id(x, yl), gmap, ".@");
  }

  printf("\n");
  // print Mid
  for (int y=yl+1; y<yu; y++) {
    
    // print left pad column
    for (int x=xl-pad; x<xl; x++) {
      print_cell(gmap->to_padded_id(x, y), gmap, ".@");
    }
    print_cell(gmap->to_padded_id(xl, y), gmap, "|*");
    for (int x=xl+1; x<xu; x++) {
      print_cell(gmap->to_padded_id(x, y), gmap, ".@");
    }
    print_cell(gmap->to_padded_id(xu, y), gmap, "|*");
    // print right pad column
    for (int x=xu+1; x<=xu+pad; x++) {
      print_cell(gmap->to_padded_id(x, y), gmap, ".@");
    }
    printf("\n");
  }
  // print SOUTH border
  // print left pad columns
  for (int x=xl-pad; x<xl; x++) {
    print_cell(gmap->to_padded_id(x, yu), gmap, ".@");
  }
  for (int x=xl; x<=xu && yu>yl; x++) {
    print_cell(gmap->to_padded_id(x, yu), gmap, "-*");
  }
  // print right pad columns
  for (int x=xu+1; x<=xu+pad; x++) {
    print_cell(gmap->to_padded_id(x, yu), gmap, ".@");
  }
  printf("\n");

  // print below pad row
  for (int y=yu+1; y<=yu+pad; y++) {
    for (int x=xl-pad; x<=xu+pad; x++) {
      print_cell(gmap->to_padded_id(x, y), gmap, ".@");
    }
    printf("\n");
  }
}

inline void update_interval_x(Interval& inv, int xl, int xu, int y, online_jump_point_locator2* jpl, warthog::gridmap* gmap) {
  // update lb/ub if it is not adjacent to an obstacle
  // (implies that it is still on the border and the border has been updated)
  int step;
  step = jpl->rayscan(warthog::jps::WEST, gmap->to_padded_id(inv.lb, y));
  inv.lb = max(xl, inv.lb-step);

  step = jpl->rayscan(warthog::jps::EAST, gmap->to_padded_id(inv.ub, y));
  inv.ub = min(xu, inv.ub+step);
}

inline void update_interval_y(Interval& inv, int yl, int yu, int x, online_jump_point_locator2* jpl, warthog::gridmap* gmap) {
  // update lb/ub if it is not adjacent to an obstacle
  // (implies that it is still on the border and the border has been updated)
  int step;
  step = jpl->rayscan(warthog::jps::NORTH, gmap->to_padded_id(x, inv.lb));
  inv.lb = max(yl, inv.lb-step);

  step = jpl->rayscan(warthog::jps::SOUTH, gmap->to_padded_id(x, inv.ub));
  inv.ub = min(yu, inv.ub+step);
}

bool expand_rect(int& xl, int& yl, int& xu, int& yu, online_jump_point_locator2* jpl) {
  int sx = xl, sy = yl;
  warthog::gridmap* gmap = jpl->get_map();

  if (verbose) {
    printf("before:\n");
    print_rect_map(xl, yl, xu, yu, gmap);
  }
  // track the intervals that are still open.
  // as we don't expand the border if it is completely closed (by obstacles)
  vector<Interval> invs = {
    {xl, xu},  // NORTH border
    {yl, yu},  // EAST border
    {xl, xu},  // SOUTH border
    {yl, yu}   // WEST border
  };
  bool res = false;
  int delta = 0;
  // Expand NORTH
  if (invs[0].lb <= invs[0].ub) {
    update_interval_x(invs[0], xl, xu, yl, jpl, gmap);
    delta = _expand_rect_y(yl, invs[0].lb, invs[0].ub, xl, xu, NORTH, jpl);
    yl -= delta;
    res |= (delta > 0);
  }
  if (verbose) {
    printf("expand north, delta=%d\n", delta);
    print_rect_map(xl, yl, xu, yu, gmap);
  }

  // Expand EAST
  if (invs[1].lb <= invs[1].ub) {
    update_interval_y(invs[1], yl, yu, xu, jpl, gmap);
    delta = _expand_rect_x(xu, invs[1].lb, invs[1].ub, yl, yu, EAST, jpl);
    xu += delta;
    res |= (delta > 0);
  }
  if (verbose) {
    printf("expand east, delta=%d\n", delta);
    print_rect_map(xl, yl, xu, yu, gmap);
  }

  // Expand SOUTH
  if (invs[2].lb <= invs[2].ub) {
    update_interval_x(invs[2], xl, xu, yu, jpl, gmap);
    delta = _expand_rect_y(yu, invs[2].lb, invs[2].ub, xl, xu, SOUTH, jpl);
    yu += delta;
    res |= (delta > 0);
  }
  if (verbose) {
    printf("expand south, delta=%d\n", delta);
    print_rect_map(xl, yl, xu, yu, gmap);
  }

  // Expand WEST
  if (invs[3].lb <= invs[3].ub) {
    update_interval_y(invs[3], yl, yu, xl, jpl, gmap);
    delta = _expand_rect_x(xl, invs[3].lb, invs[3].ub, yl, yu, WEST, jpl);
    xl -= delta;
    res |= (delta > 0);
  }
  if (verbose) {
    printf("expand west, delta=%d\n", delta);
    print_rect_map(xl, yl, xu, yu, gmap);
  }

  if (invs[0].lb <= invs[0].ub) update_interval_x(invs[0], xl, xu, yl, jpl, gmap); // N
  if (invs[1].lb <= invs[1].ub) update_interval_y(invs[1], yl, yu, xu, jpl, gmap); // E
  if (invs[2].lb <= invs[2].ub) update_interval_x(invs[2], xl, xu, yu, jpl, gmap); // S
  if (invs[3].lb <= invs[3].ub) update_interval_y(invs[3], yl, yu, xl, jpl, gmap); // W

  vector<bool> flag;
  REQUIRE(is_convex(xl, yl, xu, yu, sx, sy, flag, jpl));

  // sanity checking NORTH border
  for (int i=xl; i<=xu; i++) {
    int id = yl*gmap->header_width() + i;
    REQUIRE(id >= 0);
    REQUIRE(id < flag.size());
    if (i>=invs[0].lb && i<=invs[0].ub) REQUIRE(flag[id]);
    else REQUIRE(!flag[id]);
  }

  // sanity checking EAST border
  for (int i=yl; i<=yu; i++) {
    int id = i*gmap->header_width()+xu;
    REQUIRE(id >= 0);
    REQUIRE(id < flag.size());
    if (i>=invs[1].lb && i<=invs[1].ub) REQUIRE(flag[id]);
    else REQUIRE(!flag[id]);
  }

  // sanity checking SOUTH border
  for (int i=xl; i<=xu; i++) {
    int id = yu*gmap->header_width()+i;
    REQUIRE(id >= 0);
    REQUIRE(id < flag.size());
    if (i>=invs[2].lb && i<=invs[2].ub) REQUIRE(flag[id]);
    else REQUIRE(!flag[id]);
  }

  // sanity checking WEST border
  for (int i=yl; i<=yu; i++) {
    int id = i*gmap->header_width()+xl;
    REQUIRE(id >= 0);
    REQUIRE(id < flag.size());
    if (i>=invs[3].lb && i<=invs[3].ub) REQUIRE(flag[id]);
    else REQUIRE(!flag[id]);
  }
  return res;
}

bool cmp(const Rect& a, const Rect& b) {
  return (a.w*a.h) >= (b.w*b.h);
}

void run_ext_rect(string rfile) {
  RectMap rmap(rfile.c_str());
  rect_jump_point_locator rjpl(&rmap);
  sort(rmap.rects.begin(), rmap.rects.end(), cmp);
  for (int i=0; i<min(10, (int)rmap.rects.size()); i++) {
    Rect& r = rmap.rects[i];
    int xl=r.x, yl=r.y, xu=r.x+r.w-1, yu=r.y+r.h-1;
    printf("i=%d\n", i);
    if (expand_rect(xl, yl, xu, yu, rjpl.get_jpl())) {
      if (verbose) {
        printf("before:\n");
        print_rect_map(r.x, r.y, r.x+r.w-1, r.y+r.h-1, rjpl.get_jpl()->get_map());
        printf("after:\n");
        print_rect_map(xl, yl, xu, yu, rjpl.get_jpl()->get_map());
      }
    }
  }
}

TEST_CASE("ext-rect") {
  vector<string> rectids = {
   "./test/rectid/arena_1.rectid",
   "./test/rectid/scene_sp_endmaps_1.rectid",
   "./test/rectid/GreenerPastures_1.rectid",
   "./test/rectid/CatwalkAlley_1.rectid",
  };
  for (auto& rid: rectids) {
    cout << "Map: " << rid << endl;
    run_ext_rect(rid);
  }
}

void gen_rectid(string mapfile, string writeto) {
  RectMap rmap(mapfile.c_str());
  if (!writeto.empty()) {
    ofstream out;
    out.open(writeto.c_str());
    rmap.print_idmap(out);
    out.close();
    RectMap newmap(writeto.c_str());
    REQUIRE(newmap == rmap);
    REQUIRE(newmap.equal(*(newmap.gmap)));
  }
  else {
    rmap.print_idmap(cout);
  }
}

TEST_CASE("gen-rectid") {
  vector<pair<string, string>> cases = {
    {"../maps/dao/arena.map", "./test/rectid/arena.rectid"},
    {"../maps/rooms/64room_000.map", "./test/rectid/64room.rectid"},
    {"../maps/bgmaps/AR0042SR.map", "./test/rectid/AR0042SR.rectid"},
    {"../maps/street/Boston_2_256.map", "./test/rectid/Boston_2_256.rectid"}
  };
  if (infile.empty()) {
    for (auto& each: cases) {
      string mapfile = each.first;
      string writeto = each.second;
      gen_rectid(mapfile, writeto);
    }
  } else {
    gen_rectid(infile, outfile);
  }
}

TEST_CASE("gen-rectlist") {
  vector<string> maps = {
    "./test/rectid/CatwalkAlley_1.rectid",
    "./test/rectid/scene_sp_endmaps_1.rectid",
    "./test/rectid/GreenerPastures_1.rectid"
  };
  string headers="map,maph,mapw,x,y,h,w,traversable";
  cout << headers << endl;
  for (string& m: maps) {
    RectMap rmap(m.c_str());
    assert(rmap.equal(*(rmap.gmap)));
    for (Rect r: rmap.rects) {
      cout << m.c_str() << "," << rmap.maph << ","
           << rmap.mapw << "," << r.x << "," << r.y << ","
           << r.h << "," << r.w << "," << true << endl;
    }
    for (int y=0; y<rmap.maph; y++)
    for (int x=0; x<rmap.mapw; x++) if (rmap.idmap[y*rmap.mapw+x] == -1) {
      cout << m.c_str() << "," << rmap.maph << ","
           << rmap.mapw << "," << x << "," << y << ","
           << 1 << "," << 1 << "," << false << endl;
    }
  }
}

TEST_CASE("gen-distr") {
  // generate statistic data of rectangles
  vector<string> maps = {
     "./test/rectid/arena_1.rectid",
     "./test/rectid/arena_2.rectid",
     "./test/rectid/arena_3.rectid",
     "./test/rectid/arena_4.rectid",
     "./test/rectid/arena_5.rectid",
     "./test/rectid/arena_6.rectid",
     "./test/rectid/arena_7.rectid",
     "./test/rectid/arena_8.rectid",
     "./test/rectid/scene_sp_endmaps_1.rectid",
     "./test/rectid/scene_sp_endmaps_2.rectid",
     "./test/rectid/scene_sp_endmaps_3.rectid",
     "./test/rectid/scene_sp_endmaps_4.rectid",
     "./test/rectid/scene_sp_endmaps_5.rectid",
     "./test/rectid/scene_sp_endmaps_6.rectid",
     "./test/rectid/scene_sp_endmaps_7.rectid",
     "./test/rectid/scene_sp_endmaps_8.rectid",
     "./test/rectid/GreenerPastures_1.rectid",
     "./test/rectid/GreenerPastures_2.rectid",
     "./test/rectid/GreenerPastures_3.rectid",
     "./test/rectid/GreenerPastures_4.rectid",
     "./test/rectid/GreenerPastures_5.rectid",
     "./test/rectid/GreenerPastures_6.rectid",
     "./test/rectid/GreenerPastures_7.rectid",
     "./test/rectid/GreenerPastures_8.rectid",
     "./test/rectid/CatwalkAlley_1.rectid",
     "./test/rectid/CatwalkAlley_2.rectid",
     "./test/rectid/CatwalkAlley_3.rectid",
     "./test/rectid/CatwalkAlley_4.rectid",
     "./test/rectid/CatwalkAlley_5.rectid",
     "./test/rectid/CatwalkAlley_6.rectid",
     "./test/rectid/CatwalkAlley_7.rectid",
     "./test/rectid/CatwalkAlley_8.rectid",
  };
  RectMap rmap;
  string headers = "map,mapw,maph,rnum,rid,rw,rh";
  cout << headers << endl;
  for (auto& m: maps) {
    rmap.init(m.c_str());
    for (auto& r: rmap.rects) {
      cout << m << ","
           << rmap.mapw << ","
           << rmap.maph << ","
           << rmap.rects.size() << ","
           << r.rid << ","
           << r.w << ","
           << r.h << endl;
    }
  }
}


int main(int argv, char* args[]) {
  using namespace Catch::clara;
  Catch::Session session;
  auto cli = 
      Opt(infile, "testfile")["--in"]("")
    | Opt(outfile, "outfile")["--out"]("")
    | Opt( verbose )["-v"]["--verbose"]("verbose")
    | session.cli();
  session.cli(cli);
  int resCode = session.applyCommandLine(argv, args);
  if (resCode != 0)
    return resCode;

	cerr << "Running test cases..." << endl;
	return session.run(argv, args);

}
