#define CATCH_CONFIG_RUNNER
#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>

#include "catch.hpp"
#include "rectmap.h"
#include "constants.h"
#include "jps2_expansion_policy.h"
#include "flexible_astar.h"
#include "octile_heuristic.h"
#include "rect_jump_point_locator.h"
#include "rect_expansion_policy.h"
#include "convrect_jump_point_locator.h"
#include "grid2convex_rect.h"
#include "convex_rectmap.h"

using namespace warthog::jps;
using namespace std;
namespace cr = convrectgen;
namespace cv = warthog::convrectscan;
namespace rs = warthog::rectscan;
typedef cr::Point Point;
typedef rs::Rect Rect;
typedef rs::RectMap RectMap;
typedef rs::ConvRectMap ConvRectMap;
typedef rs::ConvRect ConvRect;
string infile, outfile;
bool verbose = false;
const double EPS = 1e-3;
const vector<string> desc = {
  "NORTH", "SOUTH", "EAST", "WEST",
  "NORTHEAST", "NORTHWEST", "SOUTHEAST", "SOUTHWEST"
};

struct Visual {
  int x, y, h, w, size;
  int mapw, maph;
  int traversable;
  string map;
  const string header = "map,maph,mapw,x,y,h,w,size,traversable";

  Visual(int xl, int yl, int xu, int yu, int _size, int mw, int mh, string mapfile, int label) {
    x = xl, y = yl, h = yu-yl+1, w = xu-xl+1;
    size = _size;
    mapw = mw, maph = mh;
    map = mapfile;
    traversable = label;
  }
  Visual() {};

  string to_str() {
    return map + "," + to_string(maph) + "," + to_string(mapw) + "," 
          + to_string(x) + "," + to_string(y) + "," + to_string(h) + "," + to_string(w) + ","
          + to_string(size) + "," + to_string(traversable);
  }
};

int get_convex_size(vector<int>& idmap, int rid, int sx, int sy, int mapw, int maph) {
  int res = 0;
  set<Point> vis;
  queue<Point> q;
  q.push({sx, sy});
  vis.insert({sx, sy});
  while (!q.empty()) {
    Point c = q.front(); q.pop();
    res++;
    for (int i=0; i<4; i++) {
      Point nxt = {c.x+warthog::dx[i], c.y+warthog::dy[i]};
      if (0<=nxt.x&&nxt.x<mapw && 0<=nxt.y&&nxt.y<maph) {
        if (idmap[nxt.y*mapw+nxt.x] == rid && vis.find(nxt) == vis.end()) {
          vis.insert(nxt);
          q.push(nxt);
        }
      }
    }
  }
  return res;
}

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

bool is_convex(int xl, int yl, int xu, int yu, int sx, int sy, int rid, vector<int>& idmap, online_jump_point_locator2* jpl) {
  // expanded rectangle: top-left (xl, yl) bottom-right (xu, yu)
  // seed position (sx, sy) a traversable tile of the original empty rectangle
  // the function check whether the traversable component is "convex" in the expanded rectangle by:
  // 0. flood fill from (sx, sy) to mark the current traversable component
  // 1. get the top, is it "convex"?
  // 2. get the bottom, is it "convex"?;
  // 3. get the left outline, ....?
  // 4. get the right outline, ...?
  warthog::gridmap* gmap = jpl->get_map();
  int w = gmap->header_width();
  set<Point> visit;
  queue<Point> q;
  Point tl, tr, bl, br, lt, lb, rt, rb;
  tl = tr = bl = br = lt = lb = rt = rb = Point{sx, sy};
  // flood fill
  q.push({sx, sy});
  visit.insert({sx, sy});
  // all connected tiles in [xl, xu], [yl, yu] must be marked by rid
  while (!q.empty()) {
    Point c = q.front(); q.pop();
    REQUIRE(idmap[c.y*w+c.x] == rid);
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
      if (visit.find(nxt) != visit.end()) continue;
      // it is illegal if (x, y) is marked by other rectangle
      REQUIRE(idmap[nxt.y*w+nxt.x] == rid);
      visit.insert(nxt);
      q.push(nxt);
    }
  }
  int l, r;
  // check top border from left
  l = cr::check_border_x(xl, yl, xu, yu, lt, rt, 1, -1, rid, w, idmap);
  // check top border from right
  r = cr::check_border_x(xl, yl, xu, yu, rt, lt, -1, -1, rid, w, idmap);
  REQUIRE(l > r);
  // check bot border from left
  l = cr::check_border_x(xl, yl, xu, yu, lb, rb, 1, 1, rid, w, idmap);
  // check bot border from right
  r = cr::check_border_x(xl, yl, xu, yu, rb, lb, -1, 1, rid, w, idmap);
  REQUIRE(l > r);

  // check left border from top 
  l = check_border_y(xl, yl, xu, yu, tl, bl, -1, 1, rid, w, idmap);
  // check left border from bot 
  r = check_border_y(xl, yl, xu, yu, bl, tl, -1, -1, rid, w, idmap);
  REQUIRE(l > r);
  
  // check right border from top
  l = check_border_y(xl, yl, xu, yu, tr, br, 1, 1, rid, w, idmap);
  // check right border from bot
  r = check_border_y(xl, yl, xu, yu, br, tr, 1, -1, rid, w, idmap);
  REQUIRE(l > r);
  return true;
}

bool expand_rect(int& xl, int& yl, int& xu, int& yu, int rid, 
    int lb[4], int ub[4], vector<int>& idmap, online_jump_point_locator2* jpl) {
  // precondition:
  // * [xl, yl, xu, yu] is a purely empty rectangle
  // * idmap may contains other convex rectangles
  // * labels in jpl may changed by other convex rectangles
  int _xu=xu, _yu=yu;
  int mapw = jpl->get_map()->header_width();

  // mark the rectangle before expand
  for (int y=yl; y<=yu; y++)
  for (int x=xl; x<=xu; x++) {
    REQUIRE(idmap[y*mapw + x] == 0);
    idmap[y*mapw+x] = rid;
    jpl->set_label(x, y, false);
  }

  // bool res = cr::expand_rect(xl, yl, xu, yu, rid, lb, ub, idmap, jpl);
  bool res = cr::expand_rect_naive(xl, yl, xu, yu, rid, lb, ub, idmap, jpl);
  REQUIRE(is_convex(xl, yl, xu, yu, _xu, _yu, rid, idmap, jpl));

  // check the correctness of lb[4], ub[4]
  for (int x=lb[0]; x<=ub[0]; x++) 
    REQUIRE(idmap[yl*mapw+x] == rid);
  for (int x=lb[2]; x<=ub[2]; x++)
    REQUIRE(idmap[yu*mapw+x] == rid);
  for (int y=lb[1]; y<=ub[1]; y++)
    REQUIRE(idmap[y*mapw+xu] == rid);
  for (int y=lb[3]; y<=ub[3]; y++)
    REQUIRE(idmap[y*mapw+xl] == rid);
  return res;
}

bool cmp(const Rect& a, const Rect& b) {
  return (a.w*a.h) > (b.w*b.h);
}

void run_ext_rect(string rfile, vector<Visual>& vis) {
  RectMap rmap(rfile.c_str());
  rs::rect_jump_point_locator rjpl(&rmap);
  online_jump_point_locator2* jpl = rjpl.get_jpl();

  // init rectangle list
  sort(rmap.rects.begin(), rmap.rects.end(), cmp);
  vector<int>& idmap = cr::_idmap;
  idmap.resize(rmap.maph*rmap.mapw);
  fill(idmap.begin(), idmap.end(), 0);
  cr::init(rmap.mapw, rmap.maph, verbose);

  // init idmaps
  for (int y=0; y<rmap.maph; y++)
  for (int x=0; x<rmap.mapw; x++)
  if (!jpl->get_label(x, y)) {
    idmap[y*rmap.mapw+x] = -1;
    vis.push_back(Visual(x, y, x, y, 1, rmap.mapw, rmap.maph, rfile, 0));
  }

  for (int i=0; i<(int)rmap.rects.size() && i<=20; i++) {
    Rect& r = rmap.rects[i];
    int xl=r.x, yl=r.y, xu=r.x+r.w-1, yu=r.y+r.h-1;

    printf("i=%d\n", i);
    if (verbose) {
      printf("cur pure rect:\n");
      cr::print_rect_map(xl, yl, xu, yu, i+1, idmap, jpl->get_map());
    }
    cr::SearchNode bestr = cr::get_best_rect({xu, yu, xu-xl+1, yu-yl+1}, idmap, jpl);
    if (bestr.hvalue() == 0) continue;
    xl=bestr.x-bestr.w+1, xu=bestr.x, yl=bestr.y-bestr.h+1, yu=bestr.y;
    if (verbose && bestr.hvalue() != cr::get_heuristic(r.h, r.w)) {
      printf("best rect from current:\n");
      cr::print_rect_map(xl, yl, xu, yu, i+1, idmap, jpl->get_map());
    }
    int lb[4] = {xl, yl, xl, yl}, ub[4] = {xu, yu, xu, yu};
    if (expand_rect(xl, yl, xu, yu, i+1, lb, ub, idmap, jpl)) {
      if (verbose) {
        printf("after expand:\n");
        cr::print_rect_map(xl, yl, xu, yu, i+1, idmap, jpl->get_map());
      }
    }
    int size = get_convex_size(idmap, r.rid, r.x, r.y, rmap.mapw, rmap.maph);
    vis.push_back(Visual(xl, yl, xu, yu, size, rmap.mapw, rmap.maph, rfile, 1));
  }
}

TEST_CASE("ext-rect") {
  vector<string> rectids = {
    "./test/rectid/arena_1.rectid",
    "./test/rectid/scene_sp_endmaps_1.rectid",
    "./test/rectid/GreenerPastures_1.rectid",
    "./test/rectid/CatwalkAlley_1.rectid",
  };
  vector<Visual> vis;
  for (auto& rid: rectids) {
    cout << "Map: " << rid << endl;
    run_ext_rect(rid, vis);
  }
  if (outfile.empty())
    outfile = "./data/rect-ext.visual";
  ofstream out(outfile);
  out << Visual().header << endl;
  for (auto& v: vis)
    out << v.to_str() << endl;
}

TEST_CASE("ext-rect2") {
  map<string, vector<int>> cases = {
    {
      "./data/CatwalkAlley_1.map",
      // xl, yl, xu, yu, rid
      {342, 341, 372, 354, 1}
    }
  };
  for (auto& c: cases) {
    string mapfile = c.first;
    int xl = c.second[0], yl = c.second[1],
        xu = c.second[2], yu = c.second[3],
        rid = c.second[4];
    warthog::gridmap* gmap = new warthog::gridmap(mapfile.c_str());
    online_jump_point_locator2* jpl = new online_jump_point_locator2(gmap);

    int mapw = gmap->header_width(), maph = gmap->header_height();
    cr::init(mapw, maph, verbose);
    vector<int>& idmap = cr::_idmap;
    idmap.resize(mapw*maph);
    for (int y=0; y<maph; y++)
    for (int x=0; x<mapw; x++)
    if (!jpl->get_label(x, y))
      idmap[y*mapw+x] = -1;
    int lb[4] = {xl, yl, xl, yl}, ub[4] = {xu, yu, xu, yu};
    if (expand_rect(xl, yl, xu, yu, rid, lb, ub, idmap, jpl)) {
      if (verbose) {
        printf("after expand:\n");
        cr::print_rect_map(xl, yl, xu, yu, rid, idmap, gmap);
      }
    }

    delete gmap;
    delete jpl;
  }
}

void gen_ext_rect(string mapfile, vector<Visual>& vis) {
  warthog::gridmap* gmap = new warthog::gridmap(mapfile.c_str());
  online_jump_point_locator2* jpl = new online_jump_point_locator2(gmap);
  vector<int>&idmap = cr::_idmap;
  int mapw = gmap->header_width(), maph = gmap->header_height();
  cr::init(mapw, maph, verbose);
  cr::make_rectangles(jpl, idmap);

  for (auto& r: cr::finals) {
    REQUIRE(is_convex(r.xl, r.yl, r.xu, r.yu, r.sx, r.sy, r.rid, idmap, jpl));
    int size = get_convex_size(idmap, r.rid, r.sx, r.sy, mapw, maph);
    vis.push_back(Visual(r.xl, r.yl, r.xu, r.yu, size, mapw, maph, mapfile, 1));
  }
  for (int y=0; y<(int)gmap->header_height(); y++)
  for (int x=0; x<(int)gmap->header_width(); x++)
  if (idmap[y*mapw+x]==-1) {
    vis.push_back(Visual(x, y, x, y, 1, mapw, maph, mapfile, 0));
  }
}

TEST_CASE("gen-ext-rect") {
  vector<string> maps = {
    "./data/arena_1.map",
    "./data/scene_sp_endmaps_1.map",
    "./data/GreenerPastures_1.map",
    "./data/CatwalkAlley_1.map",
    "./data/Aurora_1.map"
  };
  vector<Visual> vis;
  for (auto& map: maps) {
    gen_ext_rect(map, vis);
  }
  if (outfile.empty())
    outfile = "./data/rect-ext.visual";
  ofstream out(outfile);
  out << Visual().header << endl;
  for (auto& v: vis)
    out << v.to_str() << endl;
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

void gen_convrectid(string mapfile, string writeto) {
  cr::verbose = verbose;
  cv::ConvRectMap crmap(mapfile.c_str());
  if (!writeto.empty()) {
    ofstream out;
    out.open(writeto.c_str());
    crmap.print_idmap(out);
    out.close();
    cv::ConvRectMap newmap(writeto.c_str());
    REQUIRE(newmap == crmap);
    REQUIRE(newmap.equal(*(newmap.gmap)));
  }
  else {
    crmap.print_idmap(cout);
  }
}

void gen_convrect(string mapfile, string writeto) {
  cr::verbose = verbose;
  cv::ConvRectMap crmap(mapfile.c_str());
  if (!writeto.empty()) {
    ofstream out;
    out.open(writeto.c_str());
    crmap.print(out);
    out.close();
    cv::ConvRectMap newmap(writeto.c_str());
    REQUIRE(newmap == crmap);
    REQUIRE(newmap.equal(*(newmap.gmap)));
  }
  else {
    crmap.print(cout);
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

TEST_CASE("gen-convrectid") {
  vector<pair<string, string>> cases = {
    {"../maps/dao/arena.map", "./test/rectid/arena.convrectid"},
    {"../maps/dao/isound1.map", "./test/rectid/isound1.convrectid"},
    {"../maps/dao/lak101d.map", "./test/rectid/lak101d.convrectid"},
    {"../maps/dao/lak105d.map", "./test/rectid/lak105d.convrectid"},
    {"../maps/dao/lak107d.map", "./test/rectid/lak107d.convrectid"},
    {"../maps/dao/lak108d.map", "./test/rectid/lak108d.convrectid"},
  };
  if (infile.empty()) {
    for (auto& each: cases) {
      string mapfile = each.first;
      string writeto = each.second;
      cerr << "Running on " << mapfile << endl;
      gen_convrectid(mapfile, writeto);
    }
  } else {
    gen_convrectid(infile, outfile);
  }
}

TEST_CASE("gen-convrect") {
  vector<pair<string, string>> cases = {
    {"../maps/dao/arena.map", "./test/rects/arena.convrect"},
    {"../maps/dao/isound1.map", "./test/rects/isound1.convrect"},
    {"../maps/dao/lak101d.map", "./test/rects/lak101d.convrect"},
    {"../maps/dao/lak105d.map", "./test/rects/lak105d.convrect"},
    {"../maps/dao/lak107d.map", "./test/rects/lak107d.convrect"},
    {"../maps/dao/lak108d.map", "./test/rects/lak108d.convrect"},
    {"./test/maps/Cat0.map", "./test/rects/Cat0.convrect"},
  };
  if (infile.empty()) {
    for (auto& each: cases) {
      string mapfile = each.first;
      string writeto = each.second;
      cerr << "Running on " << mapfile << endl;
      gen_convrect(mapfile, writeto);
    }
  } else {
    gen_convrect(infile, outfile);
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

void test_internalJump(string mfile) {
  cv::ConvRectMap convmap(mfile);
  rs::convrect_jump_point_locator* cjpl = new rs::convrect_jump_point_locator(&convmap);

  warthog::jps2_expansion_policy expander(convmap.gmap);
  warthog::octile_heuristic heur(convmap.mapw, convmap.maph);
  warthog::pqueue_min open;
  warthog::flexible_astar<
    warthog::octile_heuristic,
    warthog::jps2_expansion_policy,
    warthog::pqueue_min> jps2(&heur, &expander, &open);

  for (cv::ConvRect& r: convmap.rects) {
    vector<Point> nodes;
    for (int y=r.yl(); y<=r.yu(); y++)
    for (int x=r.xl(); x<=r.xu(); x++) 
    if (convmap.idmap[y*convmap.mapw+x]==r.rid) {
      nodes.push_back({x, y});
    }
    // compute path from u to v
    for (Point u: nodes)
    for (Point v: nodes) if (u != v) {
      int delx = v.x - u.x, dely = v.y - u.y;
      int cid = u.y*convmap.mapw+u.x;
      int gid = v.y*convmap.mapw+v.x;
      double plen = rs::INF;
      for (int i=0; i<8; i++) {
        int vx = warthog::dx[i]*delx;
        int vy = warthog::dy[i]*dely;
        if (vx>=0 && vy>=0) {
          direction d = v2d(warthog::dx[i], warthog::dy[i]);
          cerr << "s( " << u.x << ", " << u.y << " ) "
               << "t( " << v.x << ", " << v.y << " ) "
               << "dir: " << desc[i] << " i=" << i << endl;
          cjpl->reset();
          cjpl->jump(d, cid, gid, &r);
          if (cjpl->get_jpts().size() && (int)cjpl->get_jpts().back()==gid) {
            // path length must be exactly same if reach target via another starting direction
            if (plen != rs::INF)
              REQUIRE(plen == cjpl->get_costs().back());
            plen = cjpl->get_costs().back();
          }
        }
      }
      warthog::problem_instance pi(cid, gid, verbose);
      warthog::solution sol;
      jps2.get_path(pi, sol);
      REQUIRE(plen - sol.sum_of_edge_costs_ < EPS);
    }
  }
  delete cjpl;
}

void test_cardinaljump(string mfile) {
  cv::ConvRectMap convmap(mfile);
  rs::RectMap rmap(mfile.c_str());
  rs::convrect_jump_point_locator* cjpl = new rs::convrect_jump_point_locator(&convmap);
  rs::rect_jump_point_locator* rjpl = new rs::rect_jump_point_locator(&rmap);

  int mapw = convmap.mapw, maph = convmap.maph;
  REQUIRE(convmap.equal(*rmap.gmap));
  REQUIRE(rmap.equal(*convmap.gmap));

  if (verbose) convmap.print(cout);

  direction ds[] = {NORTH, SOUTH, EAST, WEST}; 
  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++) 
  if (convmap.idmap[y*mapw+x] != -1)
  for (int i=0; i<4; i++)
  {
    direction d = ds[i];
    cerr << "s( " << x << ", " << y << " ) " 
         << "dir: " << desc[i] << " i=" << i << endl;
    int cid = y*mapw+x;
    cjpl->reset();
    cjpl->jump(d, cid, rs::INF, convmap.get_rect(x, y));
    rjpl->reset();
    rjpl->jump(d, cid, rs::INF, rmap.get_rect(x, y));
    REQUIRE(cjpl->get_costs().size() == rjpl->get_costs().size());
    REQUIRE(cjpl->get_jpts().size() == rjpl->get_jpts().size());
    if (cjpl->get_costs().size()) {
      REQUIRE(cjpl->get_costs().back() == rjpl->get_costs().back());
      REQUIRE(cjpl->get_jpts().back() == rjpl->get_jpts().back());
    }
  }
  delete cjpl;
  delete rjpl;
} 

void pseudo_convex_expand(rs::ConvRectMap* cmap, int x, int y, 
    direction d, rs::rect_expansion_policy* expd, 
    map<uint32_t, warthog::cost_t>& jpts) {
  queue<warthog::search_node*> q;
  int cid = y*cmap->mapw+x;
  int dx, dy;
  int nx, ny, px, py, cntd;
  d2v(d, dx, dy);
  warthog::problem_instance pi(cid, rs::INF, verbose);
  warthog::search_node* snode = expd->generate_start_node(&pi);
  expd->get_jpl()->reset();
  expd->get_jpl()->jump(d, cid, rs::INF, expd->get_map()->get_rect(x, y));
  vector<uint32_t> sucids = expd->get_jpl()->get_jpts();
  vector<warthog::cost_t> cs = expd->get_jpl()->get_costs();
  for (int i=0; i<(int)sucids.size(); i++) {
    warthog::search_node* n = expd->generate(sucids[i]);
    n->init(snode->get_search_number(), snode->get_id(), cs[i], cs[i]);

    expd->get_xy(n->get_id(), nx, ny);
    cntd = min(abs(nx-x), abs(ny-y));
    px = x + cntd * dx;
    py = y + cntd * dy;
    int rid = cmap->get_rid(px, py);
    bool cardinal_x = (px != nx);

    ConvRect* cr = cmap->get_rect(nx, ny);
    if (cr->rid != rid || 
        (!cardinal_x && (nx == cr->xl() || nx == cr->xu())) ||
        (cardinal_x  && (ny == cr->yl() || ny == cr->yu()))) {
      jpts[n->get_id()] = cs[i];
      if(pi.verbose_) {
          cerr << "  generating (edgecost=" << cs[i]<<") ("<< nx <<", "<< ny <<") crid:"
               << cmap->get_rid(nx, ny) << ", rid:" << rid << "...";
          n->print(cerr);
          cerr << endl;
      }
    }
    else q.push(n);
  }
  map<int, warthog::cost_t> gtable;

  while (!q.empty()) {
    warthog::search_node* c = q.front(); q.pop();
    c->set_expanded(true);

    if(pi.verbose_) {
      int32_t x, y;
      expd->get_xy(c->get_id(), x, y);
      cerr << ". expanding ("<<x<<", "<<y<<")..."; c->print(cerr);
      cerr << endl;
    }
    expd->expand(c, &pi);
    warthog::search_node* n = 0;
    warthog::cost_t cn = 0;
    for (expd->first(n, cn); n != 0; expd->next(n, cn)) {
      expd->get_xy(n->get_id(), nx, ny);
      expd->get_xy(c->get_id(), px, py);
      cntd = min(abs(nx-px), abs(ny-py));
      int rid = cmap->get_rid(px+cntd*dx, py+cntd*dy);
      warthog::cost_t gval = c->get_g() + cn;
      n->init(c->get_search_number(), c->get_id(), gval, gval);
      if (gtable.find(n->get_id()) == gtable.end() || gtable[n->get_id()] > gval) {
        gtable[n->get_id()] = gval;
        ConvRect* cr = cmap->get_rect(nx, ny);
        bool cardinal_x = (px+cntd*dx != nx);
        if (cr->rid != rid || 
            (!cardinal_x && (nx == cr->xl() || nx == cr->xu())) ||
            (cardinal_x  && (ny == cr->yl() || ny == cr->yu()))) {
          jpts[n->get_id()] = gval;
          if(pi.verbose_) {
              cerr << "  generating (edgecost=" << cn<<") ("<< nx <<", "<< ny <<") crid:"
                   << cmap->get_rid(nx, ny) << ", rid:" << rid << "...";
              n->print(cerr);
              cerr << endl;
          }
        }
        else q.push(n);
      }
    }
  }
}

void report(map<uint32_t, warthog::cost_t> p1, vector<uint32_t> cp, ConvRectMap& m) {
  set<uint32_t> sp(cp.begin(), cp.end());
  int x, y;
  for (auto& it: p1) {
    if (sp.find(it.first) == sp.end()) {
      m.to_xy(it.first, x, y);
      cerr << "cjpl missing: (" << x << ", " << y << ")" << endl;
    }
  }
  for (uint32_t id: cp) {
    if (p1.find(id) == p1.end()) {
      m.to_xy(id, x, y);
      cerr << "jpl missing: (" << x << ", " << y << ")" << endl;
    }
  }
  sort(cp.begin(), cp.end());
  for (int i=1; i<(int)cp.size(); i++) {
    if (cp[i] == cp[i-1]) {
      m.to_xy(cp[i], x, y);
      cerr << "cjpl duplicate: (" << x << ", " << y << ")" << endl; 
    }
  }
}

void test_diagJump(string rfile, string crfile) {
  cv::ConvRectMap convmap(crfile);
  rs::convrect_jump_point_locator* cjpl = new rs::convrect_jump_point_locator(&convmap);
  rs::RectMap rmap(rfile.c_str());
  rs::rect_expansion_policy expd(&rmap);
  cjpl->verbose = verbose;

  int mapw = convmap.mapw, maph = convmap.maph;
  REQUIRE(convmap.equal(*rmap.gmap));
  REQUIRE(rmap.equal(*convmap.gmap));
  map<uint32_t, warthog::cost_t> jpts;

  if (verbose) convmap.print(cout);
  direction ds[] = {NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST}; 
  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++)
  if (convmap.idmap[y*mapw+x] != -1)
  for (int i=0; i<4; i++) {
    direction d = ds[i];
    cerr << "s( " << x << ", " << y << " ) " 
         << "dir: " << desc[4+i] << " i=" << i << endl;
    int cid = y*mapw+x;
    cjpl->reset();
    cjpl->jump(d, cid, rs::INF, convmap.get_rect(x, y));

    jpts.clear();
    pseudo_convex_expand(&convmap, x, y, d, &expd, jpts);
    vector<uint32_t> cjpts = cjpl->get_jpts();
    vector<warthog::cost_t> costs = cjpl->get_costs();
    report(jpts, cjpts, convmap);
    for (int i=0; i<(int)cjpts.size(); i++) {
      REQUIRE(jpts.find(cjpts[i]) != jpts.end());
      REQUIRE(jpts[cjpts[i]] == costs[i]);
    }
    REQUIRE(jpts.size() == cjpts.size());
  }
  delete cjpl;
}


TEST_CASE("internalJump") {
  vector<string> cases = {
    "./test/rects/stairBorder.convrect",
    "./test/rects/stairNE.convrect",
    "./test/rects/stairSW.convrect",
    "./test/rects/cross1.convrect",
    "./test/rects/arena.convrect",
    "./test/rects/isound1.convrect",
    "./test/rects/lak101d.convrect",
    "./test/rects/lak105d.convrect",
    "./test/rects/lak107d.convrect",
    "./test/rects/lak108d.convrect",
    "./test/rects/Cat0.convrect",
  };
  for (string f: cases) {
    cerr << "Running map: " << f << endl;
    test_internalJump(f);
  }
}

TEST_CASE("cardinalJump") {
  vector<string> cases = {
    "../maps/dao/arena.map",
    "../maps/bgmaps/AR0042SR.map",
    "../maps/starcraft/CatwalkAlley.map",
    "../maps/starcraft/GreenerPastures.map",
    "../maps/iron/scene_sp_endmaps.map"
  };
  for (string f: cases) {
    cerr << "Running map: " << f << endl;
    test_cardinaljump(f);
  }
}

TEST_CASE("diagJump") {
  vector<pair<string, string>> cases {
    {"../maps/dao/arena.map", "./test/rects/arena.convrect"},
    {"../maps/dao/isound1.map", "./test/rects/isound1.convrect"},
    {"../maps/dao/lak101d.map", "./test/rects/lak101d.convrect"},
    {"../maps/dao/lak105d.map", "./test/rects/lak105d.convrect"},
    {"../maps/dao/lak107d.map", "./test/rects/lak107d.convrect"},
    {"../maps/dao/lak108d.map", "./test/rects/lak108d.convrect"},
    {"./test/maps/Cat0.map", "./test/rects/Cat0.convrect"},
  };
  for (auto& c: cases) {
    cerr << "Running map: " << c.first << endl;
    test_diagJump(c.first, c.second);
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
