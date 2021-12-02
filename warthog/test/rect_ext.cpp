#define CATCH_CONFIG_RUNNER
#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>

#include "catch.hpp"
#include "rectmap.h"
#include "constants.h"
#include "rect_jump_point_locator.h"
#include "grid2convex_rect.h"

using namespace warthog::rectscan;
using namespace warthog::jps;
using namespace std;
namespace cr = convrectgen;
typedef cr::Point Point;
string infile, outfile;
bool verbose = false;
const vector<string> desc = {
  "NORTH", "SOUTH", "EAST", "WEST",
  "NORTHEAST", "NORTHWEST", "SOUTHEAST", "SOUTHWEST"
};

struct Visual {
  int x, y, h, w;
  int mapw, maph;
  int traversable;
  string map;
  const string header = "map,maph,mapw,x,y,h,w,traversable";

  Visual(int xl, int yl, int xu, int yu, int mw, int mh, string mapfile, int label) {
    x = xl, y = yl, h = yu-yl+1, w = xu-xl+1;
    mapw = mw, maph = mh;
    map = mapfile;
    traversable = label;
  }
  Visual() {};

  string to_str() {
    return map + "," + to_string(maph) + "," + to_string(mapw) + "," 
          + to_string(x) + "," + to_string(y) + "," + to_string(h) + "," + to_string(w) + ","
          + to_string(traversable);
  }
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
  rect_jump_point_locator rjpl(&rmap);
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
    vis.push_back(Visual(x, y, x, y, rmap.mapw, rmap.maph, rfile, 0));
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
    if (bestr.hvalue == 0) continue;
    xl=bestr.x-bestr.w+1, xu=bestr.x, yl=bestr.y-bestr.h+1, yu=bestr.y;
    if (verbose && bestr.hvalue != cr::get_heuristic(r.h, r.w)) {
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
    vis.push_back(Visual(xl, yl, xu, yu, rmap.mapw, rmap.maph, rfile, 1));
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
    vis.push_back(Visual(r.xl, r.yl, r.xu, r.yu, mapw, maph, mapfile, 1));
  }
  for (int y=0; y<(int)gmap->header_height(); y++)
  for (int x=0; x<(int)gmap->header_width(); x++)
  if (idmap[y*mapw+x]==-1) {
    vis.push_back(Visual(x, y, x, y, mapw, maph, mapfile, 0));
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
