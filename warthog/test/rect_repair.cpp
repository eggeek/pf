#define CATCH_CONFIG_RUNNER
#include <cstdlib>
#include "scenario_manager.h"
#include <catch2/catch.hpp>
#include "constants.h"
#include "rectmap.h"
#include "jps.h"
#include "timer.h"
#include "jpsplus_expansion_policy.h"

using namespace std;
using namespace warthog;
using namespace warthog::jps;
string infile, outfile;
double perc = 0.01;
int l = 1;
const int max_update = 10000;
bool verbose = false;
// const double EPS = 1e-3;
typedef rectscan::RectMap RectMap;
typedef rectscan::Rect Rect;

void visual_repair(vector<int>& idmap, vector<string>& c, int mh, int mw, int x, int y) {
  // idmap: stores the traversable information, -1 is blocked, otherwise is traversable
  // mh, mw: height and width of the map
  // x, y: the location the a empty tile becomes blocked

  c.resize(mh);
  for (int i=0; i<mh; i++) {
    c[i].resize(mw);
    for (int j=0; j<mw; j++) {
      if (idmap[i*mw+j]==-1) c[i][j] = '@';
      else c[i][j] = '.';
    }
  }
  c[y][x] = '*';

  auto traversable = [&](int cx, int cy) {
    if (cx >= 0 && cx < mw && cy >= 0 && cy < mh && idmap[cy*mw+cx] != -1) return true;
    else return false;
  };

  int cx, cy, cardinal_step=0;
  for (int i=0; i<4; i++) {
    direction d = (direction)(1<<i);
    int dx, dy;
    d2v(d, dx, dy);
    cx = x, cy = y;

    // update cardinal
    while (traversable(cx+dx, cy+dy)) {
      cx += dx, cy += dy, cardinal_step++;
      if (dx==0) c[cy][cx] = dy>0?'^': 'v';
      else c[cy][cx] = dx>0?'<': '>';

      // update diagonal
      int tx, ty, tdx, tdy;
      tx = cx, ty = cy;
      tdx=dx==0?1: dx, tdy=dy==0?1: dy;
      int diag_step = 0;
      while (traversable(tx+tdx, ty) && traversable(tx, ty+tdy)) {
        tx += tdx, ty += tdy, diag_step++;
        if (tdx>0) c[ty][tx] = tdy>0?'\\':'/';
        else c[ty][tx] = tdy>0?'/': '\\';
      }
      tx = cx, ty = cy, tdx=dx==0?-1: dx, tdy=dy==0?-1: dy;
      while (traversable(tx+tdx, ty) && traversable(tx, ty+tdy)) {
        tx += tdx, ty += tdy, diag_step++;
        if (tdx>0) c[ty][tx] = tdy>0?'\\':'/';
        else c[ty][tx] = tdy>0?'/': '\\';
      }
    }
    cout << d2s(d) << endl;
    for (string s: c) cout << s << endl;
    cout << endl;
  }
  for (int i=4; i<8; i++) {
    direction d = (direction)(1<<i);
    int dx, dy, diag_step = 0;
    d2v(d, dx, dy);
    cx = x, cy = y;
    while (traversable(cx+dx, cy) && traversable(cx, cy+dy)) {
      cx += dx, cy += dy, diag_step++;
      if (dx>0) c[cy][cx] = dy>0?'\\': '/';
      else c[cy][cx] = dy>0?'/': '\\';
    }

    cout << d2s(d) << endl;
    for (string s: c) cout << s << endl;
    cout << endl;
  }
}


void repair_jpsp(vector<int>& idmap, vector<int>& db, int mh, int mw, int x, int y) {
  // idmap: stores the traversable information, -1 is blocked, otherwise is traversable
  // mh, mw: height and width of the map
  // x, y: the location the a empty tile becomes blocked

  auto traversable = [&](int cx, int cy) {
    if (cx >= 0 && cx < mw && cy >= 0 && cy < mh && idmap[cy*mw+cx] != -1) return true;
    else return false;
  };

  for (int i=0; i<4; i++) {
    direction d = (direction)(1<<i);
    int dx, dy;
    d2v(d, dx, dy);

    int cx=x, cy=y, cardinal_step=0;
    // update cardinal
    while (traversable(cx+dx, cy+dy)) {
      cx += dx, cy += dy, cardinal_step++;
      db[(cy*mw+dx)*i] = cardinal_step;

      // update diagonal
      int tx = cx, ty = cy, tdx=dx==0?1: dx, tdy=dy==0?1: dy;
      int diag_step = 0;
      direction diag = v2d(tdx, tdy);
      while (traversable(tx+tdx, ty) && traversable(tx, ty+tdy)) {
        tx += tdx, ty += tdy, diag_step++;
        int idx = __builtin_ffs((int)(diag))-1;
        db[(ty*mw+tx)*8+idx] = diag_step;
      }

      tx = cx, ty = cy, tdx=dx==0?-1: dx, tdy=dy==0?-1: dy;
      diag_step = 0;
      diag = v2d(tdx, tdy);
      while (traversable(tx+tdx, ty) && traversable(tx, ty+tdy)) {
        tx += tdx, ty += tdy, diag_step++;
        int idx = __builtin_ffs((int)(diag))-1;
        db[(ty*mw+tx)*8+idx] = diag_step;
      }
      if (!traversable(cx+dx, cy) || !traversable(cx, cy+dy)) break;
    }
  }
  for (int i=4; i<8; i++) {
    direction d = (direction)(1<<i);
    int dx, dy, diag_step = 0;
    d2v(d, dx, dy);
    int cx = x, cy = y;
    while (traversable(cx+dx, cy) && traversable(cx, cy+dy)) {
      cx += dx, cy += dy, diag_step++;
      int idx = __builtin_ffs((int)(d))-1;
      db[(cy*mw+cx)*8+idx] = diag_step;
    }
  }
}

int run_jps_repair(string mfile, string rfile) {
  RectMap r(mfile.c_str());
  vector<int> db(r.mapw*r.maph*8);

  ifstream in(rfile);
  int num, x, y, h, w, f;
  in >> num;
  num = min(num, max_update);
  for (int i=0; i<num; i++) {
    in >> x >> y >> h >> w >> f;
    repair_jpsp(r.idmap, db, r.maph, r.mapw, x, y);
  }
  return num;
}

int run_rect_repair(string mfile, string rfile) {
  RectMap r(mfile.c_str());
  vector<int> db(r.mapw*r.maph*8);

  ifstream in(rfile);
  int num, x, y, h, w, f;
  in >> num;
  num = min(num, max_update);
  for (int i=0; i<num; i++) {
    in >> x >> y >> h >> w >> f;
    r.update_map(Rect(0, x, y, 1, 1), f);
  }
  return num;
}

bool gen_update_block(RectMap& r, vector<int>& ids, int l, int& x, int& y) {
// genereate a rect [x, x+l-1]*[y, y+l-1] that all tiles insides are empty;

  // try at most ids.size() times
  for (int i=0; i<(int)ids.size(); i++) {
    int id = ids[rand() % ids.size()];
    r.to_xy(id, x, y);
    bool f = true;
    for (int j=x; j<x+l && f; j++)
    for (int k=y; k<y+l && f; k++) 
    if (r.idmap[k*r.mapw+j] == -1){
      f = false; break;
    }
    if (f) return true;
  }
  return false;
}

void gen_blocked(string mfile, string sfile, vector<int>& ids, vector<Rect>& rects) {
  RectMap r(mfile.c_str());
  if (verbose) {
    cout << "before: " << endl;
    r.print(cout);
  }
  set<int> ex;
  scenario_manager smgr;
  if (!sfile.empty()) {
    smgr.load_scenario(sfile.c_str());
  }
  for (int i=0; i<(int)smgr.num_experiments(); i++) {
    int sx, sy, tx, ty;
    sx = smgr.get_experiment(i)->startx();
    sy = smgr.get_experiment(i)->starty();
    tx = smgr.get_experiment(i)->goalx();
    ty = smgr.get_experiment(i)->goaly();
    ex.insert(sy*r.mapw+sx);
    ex.insert(ty*r.mapw+tx);

    // precondition: all starts and tarets must be traversable
    REQUIRE(r.get_label(sx, sy));
    REQUIRE(r.get_label(tx, ty));
  }
  for (int x=0; x<r.mapw; x++)
  for (int y=0; y<r.maph; y++)
  if (r.idmap[y*r.mapw+x] != -1 && ex.count(y*r.mapw+x) == 0)
    ids.push_back(y*r.mapw+x);

  int num = (int)(ids.size() * perc) / l*l + 1;
  num = min(max_update, num);
  for (int i=0; i<num; i++) {
    int x, y;
    if (gen_update_block(r, ids, l, x, y)) {
      rects.push_back(Rect(0, x, y, l, l));
      r.update_map(rects.back(), false);
    }
    else break;
  }
  for (int i=0; i<(int)smgr.num_experiments(); i++) {
    int sx, sy, tx, ty;
    sx = smgr.get_experiment(i)->startx();
    sy = smgr.get_experiment(i)->starty();
    tx = smgr.get_experiment(i)->goalx();
    ty = smgr.get_experiment(i)->goaly();
    // invariant: start and target node must be traversable
    REQUIRE(r.get_label(sx, sy));
    REQUIRE(r.get_label(tx, ty));
  }
  if (verbose) {
    cout << "after: " << endl;
    r.print(cout);
  }
}

TEST_CASE("gen-block") {
  vector<vector<string>> cases = {
    // domain: bgmaps
    {"./data/AR0044SR_1.rectid", "./data/AR0044SR_1.repair", "./data/AR0044SR_1.map.scen"},
    {"./data/AR0044SR_2.rectid", "./data/AR0044SR_2.repair", "./data/AR0044SR_2.map.scen"},
    {"./data/AR0044SR_8.rectid", "./data/AR0044SR_8.repair", "./data/AR0044SR_8.map.scen"},

    {"./data/CatwalkAlley_1.rectid", "./data/CatwalkAlley_1.repair", "./data/CatwalkAlley_1.map.scen"},
    {"./data/CatwalkAlley_2.rectid", "./data/CatwalkAlley_2.repair", "./data/CatwalkAlley_2.map.scen"},
    {"./data/CatwalkAlley_8.rectid", "./data/CatwalkAlley_8.repair", "./data/CatwalkAlley_8.map.scen"},

    // domain: starcraft
    {"./data/GreenerPastures_1.rectid", "./data/GreenerPastures_1.repair", "./data/GreenerPastures_1.map.scen"},
    {"./data/GreenerPastures_2.rectid", "./data/GreenerPastures_2.repair", "./data/GreenerPastures_2.map.scen"},
    {"./data/GreenerPastures_8.rectid", "./data/GreenerPastures_8.repair", "./data/GreenerPastures_8.map.scen"},

    // domain: iron
    {"./data/scene_sp_cha_02_1.rectid", "./data/scene_sp_cha_02_1.repair", "./data/scene_sp_cha_02_1.map.scen"},
    {"./data/scene_sp_cha_02_2.rectid", "./data/scene_sp_cha_02_2.repair", "./data/scene_sp_cha_02_2.map.scen"},
    {"./data/scene_sp_cha_02_8.rectid", "./data/scene_sp_cha_02_8.repair", "./data/scene_sp_cha_02_8.map.scen"},

    // domain: room
    {"./data/64room_000_1.rectid", "./data/64room_000_1.repair", "./data/64room_000_1.map.scen"},
    {"./data/64room_000_2.rectid", "./data/64room_000_2.repair", "./data/64room_000_2.map.scen"},
    {"./data/64room_000_8.rectid", "./data/64room_000_8.repair", "./data/64room_000_8.map.scen"},

    // domain: maze512
    {"./data/maze512-32-0_1.rectid", "./data/maze512-32-0_1.repair", "./data/maze512-32-0_1.map.scen"},
    {"./data/maze512-32-0_2.rectid", "./data/maze512-32-0_2.repair", "./data/maze512-32-0_2.map.scen"},
    {"./data/maze512-32-0_8.rectid", "./data/maze512-32-0_8.repair", "./data/maze512-32-0_8.map.scen"},

    // domain: street
    {"./data/Denver_2_1024_1.rectid", "./data/Denver_2_1024_1.repair", "./data/Denver_2_1024_1.map.scen"},
    {"./data/Denver_2_1024_2.rectid", "./data/Denver_2_1024_2.repair", "./data/Denver_2_1024_2.map.scen"},
    {"./data/Denver_2_1024_8.rectid", "./data/Denver_2_1024_8.repair", "./data/Denver_2_1024_8.map.scen"},
  };

  vector<int> ids;
  vector<Rect> rects;

  if (infile.empty()) {
    for (auto& each: cases) {
      string mfile = each[0];
      outfile = each[1];
      string sfile = each[2];
      cerr << "Running on " << mfile << endl;
      ids.clear(); rects.clear();
      gen_blocked(mfile, sfile, ids, rects);

      ofstream out(outfile);
      out << rects.size() << endl;
      for (Rect& r: rects) {
        out << r.x << " " << r.y << " " << r.h << " " 
             << r.w << " " << false << endl;
      }
    }
  }
  else {
    ids.clear(); rects.clear();
    gen_blocked(infile, "", ids, rects);

    if (outfile.empty()) {
      cout << rects.size() << endl;
      for (Rect& r: rects) {
        cout << r.x << " " << r.y << " " << r.h << " " 
             << r.w << " " << false << endl;
      }
    }
    else {
      ofstream out(outfile);
      out << rects.size() << endl;
      for (Rect& r: rects) {
        out << r.x << " " << r.y << " " << r.h << " " 
             << r.w << " " << false << endl;
      }
    }
  }
}


TEST_CASE("visual_repair") {
  int mh = 20, mw = 20, x=10, y=10;
  vector<int> idmap(mh*mw, 0);
  vector<string> c;
  visual_repair(idmap, c, mh, mw, x, y);
}

void gen_changed_map(string mpath, string rpath, string opath, string spath) {
  scenario_manager smgr;
  smgr.load_scenario(spath.c_str());
  RectMap r(mpath.c_str());

  vector<string> m;
  m.resize(r.maph);
  for (int y=0; y<r.maph; y++) {
    for (int x=0; x<r.mapw; x++) m[y].push_back(r.get_label(x, y)?'.': 'T');
  }

  // precondition: before apply, all starts/targets must be empty
  for (int i=0; i<(int)smgr.num_experiments(); i++) {
    int sx, sy, tx, ty;
    sx = smgr.get_experiment(i)->startx();
    sy = smgr.get_experiment(i)->starty();
    tx = smgr.get_experiment(i)->goalx();
    ty = smgr.get_experiment(i)->goaly();
    REQUIRE(sy < (int)m.size());
    REQUIRE(ty < (int)m.size());
    REQUIRE(sx < (int)m[sy].size());
    REQUIRE(tx < (int)m[ty].size());
    // cerr << "sx: " << sx << " sy: " << sy << endl;
    REQUIRE(m[sy][sx]=='.');
    // cerr << "tx: " << tx << " ty: " << ty << endl;
    REQUIRE(m[ty][tx]=='.');
  }
  ifstream in(rpath);
  int num, x, y, h, w, f;
  in >> num;
  num = min(num, max_update);
  for (int i=0; i<num; i++) {
    in >> x >> y >> h >> w >> f;
    if (f==0) m[y][x] = 'T';
    else m[y][x] = '.';
  }
  // postcondition: after apply, still be empty
  for (int i=0; i<(int)smgr.num_experiments(); i++) {
    int sx, sy, tx, ty;
    sx = smgr.get_experiment(i)->startx();
    sy = smgr.get_experiment(i)->starty();
    tx = smgr.get_experiment(i)->goalx();
    ty = smgr.get_experiment(i)->goaly();
    REQUIRE(m[sy][sx]=='.');
    REQUIRE(m[ty][tx]=='.');
  }
  ofstream out(opath);
  out << "type octile" << endl; 
  out << "height " << r.maph << endl;
  out << "width " << r.mapw << endl;
  out << "map" << endl;
  for (string row: m) out << row << endl;

}

TEST_CASE("gen-changed") {
  vector<vector<string>> cases = {

    // domain: bgmaps
    {"./data/AR0044SR_1.rectid", "./data/AR0044SR_1.repair", "./data/changed/AR0044SR_1.map", "./data/AR0044SR_1.map.scen"},
    {"./data/AR0044SR_2.rectid", "./data/AR0044SR_2.repair", "./data/changed/AR0044SR_2.map", "./data/AR0044SR_2.map.scen"},
    {"./data/AR0044SR_8.rectid", "./data/AR0044SR_8.repair", "./data/changed/AR0044SR_8.map", "./data/AR0044SR_8.map.scen"},

    // domain: iron
    {"./data/scene_sp_cha_02_1.rectid", "./data/scene_sp_cha_02_1.repair", "./data/changed/scene_sp_cha_02_1.map", "./data/scene_sp_cha_02_1.map.scen"},
    {"./data/scene_sp_cha_02_2.rectid", "./data/scene_sp_cha_02_2.repair", "./data/changed/scene_sp_cha_02_2.map", "./data/scene_sp_cha_02_2.map.scen"},
    {"./data/scene_sp_cha_02_8.rectid", "./data/scene_sp_cha_02_8.repair", "./data/changed/scene_sp_cha_02_8.map", "./data/scene_sp_cha_02_8.map.scen"},

    // domain: starcraft
    {"./data/GreenerPastures_1.rectid", "./data/GreenerPastures_1.repair", "./data/changed/GreenerPastures_1.map", "./data/GreenerPastures_1.map.scen"},
    {"./data/GreenerPastures_2.rectid", "./data/GreenerPastures_2.repair", "./data/changed/GreenerPastures_2.map", "./data/GreenerPastures_2.map.scen"},
    {"./data/GreenerPastures_8.rectid", "./data/GreenerPastures_8.repair", "./data/changed/GreenerPastures_8.map", "./data/GreenerPastures_8.map.scen"},

    // domain: room
    {"./data/64room_000_1.rectid", "./data/64room_000_1.repair", "./data/changed/64room_000_1.map", "./data/64room_000_1.map.scen"},
    {"./data/64room_000_2.rectid", "./data/64room_000_2.repair", "./data/changed/64room_000_2.map", "./data/64room_000_2.map.scen"},
    {"./data/64room_000_8.rectid", "./data/64room_000_8.repair", "./data/changed/64room_000_8.map", "./data/64room_000_8.map.scen"},

    // domain: maze512
    {"./data/maze512-32-0_1.rectid", "./data/maze512-32-0_1.repair", "./data/changed/maze512-32-0_1.map", "./data/maze512-32-0_1.map.scen"},
    {"./data/maze512-32-0_2.rectid", "./data/maze512-32-0_2.repair", "./data/changed/maze512-32-0_2.map", "./data/maze512-32-0_2.map.scen"},
    {"./data/maze512-32-0_8.rectid", "./data/maze512-32-0_8.repair", "./data/changed/maze512-32-0_8.map", "./data/maze512-32-0_8.map.scen"},

    // domain: street
    {"./data/Denver_2_1024_1.rectid", "./data/Denver_2_1024_1.repair", "./data/changed/Denver_2_1024_1.map", "./data/Denver_2_1024_1.map.scen"},
    {"./data/Denver_2_1024_2.rectid", "./data/Denver_2_1024_2.repair", "./data/changed/Denver_2_1024_2.map", "./data/Denver_2_1024_2.map.scen"},
    {"./data/Denver_2_1024_8.rectid", "./data/Denver_2_1024_8.repair", "./data/changed/Denver_2_1024_8.map", "./data/Denver_2_1024_8.map.scen"},
  };
  for (vector<string>& each: cases) {
    cerr << "Processing: " << each[0] << ", write to: " << each[2] << endl;
    gen_changed_map(each[0], each[1], each[2], each[3]);
  }
}

TEST_CASE("preproc") {
  vector<string> cases = {
    // "./data/changed/CatwalkAlley_8.map",
    "./data/changed/GreenerPastures_8.map",
    "./data/changed/scene_sp_cha_02_8.map",
    "./data/changed/64room_000_8.map",
    "./data/changed/maze512-32-0_8.map",
    "./data/changed/Denver_2_1024_8.map"
  };
  string header = "map,alg,cost";
  timer t;
  cost_t prepc;
  ofstream out(outfile.empty()?"preproc_cost.log": outfile);
  out << header << endl;
  for (string each: cases) {
    gridmap* gmap = new gridmap(each.c_str());
    t.start();
    offline_jump_point_locator* jpl = new offline_jump_point_locator(gmap);
    t.stop();
    prepc = t.elapsed_time_nano();
    out << each << ",jpsplus," << prepc << endl;

    t.start();
    RectMap r(each.c_str());
    t.stop();
    prepc = t.elapsed_time_nano();
    out << each << ",rect," << prepc << endl;
    delete gmap;
    delete jpl;
  }
}

TEST_CASE("repair_cost") {
  vector<pair<string, string>> cases = {
    // {"./data/CatwalkAlley_1.rectid", "./data/CatwalkAlley_1.repair"},
    // {"./data/CatwalkAlley_2.rectid", "./data/CatwalkAlley_2.repair"},
    // {"./data/CatwalkAlley_8.rectid", "./data/CatwalkAlley_8.repair"},

    {"./data/GreenerPastures_1.rectid", "./data/GreenerPastures_1.repair"},
    {"./data/GreenerPastures_2.rectid", "./data/GreenerPastures_2.repair"},
    {"./data/GreenerPastures_8.rectid", "./data/GreenerPastures_8.repair"},

    {"./data/scene_sp_cha_02_1.rectid", "./data/scene_sp_cha_02_1.repair"},
    {"./data/scene_sp_cha_02_2.rectid", "./data/scene_sp_cha_02_2.repair"},
    {"./data/scene_sp_cha_02_8.rectid", "./data/scene_sp_cha_02_8.repair"},

    {"./data/64room_000_1.rectid", "./data/64room_000_1.repair"},
    {"./data/64room_000_2.rectid", "./data/64room_000_2.repair"},
    {"./data/64room_000_8.rectid", "./data/64room_000_8.repair"},

    {"./data/maze512-32-0_1.rectid", "./data/maze512-32-0_1.repair"},
    {"./data/maze512-32-0_2.rectid", "./data/maze512-32-0_2.repair"},
    {"./data/maze512-32-0_8.rectid", "./data/maze512-32-0_8.repair"},

    {"./data/AR0044SR_1.rectid", "./data/AR0044SR_1.repair"},
    {"./data/AR0044SR_2.rectid", "./data/AR0044SR_2.repair"},
    {"./data/AR0044SR_8.rectid", "./data/AR0044SR_8.repair"},

    {"./data/Denver_2_1024_1.rectid", "./data/Denver_2_1024_1.repair"},
    {"./data/Denver_2_1024_2.rectid", "./data/Denver_2_1024_2.repair"},
    {"./data/Denver_2_1024_8.rectid", "./data/Denver_2_1024_8.repair"},
  };
  timer t;
  string header = "map,alg,cost,num";
  int num;
  double crect, cjpsp;
  ofstream out(outfile.empty()?"repair_cost.log": outfile);

  out << header << endl;
  for (auto& each: cases) {
    string mfile = each.first;
    string rfile = each.second;

    t.start();
    num = run_jps_repair(mfile, rfile);
    t.stop();
    cjpsp = t.elapsed_time_nano();
    out << mfile << ",jpsp" << "," << cjpsp << "," << num << endl;

    t.start();
    run_rect_repair(mfile, rfile);
    t.stop();
    crect = t.elapsed_time_nano();
    out << mfile << ",rect" << "," << crect << "," << num << endl;
  }
}

int main(int argv, char* args[]) {
  using namespace Catch::clara;
  Catch::Session session;
  auto cli = 
      Opt(infile, "testfile")["-in"]("")
    | Opt(outfile, "outfile")["-out"]("")
    | Opt( verbose )["-v"]["--verbose"]("verbose")
    | session.cli();
  session.cli(cli);
  int resCode = session.applyCommandLine(argv, args);
  if (resCode != 0)
    return resCode;

	cerr << "Running test cases..." << endl;
  srand (0);
	return session.run(argv, args);
}
