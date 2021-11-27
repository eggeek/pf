#define CATCH_CONFIG_RUNNER

#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "catch.hpp"
#include "constants.h"
#include "gridmap.h"
#include "jps.h"
#include "octile_heuristic.h"
#include "rect_jump_point_locator.h"
#include "online_jump_point_locator2.h"
#include "rectmap.h"
#include "scenario_manager.h"
#include "jps2_expansion_policy.h"
#include "rect_expansion_policy.h"
#include "flexible_astar.h"

using namespace warthog::rectscan;
using namespace std;
bool Verbose = false;
const vector<string> desc = {
  "NORTH", "SOUTH", "EAST", "WEST",
  "NORTHEAST", "NORTHWEST", "SOUTHEAST", "SOUTHWEST"
};

TEST_CASE("gen-rect") {
  vector<pair<string, string>> cases = {
    {"../maps/dao/arena.map", "./test/rects/arena.rect"},
    {"../maps/bgmaps/AR0042SR.map", "./test/rects/AR0042SR.rect"},
    {"../maps/starcraft/CatwalkAlley.map", "./test/rects/CatwalkAlley.rect"},
    {"../maps/street/Boston_2_256.map", "./test/rects/Boston_2_256.rect"}
  };
  RectMap rectmap;
  for (auto& each: cases) {
    string mapfile = each.first;
    string writeto = each.second;
    rectmap.init(mapfile.c_str()); 
    ofstream out;
    out.open(writeto.c_str());
    rectmap.print(out);
    cout << "#rects: " << rectmap.rects.size() << endl;
    out.close();
  }
}

TEST_CASE("adj") {
  vector<pair<string, string>> cases = {
    {"./test/rects/simple0.rect", "./test/rects/simple0.adj"},
  };

  RectMap rectmap;
  for (auto &each: cases) {
    string rectfile = each.first;
    string adjfile = each.second;
    rectmap.init(rectfile.c_str());
    ifstream in;
    in.open(adjfile.c_str());
    vector<Rect> rects;
    rects.resize(rectmap.rects.size());
    for (int i=0; i < (int)rects.size(); i++) rects[i].rid = -1;
    for (auto& r: rectmap.rects) {
      rects[r.rid].read(in);
    }
    for (int i=0; i < (int)rectmap.rects.size(); i++) {
      Rect rect = rectmap.rects[i];
      REQUIRE(rects[i].equal(rect));
    }
  }
}


void cmp_jpts(vector<uint32_t> &jpts, vector<uint32_t> &jpts2, warthog::gridmap* gmap, RectMap* rmap) { 

  REQUIRE(jpts.size() == jpts2.size());
  const uint32_t id_mask = (1 << 24) - 1;
  set<int> m1, m2;
  for (int i=0; i<(int)jpts.size(); i++) {
    int x, y;
    uint32_t x2, y2;
    rmap->to_xy(jpts[i] & id_mask, x, y);
    gmap->to_unpadded_xy(jpts2[i] & id_mask, x2, y2);
    
    m1.insert(rmap->to_id(x, y));
    m2.insert(y * gmap->header_width() + x);
  }
  for (auto &it: m1) {
    REQUIRE(m2.count(it) == 1);
  }
  for (auto &it: m2) {
    REQUIRE(m1.count(it) == 1);
  }
}


TEST_CASE("jump") {
  vector<string> cases = {
    "./test/rects/simple0.rect",
    "./test/rects/arena.rect",
    "../maps/rooms/64room_000.map",
    "../maps/starcraft/CatwalkAlley.map"
  };
  for (auto &each: cases) {
    string rectfile = each;
    cout << "map: " << rectfile << endl;
    // check if it is end with .map
    RectMap rectmap(rectfile.c_str());
    warthog::gridmap* gmap = rectmap.gmap;
    REQUIRE(rectmap.equal(*gmap));
    warthog::jps::online_jump_point_locator2* jpl2 = 
      new warthog::jps::online_jump_point_locator2(gmap);
    rect_jump_point_locator jpl = rect_jump_point_locator(&rectmap); 

    vector<uint32_t> jpts; 
    vector<uint32_t> jpts2;
    vector<warthog::cost_t> costs2;
    for (int d=0; d<8; d++)
    for (int x=0; x<rectmap.mapw; x++) {
      for (int y=0; y<rectmap.maph; y++) {
        warthog::jps::direction dir = (warthog::jps::direction)(1<<d);
        int padded_nid = gmap->to_padded_id(x, y);
        if (gmap->get_label(padded_nid) == 0) continue;
        Rect* r = rectmap.get_rect(x, y);

        // cout << "id: " << r->rid << ", x: " << x << ", y: " << y
        //      << " d: " << d << " " << desc[d] << endl;

        jpts2.clear(); costs2.clear();
        jpl2->jump(dir, padded_nid, INF, jpts2, costs2);

        jpl.get_jpts().clear();
        jpl.get_costs().clear();
        jpl.jump(dir, y*rectmap.mapw+x, INF, r);
        jpts = jpl.get_jpts();
        cmp_jpts(jpts, jpts2, gmap, &rectmap);
      }
    }
    delete jpl2;
  }
}

TEST_CASE("speed-empty") {
  vector<int> idmaps;

  vector<vector<int>> cases = {
    // h,  w,  side
    {256, 256, 1},
    {256, 256, 16},
    {256, 256, 32},
    {256, 256, 64},
    {512, 512, 2},
    {512, 512, 16},
    {512, 512, 32},
    {512, 512, 64},
    {1024, 1024, 4},
    {1024, 1024, 8},
    {1024, 1024, 16},
    {1024, 1024, 32},
    {1024, 1024, 64},
    {2048, 2048, 8},
    {4096, 4096, 16},
    {4096, 4096, 32},
    {4096, 4096, 64},
    {4096, 4096, 128},
    {4096, 4096, 256},
  };
  RectMap rmap;
  string header = "len\tside\tmem\tloc\tconvert\tcalcLR\trect\tjps2\tspeed";
  string header2 = "len\tside\tmem\trscan\tbscan\tscan_speed";
  vector<vector<int>> mapsize;
  vector<pair<double, double>> scancosts;
  cout << header << endl;
  for (auto& c: cases) {
    int maph = c[0], mapw = c[1], s = c[2];
    idmaps.resize(maph * mapw);
    for (int y=0; y<maph; y++)
    for (int x=0; x<mapw; x++) {
      int block_id = (y / s) * (mapw / s) + (x / s);
      idmaps[y * mapw + x] = block_id;
    }
    rmap.init(maph, mapw, idmaps);
    REQUIRE((int)rmap.rects.size() == (maph*mapw) / (s*s));
    rect_jump_point_locator jpl = rect_jump_point_locator(&rmap); 
    // jpl.set_minarea(0);
    jpl.set_minstep(0);

    warthog::gridmap* gmap = rmap.gmap;
    warthog::jps::online_jump_point_locator2* jpl2 = 
      new warthog::jps::online_jump_point_locator2(gmap);

    REQUIRE(rmap.equal(*gmap));
    for (int rep=0; rep<1; rep++) {
      warthog::timer t;
      long long cost_rect, cost_jp2, cost_loc, cost_convert, cost_scanLR;
      double rscan_cost, bscan_cost;
      int sx = 0, sy = 0, tx = mapw-1, ty = maph-1, id;
      uint32_t unpad_sid = rmap.to_id(sx, sy), unpad_tid = rmap.to_id(tx, ty);
      uint32_t pad_sid = gmap->to_padded_id(unpad_sid), pad_tid = gmap->to_padded_id(unpad_tid);
      vector<uint32_t> jpts; 
      vector<warthog::cost_t> cost;

      t.start();
      jpl2->jump(warthog::jps::direction::SOUTHEAST, pad_sid, pad_tid, jpts, cost);
      t.stop();
      cost_jp2 = t.elapsed_time_nano();
      bscan_cost = (double)cost_jp2 / (maph*mapw/32);

      t.start();
      jpl.jump(warthog::jps::direction::SOUTHEAST, unpad_sid, unpad_tid, rmap.get_rect(rmap.idmap[sy*rmap.mapw+sx]));
      t.stop();
      cost_rect = t.elapsed_time_nano();
      rscan_cost = (double)cost_rect / (mapw*maph/(s*s));

      srand(0);
      vector<int> xs, ys;
      for (int x=0; x<mapw; x++) xs.push_back(x);
      for (int y=0; y<maph; y++) ys.push_back(y);
      random_shuffle(xs.begin(), xs.end());
      random_shuffle(ys.begin(), ys.end());
      t.start();
      for (int x: xs)
      for (int y: ys) {
        rmap.get_rect(x, y);
      }
      t.stop();
      cost_loc = t.elapsed_time_nano();

      t.start();
      for (int x: xs)
      for (int y: ys) {
        id = gmap->to_padded_id(y*mapw+x);
      }
      t.stop();
      cost_convert = t.elapsed_time_nano();

      t.start();
      for (int i=0; i<4; i++)
      for (int x: xs)
      for (int y: ys) {
        Rect* r = rmap.get_rect(y*rmap.mapw+x);
        int dx = warthog::dx[i], dy = warthog::dy[i];
        r->disLR(rdirect::L, dx, dy, x, y);
        r->disLR(rdirect::R, dx, dy, x, y);
        r->disF(dx, dy, x, y);
      }
      t.stop();
      cost_scanLR = t.elapsed_time_nano();

      cout << maph << "\t"
           << s << "\t"
           << (rmap.rects.size() * sizeof(Rect)>>10) << "Kb\t" 
           << cost_loc << "\t" << cost_convert << "\t" << cost_scanLR << "\t"
           << cost_rect << "\t" << cost_jp2 << "\t" 
           << (double)cost_jp2 / (double)cost_rect << endl;
      scancosts.push_back({rscan_cost, bscan_cost});
      mapsize.push_back({maph,s,(int)(rmap.rects.size() * sizeof(Rect)>>10)});
    }
    delete jpl2;
  }

  cout << "\n" << header2 << endl;
  for (int i=0; i<(int)scancosts.size(); i++) {
    int len = mapsize[i][0];
    int s   = mapsize[i][1];
    int mem = mapsize[i][2];
    double rscan_cost = scancosts[i].first;
    double bscan_cost = scancosts[i].second;
    cout << len << "\t" << s << "\t" << mem << "\t"
         << rscan_cost << "\t" << bscan_cost << "\t"
         << bscan_cost / rscan_cost << endl;
  }
}

TEST_CASE("intervalScan") {
  vector<string> cases = {
    "./test/rects/simple0.rect",
    "./test/rects/arena.rect",
    "../maps/rooms/64room_000.map",
    "../maps/starcraft/CatwalkAlley.map"
  };
  for (auto &each: cases) {
    string rectfile = each;
    cout << "map:" << rectfile << endl; 
    RectMap rectmap(rectfile.c_str());
    warthog::gridmap* gmap = new warthog::gridmap(rectfile.c_str());
    warthog::jps::online_jump_point_locator2* jpl2 = 
      new warthog::jps::online_jump_point_locator2(gmap);
    rect_jump_point_locator jpl = rect_jump_point_locator(&rectmap); 

    vector<uint32_t> jpts; 
    vector<uint32_t> jpts2;
    vector<warthog::cost_t> costs, costs2;
    for (auto& it: rectmap.rects) {
      for (int d=0; d<4; d++) {
        warthog::jps::direction dir = (warthog::jps::direction)(1<<d);
        int dx = warthog::dx[d];
        int dy = warthog::dy[d];
        int lb=0, ub=0, ax;
        eposition cure = R2E(dx, dy, rdirect::B);
        it.get_range(cure, lb, ub);
        ax = it.axis(cure);
        jpts2.clear(); costs2.clear();
        // cout << "id: " << it.rid << ", lb: " << lb << ", ub: " << ub
        //      << ", ax: " << ax << ", d: " << desc[d] << endl;
        for (int i=lb; i<=ub; i++) {
          int padded_nid = gmap->to_padded_id(dx?ax: i, dx?i: ax);
          jpl2->jump(dir, padded_nid, INF, jpts2, costs2);
        }

        jpl.get_jpts().clear(); jpl.get_costs().clear();
        switch (d) {
          case 0:
            jpl.scanInterval<0, -1>(lb, ub, &(it));
            break;
          case 1:
            jpl.scanInterval<0, 1>(lb, ub, &(it));
            break;
          case 2:
            jpl.scanInterval<1, 0>(lb, ub, &(it));
            break;
          default:
            jpl.scanInterval<-1, 0>(lb, ub, &(it));
            break;
        };
        jpts = jpl.get_jpts();
        cmp_jpts(jpts, jpts2, gmap, &rectmap);
      }
    }
    delete gmap;
    delete jpl2;
  }
}

void run_scen(warthog::scenario_manager& scenmgr, string mapfile, bool verbose=false) {

  warthog::gridmap map(mapfile.c_str());
  warthog::jps2_expansion_policy expander(&map);
  warthog::octile_heuristic heuristic(map.width(), map.height());
  warthog::pqueue_min open;
  warthog::flexible_astar<
    warthog::octile_heuristic,
    warthog::jps2_expansion_policy,
    warthog::pqueue_min> jps2(&heuristic, &expander, &open);

  RectMap rmap(mapfile.c_str());
  rect_expansion_policy rexpan(&rmap);
  warthog::octile_heuristic rheur(rmap.mapw, rmap.maph);
  warthog::pqueue_min open2;
  warthog::flexible_astar<
    warthog::octile_heuristic,
    rect_expansion_policy,
    warthog::pqueue_min> ra(&rheur, &rexpan, &open2);

  for (int i=0; i<(int)scenmgr.num_experiments(); i++) {
    warthog::experiment* exp = scenmgr.get_experiment(i);
    uint32_t sid = exp->starty() * exp->mapwidth() + exp->startx();
    uint32_t tid = exp->goaly() * exp->mapwidth() + exp->goalx();
    warthog::problem_instance pi(sid, tid, verbose);
    warthog::solution jps2_sol, ra_sol;
    jps2.get_path(pi, jps2_sol);
    ra.get_path(pi, ra_sol);
    REQUIRE(abs(jps2_sol.sum_of_edge_costs_ - ra_sol.sum_of_edge_costs_) == 0);
  }
}

TEST_CASE("query") {
  vector<pair<string, string>> cases = {
    {"../maps/dao/arena.map", "../scenarios/movingai/dao/arena.map.scen"},
    {"../maps/dao/brc202d.map", "../scenarios/movingai/dao/brc202d.map.scen"},
    {"../maps/starcraft/CatwalkAlley.map", "../scenarios/movingai/starcraft/CatwalkAlley.map.scen"},
  };
  for (auto& it: cases) {
    string mpath = it.first;
    string spath = it.second;
    warthog::scenario_manager* scenmgr = new warthog::scenario_manager();
    scenmgr->load_scenario(spath.c_str());
    cerr << "Running map: " << mpath << endl;
    run_scen(*scenmgr, mpath, Verbose);
    delete scenmgr;
  }
}

int main(int argv, char* args[]) {
  using namespace Catch::clara;
  Catch::Session session;
  auto cli = Opt( Verbose )["-v"]["--verbose"]("verbose") | session.cli();
  session.cli(cli);
  int resCode = session.applyCommandLine(argv, args);
  if (resCode != 0)
    return resCode;

	cerr << "Running test cases..." << endl;
	return session.run(argv, args);
}
