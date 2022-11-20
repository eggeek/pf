#include "constants.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "jps2_expansion_policy.h"
#include "jps2_expansion_policy_prune2.h"
#include "octile_heuristic.h"
#include "zero_heuristic.h"
#include "scenario_manager.h"
#include "timer.h"
#include "global.h"

#include <stdlib.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <map>

using namespace std;
namespace w = warthog;
namespace G = global;
const double EPS = 1e-6;

struct ExpData {
  long long exp, gen, scan, pruneable;
  // count subopt gval:
  long long subopt_expd, subopt_gen;
  double time;
  void reset() {
    exp = gen = scan = pruneable = 0;
    subopt_expd = subopt_gen = 0;
  }
  void update(const w::solution& si) {
    exp += si.nodes_expanded_;
    // gen += si.nodes_touched_;
    gen += si.nodes_inserted_;
    time += si.time_elapsed_nano_;
    scan += G::statis::scan_cnt;
  }

  void update_subopt() {
    subopt_expd += G::statis::subopt_expd;
    // subopt_gen += G::statis::subopt_gen;
    subopt_gen += G::statis::subopt_insert;
    pruneable += G::statis::prunable;
  }

  string str() {
    string res = "";
    res += to_string(exp) + "\t" + 
           to_string(gen) + "\t" + 
           to_string((long long)time) + "\t" + to_string(scan);
    return res;
  }

  string subopt_str() {
    string res = "";
    res += to_string(subopt_gen) + "\t" +
           to_string(gen) + "\t" +
           to_string(subopt_expd) + "\t" +
           to_string(pruneable) + "\t" + 
           to_string(exp) + "\t" +
           to_string(scan);
    return res;
  }

  bool valid() {
    if (subopt_expd > exp ||
        subopt_gen > gen ||
        pruneable > subopt_gen) return false;
    return true;
  }
};
string mfile, sfile, type;
bool verbose = false;

void subcnt() {
  w::gridmap map(mfile.c_str());
  w::scenario_manager scenmgr;
  scenmgr.load_scenario(sfile.c_str());
  w::octile_heuristic heur(map.width(), map.height());
  w::zero_heuristic zheur;
  w::pqueue_min open;
  w::gridmap_expansion_policy expd_g(&map);
  w::jps2_expansion_policy expd_jps2(&map);
  w::jps2_expansion_policy_prune2 expd_cjps2(&map);


  w::flexible_astar<
    w::zero_heuristic, 
    w::gridmap_expansion_policy, 
    w::pqueue_min> dij(&zheur, &expd_g, &open);

  w::flexible_astar<
    w::octile_heuristic, 
    w::jps2_expansion_policy, 
    w::pqueue_min> jps2(&heur, &expd_jps2, &open);

  w::flexible_astar<
    w::octile_heuristic, 
    w::jps2_expansion_policy_prune2, 
    w::pqueue_min> cjps2(&heur, &expd_cjps2, &open);

  ExpData cnt_jps2, cnt_cjps2;
  ExpData* cnts[] = {&cnt_jps2, &cnt_cjps2};
  for (auto& it: cnts) it->reset();

  string header = "map\tid\tsubopt_gen\ttot_gen\tsubopt_expd\tpruneable\ttot_expd\tscnt\talg";
  cout << header << endl;
  int fromidx = max((int)scenmgr.num_experiments() - 100, 0);
  int toindx = (int)scenmgr.num_experiments();
  // int fromidx = 1960;
  // int toindx = 1961;

  G::query::map = &map;
  G::query::open = &open;
  for (int i=fromidx; i<toindx; i++) {
    w::experiment* exp = scenmgr.get_experiment(i);
    uint32_t sid = exp->starty() * exp->mapwidth() + exp->startx();
    uint32_t tid = exp->goaly() * exp->mapwidth() + exp->goalx();
    // cerr << i << " " << exp->startx() << " " << exp->starty() << " " << exp->distance() << endl;
    warthog::problem_instance pi_jps2(sid, tid, verbose);
    warthog::problem_instance pi_cjps2(sid, tid, verbose);
    warthog::problem_instance pi_dij(sid, w::SN_ID_MAX, verbose);
    warthog::solution sol_dij, sol_jps, sol_cjps;
    G::statis::clear();
    G::statis::init_dist(map.width() * map.height());
    G::sol = &sol_dij;
    dij.get_path(pi_dij, sol_dij);

    G::statis::clear();
    G::nodepool = expd_jps2.get_nodepool();
    G::sol = &sol_jps;
    jps2.get_path(pi_jps2, sol_jps);
    cnt_jps2.update(sol_jps);
    cnt_jps2.update_subopt();

    G::statis::clear();
    G::nodepool = expd_cjps2.get_nodepool();
    G::sol = &sol_cjps;
    cjps2.get_path(pi_cjps2, sol_cjps);
    cnt_cjps2.update(sol_cjps);
    cnt_cjps2.update_subopt();

    cout << mfile << "\t" << i << "\t" << cnt_jps2.subopt_str() << "\tjps2" << endl;
    cout << mfile << "\t" << i << "\t" << cnt_cjps2.subopt_str() << "\tc2jps2" << endl;

    if ((!cnt_cjps2.valid()) || (!cnt_jps2.valid())) {
      cerr << i << " " << exp->startx() << " " << exp->starty() << " " << exp->distance() << endl;
      exit(0);
    }
    for (auto &i: cnts) i->reset();
    assert(map.get_label(map.to_padded_id(sid)) != 0);
    assert(map.get_label(map.to_padded_id(tid)) != 0);
  }
  G::query::clear();
}

void get_full_path(const vector<w::sn_id_t>& p, 
    vector<w::sn_id_t>& fullp, w::gridmap &map) {
  fullp.clear();
  if (p.size() == 0) return; // no path

  fullp.push_back(p[0]);
  for (size_t i=0; i+1<p.size(); i++) {
    uint32_t cx, cy, nx, ny;
    map.to_padded_xy(p[i], cx, cy);
    map.to_padded_xy(p[i+1], nx, ny);
    int dx = nx > cx?1: -1;
    int dy = ny > cy?1: -1;
    while (cx != nx || cy != ny) {
      if (cx != nx && cy != ny) { cx += dx, cy += dy; } 
      else if (cx != nx) { cx += dx; }
      else if (cy != ny) { cy += dy; }
      fullp.push_back(cy * map.width() + cx);
    }
  }
  assert(fullp[0] == p[0]);
  assert(fullp.back() == p.back());
}

void inc_subopt_expr(int id=-1) {
  warthog::scenario_manager scenmgr;
  scenmgr.load_scenario(sfile.c_str());
  warthog::experiment* exp;
  if (id == -1) exp = scenmgr.get_experiment(scenmgr.num_experiments()-1);
  else exp = scenmgr.get_experiment(id);

  uint32_t startid = exp->starty() * exp->mapwidth() + exp->startx();
  uint32_t goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
  warthog::problem_instance pi(startid, goalid, verbose);
  warthog::problem_instance pi_dij(startid, w::SN_ID_MAX, verbose);
  warthog::solution sol_jps, sol_cjps, sol_dij;
  ExpData cnt_jps2, cnt_cjps2;
  ExpData* cnts[] = {&cnt_jps2, &cnt_cjps2};

  warthog::gridmap map(mfile.c_str());
  warthog::octile_heuristic heur(map.width(), map.height());
  warthog::zero_heuristic zheur;
  warthog::pqueue_min open;
  warthog::gridmap_expansion_policy expd_g(&map);
  warthog::jps2_expansion_policy expd_jps2(&map);
  warthog::jps2_expansion_policy_prune2 expd_cjps2(&map);

  warthog::flexible_astar<
    warthog::zero_heuristic,
    warthog::gridmap_expansion_policy,
    warthog::pqueue_min> dij(&zheur, &expd_g, &open);

  warthog::flexible_astar<
    warthog::octile_heuristic,
    warthog::jps2_expansion_policy,
    warthog::pqueue_min> jps2(&heur, &expd_jps2, &open);

  warthog::flexible_astar<
    warthog::octile_heuristic,
    warthog::jps2_expansion_policy_prune2,
    warthog::pqueue_min> cjps2(&heur, &expd_cjps2, &open);

  const double ratio = 0.2;
  std::map<warthog::sn_id_t, bool> pert_rec;  // perturbation records
  int maxit = int(map.get_num_traversable_tiles() * ratio);
  pert_rec[startid] = map.get_label(startid);
  pert_rec[goalid] = map.get_label(goalid);
  // force start and target traversable
  expd_jps2.perturbation(startid, 1); 
  expd_cjps2.perturbation(startid, 1);
  expd_jps2.perturbation(goalid, 1); 
  expd_cjps2.perturbation(goalid, 1);

  // start exp
  srand( 0 );
  vector<w::sn_id_t> fullp;
  G::query::map = &map;
  G::query::open = &open;
  string header = "map\tid\tsubopt_gen\ttot_gen\tsubopt_expd\tpruneable\ttot_expd\tscnt\talg\tdist";
  cout << header << endl;
  for (int i=0; i<maxit; i++) {
    // clear ExpData cnt
    for (auto& it: cnts) it->reset();
    // run dij
    G::clear();
    G::statis::init_dist(map.width() * map.height());
    G::sol = &sol_dij;
    dij.get_path(pi_dij, sol_dij);

    // run jps2
    G::clear(); 
    G::nodepool = expd_jps2.get_nodepool();
    G::sol = &sol_jps;
    sol_jps.reset();
    jps2.get_path(pi, sol_jps);
    cnt_jps2.update(sol_jps);
    cnt_jps2.update_subopt();

    // run cjps2
    G::clear();
    G::nodepool = expd_cjps2.get_nodepool();
    G::sol = &sol_cjps;
    sol_cjps.reset();
    cjps2.get_path(pi, sol_cjps);
    cnt_cjps2.update(sol_cjps);
    cnt_cjps2.update_subopt();


    cout << mfile << "\t" << i << "\t" << cnt_jps2.subopt_str() << "\tjps2\t" << sol_jps.sum_of_edge_costs_ << endl;
    cout << mfile << "\t" << i << "\t" << cnt_cjps2.subopt_str() << "\tc2jps2\t" << sol_cjps.sum_of_edge_costs_ << endl;

    assert(fabs(sol_jps.sum_of_edge_costs_ - sol_cjps.sum_of_edge_costs_) < EPS);

    // not middle nodes, terminate
    get_full_path(sol_jps.path_, fullp, map);
    if (fullp.size() <= 2) break;
    // perturbation
    w::sn_id_t tid = fullp[1 + rand() % (fullp.size() - 2)];
    pert_rec[tid] = map.get_label(tid);
    expd_jps2.perturbation(tid, 0);
    expd_cjps2.perturbation(tid, 0);
  }

  // reset map
  for (auto& it: pert_rec) {
    expd_jps2.perturbation(it.first, it.second);
    expd_cjps2.perturbation(it.first, it.second);
  }
}

int main(int argc, char** argv) {
  // run expr and report suboptimal: ./expriment <map> <scen> subcnt
  // run expr and report incremental suboptimal: ./expriment <map> <scen> subcnt_inc
  // [suboptimal] step: for each query, run dijkstra first, 
  // then run jps2 and cjps2 and count the number of suboptimal node expansion/generation
  // [incremental suboptimal] step:
  // 0. set fix random seed
  // 1. choose the longest query (the last one) from the scenario file,
  // 2. run the query, and report suboptimal expansion and generation
  // 3. randomly block a set of nodes on the shortest path of JPS
  // 4. go step 2, repeat certain times, or until the path is fully blocked.
  mfile = string(argv[1]);
  sfile = string(argv[2]);
  type = string(argv[3]);
  if (type == "inc_subopt") {
    inc_subopt_expr(0);
  }
  else if (type == "subcnt") {
    subcnt();
  }
}
