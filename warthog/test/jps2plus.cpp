#define CATCH_CONFIG_RUNNER
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "catch.hpp"
#include "constants.h"
#include "jps.h"
#include "octile_heuristic.h"
#include "gridmap.h"
#include "scenario_manager.h"
#include "jps2_expansion_policy.h"
#include "jps2plus_expansion_policy.h"
#include "flexible_astar.h"

using namespace warthog;
using namespace std;

bool Verbose = false;
const vector<string> desc = {
  "NORTH", "SOUTH", "EAST", "WEST",
  "NORTHEAST", "NORTHWEST", "SOUTHEAST", "SOUTHWEST"
};

void run_scen(warthog::scenario_manager& scenmgr, string mapfile, bool verbose=false) {

  gridmap map(mapfile.c_str());
  octile_heuristic heuristic(map.width(), map.height());
  jps2_expansion_policy expander(&map);
  pqueue_min open;
  flexible_astar<
    octile_heuristic,
    jps2_expansion_policy,
    pqueue_min> jps2(&heuristic, &expander, &open);

  jps2plus_expansion_policy jps2p_exp(&map);
  pqueue_min open2;
  flexible_astar<
    octile_heuristic,
    jps2plus_expansion_policy,
    pqueue_min> jps2p(&heuristic, &jps2p_exp, &open2);

  for (int i=0; i<(int)scenmgr.num_experiments(); i++) {
    warthog::experiment* exp = scenmgr.get_experiment(i);
    uint32_t sid = exp->starty() * exp->mapwidth() + exp->startx();
    uint32_t tid = exp->goaly() * exp->mapwidth() + exp->goalx();
    warthog::problem_instance pi(sid, tid, verbose);
    warthog::solution jps2_sol, ra_sol;
    jps2.get_path(pi, jps2_sol);
    jps2p.get_path(pi, ra_sol);
    REQUIRE(abs(jps2_sol.sum_of_edge_costs_ - ra_sol.sum_of_edge_costs_) == 0);
  }
}

TEST_CASE("query") {
  vector<pair<string, string>> cases = {
    {"../maps/dao/arena.map", "../scenarios/movingai/dao/arena.map.scen"},
    {"../maps/dao/brc202d.map", "../scenarios/movingai/dao/brc202d.map.scen"},
    {"../maps/starcraft/CatwalkAlley.map", "../scenarios/movingai/starcraft/CatwalkAlley.map.scen"},
    // add a large map to test whether it support large state space instead of 2^24
    // {"./data/CatwalkAlley_8.map", "./data/CatwalkAlley_8.map.scen"}
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

	cout << "Running test cases..." << endl;
	return session.run(argv, args);
}
