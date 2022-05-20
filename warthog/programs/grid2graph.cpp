#include "gridmap.h"
#include "xy_graph.h"
#include "scenario_manager.h"

#include <cstring>
#include <errno.h>
#include <iostream>
#include <memory>
#include <map>
using namespace std;
using namespace warthog;

const int cw = 10; // cardinal weight
const int dw = 14; // diagonal weight
const int blocked = 100000000; // obstacle weight
const int edge_w[] = {cw, cw, cw, cw, dw, dw, dw, dw};
map<pair<int,int>, int> vertid;
int mapw, maph;

void
help()
{
    std::cerr 
       << "Converts from the format used at the Grid-based Path Planning Competition "
       << "\nand the xy_graph format used by the Warthog Pathfinding Library\n"
       << "\n"
       << "Usage: ./grid2graph [map | scen] [grid file]"
       << "\n\nParameter descriptions: " 
       << "\n\tmap: convert directly from a grid map to an xy_graph"
       << "\n\tscen: convert a gridmap scenario file into an xy_graph problem file\n";

}

bool traversable(int x, int y, gridmap* gm) {
  if (x < 0 || x >= mapw || y < 0 || y >= maph || 
      gm->get_label(gm->to_padded_id(x, y)) == 0) return false;
  return true;
}

void init_mapinfo(gridmap* gm) {
  mapw = gm->header_width(), maph = gm->header_height();
  int cnt = 0;
  vertid.clear();
  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++) 
  if (traversable(x, y, gm)) {
    vertid[{x, y}] = ++cnt;
  }
}

void grid2arcs(warthog::gridmap* gm, vector<int>& u, vector<int>& v, vector<int>& w) {
  init_mapinfo(gm);
  u.clear(), v.clear(), w.clear();
  for (auto& it: vertid) {
    pair<int, int> loc = it.first;
    int x = loc.first, y = loc.second;
    for (int i=0; i<8; i++) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if (traversable(nx, ny, gm)) {
        u.push_back(vertid[{x, y}]);
        v.push_back(vertid[{nx, ny}]);
        w.push_back(edge_w[i]);
      }
    }
  }
}

void scen2queries(string sfile, vector<int>& u, vector<int>& v) {
  scenario_manager smgr;
  smgr.load_scenario(sfile.c_str());
  u.clear(), v.clear();
  for (int i=0; i<(int)smgr.num_experiments(); i++) {
    int sx, sy, tx, ty;
    sx = smgr.get_experiment(i)->startx();
    sy = smgr.get_experiment(i)->starty();
    tx = smgr.get_experiment(i)->goalx();
    ty = smgr.get_experiment(i)->goaly();
    u.push_back(vertid[{sx, sy}]);
    v.push_back(vertid[{tx, ty}]);
  }
}

void apply_changes(gridmap* gm, string rfile, 
    vector<int>& du, vector<int>& dv, vector<int>& dw) {
  du.clear(), dv.clear(), dw.clear();
  ifstream fin(rfile);
  int num;
  fin >> num;
  for (int i=0; i<num; i++) {
    int x, y, tmp;
    fin >> x >> y >> tmp >> tmp >> tmp;
    for (int j=0; j<8; j++) {
      int px = x - dx[j];
      int py = y - dy[j];
      if (traversable(px ,py, gm)) {
        du.push_back(vertid[{x, y}]);
        dv.push_back(vertid[{px, py}]);
        dw.push_back(blocked);

        du.push_back(vertid[{px, py}]);
        dv.push_back(vertid[{x, y}]);
        dw.push_back(blocked);
      }
    }
    gm->set_label(gm->to_padded_id(x, y), false);
  }
}

void grid2dimacs(string mfile, string sfile, string rfile, 
    string gfile, string cofile, string qfile, string dfile) {
  cerr << "Translate gridmap: [" << mfile << "] To graph: " << gfile << endl;
  cerr << "Scen: [" << sfile << "] To query: [" << qfile << "]" << endl;
  cerr << "Repair: [" << rfile << "] To diff: [" << dfile << "]" << endl;

  gridmap gm(mfile.c_str());
  init_mapinfo(&gm);
  vector<int> u, v, w;
  grid2arcs(&gm, u, v, w);
  assert(u.size() == v.size() && v.size() == w.size());

  // save gr file
  ofstream out;
  out.open(gfile);
  out << "p sp " << vertid.size() << " " << u.size() << endl;
  for (int i=0; i<(int)u.size(); i++) {
    out << "a " << u[i] << " " << v[i] << " " << w[i] << endl;
  }
  out.close();

  // save co file
  out.open(cofile);
  out << "p aux sp co " << vertid.size() << endl;
  for (auto& it: vertid) {
    int id = it.second;
    int x = it.first.first;
    int y = it.first.second;
    out << "v " << id << " " << x << " " << y << endl;
  }
  out.close();

  vector<int> qu, qv;
  scen2queries(sfile, qu, qv);
  assert(qu.size() == qv.size());

  // save query file
  out.open(qfile);
  out << "p aux sp p2p-zero " << qu.size() << " pcost" << endl;
  for (int i=0; i<(int)qu.size(); i++) {
    out << "q " << qu[i] << " " << qv[i] << endl;
  }
  out.close();

  // save diff file
  vector<int> du, dv, dw;
  apply_changes(&gm, rfile, du, dv, dw);
  assert(du.size() == dv.size() && dv.size() == dw.size());
  out.open(dfile);
  out << du.size() << endl;
  for (int i=0; i<(int)du.size(); i++) {
    out << du[i] << " " << dv[i] << " " << dw[i] << endl;
  }
  out.close();
}

int 
main(int argc, char** argv)
{
    if (argc == 8) {
      grid2dimacs(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6], 
          argv[7]);
    }
    else {
      if(argc != 3)
      {
      help();
          exit(0);
      }


      if(strcmp(argv[1], "map") == 0)
      {
          warthog::gridmap gm(argv[2]);
          warthog::graph::xy_graph g;
          warthog::graph::gridmap_to_xy_graph(&gm, &g);
          std::cout << g;
      }
      else if(strcmp(argv[1], "scen") == 0)
      {
          warthog::scenario_manager scenmgr;
          scenmgr.load_scenario(argv[2]);
          if(scenmgr.num_experiments() == 0)
          {
              std::cerr << "warning: scenario file contains no experiments\n";
              return 0;
          }

          // we need to convert (x, y) coordinates into graph ids
          // so we load up the associated grid and map ids
          warthog::gridmap gm(scenmgr.get_experiment(0)->map().c_str());
          warthog::gridmap_expansion_policy exp(&gm);

          // xy graph ids are assigned by 
          std::vector<uint32_t> id_map(gm.header_height() * gm.header_width());
          uint32_t next_graph_id = 0;
          for(uint32_t y = 0; y < gm.header_height(); y++)
          {
              for(uint32_t x = 0; x < gm.header_width(); x++)
              {
                  // skip obstacles
                  uint32_t from_gm_id = y * gm.header_width() + x;
                  if(!gm.get_label(gm.to_padded_id(from_gm_id))) 
                  { continue; }

                  // add graph node (we scale up all costs and coordinates)
                  id_map[from_gm_id] = next_graph_id++;
              }
          }

          std::cout 
              << "c Zero-indexed point-to-point problem instances, converted from the gridmap scenario file\n"
              << "c " << argv[2] << std::endl
              << "c Each point identifies a traversable grid tile and the ids are generated by\n"
              << "c scanning the associated grid map left-to-right and top-to-bottom\n"
              << std::endl;

          std::cout
              << "p aux sp p2p-zero " << scenmgr.num_experiments() << std::endl;

          for(uint32_t i = 0; i < scenmgr.num_experiments(); i++)
          {
              warthog::experiment* exp = scenmgr.get_experiment(i);
              uint32_t start_id = 
                  exp->starty() * exp->mapwidth() + exp->startx();
              uint32_t goal_id  = 
                  exp->goaly() * exp->mapwidth() + exp->goalx();
              std::cout << "q " << id_map[start_id] << " " << id_map[goal_id] << std::endl;
          }
      }
      else
      {
          std::cerr << "err; must specify type of conversion and file\n";
          return EINVAL;
      }

    }
    return 0;
}

