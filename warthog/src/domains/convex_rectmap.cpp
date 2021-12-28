#include "convex_rectmap.h"
#include "jps.h"
#include "online_jump_point_locator2.h"

using namespace std;
using namespace warthog;
typedef convrectscan::ConvRectMap cr;
typedef jps::online_jump_point_locator2 locator;
namespace cg=convrectgen;

cr::ConvRectMap(const string& mfile) {
  init(mfile);
  assert(mapw == (int)gmap->header_width());
  assert(maph == (int)gmap->header_height());
}

gridmap* cr::create_gmap_from_idmap(vector<int>& _idmap) {
  gridmap* res = new gridmap((uint32_t)maph, (uint32_t)mapw);
  uint32_t cnt = 0;
  res->set_num_traversable_tiles(cnt);
  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++) {
    uint32_t id = y*mapw + x;
    if (_idmap[id] != -1) {
      cnt++;
      res->set_num_traversable_tiles(cnt);
      res->set_label(res->to_padded_id(id), 1);
    }
    else res->set_label(res->to_padded_id(id), 0);
  }
  return res;
}

vector<int> calc_adj(
    vector<int>& idmap, int mapw, int maph, 
    int xl, int xr, int yl, int yr) {
  set<int> adj_set;
  vector<int> adj;
  adj.clear();
  // adjacent rects are in order left-right, top-down
  for (int y=yl; y<=yr; y++)
  for (int x=xl; x<=xr; x++) 
  if (x>=0 && x<mapw && y>=0 && y< maph && idmap[y*mapw+x] != -1) {
    if (adj_set.find(idmap[y*mapw+x]) == adj_set.end()) {
      adj_set.insert(idmap[y*mapw+x]);
      adj.push_back(idmap[y*mapw+x]);
    }
  }
  return adj;
}

vector<int> calc_jptx(int mapw, int maph, vector<int>& idmap, 
  locator* jpl, jps::direction d, int rid, int y, int xlb, int xub) {
  vector<int> res;
  vector<uint32_t> tjps;
  vector<cost_t> tcost;
  tjps.reserve(1);
  tcost.reserve(1);
  int dx, dy;
  d2v(d, dx, dy);
  uint32_t goalid = jpl->get_map()->to_padded_id(xub+dx, y);
  for (int x=xlb; x!=xub+dx; x+=dx) if (idmap[y*mapw+x] == rid) {
    uint32_t padded_id = jpl->get_map()->to_padded_id(x, y);
    tjps.clear();
    tcost.clear();
    jpl->jump(d, padded_id, goalid, tjps, tcost);
    if (!tjps.empty() && tjps.back() != goalid) {
      int dis = (int)tcost.back();
      assert(dis > 0);
      res.push_back(y*mapw+(x+dis*dx));
      x += (dis-1)*dx;
    }
  }
  return res;
}

vector<int> calc_jpty(int mapw, int maph, vector<int>& idmap,
    locator* jpl, jps::direction d, int rid, int x, int ylb, int yub) {
  vector<int> res;
  vector<uint32_t> tjps;
  vector<cost_t> tcost;
  tjps.reserve(1);
  tcost.reserve(1);
  int dx, dy;
  d2v(d, dx, dy);
  uint32_t goalid = jpl->get_map()->to_padded_id(x, yub+dy);
  for (int y=ylb; y!=yub+dy; y+=dy) if (idmap[y*mapw+x] == rid) {
    uint32_t padded_id = jpl->get_map()->to_padded_id(x, y);
    tjps.clear();
    tcost.clear();
    jpl->jump(d, padded_id, goalid, tjps, tcost);
    if (!tjps.empty() && tjps.back() != goalid) {
      int dis = (int)tcost.back();
      assert(dis > 0);
      res.push_back((y+dis*dy)*mapw + x);
      y += (dis-1)*dy;
    }
  }

  return res;
}

void cr::init_convrects(vector<cg::FinalConvexRect>& fr, 
    vector<int>& rectids, locator* jpl) {
  rects.reserve(fr.size());
  idmap = vector<int>(rectids);

  for (int i=0; i<(int)fr.size(); i++) {
    ConvRect r = ConvRect(fr[i]);
    assert(r.rid == i+1);
    int lb[4] = {
      r.get_lb(rectscan::N),
      r.get_lb(rectscan::E),
      r.get_lb(rectscan::S),
      r.get_lb(rectscan::W),
    };
    int ub[4] = {
      r.get_ub(rectscan::N),
      r.get_ub(rectscan::E),
      r.get_ub(rectscan::S),
      r.get_ub(rectscan::W),
    };
    // NORTH border
    r.adj[0]  = calc_adj(idmap, mapw, maph, lb[0], ub[0], r.y-1, r.y-1);
    r.jptf[0] = calc_jptx(mapw, maph, idmap, jpl, jps::EAST, r.rid, r.y, r.x, r.x+r.w-1);
    r.jptr[0] = calc_jptx(mapw, maph, idmap, jpl, jps::WEST, r.rid, r.y, r.x+r.w-1, r.x);

    // EAST border
    r.adj[1]  = calc_adj(idmap, mapw, maph, r.x+r.w, r.x+r.w, lb[1], ub[1]);
    r.jptf[1] = calc_jpty(mapw, maph, idmap, jpl, jps::SOUTH, r.rid, r.x+r.w-1, r.y, r.y+r.h-1);
    r.jptr[1] = calc_jpty(mapw, maph, idmap, jpl, jps::NORTH, r.rid, r.x+r.w-1, r.y+r.h-1, r.y);

    // SOUTH border
    r.adj[2]  = calc_adj(idmap, mapw, maph, lb[2], ub[2], r.y+r.h, r.y+r.h);
    r.jptf[2] = calc_jptx(mapw, maph, idmap, jpl, jps::EAST, r.rid, r.y+r.h-1, r.x, r.x+r.w-1);
    r.jptr[2] = calc_jptx(mapw, maph, idmap, jpl, jps::WEST, r.rid, r.y+r.h-1, r.x+r.w-1, r.x);

    // WEST border
    r.adj[3]  = calc_adj(idmap, mapw, maph, r.x-1, r.x-1, lb[3], ub[3]);
    r.jptf[3] = calc_jpty(mapw, maph, idmap, jpl, jps::SOUTH, r.rid, r.x, r.y, r.y+r.h-1);
    r.jptr[3] = calc_jpty(mapw, maph, idmap, jpl, jps::NORTH, r.rid, r.x, r.y+r.h-1, r.y);

    rects.push_back(r);
  }
  if (gmap != nullptr)
    delete gmap;
  gmap = create_gmap_from_idmap(idmap);
}

void cr::init(const string& mapfile) {
  // generate convex rectangles from original map
  // if mapfile is:
  // 1. *.map, compute rectangles
  // 2. *.convrect, load from rect file (make_convrects_from_file)
  // 3. *.convrectid, load from idmap file (make_convrects_from_idmap)

  if (mapfile.back() == 'p') {
    make_convrects_from_map(mapfile);
  }
  else if (mapfile.back() == 't') {
    make_convrects_from_file(mapfile);
  }
  else if (mapfile.back() == 'd') {
    make_convrects_from_idmap(mapfile);
  }
  init_jptr();
}

void cr::make_convrects_from_map(const string& mfile) {
  gridmap* gmap0 = new gridmap(mfile.c_str());
  locator* jpl = new locator(gmap0);
  this->mapw = gmap0->header_width();
  this->maph = gmap0->header_height();

  cg::make_rectangles(jpl, cg::_idmap);
  // make_rectangles will set all nodes label false
  // need to fix this after calling make_rectangles
  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++) 
  if (cg::_idmap[y*mapw+x] != -1)
    jpl->set_label(x, y, true);

  init_convrects(cg::finals, cg::_idmap, jpl);
  delete jpl;
  delete gmap0;
}

void cr::make_convrects_from_file(const string& rectfile) {
  gm_parser parser(rectfile.c_str());
  gm_header header = parser.get_header();
  this->maph = header.height_;
  this->mapw = header.width_;
  map<char, cg::FinalConvexRect> rs;
  vector<int> rectids;
  rectids.resize(parser.get_num_tiles());
  fill(rectids.begin(), rectids.end(), -1);

  auto get_label = [](char c) {
    if ('0' <= c && c <= '9') return (int)(c-'0');
    else if ('a' <= c && c <= 'z') return (int)(c-'a')+10;
    else if ('A' <= c && c <= 'Z') return (int)(c-'A')+10+26;
    return -1;
  };

  for (int i=0; i<(int)parser.get_num_tiles(); i++) {
    char c = parser.get_tile_at(i);
    if (
      ('0' <= c && c <= '9') ||
      ('a' <= c && c <= 'z') ||
      ('A' <= c && c <= 'Z')
    ) {
      int x = i % mapw;
      int y = i / mapw;
      rectids[i] = get_label(c);
      if (rs.find(c) == rs.end()) {
        rs[c] = cg::FinalConvexRect(x, y, 1, 1, rectids[i]);
      }
      else {
        rs[c].xl = min(rs[c].xl, x);
        rs[c].yl = min(rs[c].yl, y);
        rs[c].xu = max(rs[c].xu, x);
        rs[c].yu = max(rs[c].yu, y);
      }
    }
  }
  cg::finals.clear();
  for (auto& it: rs) {
    it.second.calc_bound(rectids, mapw);
    cg::finals.push_back(it.second);
  }
  gridmap* m = create_gmap_from_idmap(rectids);
  locator* jpl = new locator(m);
  init_convrects(cg::finals, rectids, jpl);
  delete jpl;
  delete m;
}

void cr::make_convrects_from_idmap(const string& ridfile) {
  ifstream fin(ridfile);
  int id;
  fin >> this->maph >> this->mapw;
  map<int, cg::FinalConvexRect> rs;
  vector<int> rectids;

  rectids.resize(maph*mapw);

  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++) {
    fin >> id;
    rectids[y*mapw+x] = id;
    if (id != -1) {
      if (rs.find(id) == rs.end()) {
        rs[id] = cg::FinalConvexRect(x, y, 1, 1, id);
      }
      else {
        rs[id].xl = min(rs[id].xl, x);
        rs[id].yl = min(rs[id].yl, y);
        rs[id].xu = max(rs[id].xu, x);
        rs[id].yu = max(rs[id].yu, y);
      }
    }
  }
  cg::finals.clear();
  for (auto& it: rs) {
    it.second.calc_bound(rectids, mapw);
    cg::finals.push_back(it.second);
  }
  gridmap* m = create_gmap_from_idmap(rectids);
  locator* jpl = new locator(m);
  init_convrects(cg::finals, rectids, jpl);
  delete jpl;
  delete m;
}

void cr::init_jptr() {
  for (int pre=0; pre<4; pre++)
  for (int cur=0; cur<4; cur++) {
    if (((pre & 2) && (cur & 2) == 0) ||
        ((pre & 1) && (cur & 1) == 0)
       ) isjptr[pre][cur] = true;
    else isjptr[pre][cur] = false;
  }
}
