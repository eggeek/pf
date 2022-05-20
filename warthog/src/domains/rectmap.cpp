#include "rectmap.h"
#include "grid2rect.h"
#include <vector>

typedef warthog::rectscan::RectMap rmap;

void rmap::create_gmap_from_rects() {
  if (gmap != nullptr)
    delete gmap;
  gmap = new gridmap((uint32_t)maph, (uint32_t)mapw);
  uint32_t cnt = 0;
  gmap->set_num_traversable_tiles(cnt);
  for (int y=0; y<maph; y++)
  for (int x=0; x<mapw; x++) {
    uint32_t id = y*mapw + x;
    if (idmap[id] != -1) {
      cnt++;
      gmap->set_num_traversable_tiles(cnt);
      gmap->set_label(gmap->to_padded_id(id), 1);
    }
    else {
      gmap->set_label(gmap->to_padded_id(id), 0);
    }
  }
}

rmap::RectMap(const char* mapfile, bool quadtree) {
  _filename = string(mapfile);
  init(mapfile);
  create_gmap_from_rects();
  assert(mapw == (int)gmap->header_width());
  assert(maph == (int)gmap->header_height());
}

void rmap::init(const string& mapfile, bool quadtree) {
  // generate rectangle from original map
  // if mapfile is: 
  // 1. "*.map", compute rectangles
  // 2. "*.rect", load from rect file (make_rectangles_from_file)
  // 3. "*.rectid", load from idmap file (make_rectangles_from_idmap)

  if (mapfile.back() == 'p') {
    // *.map
    ifstream fin;
    fin.open(mapfile.c_str());
    rectgen::read_map(fin);
    if (quadtree)
      rectgen::make_rectangles_from_quadtree();
    else
      rectgen::make_rectangles();
    maph = rectgen::map_height;
    mapw = rectgen::map_width;
  }
  else if (mapfile.back() == 't') {
    // *.rect
    // or load rectangles from a user specified rectangles
    // each rectangle is labelled by 0~9 a~z A~Z
    // only work if #rect is not large than 10+26*2 
    make_rectangles_from_file(mapfile.c_str());
  }
  else if (mapfile.back() == 'd') {
    // *.rectid
    make_rectangles_from_idmap(mapfile.c_str());
  }
  init_rects();
  // init isjptr
  for (int pre=0; pre<4; pre++)
  for (int cur=0; cur<4; cur++) {
    if (((pre & 2) && (cur & 2) == 0) ||
        ((pre & 1) && (cur & 1) == 0)
       ) isjptr[pre][cur] = true;
    else isjptr[pre][cur] = false;
  }
}

void rmap::init(int maph_, int mapw_, const vector<int>& rectids) {
  this->maph = maph_;
  this->mapw = mapw_;
  this->idmap = vector<int>(rectids.begin(), rectids.end());

  map<int, rectgen::FinalRect> rs;
  for (int y=0; y<maph; y++) {
    for (int x=0; x<mapw; x++) {
      int id = idmap[y*mapw + x];
      if (id != -1) {
        if (rs.find(id) == rs.end()) {
          rs[id].x = x;
          rs[id].y = y;
          rs[id].height = y;
          rs[id].width = x;
        }
        else {
          rs[id].x = min(rs[id].x , x);
          rs[id].y = min(rs[id].y , y);
          rs[id].height = max(rs[id].height, y);
          rs[id].width = max(rs[id].width, x);
        }
      }
    }
  }
  rectgen::final_rectangles.clear();
  for (auto& it: rs) {
    it.second.height -= it.second.y - 1;
    it.second.width -= it.second.x - 1;
    rectgen::final_rectangles.push_back(it.second);
  }

  init_rects();
  for (int pre=0; pre<4; pre++)
  for (int cur=0; cur<4; cur++) {
    if (((pre & 2) && (cur & 2) == 0) ||
        ((pre & 1) && (cur & 1) == 0)
       ) isjptr[pre][cur] = true;
    else isjptr[pre][cur] = false;
  }
  create_gmap_from_rects();
}

void rmap::make_rectangles_from_file(const char* rectfile) {
    gm_parser parser(rectfile);
    gm_header header = parser.get_header();
    maph = header.height_;
    mapw = header.width_;
    map<char, rectgen::FinalRect> rs;

    for (int i=0; i<(int)parser.get_num_tiles(); i++) {
      char c = parser.get_tile_at(i);
      if (
        ('0' <= c && c <= '9') || 
        ('a' <= c && c <= 'z') ||
        ('A' <= c && c <= 'Z') 
      ) {
        int x = i % mapw;
        int y = i / mapw;
        if (rs.find(c) == rs.end()) {
          rs[c].x = x;
          rs[c].y = y;
          rs[c].height = y;
          rs[c].width = x;
        }
        else {
          rs[c].x = min(rs[c].x, x);
          rs[c].y = min(rs[c].y, y);
          rs[c].height = max(rs[c].height, y);
          rs[c].width = max(rs[c].width, x);
        }
      }
    }
    rectgen::final_rectangles.clear();
    for (auto& it: rs) {
      it.second.height -= it.second.y - 1;
      it.second.width -= it.second.x - 1;
      rectgen::final_rectangles.push_back(it.second);
    }
  }

void rmap::make_rectangles_from_idmap(const char* ridfile) {
  ifstream fin(ridfile);
  int id;
  fin >> this->maph >> this->mapw;
  map<int, rectgen::FinalRect> rs;
  for (int y=0; y<maph; y++) {
    for (int x=0; x<mapw; x++) {
      fin >> id;
      if (id != -1) {
        if (rs.find(id) == rs.end()) {
          rs[id].x = x;
          rs[id].y = y;
          rs[id].height = y;
          rs[id].width = x;
        }
        else {
          rs[id].x = min(rs[id].x , x);
          rs[id].y = min(rs[id].y , y);
          rs[id].height = max(rs[id].height, y);
          rs[id].width = max(rs[id].width, x);
        }
      }
    }
  }
  rectgen::final_rectangles.clear();
  for (auto& it: rs) {
    it.second.height -= it.second.y - 1;
    it.second.width -= it.second.x - 1;
    rectgen::final_rectangles.push_back(it.second);
  }
}

void rmap::init_rect(Rect& r) {
  auto calc_adj = [&](
      int xl, int xr, int yl, int yr) {
      
    set<int> adj_set;
    vector<int> adj;
    adj.clear();
    // adjacent rects are in order left-right, top-down
    for (int x=xl; x<=xr; x++)
    for (int y=yl; y<=yr; y++) 
    if (x >= 0 && x < mapw && y >= 0 && y < maph) {
      // ignore adjacent obstacles
      if (idmap[y * mapw + x] == -1) continue;
      if (adj_set.find(idmap[y * mapw + x]) == adj_set.end()) {
        adj_set.insert(idmap[y * mapw + x]);
        adj.push_back(idmap[y * mapw + x]);
      }
    }
    return adj;
  };

  auto calc_jptsf = [&] (int xl, int xr, int yl, int yr, int dx, int dy) {
    int pre = 0, cur;
    vector<int> res;
    for (int x=xl; x<=xr; x++)
    for (int y=yl; y<=yr; y++)
    if (x >= 0 && x < mapw && y >= 0 && y < maph) {
      cur = idmap[y * mapw + x];
      // from obstacle to traversable
      // (x-dx, y-dy) is on border
      // (x, y) is the adjacent
      if (pre < 0 && cur >= 0) res.push_back((y - dy) * mapw + (x - dx));
      pre = cur;
    }
    return res;
  };
  
  auto calc_jptsr = [&] (int xl, int xr, int yl, int yr, int dx, int dy) {
    int pre = 0, cur;
    vector<int> res;
    for (int x=xr; x>=xl; x--)
    for (int y=yr; y>=yl; y--)
    if (x >= 0 && x < mapw && y >= 0 && y < maph) {
      cur = idmap[y * mapw + x];
      // from obstacle to traversable
      // (x-dx, y-dy) is on border
      // (x, y) is the adjacent
      if (pre < 0 && cur >= 0) res.push_back((y - dy) * mapw + (x - dx));
      pre = cur;
    }
    return res;
  };

  for (int j=0; j<4; j++) {
    r.adj[j].clear();
    r.jptf[j].clear();
    r.jptr[j].clear();
  }
  r.adj[0]  = calc_adj(r.x, r.x+r.w-1, r.y-1, r.y-1);
  r.jptf[0] = calc_jptsf(r.x, r.x+r.w-1, r.y-1, r.y-1, 0, -1);
  r.jptr[0] = calc_jptsr(r.x, r.x+r.w-1, r.y-1, r.y-1, 0, -1);

  r.adj[1]  = calc_adj(r.x+r.w, r.x+r.w, r.y, r.y+r.h-1);
  r.jptf[1] = calc_jptsf(r.x+r.w, r.x+r.w, r.y, r.y+r.h-1, 1, 0);
  r.jptr[1] = calc_jptsr(r.x+r.w, r.x+r.w, r.y, r.y+r.h-1, 1, 0);

  r.adj[2]  = calc_adj(r.x, r.x+r.w-1, r.y+r.h, r.y+r.h);
  r.jptf[2] = calc_jptsf(r.x, r.x+r.w-1, r.y+r.h, r.y+r.h, 0, 1);
  r.jptr[2] = calc_jptsr(r.x, r.x+r.w-1, r.y+r.h, r.y+r.h, 0, 1);

  r.adj[3]  = calc_adj(r.x-1, r.x-1, r.y, r.y+r.h-1);
  r.jptf[3] = calc_jptsf(r.x-1, r.x-1, r.y, r.y+r.h-1, -1, 0);
  r.jptr[3] = calc_jptsr(r.x-1, r.x-1, r.y, r.y+r.h-1, -1, 0);
}
  
void rmap::init_rects() {

  const vector<rectgen::FinalRect>& frects = rectgen::final_rectangles;
  rects.resize(frects.size());

  idmap.resize(maph * mapw);

  // idmap stores the rect id of each traversable tile
  // so that we can compute the neighbor rects of each edge
  fill(idmap.begin(), idmap.end(), -1);
  for (int i=0; i<(int)frects.size(); i++) {
    for (int x=frects[i].x; x<frects[i].x + frects[i].width; x++) {
      for (int y=frects[i].y; y<frects[i].y + frects[i].height; y++) {
        idmap[y * mapw + x] = i;
      }
    }
  }

  for (int i=0; i<(int)rects.size(); i++) {
    Rect& r = rects[i];
    rectgen::FinalRect fr = rectgen::final_rectangles[i];
    r.rid = i;
    r.x = fr.x;
    r.y = fr.y;
    r.h = fr.height;
    r.w = fr.width;
    init_rect(r);
  }
}

void rmap::update_to_empty(Rect r) {
  rects.push_back(r); // add new rect
  for (int x=r.x; x<r.x+r.w; x++)
  for (int y=r.y; y<r.y+r.h; y++) {
    // sanity checking: all tiles in r must be nontraversable now
    assert(idmap[y*mapw+x] == -1);
    // update idmap
    idmap[y*mapw+x] = rects.size()-1;
  }
  init_rect(rects.back());
}

void rmap::update_to_obstacle(Rect r) {
  set<int> ids;
  int cntr = rects.size();
  for (int x=r.x; x<r.x+r.w; x++)
  for (int y=r.y; y<r.y+r.h; y++) {
    // sanity checking: all tiles in r must be traversable now
    assert(idmap[y*mapw+x] != -1);
    ids.insert(idmap[y*mapw+x]);
  }
  vector<Rect> newr;
  newr.reserve(ids.size()<<2);
  for (int id: ids) {
    Rect c = rects[id];
    int xl = c.x, xu = c.x+c.w-1, yl=c.y, yu=c.y+c.h-1;
    int sid = newr.size();
    if (xl < r.x) { // [xl, r.x-1] * [yl, yu]
      newr.push_back(Rect(cntr++, xl, yl, yu-yl+1, r.x-xl));
      xl = r.x;
    }
    if (xu > r.x+r.w-1) { // [r.x+r.w, xu] * [yl, yu]
      newr.push_back(Rect(cntr++, r.x+r.w, yl, yu-yl+1, xu-(r.x+r.w)+1));
      xu = r.x+r.w-1;
    }
    if (yl < r.y) { // [xl, xu] * [yl, r.y-1]
      newr.push_back(Rect(cntr++, xl, yl, r.y-yl, xu-xl+1));
      yl = r.y;
    }
    if (yu > r.y+r.h-1) { // [xl, xu] * [r.y+r.h, yu]
      newr.push_back(Rect(cntr++, xl, r.y+r.h, yu-(r.y+r.h)+1, xu-xl+1));
      yu = r.y+r.h-1;
    }
    // update idmap and gridmap
    for (int x=xl; x<=xu; x++)
    for (int y=yl; y<=yu; y++) {
      // before rewrite idmap, all tiles must marked by id
      assert(idmap[y*mapw+x] == id);
      // rewrite them to -1 (obstacle)
      idmap[y*mapw+x] = -1;
      gmap->set_label(gmap->to_padded_id(y*mapw+x), false);
    }
    for (int i=sid; i<(int)newr.size(); i++) {
      Rect& nr = newr[i];
      for (int x=nr.x; x<nr.x+nr.w; x++)
      for (int y=nr.y; y<nr.y+nr.h; y++) {
        // before rewrite, it must be the original rect
        assert(idmap[y*mapw+x] == id);
        // rewrite to the new rect
        idmap[y*mapw+x] = nr.rid;
        gmap->set_label(gmap->to_padded_id(y*mapw+x), true);
      }
    }
  }
  // add new to rect collection
  rects.insert(rects.end(), newr.begin(), newr.end());
}

void rmap::update_map(Rect r, bool f) {
  // mark all tiles in r from nontraversable to traversable
  if (f) update_to_empty(r);
  // mark all tiles in r from traversable to nontraversable
  else update_to_obstacle(r); 
}
