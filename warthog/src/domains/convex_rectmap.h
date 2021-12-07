#pragma once

#include <vector>

#include "jps.h"
#include "gridmap.h"
#include "gm_parser.h"
#include "rectmap.h"
#include "grid2convex_rect.h"
#include "online_jump_point_locator.h"
namespace warthog {
namespace convrectscan {
using namespace std;
namespace rs = rectscan;
namespace cg = convrectgen;
typedef rs::eposition epos;
typedef rs::rdirect rdir;

inline epos R2E(const int& dx, const int& dy, const rdir& rd) {
  return rs::R2E(dx, dy, rd);
}

inline rdir E2R(const int& dx, const int& dy, const epos& ep) {
  return rs::E2R(dx, dy, ep);
}

// a single convex rectangle
class ConvRect: public rs::Rect {
public:
  // the first/last traversable tile of each edge
  int sx, sy;
  int lb[4], ub[4];

  ConvRect() {};
  // init ConvRect from FinalConvexRect obtained from "grid2convex_rect"
  ConvRect(cg::FinalConvexRect f) {
    rid = f.rid, x = f.xl, y = f.yl, sx = f.sx, sy = f.sy;
    h = f.yu-f.yl+1, w = f.xu-f.xl+1;
    for (int i=0; i<4; i++) {
      lb[i] = f.lb[i], ub[i] = f.ub[i];
    }
  }
  bool operator==(const ConvRect& other) const {
    return equal(other);
  }
  bool equal(const ConvRect& other) const {
    if (rid != other.rid) return false;
    if (x!=other.x || y!=other.y || h!= other.h || w!=other.w) return false;
    for (int i=0; i<4; i++) {

      if (adj[i].size() != other.adj[i].size()) return false;
      for (int j=0; j<(int)adj[i].size(); j++)
        if (adj[i][j] != other.adj[i][j]) return false;

      if (lb[i] != other.lb[i] || ub[i] != other.ub[i]) return false;
    }
    return true;
  }
  inline int xl() const { return x; }
  inline int xu() const { return x+w-1; }
  inline int yl() const { return y; }
  inline int yu() const { return y+h-1; }
};

// a collection of convec rectangles 
// that decompose the grid map
class ConvRectMap {
public:
  vector<ConvRect> rects;
  int maph, mapw;
  gridmap* gmap = nullptr;
  bool isjptr[4][4];
  vector<int> idmap;

  ConvRectMap() {
    gmap = nullptr;
  }
  ConvRectMap(const string& mfile);
  ~ConvRectMap() {
    if (gmap != nullptr)
      delete gmap;
  }

  inline bool operator==(const ConvRectMap& rhs) const {
    if (rhs.mapw != mapw || rhs.maph != maph) return false;
    if (idmap.size() != rhs.idmap.size()) return false;
    assert((int)idmap.size() == mapw*maph);
    for (int y=0; y<maph; y++)
    for (int x=0; x<mapw; x++) {
      if (this->idmap[y*mapw+x] != rhs.idmap[y*mapw+x]) return false;
    }
    return true;
  }

  inline bool equal(gridmap& rhs) const {
    if ((int)rhs.header_height() != maph || (int)rhs.header_width() != mapw)
      return false;
    for (int y=0; y<maph; y++)
    for (int x=0; x<mapw; x++) {
      uint32_t padded_id = rhs.to_padded_id(y*mapw + x);
      if (rhs.get_label(padded_id)==0 && idmap[y*mapw+x]!=-1) return false;
      if (rhs.get_label(padded_id) && idmap[y*mapw+x]==-1) return false;
    }
    return true;
  }
  gridmap* create_gmap_from_idmap(vector<int>& _idmap);
  void init(const string& mapfile);
  void init(int mw, int mh, const vector<int>& rectids);
  void make_convrects_from_map(const string& mapfile);
  void make_convrects_from_idmap(const string& mapfile);
  void make_convrects_from_file(const string& mapfile);


  // return the mask horizontal, i.e. left,right
  inline int get_maskh(const int x, const int y) {
    int res = 3;
    if (get_label(x-1, y)) res ^= 2;
    if (get_label(x+1, y)) res ^= 1;
    return res;
  }

  inline int get_label(int x, int y) {
    int padid = gmap->to_padded_id(x, y);
    return gmap->get_label(padid);
  }

  inline int get_label(const int id) {
    int x, y;
    this->to_xy(id, x, y);
    int padid = gmap->to_padded_id(x, y);
    return gmap->get_label(padid);
  }

  // return the mask vertical, i.e. up,down
  inline int get_maskw(const int& x, const int& y) {
    int res = 3;
    if (get_label(x, y-1)) res ^= 2;
    if (get_label(x, y+1)) res ^= 1;
    return res;
  }

  inline void to_xy(int id, int& x, int& y) {
    y = id / mapw;
    x = id % mapw;
  }

  inline int to_id(int x, int y) const {
    return y * mapw + x;
  }

  inline int get_rid(int x, int y) {
    if (x < 0 || x >= mapw || y < 0 || y >= maph) return -1;
    return idmap[y * mapw + x];
  }

  inline ConvRect* get_rect(int x, int y) {
    return &(rects[idmap[y * mapw + x]]);
  }

  inline ConvRect* get_rect(int id) {
    return &(rects[idmap[id]]);
  }

  int mem() { return sizeof(*this); }
  void print(ostream& out) {
    out << "type "<< "octile" << std::endl;
    out << "height "<< maph << std::endl;
    out << "width "<< mapw << std::endl;
    out << "map" << std::endl;

    auto get_label = [&](int id) {
      if (id == -1) return '@';
      id %= 10 + 26 * 2;
      if (id<10) return char('0' + id);
      id -= 10;
      if (id<26) return char('a' + id);
      id -= 26;
      return char('A' + id);
    };

    for (int y=0; y<maph; y++)
    {
      for (int x=0; x<mapw; x++) {
        int pos = y * mapw + x;
        char c = get_label(idmap[pos]);
        out << c;
      }
      out << endl;
    }
  }

  void print_idmap(ostream& out) {
    out << maph << " " << mapw << endl;
    for (int y=0; y<maph; y++) {
      for (int x=0; x<mapw; x++) {
        out << idmap[y*mapw + x] << ((x+1<mapw)?" ": "\n");
      }
    }
  }

private:
  void init_convrects(vector<cg::FinalConvexRect>& fr, vector<int>& rectids, 
      jps::online_jump_point_locator2* jpl);
  void init_jptr();
};

}
}
