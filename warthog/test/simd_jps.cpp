#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include "gridmap.h"
#include "timer.h"
#include <bitset>
#include <ctime>

using namespace std;
using namespace warthog;

bool Verbose = false;

void print_padded(gridmap* gmap) {
  printf("type octile\nheight %d\nwidth %d\nmap\n", gmap->height(), gmap->width());
  for (uint32_t y=0; y<gmap->height(); y++) {
    string s = "";
    for (uint32_t x=0; x<gmap->width(); x++) {
      if (gmap->get_label(x, y)) s += ".";
      else s += "@";
    }
    printf("%s\n", s.c_str());
  }
}

inline uint32_t calc_stopbits_simd(gridmap* gmap, uint32_t id, 
    SIMD_tiles& s, SIMD_tiles& a, SIMD_tiles& b, SIMD_tiles& l, SIMD_tiles& r) {

  // step 1, load:
  while (true) {
    gmap->get_neighbours_32bit_simd(id, s);
    if (_mm256_movemask_epi8(~s.v)) break;
    id += 31;
  }
  // step 2, compute force-bits: 
  //  * step 2.1: ~neis[i] << 1 & neis[i]
  a.v = _mm256_and_si256(_mm256_slli_epi32(~s.v, 1), s.v);

  //  * step 2.2:
  //    * ~neis[0] << 1 & neis[0]
  //    * ~neis[2] << 1 & neis[2]
  lftshift32bit(a, l);
  rhtshift32bit(a, r);
  a.v = _mm256_or_si256(
      l.v,
      r.v
    );
  // step 3, compute stop-bits: ~neis[1] | force[0] | force[2]
  // ~neis[1] is deadend bits
  b.v = _mm256_or_si256(~s.v, a.v);
  return id;
}

inline uint32_t calc_stopbits(gridmap* gmap, uint32_t id, uint32_t maxid,
    uint32_t neis[3],
    uint32_t& force_bits,
    uint32_t& deadend_bits,
    uint32_t& stopbits) {
  while (id <= maxid) {
    gmap->get_neighbours_32bit(id, neis);
    if ((~neis[0]) | (~neis[1]) | (~neis[2])) break;
    id += 31;
  }
  force_bits = (~neis[0] << 1) & neis[0];
  force_bits |= (~neis[2] << 1) & neis[2];
  deadend_bits = ~neis[1];
  stopbits = force_bits | deadend_bits;
  return id;
}

TEST_CASE("simd-calc") {
  vector<string> maps = {
    "./data/CatwalkAlley_1.map", "./data/CatwalkAlley_2.map",
    "./data/CatwalkAlley_4.map", "./data/CatwalkAlley_8.map",
    "./data/scene_sp_endmaps_1.map", "./data/scene_sp_endmaps_2.map",
    "./data/scene_sp_endmaps_4.map", "./data/scene_sp_endmaps_8.map",
    "./data/GreenerPastures_1.map", "./data/GreenerPastures_2.map",
    "./data/GreenerPastures_4.map", "./data/GreenerPastures_8.map",
  };
  srand(0);
  int step = 6;
  SIMD_tiles s, a, b, l, r;
  uint32_t neis[3] = {0, 0, 0}, force_bits, stop_bits, deadend_bits;

  warthog::timer t;
  uint64_t tcost_simd, tcost_norm;
  for (auto& m: maps) {
    gridmap* gmap = new gridmap(m.c_str());
    vector<uint32_t> xs;
    for (int i=0; i<(int)gmap->header_width(); i++) xs.push_back((uint32_t)i);
    random_shuffle(xs.begin(), xs.end());
    t.start();
    for (uint32_t x: xs)
    for (uint32_t y=0; y+step<=gmap->header_height(); y+=step) {
      calc_stopbits_simd(gmap, gmap->to_padded_id(x, y), s, a, b, l, r);
    }
    t.stop();
    tcost_simd = t.elapsed_time_nano();

    t.reset();
    t.start();
    for (uint32_t x: xs)
    for (uint32_t y=0; y+step<=gmap->header_height(); y+=step) {
      for (int i=0; i<step; i++) {
        calc_stopbits(gmap, gmap->to_padded_id(x, y+i), warthog::INF32, neis,
            force_bits, deadend_bits, stop_bits);
      }
    }
    t.stop();
    tcost_norm = t.elapsed_time_nano();

    cout << "Calc stopbits in Map: " << m
         << ", SIMD speed up: " << (double)tcost_norm / (double)tcost_simd << endl;
    // sanity checking
    for (uint32_t x: xs)
    for (uint32_t y=0; y+step<=gmap->header_height(); y+=step) {
      uint32_t id0 = calc_stopbits_simd(gmap, gmap->to_padded_id(x, y), s, a, b, l, r);
      for (int i=0; i<step; i++) {
        uint32_t id1 = calc_stopbits(gmap, gmap->to_padded_id(x, y+i), id0, neis, 
            force_bits, deadend_bits, stop_bits);
        if (id1 == id0) {
          REQUIRE(force_bits == a.tiles[i+1]);
          REQUIRE(stop_bits == (int32_t)b.tiles[i+1]);
        }
      }
    }
    delete gmap;
  }
}

TEST_CASE("simd-load") {
// load <neis[i] ... neis[i+step-1]>
  vector<string> maps = {
    "test/maps/diagonal.map"
  };

  for (auto& mp: maps) {
    gridmap* gmap = new gridmap(mp.c_str());
    print_padded(gmap);

    int step = 8;
    for (uint32_t x=0; x<gmap->header_width(); x++)
    for (uint32_t y=0; y+step<=gmap->header_height(); y++) {
      uint32_t px, py;
      uint32_t neis[3] = {0, 0, 0};
      vector<uint32_t> dat(step, 0);
      SIMD_tiles s;
      gmap->to_padded_xy(gmap->to_padded_id(x, y), px, py);

      gmap->get_neighbours_32bit_simd(gmap->to_padded_id(x, y+1), s);
      for (int i=0; i<step; i++) {
        gmap->get_neighbours_32bit(gmap->to_padded_id(x, y+i), neis);
        dat[i] = neis[1];
      }

      cout << "get lower bits: " << endl;
      for (int i=0; i<step; i++) {
        bitset<32> b1(dat[i]);
        bitset<32> b2(s.tiles[i]);
        cout << "i: " << i << ", dat: " << b1 << ", simd: " << b2 << endl;
      }

      for (int i=0; i<step; i++) {
        int mask = 0;
        if (x <= y+i) mask |= 1<<(y+i-x);
        if (x+31>gmap->header_width()-1) {
          int numb = x+32-gmap->header_width();
          int offs = gmap->header_width()-x;
          mask |= ((1<<numb)-1)<<offs;
        }
        REQUIRE(~dat[i] == mask);
        REQUIRE(dat[i] == s.tiles[i]);
      }

      gmap->get_neighbours_upper_32bit_simd(gmap->to_padded_id(x, y+1), s);

      for (int i=0; i<step; i++) {
        gmap->get_neighbours_upper_32bit(gmap->to_padded_id(x, y+i), neis);
        dat[i] = neis[1];
      }

      cout << "get upper bits: " << endl;
      for (int i=0; i<step; i++) {
        bitset<32> b1(dat[i]);
        bitset<32> b2(s.tiles[i]);
        cout << "i: " << i << ", dat: " << b1 << ", simd: " << b2 << endl;
      }

      for (int i=0; i<step; i++) {
        int mask = 0;
        if (x >= y+i) mask |= 1<<(31-(x-(y+i)));
        if (x < 31) mask |= (1<<(31-x)) - 1;
        REQUIRE(~dat[i] == mask);
        REQUIRE(dat[i] == s.tiles[i]);
      }
    }
    delete gmap;
  }
}

TEST_CASE("load-speed") {
  vector<string> maps = {
    "./data/CatwalkAlley_1.map", "./data/CatwalkAlley_2.map",
    "./data/CatwalkAlley_4.map", "./data/CatwalkAlley_8.map",
    "./data/scene_sp_endmaps_1.map", "./data/scene_sp_endmaps_2.map",
    "./data/scene_sp_endmaps_4.map", "./data/scene_sp_endmaps_8.map",
    "./data/GreenerPastures_1.map", "./data/GreenerPastures_2.map",
    "./data/GreenerPastures_4.map", "./data/GreenerPastures_8.map",
  };
  srand(0);
  for (auto& m: maps) {
    gridmap* gmap = new gridmap(m.c_str());
    vector<uint32_t> xs;
    int step = 6;
    uint64_t tcost_simd, tcost_norm;
    SIMD_tiles s;
    warthog::timer t;
    for (int i=0; i<(int)gmap->header_width(); i++) xs.push_back((uint32_t)i);
    random_shuffle(xs.begin(), xs.end());
    t.start();
    for (uint32_t x: xs)
    for (uint32_t y=0; y+step<=gmap->header_height(); y+=step) {
      // load [y-1, y+6];
      gmap->get_neighbours_32bit_simd(gmap->to_padded_id(x, y), s);
    }
    t.stop();
    tcost_simd = t.elapsed_time_nano();

    t.reset();
    t.start();
    uint32_t tiles[3] = {0, 0, 0};
    for (uint32_t x: xs)
    for (uint32_t y=0; y+step<=gmap->header_height(); y++) {
      for (int i=0; i<step; i++) {
        uint32_t id = gmap->to_padded_id(x, y+i);
        gmap->get_neighbours_32bit(id, tiles);
      }
    }
    t.stop();
    tcost_norm = t.elapsed_time_nano();
    cout << "Get neighbour 32 bit in Map:" << m 
         << ", SIMD speed up: " << (double)tcost_norm / (double)tcost_simd << endl;

    for (uint32_t x: xs)
    for (uint32_t y=0; y+step<=gmap->header_height(); y+=step) {
      gmap->get_neighbours_32bit_simd(gmap->to_padded_id(x, y), s);
      for (int i=0; i<step; i++) {
        uint32_t id = gmap->to_padded_id(x, y+i);
        gmap->get_neighbours_32bit(id, tiles);
        REQUIRE(tiles[1] == s.tiles[i+1]);
      }
    }
    delete gmap;
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
