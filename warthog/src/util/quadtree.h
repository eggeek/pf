#pragma once

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

using namespace std;

typedef enum {
  WHITE = 0,
  BLACK = 1,
  GREY  = 2
} color;

string cdesc[3] = {"WHITE", "BLACK", "GREY"}; 


/*
 id: index in vector<Node>
 parent id = id / 4
 id = 0 -> null
 id = 1 -> root
          
          |
  c1      |      c2
 id<<2    |  id<<2|1
          |
----------+-----------> x
          |
  c4      |      c3
 id<<2|3  |  id<<2|2
          |
          v
          y
*/
struct Node {
  color c;
  int id;

  void init(int i) {
    c = color::WHITE;
    id = i;
  }
};

class QuadTree {

public:
  vector<Node> nodes;
  int num, h, w;

  QuadTree(int maph, int mapw) {
    num = max(maph, mapw) * max(maph, mapw);
    nodes.resize(num << 3);
    for (int i=0; i<(int)nodes.size(); i++) nodes[i].init(i);
  }

  void insertBlack(int xl, int xu, int yl, int yu, int px, int py, int idx) {
    assert(nodes[idx].id == idx);
    if (xl == xu && yl == yu) {
      assert(idx <= (int)nodes.size());
      nodes[idx].c = color::BLACK;
      return;
    }

    assert((idx << 2 | 3 ) < (int)nodes.size());
    int mx = (xl + xu) >> 1;
    int my = (yl + yu) >> 1;


    if (px <= mx) { // c1 or c4
      if (py <= my) { //c1
        insertBlack(xl, mx, yl, my, px, py, idx<<2);
      }
      else { // c4
        insertBlack(xl, mx, my+1, yu, px, py, idx<<2|3);
      }
    } else { // c2 or c3
      if (py <= my) { // c2
        insertBlack(mx+1, xu, yl, my, px, py, idx<<2|1);
      }
      else { // c3
        insertBlack(mx+1, xu, my+1, yu, px, py, idx<<2|2);
      }
    }
    int blk_cnt = 0, grey_cnt = 0;
    for (int i=0; i<4; i++) {
      if (nodes[idx<<2|i].c == BLACK) blk_cnt++;
      else if (nodes[idx<<2|i].c == GREY) grey_cnt++;
    }
    if (blk_cnt == 4) nodes[idx].c = BLACK;
    else nodes[idx].c = GREY;
    assert(blk_cnt + grey_cnt > 0);
    if (nodes[idx].c != GREY) {
    }
  }

  color query(int xl, int xu, int yl, int yu, int px, int py, int idx) {
    int mx = (xl + xu) >> 1;
    int my = (yl + yu) >> 1;
    if (nodes[idx].c != GREY) return nodes[idx].c;

    assert((idx << 2 | 3 ) < (int)nodes.size());
    if (px <= mx) {
      if (py <= my) return query(xl, mx, yl, my, px, py, idx<<2);
      else return query(xl, mx, my+1, yu, px, py, idx<<2|3);
    }
    else {
      if (py <= my) return query(mx+1, xu, yl, my, px, py, idx<<2|1);
      else return query(mx+1, xu, my+1, yu, px, py, idx<<2|2);
    }
  }
};
