#include "rect_expansion_policy.h"
#include "jps.h"
#include "rect_jump_point_locator.h"

typedef warthog::rectscan::rect_expansion_policy rexpand;

rexpand::rect_expansion_policy(RectMap* map): expansion_policy(map->maph * map->mapw) {
  map_ = map;
  jpl_ = new warthog::rectscan::rect_jump_point_locator(map);
}

rexpand::~rect_expansion_policy() {
  delete jpl_;
}

void rexpand::expand(
    warthog::search_node* cur, warthog::problem_instance* prob) {
  reset();
  jpl_->reset();

  jps::direction dir = this->compute_direction((uint32_t)cur->get_parent(), (uint32_t)cur->get_id());
  uint32_t c_tiles;
  uint32_t cur_id = cur->get_id();
  map_->get_neighbours(cur_id, (uint8_t*)&c_tiles);

  uint32_t succ_dirs = warthog::jps::compute_successors(dir, c_tiles);
  uint32_t goal_id = (uint32_t)prob->target_id_;

  Rect* curr = map_->get_rect(cur_id);
  vector<uint32_t> &jpts = jpl_->get_jpts();
  vector<cost_t> &costs = jpl_->get_costs();

  if (curr->rid == map_->get_rect(goal_id)->rid) {
    jpts.push_back(goal_id);
    int curx, cury, gx, gy;
    map_->to_xy(cur_id, curx, cury);
    map_->to_xy(goal_id, gx, gy);
    costs.push_back(octile_dist(curx, cury, gx, gy));
  }
  else {
    for (int i=0; i<8; i++) {
      if (succ_dirs & (1<<i)) {
        jpl_->jump((jps::direction)(1<<i), cur_id, goal_id, curr);
      }
    }
  }

  for (int i=0; i<(int)jpts.size(); i++) {
    uint32_t jp_id = jpts.at(i);
    search_node* mynode = this->generate(jp_id);
    add_neighbour(mynode, costs.at(i));
  }
}

void rexpand::get_xy(warthog::sn_id_t sn_id, int32_t& x, int32_t& y) {
    map_->to_xy(sn_id, x, y);
}

warthog::search_node* rexpand::generate_start_node(
        warthog::problem_instance* pi)
{ 
    uint32_t start_id = (uint32_t)pi->start_id_;
    uint32_t max_id = map_->mapw * map_->maph;

    if(start_id >= max_id) { return 0; }
    if(map_->get_label(start_id) == 0) { return 0; }
    return generate(start_id);
}


warthog::search_node* rexpand::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t target_id = (uint32_t)pi->target_id_;
    uint32_t max_id = map_->mapw * map_->maph;

    if(target_id  >= max_id) { return 0; }
    if(map_->get_label(target_id) == 0) { return 0; }
    return generate(target_id);
}
