#include "convrect_expansion_policy.h"

typedef warthog::rectscan::convrect_expansion_policy conexpd;

conexpd::convrect_expansion_policy(ConvRectMap* map): expansion_policy(map->maph*map->mapw) {
  map_ = map;
  jpl_ = new convrect_jump_point_locator(map);
  }

conexpd::~convrect_expansion_policy() { delete jpl_; }

void conexpd::expand(
    warthog::search_node* cur, warthog::problem_instance* prob) {
  reset();
  jpl_->reset();
  jpl_->set_search_number(cur->get_search_number());
  jpl_->set_parent_gvalue(cur->get_g());

  uint32_t c_tiles;
  uint32_t cur_id = cur->get_id();
  map_->get_neighbours(cur_id, (uint8_t*)&c_tiles);
  // jps::direction dir = this->compute_direction((uint32_t)cur->get_parent(), (uint32_t)cur->get_id());
  jps::direction dir = jpl_->get_pdir(cur_id);

  uint32_t succ_dirs = warthog::jps::compute_successors(dir, c_tiles);
  uint32_t goal_id = (uint32_t)prob->target_id_;

  ConvRect* curr = map_->get_rect(cur_id);
  assert(curr->rid == map_->idmap[cur_id]);
  vector<uint32_t> &jpts = jpl_->get_jpts();
  vector<cost_t> &costs = jpl_->get_costs();

  
  for (int i=0; i<8; i++) {
    if (succ_dirs & (1<<i)) {
      jpl_->jump((jps::direction)(1<<i), cur_id, goal_id, curr);
    }
  }

  for (int i=0; i<(int)jpts.size(); i++) {
    uint32_t jp_id = jpts.at(i);
    search_node* mynode = this->generate(jp_id);
    add_neighbour(mynode, costs.at(i));
  }
}

void conexpd::get_xy(warthog::sn_id_t sn_id, int32_t& x, int32_t& y) {
  map_->to_xy(sn_id, x, y);
}

warthog::search_node* conexpd::generate_start_node(warthog::problem_instance* pi) {
    uint32_t start_id = (uint32_t)pi->start_id_;
    uint32_t max_id = map_->mapw * map_->maph;

    if(start_id >= max_id) { return 0; }
    if(map_->get_label(start_id) == 0) { return 0; }
    return generate(start_id);
}

warthog::search_node* conexpd::generate_target_node(
        warthog::problem_instance* pi)
{
    uint32_t target_id = (uint32_t)pi->target_id_;
    uint32_t max_id = map_->mapw * map_->maph;

    if(target_id  >= max_id) { return 0; }
    if(map_->get_label(target_id) == 0) { return 0; }
    return generate(target_id);
}
