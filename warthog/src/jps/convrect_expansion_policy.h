#pragma once
// rect_expansion_policy.h
//
// similar to jps2, but scanning is based on rectangle mesh
//
// @author: shizhe
// @created: 14/08/2021

#include "forward.h"
#include "rectmap.h"
#include "jps.h"
#include "problem_instance.h"
#include "convrect_jump_point_locator.h"
#include "search_node.h"
#include "expansion_policy.h"

#include "stdint.h"

namespace warthog
{
namespace rectscan {

class convrect_expansion_policy: public expansion_policy
{
	public:
		convrect_expansion_policy(ConvRectMap* map);
		virtual ~convrect_expansion_policy();

		virtual void 
		expand(search_node*, problem_instance*);

		virtual inline size_t
		mem() {
      return expansion_policy::mem() +
          sizeof(*this) + map_->mem() + jpl_->mem();
		}

    virtual void
    get_xy(warthog::sn_id_t node_id, int32_t& x, int32_t& y); 

    virtual warthog::search_node* 
    generate_start_node(warthog::problem_instance* pi);

    virtual warthog::search_node*
    generate_target_node(warthog::problem_instance* pi);

    convrect_jump_point_locator* get_jpl() { return jpl_; }
    ConvRectMap* get_map() { return map_; }

	private:
		ConvRectMap* map_;
		convrect_jump_point_locator* jpl_;

    inline warthog::jps::direction
    compute_direction(
            uint32_t n1_id, uint32_t n2_id)
    {
        if(n1_id == warthog::GRID_ID_MAX) { return warthog::jps::NONE; }

        int32_t x, y, x2, y2;
        warthog::helpers::index_to_xy(n1_id, map_->mapw, x, y);
        warthog::helpers::index_to_xy(n2_id, map_->mapw, x2, y2);
        int32_t dx = abs(x2 - x);
        int32_t dy = abs(y2 - y);

        if(dx > dy)
        {
            if(x2 > x)
            { return warthog::jps::EAST; }

            return warthog::jps::WEST;
        }

        if(y2 > y) 
        { return warthog::jps::SOUTH; }

        return warthog::jps::NORTH;
    }

};

}}
