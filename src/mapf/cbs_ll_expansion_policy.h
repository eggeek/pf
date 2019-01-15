#ifndef WARTHOG_CBS_LL_EXPANSION_POLICY_H
#define WARTHOG_CBS_LL_EXPANSION_POLICY_H

// mapf/cbs_ll_expansion_policy.h
//
// An time-based expansion policy for uniform-cost manhattan grids.
// At each location there are two different types of actions:
// - cardinal moves (at a cost of 1)
// - wait moves (at a cost of 1)
//
// Each action, except wait, moves the agent from its current grid
// location to an adjacent grid location. Each action (including wait)
// advances time by one time-step.
//
// @author: dharabor
// @created: 2018-11-01
//

#include "cbs_ll_heuristic.h"
#include "expansion_policy.h"
#include "forward.h"
#include "gridmap.h"
#include "search_node.h"
#include "time_constraints.h"

#include <memory>

namespace warthog
{

class cbs_ll_expansion_policy 
{
	public:
		cbs_ll_expansion_policy(
                warthog::gridmap* map, 
                warthog::cbs_ll_heuristic* h);

		~cbs_ll_expansion_policy();

		inline void
		reset()
		{
			current_ = 0;
            neis_->clear();
		}

		inline void
		first(warthog::search_node*& ret, double& cost)
		{
            current_ = 0;
            n(ret, cost);
		}

		inline void
		n(warthog::search_node*& ret, double& cost)
		{
            if(current_ < neis_->size())
            {
                ret = (*neis_)[current_].node_;
                cost = (*neis_)[current_].cost_;
            }
            else
            {
                ret = 0;
                cost = 0;
            }
		}

		inline void
		next(warthog::search_node*& ret, double& cost)
		{
            current_++;
            n(ret, cost);
		}

		void 
		expand(warthog::search_node*, warthog::problem_instance*);

        void
        get_xy(warthog::sn_id_t nid, int32_t& x, int32_t& y);

        warthog::search_node* 
        generate_start_node(warthog::problem_instance* pi);

        warthog::search_node*
        generate_target_node(warthog::problem_instance* pi);
        
        warthog::search_node*
        generate(warthog::sn_id_t node_id)
        {
            uint32_t xy_id = (uint32_t)(node_id & UINT32_MAX);
            uint32_t timestep = (uint32_t)(node_id >> 32);
            return __generate(xy_id, timestep);
        }

        // an agent arrives at its target if:
        // (1) its current location is the xy location of the target
        // (2) the agent is never required to move again
        // this function returns true only if both conditions are satisfied,
        // with @param n acting as the current location
        inline bool
        is_target(warthog::search_node* n, warthog::problem_instance* pi)
        {
            // agents must arrive at the xy location of the target
            uint32_t xy_id = (uint32_t)(n->get_id() & UINT32_MAX);
            if(xy_id != (uint32_t)pi->target_id_) { return false; }

            uint32_t arrival_time = (uint32_t)(n->get_id() >> 32);
            std::vector<warthog::cbs::cbs_constraint>& xy_cons = 
                cons_->get_constraint_set((uint32_t)(n->get_id()));

            // the arrival is _safe_ if the agent never has to move again
            for(uint32_t i = 0; i < xy_cons.size(); i++)
            {
                if((xy_cons.at(i).timestep_ >= arrival_time) &&
                    xy_cons.at(i).v_) 
                { return false; }
            }
            return true;
        }

        warthog::mapf::time_constraints<warthog::cbs::cbs_constraint>*
        get_time_constraints() { return cons_; }

		size_t 
        mem();

	
	private:
		warthog::gridmap* map_;
        uint32_t map_xy_sz_;
        std::vector<warthog::mem::node_pool*>* time_map_;
        warthog::cbs_ll_heuristic* h_;
        warthog::mapf::time_constraints<warthog::cbs::cbs_constraint>* cons_;

        struct neighbour_record
        {
            neighbour_record(warthog::search_node* node, double cost)
            {
                node_ = node;
                cost_ = cost;
            }
            warthog::search_node* node_;
            double cost_;
        };

        arraylist<neighbour_record>* neis_;
        uint32_t current_;

        inline warthog::search_node* 
        __generate(uint32_t xy_id, uint32_t timestep)
        {
            while(timestep >= time_map_->size())
            {
                time_map_->push_back(
                            new warthog::mem::node_pool(
                                map_->height() * map_->width()));
            }
            warthog::search_node* nei = time_map_->at(timestep)->generate(xy_id);
            warthog::sn_id_t node_id = ((uint64_t)timestep << 32) | xy_id;
            nei->set_id(node_id);
            return nei;
        }


        inline void 
        add_neighbour(warthog::search_node* nei, double cost)
        {
            neis_->push_back(neighbour_record(nei, cost));
            //std::cout << " neis_.size() == " << neis_->size() << std::endl;
        }

};

}

#endif

