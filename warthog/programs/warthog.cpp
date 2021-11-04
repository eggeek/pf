// warthog.cpp
//
// Pulls together a variety of different algorithms 
// for pathfinding on grid graphs.
//
// @author: dharabor
// @created: 2016-11-23
//

#include "cbs.h"
#include "cbs_ll_expansion_policy.h"
#include "cbs_ll_heuristic.h"
#include "cfg.h"
#include "constants.h"
#include "depth_first_search.h"
#include "flexible_astar.h"
#include "four_connected_jps_locator.h"
#include "greedy_depth_first_search.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "rectmap.h"
#include "jps_expansion_policy.h"
#include "jps2_expansion_policy.h"
#include "jps2plus_expansion_policy.h"
#include "jps4c_expansion_policy.h"
#include "jpsplus_expansion_policy.h"
#include "rect_expansion_policy.h"
#include "ll_expansion_policy.h"
#include "manhattan_heuristic.h"
#include "octile_heuristic.h"
#include "scenario_manager.h"
#include "timer.h"
#include "labelled_gridmap.h"
#include "sipp_expansion_policy.h"
#include "vl_gridmap_expansion_policy.h"
#include "zero_heuristic.h"

#include "getopt.h"

#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <memory>

#include "time_constraints.h"

// check computed solutions are optimal
int checkopt = 0;
// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;

double preproc_nano = 0;

void
help()
{
    std::cerr 
        << "==> manual <==\n"
        << "This program solves/generates grid-based pathfinding problems using the\n"
        << "map/scenario format from the 2014 Grid-based Path Planning Competition\n\n";

	std::cerr 
    << "The following are valid parameters for SOLVING instances:\n"
	<< "\t--alg [alg] (required)\n"
    << "\t--scen [scen file] (required) \n"
    << "\t--map [map file] (optional; specify this to override map values in scen file) \n"
	<< "\t--checkopt (optional; compare solution costs against values in the scen file)\n"
	<< "\t--verbose (optional; prints debugging info when compiled with debug symbols)\n"
    << "Invoking the program this way solves all instances in [scen file] with algorithm [alg]\n"
    << "Currently recognised values for [alg]:\n"
    << "\tcbs_ll, cbs_ll_w, dijkstra, astar, astar_wgm, astar4c, sipp\n"
    << "\tsssp, jps, jps2, jps+, jps2+, jps, jps4c\n"
    << "\tdfs, gdfs\n\n"
    << ""
    << "The following are valid parameters for GENERATING instances:\n"
    << "\t --gen [map file (required)]\n"
    << "Invoking the program this way generates at random 1000 valid problems for \n"
    << "gridmap [map file]\n";
}

bool
check_optimality(warthog::solution& sol, warthog::experiment* exp)
{
	uint32_t precision = 2;
	double epsilon = (1.0 / (int)pow(10, precision)) / 2;
	double delta = fabs(sol.sum_of_edge_costs_ - exp->distance());

	if( fabs(delta - epsilon) > epsilon)
	{
		std::stringstream strpathlen;
		strpathlen << std::fixed << std::setprecision(exp->precision());
		strpathlen << sol.sum_of_edge_costs_;

		std::stringstream stroptlen;
		stroptlen << std::fixed << std::setprecision(exp->precision());
		stroptlen << exp->distance();

		std::cerr << std::setprecision(exp->precision());
		std::cerr << "optimality check failed!" << std::endl;
		std::cerr << std::endl;
		std::cerr << "optimal path length: "<<stroptlen.str()
			<<" computed length: ";
		std::cerr << strpathlen.str()<<std::endl;
		std::cerr << "precision: " << precision << " epsilon: "<<epsilon<<std::endl;
		std::cerr<< "delta: "<< delta << std::endl;
		exit(1);
	}
    return true;
}

void
run_experiments(warthog::search* algo, std::string alg_name,
        warthog::scenario_manager& scenmgr, bool verbose, bool checkopt,
        std::ostream& out)
{
	std::cout 
        << "id\talg\texpanded\tinserted\tupdated\ttouched\tsurplus"
        << "\tnanos\tpcost\tplen\tscnt\tpreproc\tmap\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		uint32_t startid = exp->starty() * exp->mapwidth() + exp->startx();
		uint32_t goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
        warthog::problem_instance pi(startid, goalid, verbose);
        warthog::solution sol;
    warthog::jps::scan_cnt = 0;
    algo->get_path(pi, sol);

		out
            << i<<"\t" 
            << alg_name << "\t" 
            << sol.nodes_expanded_ << "\t" 
            << sol.nodes_inserted_ << "\t"
            << sol.nodes_updated_ << "\t"
            << sol.nodes_touched_ << "\t"
            << sol.nodes_surplus_ << "\t"
            << sol.time_elapsed_nano_ << "\t"
            << sol.sum_of_edge_costs_ << "\t" 
            << (sol.path_.size()-1) << "\t" 
            << warthog::jps::scan_cnt << "\t"
            << preproc_nano << "\t"
            << scenmgr.last_file_loaded() 
            << std::endl;

        if(checkopt) { check_optimality(sol, exp); }
	}
}


void
run_jpsplus(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::jpsplus_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jpsplus_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);

	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps2plus(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::timer t;
    warthog::gridmap map(mapname.c_str());
  t.start();
	warthog::jps2plus_expansion_policy expander(&map);
  t.stop();
  preproc_nano = t.elapsed_time_nano();
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps2plus_expansion_policy,
        warthog::pqueue_min> astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);

	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps2(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::jps2_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps2_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}


void
run_jps(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::jps_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps4c(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::jps4c_expansion_policy expander(&map);
	warthog::manhattan_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::manhattan_heuristic,
	   	warthog::jps4c_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_rect(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
  warthog::timer t;
  warthog::rectscan::RectMap map(mapname.c_str());
  t.start();
	warthog::rectscan::rect_expansion_policy expander(&map);
  t.stop();
  preproc_nano = t.elapsed_time_nano();
	warthog::octile_heuristic heuristic(map.mapw, map.maph);
  warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	  warthog::rectscan::rect_expansion_policy,
    warthog::pqueue_min> 
    astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_astar(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::gridmap_expansion_policy, 
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_astar4c(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map, true);
	warthog::manhattan_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::manhattan_heuristic,
	   	warthog::gridmap_expansion_policy, 
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_sipp(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap gm(mapname.c_str());
	warthog::manhattan_heuristic heuristic(gm.header_width(), gm.header_height());
    warthog::sipp_gridmap sipp_map(&gm);
	warthog::sipp_expansion_policy expander(&sipp_map);
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::manhattan_heuristic,
	   	warthog::sipp_expansion_policy,
        warthog::pqueue_min> astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_cbs_ll(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap gm(mapname.c_str());
	warthog::cbs_ll_heuristic heuristic(&gm);
	warthog::cbs_ll_expansion_policy expander(&gm, &heuristic);

    // the reservation table is just here as an example
    // for single agent search, or prioritised/rule planning, we 
    // don't need it. only for decomposition-based algos like CBS
    warthog::reservation_table restab(gm.width()*gm.height());
    warthog::cbs::cmp_cbs_ll_lessthan lessthan(&restab);
    warthog::cbs::pqueue_cbs_ll open(&lessthan);

	warthog::flexible_astar<
		warthog::cbs_ll_heuristic,
	   	warthog::cbs_ll_expansion_policy,
        warthog::cbs::pqueue_cbs_ll>
            astar(&heuristic, &expander, &open);

	std::cout 
        << "id\talg\texpanded\tinserted\tupdated\ttouched"
        << "\tnanos\tpcost\tplen\tmap\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		uint32_t startid = exp->starty() * exp->mapwidth() + exp->startx();
		uint32_t goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
        warthog::problem_instance pi(startid, goalid, verbose);
        warthog::solution sol;

        // solve and print results
        astar.get_path(pi, sol);
		std::cout
            << i<<"\t" 
            << alg_name << "\t" 
            << sol.nodes_expanded_ << "\t" 
            << sol.nodes_inserted_ << "\t"
            << sol.nodes_updated_ << "\t"
            << sol.nodes_touched_ << "\t"
            << sol.time_elapsed_nano_ << "\t"
            << sol.sum_of_edge_costs_ << "\t" 
            << (sol.path_.size()-1) << "\t" 
            << scenmgr.last_file_loaded() 
            << std::endl;
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() + restab.mem() << "\n";
}

// cbs low-level with variable edge costs
// (each action still takes one timestep, regardless of cost)
void
run_cbs_ll_w(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap gm(mapname.c_str());
	warthog::cbs_ll_heuristic heuristic(&gm);
	warthog::ll_expansion_policy expander(&gm, &heuristic);

    warthog::reservation_table restab(gm.width()*gm.height());
    warthog::cbs::cmp_cbs_ll_lessthan lessthan(&restab);
    warthog::cbs::pqueue_cbs_ll open(&lessthan);

	warthog::flexible_astar<
		warthog::cbs_ll_heuristic,
	   	warthog::ll_expansion_policy,
        warthog::cbs::pqueue_cbs_ll>
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";

}

void
run_dijkstra(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::zero_heuristic heuristic;
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::zero_heuristic,
	   	warthog::gridmap_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_wgm_astar(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::vl_gridmap map(mapname.c_str());
	warthog::vl_gridmap_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;
    
    // cheapest terrain (movingai benchmarks) has ascii value '.'; we scale
    // all heuristic values accordingly (otherwise the heuristic doesn't 
    // impact f-values much and search starts to behave like dijkstra)
    heuristic.set_hscale('.');

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::vl_gridmap_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_wgm_sssp(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::vl_gridmap map(mapname.c_str());
	warthog::vl_gridmap_expansion_policy expander(&map);
	warthog::zero_heuristic heuristic;
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::zero_heuristic,
	   	warthog::vl_gridmap_expansion_policy,
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_sssp(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::zero_heuristic heuristic;
    warthog::pqueue_min open;

	warthog::flexible_astar<
		warthog::zero_heuristic,
	   	warthog::gridmap_expansion_policy, 
        warthog::pqueue_min> 
            astar(&heuristic, &expander, &open);

    run_experiments(&astar, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_dfs(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::zero_heuristic heuristic;
    warthog::pqueue_min open;

    warthog::depth_first_search<
        warthog::zero_heuristic, 
        warthog::gridmap_expansion_policy, 
        warthog::pqueue_min> 
            alg(&heuristic, &expander, &open);

    run_experiments(&alg, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< alg.mem() + scenmgr.mem() << "\n";
}

void
run_gdfs(warthog::scenario_manager& scenmgr, std::string mapname, std::string alg_name)
{
    warthog::gridmap map(mapname.c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::pqueue_min open;

    warthog::greedy_depth_first_search<
        warthog::octile_heuristic, 
        warthog::gridmap_expansion_policy, 
        warthog::pqueue_min> 
            alg(&heuristic, &expander, &open);

    run_experiments(&alg, alg_name, scenmgr, 
            verbose, checkopt, std::cout);
	std::cerr << "done. total memory: "<< alg.mem() + scenmgr.mem() << "\n";
}

int 
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] = 
	{
		{"alg",  required_argument, 0, 1},
		{"scen",  required_argument, 0, 0},
		{"map",  required_argument, 0, 1},
		{"gen", required_argument, 0, 3},
		{"help", no_argument, &print_help, 1},
		{"checkopt",  no_argument, &checkopt, 1},
		{"verbose",  no_argument, &verbose, 1},
		{0,  0, 0, 0}
	};

	warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, "a:b:c:def", valid_args);

    if(argc == 1 || print_help)
    {
		help();
        exit(0);
    }

    std::string sfile = cfg.get_param_value("scen");
    std::string alg = cfg.get_param_value("alg");
    std::string gen = cfg.get_param_value("gen");
    std::string mapname = cfg.get_param_value("map");

	if(gen != "")
	{
		warthog::scenario_manager sm;
		warthog::gridmap gm(gen.c_str());
		sm.generate_experiments(&gm, 1000) ;
		sm.write_scenario(std::cout);
        exit(0);
	}

    // running experiments
	if(alg == "" || sfile == "")
	{
        help();
		exit(0);
	}

    // load up the instances
	warthog::scenario_manager scenmgr;
	scenmgr.load_scenario(sfile.c_str());

    if(scenmgr.num_experiments() == 0)
    {
        std::cerr << "err; scenario file does not contain any instances\n";
        exit(0);
    }

    // the map filename can be given or (default) taken from the scenario file
    if(mapname == "")
    { mapname = scenmgr.get_experiment(0)->map().c_str(); }


    if(alg == "jps+")
    {
        run_jpsplus(scenmgr, mapname, alg);
    }
    else if (alg == "rect")
    {
      run_rect(scenmgr, mapname, alg);
    }
    else if(alg == "jps2")
    {
        run_jps2(scenmgr, mapname, alg);
    }

    else if(alg == "jps2+")
    {
        run_jps2plus(scenmgr, mapname, alg);
    }

    else if(alg == "jps")
    {
        run_jps(scenmgr, mapname, alg);
    }
    else if(alg == "jps4c")
    {
        run_jps4c(scenmgr, mapname, alg);
    }

    else if(alg == "dijkstra")
    {
        run_dijkstra(scenmgr, mapname, alg); 
    }

    else if(alg == "astar")
    {
        run_astar(scenmgr, mapname, alg); 
    }
    else if(alg == "astar4c")
    {
        run_astar4c(scenmgr, mapname, alg); 
    }

    else if(alg == "cbs_ll")
    {
        run_cbs_ll(scenmgr, mapname, alg); 
    }
    else if(alg == "cbs_ll_w")
    {
        run_cbs_ll_w(scenmgr, mapname, alg); 
    }
    else if(alg == "sipp")
    {
        run_sipp(scenmgr, mapname, alg);
    }

    else if(alg == "astar_wgm")
    {
        run_wgm_astar(scenmgr, mapname, alg); 
    }

    else if(alg == "sssp")
    {
        run_sssp(scenmgr, mapname, alg);
    }

    else if(alg == "sssp")
    {
        run_wgm_sssp(scenmgr, mapname, alg); 
    }
    else if(alg == "dfs")
    {
        run_dfs(scenmgr, mapname, alg); 
    }
    else if(alg == "gdfs")
    {
        run_gdfs(scenmgr, mapname, alg); 
    }
    else
    {
        std::cerr << "err; invalid search algorithm: " << alg << "\n";
    }
}


