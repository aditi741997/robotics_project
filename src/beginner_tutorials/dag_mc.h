#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include<algorithm>
#include <tuple>
#include <unordered_map>
#include "fusion.h"
#include <set>
#include <assert.h>
#include <math.h>
#include <boost/circular_buffer.hpp>
#include <utility>

using namespace mosek::fusion;
using namespace monty;

#include <dag_multi_core_approx.h>

class DAGMultiCore: public DAG
{
	public:
		DAGMultiCore();
		// DAGMultiCore(); // Needs a DAG's stuff and a core assgt and num_cores 
		DAGMultiCore(std::string fname) : DAG(fname) 
		{ 
			fill_trigger_nodes(); 
			assign_publishing_rates();
			assign_src_rates();
		};

		void set_params(int numc, std::vector< std::vector<int> >& sc_core_assgt);

		int num_cores;
		std::vector< std::vector<int> > sc_core_assgt; // in order of exec_order.
		std::map<int, std::vector<int> > core_sc_list; // list of subchains on each core.

		std::map<int, double> sc_id_alone_per; // save fixed period for alone subchains.
		std::map<int, int> sc_id_frac_id; // sc id [as in exec_order] -> frac variable id. Also denotes if a SC is not alone. 
		std::map<int, int> node_id_sc_id; // node if -> sc id
		std::map< int, std::vector<std::vector<double> > > per_core_period_mono_powers;
		std::map<int, std::vector<double> > per_core_period_mono_const;

		// SOLVING For fractions:
		// Step1: Check for subchains alone, assign fixed period
		void assign_fixed_periods();
		void get_period_map_core(int i, int total_vars, std::vector<std::vector<int> >& exec_order);
		double get_fixed_period_node(int node_id);
		std::pair< std::vector<std::vector<double> >, std::vector<double> > get_period_node(int node_id);

		bool is_sc_streaming(std::vector<int>& sc);

		std::vector<int> compute_rt_solve();
};
