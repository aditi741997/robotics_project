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

using namespace mosek::fusion;
using namespace monty;

class DAGNode
{
public:
	int id;
	std::string name;
	std::vector<int> in_edges; // ids of all nodes that publish to this node.

	/* Value denotes the rate at which this node pubs to each of the out edges.
	   -1 denotes pubs each output. 0 denotes f1, 1 denotes f2 and so on. 
	   In call_solver step, each unique nonzero number in this vector becomes a variable. */
	std::map<int, int> out_edges; // [ids, pub_rate as id of global var array.] of all nodes to which this node publishes.

	// Done: see if we should add a corresponding vector that denotes the criticality number for each edge : max of crit # of all chains which include this edge.
	// this will be helpful in scheduling algo.

	int most_critical_se = -1; // denotes the id of the node which is the most critical out of all sync. nodes in out_edges.
	

	int trigger_node = -1; // id of triggering node. Done: default should be -1

	int pub_rate = -1; // this being non -ve denotes that this is a src/sensor node.
	// this variable shall be represented by node.name_pubrate, while trying to 
	int pub_rate_frac_var_id = -1; // if pub_rate >= 0, this denotes the index of this_node_pub_rate/critical_chain_rate in the variable arr.
	
	float compute; // compute requirement

	// the set of chains this node is in?
	std::vector<int> n_chains;
};

class Monomial
{
public:
	double c; // constant
	std::string powers_str; // this is needed for adding monomials : only if the power string is same.
	std::vector<int> powers; // for ith frac_var, powers_i denotes its power. 

	Monomial();
	Monomial(double x, std::vector<int> vars_in_mono, int len, int pow);
	
	void print();
};

class MaxMonomial
{
public:
	// Assuming we take max of monomials of the kind 1/(product fi), the power_str shall be the unique identifier.
	std::map<std::string, Monomial> monos;
	int gvc_id; // denotes the id of the variable denoting the max.
	bool add_to_period = true; // there might be max monos which shouldn't be added to #periods, but be added as constraints.

	MaxMonomial();
	MaxMonomial(int id);
	MaxMonomial(int id, bool atp);

	void insert_mono(Monomial m); // insert another mono into monos set.
	void print();
};

class MinMonomial
{
public:
	// Monos with c=1, over which we take min
	std::map<std::string, Monomial> monos;
	// MaxMonos over which we take min
	std::set<int> maxm_gvc_ids;

	int gvc_id;

	MinMonomial();
	MinMonomial(int id);

	void insert_mono(Monomial m); // insert another mono into monos map.
	void insert_maxmono(MaxMonomial mm); // insert another max mono.
	
	void print();
};

class DAG
{
public:
	// vector of sources
	std::vector<int> src_ids;
	std::map<int, DAGNode> id_node_map;
	std::map<int, std::string> id_name_map;
	std::map<std::string, int> name_id_map;

	DAG();
	DAG(std::string fname);
	
	// Each chain could be a vector of ids. : can find sum ci
	// Sorted by the criticality : first chain is the most critical.
	// ci/wi, chain, criticality_no., id	
	std::vector<std::tuple<float, std::vector<int>, float, int > > all_chains; 
	// maintains the ids of all nodes in most_critical_chain.
	std::map<int, int> nodes_in_most_critical_chain;
	
	// True : constraints, False : weights
	bool is_constraint;

	// Fields related to mosek solver:
	Model::t mosek_model;
	
	int global_var_count;
	// for each variable, this vec contains the description.
	// s1, i1 : name, id of publishing node. 
	// s2 : 'OUT' denotes this is the OP producing rate of a src o.w. s2,i2 : name, id of subscribing node which gets this fraction of all n1 outputs.
	std::vector<std::tuple<std::string, int, std::string, int> > global_var_desc; 

	// these steps need to be repeated only if the criticality ordering changes at runtime:
	
	// Given a list of chains, it finds the criticality number for each and sorts and updates all_chains
	bool order_chains_criticality(std::vector<std::tuple<float, std::vector<int>, float, int > > chains);
	
	/* iterates over the whole DAG, and for each node, assigns which node will trigger it.
	This is also useful for multi core */
	void fill_trigger_nodes(); 

	void assign_publishing_rates(); // iterates over the DAG, and assigns rates at which each node publishes its outputs to its succeding nodes.
	void assign_src_rates(); // one src node will have the parent period [most critical chain]. Other src nodes : fraction.

	// this function returns the order of execution of sub-chains [where all nodes in a sub-chain have the same frac_rate. ]
	std::vector<std::vector<int> > get_exec_order();
	
	std::map<int, std::vector<int>> get_period();
	void compute_rt_chain(int i, std::map<int, std::vector<int>>& period_map);
	
	std::map<int, std::vector<int>> period_map; // node -> set of frac vars, e.g. f1*f4.
	std::vector< std::map<std::string, Monomial> > all_rt_periods;
	std::vector< std::map<int, MaxMonomial> > all_rt_maxmono_periods;
	std::vector< std::map<int, MinMonomial> > all_rt_minmono_periods;
	std::vector<int> compute_rt_solve(); // need to find formula for RT along each chain, as a func of all fraction variables & then solve.


	// Helper functions:
	void add_mono_to_map(std::map<std::string, Monomial>& monos, Monomial m);
	void update_chain_min_rate(std::vector<int>& a, std::vector<std::vector<int> >& x);
	void update_monos(int total_vars);
	void add_monos_async_edge(std::vector<int>& b_frac_set, int mono_len, std::map<std::string, Monomial>& rt_ps, std::map<int, MaxMonomial>& rt_maxm_ps, std::map<int, MinMonomial>& rt_mm_ps, int a_id, int b_id, MaxMonomial mm);
	std::map<std::string, Monomial> convert_period_to_monos(std::map<int, std::vector<int>>& period_map, int total_vars);
	void multiply_monos_with_period(std::map<std::string, Monomial>& period_mono_set);
	Monomial multiply_monos(Monomial m1, Monomial m2);
	void print_vec(std::vector<int>& v, std::string s);
	void print_dvec(std::vector<double>& v, std::string s);
	void print_global_vars_desc();
	void print_mono_map(std::map<std::string, Monomial>& s);
	bool is_edge_in_cc(int a, int b);
	
	void add_constraints_for_max_monos(int total_vars, Variable::t all_lfrac_vars);
	void add_constraints_for_min_monos(int total_vars, Variable::t all_lfrac_vars);
};
