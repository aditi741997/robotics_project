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

using namespace mosek::fusion;
using namespace monty;

class DAGNode
{
public:
	int id;
	std::string name;
	std::vector<int> in_edges; // ids of all nodes that publish to this node.

	/* Value denotes the rate at which this node pubs to each of the out edges.
	   0 denotes pubs each output. 1 denotes f1, 2 denotes f2 and so on. 
	   In call_solver step, each unique nonzero number in this vector becomes a variable. */
	std::map<int, int> out_edges; // [ids, pub_rate] of all nodes to which this node publishes.

	// Done: see if we should add a corresponding vector that denotes the criticality number for each edge : max of crit # of all chains which include this edge.
	// this will be helpful in scheduling algo.

	int most_critical_se = -1; // denotes the id of the node which is the most critical out of all sync. nodes in out_edges.
	

	int trigger_node = -1; // id of triggering node. TODO: default should be -1

	int pub_rate = -1; // this being non -ve denotes that this is a src/sensor node.
	// this variable shall be represented by node.name_pubrate, while trying to 
	Variable::t pub_rate_frac_var; // if pub_rate >= 0, this variable denotes this_node_pub_rate/critical_chain_rate.
	
	float compute; // compute requirement

	// the set of chains this node is in?
	std::vector<int> n_chains;
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

	// these steps need to be repeated only if the criticality ordering changes at runtime:
	
	// Given a list of chains, it finds the criticality number for each and sorts and updates all_chains
	bool order_chains_criticality(std::vector<std::tuple<float, std::vector<int>, float, int > > chains);
	
	/* iterates over the whole DAG, and for each node, assigns which node will trigger it.
	This is also useful for multi core */
	void fill_trigger_nodes(); 

	void assign_publishing_rates(); // iterates over the DAG, and assigns rates at which each node publishes its outputs to its succeding nodes.
	void assign_src_rates(); // one src node will have the parent period [most critical chain]. Other src nodes : fraction.
	void create_mosek_vars(); // instantiates vars for all fractions.
	void compute_rt(); // need to somehow save formula for RT along each chain, as a func of all fraction variables.

	// This step will always be executed whenever compute times change:
	void call_solver(); // 1core


	// Helper functions:
	void print_vec(std::vector<int>& v, std::string s);
};
