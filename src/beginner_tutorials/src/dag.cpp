#include <dag.h>

DAG::DAG()
{
}

DAG::DAG(std::string fname)
{
	// Read out DAG structure from file.
	std::ifstream df(fname);
	std::vector<std::tuple<float, std::vector<int>, float, int > > chains;
	if (df.is_open())
	{
		std::string line;
		int node_count = 70;
		int chain_count = 0;
		while (std::getline(df, line))	
		{
			std::istringstream ss(line);
			std::string line_type;
			ss >> line_type;
			if (line_type.find("N") != std::string::npos)
			{
				//its a node!!
				DAGNode n;
				std::string name;
				float ci;
				ss >> name >> ci;

				n.name = name;
				n.id = node_count;
				n.compute = ci;				

				std::cout << "Adding node, name : " << name << ", id : " << n.id << ", ci :" << ci << std::endl;

				id_name_map[node_count] = name;
				id_node_map[node_count] = n;
				name_id_map[name] = node_count;

				node_count += 1;
			}
			else if (line_type.find("E") != std::string::npos)
			{
				// its edges!!
				std::string a;
				ss >> a;
				std::string b;
				
				DAGNode& na = id_node_map[name_id_map[a]];
				ss >> b;
				while (b.find("X") == std::string::npos )
				{
					std::cout << "Addign edge " << na.name << " to " << b << std::endl;
					na.out_edges[name_id_map[b]] = 0;
					id_node_map[name_id_map[b]].in_edges.push_back(na.id);
					ss >> b;
				}
						
			}
			else if (line_type.find("C") != std::string::npos)
			{
				float cw; // constraint/weight
				ss >> cw;
				std::string n;
				std::vector<int> ch;
				ss >> n;
				while (n.find("X") == std::string::npos)
				{
					ch.push_back(name_id_map[n]);
					ss >> n;
				}
				std::cout << "Adding chain with cw : " << cw << " and id : " << chain_count << std::endl;
				print_vec(ch, "Chain:");
				chains.push_back(std::make_tuple(cw, ch, 0.0, chain_count) );
				chain_count += 1;
			}
			else
			{
				is_constraint = (line_type.find("constr") != std::string::npos);
			} 
		}
		order_chains_criticality(chains);	
	}
	
	// Initialize mosek solver:
	mosek_model = new Model("single_core_scheduler_algo"); auto _M = finally([&]() { mosek_model->dispose(); });
}

void DAG::print_vec(std::vector<int>& v, std::string s)
{
	std::cout << s;
	for (int i = 0; i < v.size(); i++)
		std::cout << " " << v[i];
	std::cout << "\n";
}

// Finds the criticality no. for each chain and then sorts it & updates all_chains.
// returns if the ordering of chains vector has changed.
bool DAG::order_chains_criticality(std::vector<std::tuple<float, std::vector<int>, float, int > > chains)
{
	std::cout << "##### STARTING Step1 : criticality" << std::endl;
	
	// We append sum ci at end of each vector, just for sorting.
	for (int i = 0; i < chains.size(); i++)
	{
		float si = 0.0;
		for (int j = 0; j < std::get<1>(chains[i]).size(); j++)
		{
			si += id_node_map[std::get<1>(chains[i])[j]].compute;
		}
		if (is_constraint)
			std::get<2>(chains[i]) = ( std::get<0>(chains[i]) / si  );
		else
			std::get<2>(chains[i]) = ( si / std::get<0>(chains[i]) );
	}
	std::sort(chains.begin(), chains.end(), [](const std::tuple<float, std::vector<int>, float, int >& c1, const std::tuple<float, std::vector<int>, float, int >& c2) {return ( std::get<2>(c1) < std::get<2>(c2) ); });
	bool ret = false;
	if (all_chains.size() != chains.size())
		ret = true;
	else
		for (int i = 0; i < chains.size(); i++)
		{
			if (std::get<3>(chains[i]) != std::get<3>(all_chains[i]) )
				ret = true;
		}
	all_chains = chains;
	nodes_in_most_critical_chain.clear(); // empties the map
	for (int i = 0; i < std::get<1>(all_chains[0]).size(); i++)
		nodes_in_most_critical_chain[std::get<1>(all_chains[0])[i]] = 0;

	for (int i = 0; i < all_chains.size(); i++)
		print_vec(std::get<1>(all_chains[i]), "Chain : ");
	return ret;	
}

void DAG::fill_trigger_nodes()
{
	std::cout << "##### STARTING Step2 : fill_trigger_nodes" << std::endl;
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::vector<int>& chain = std::get<1>(all_chains[i]);
		for (int j = 1; j < chain.size(); j++)
		{
			if (id_node_map[chain[j]].trigger_node == -1)
			{
				std::cout << "Node " << id_name_map[chain[j-1]] << " triggers node " << id_name_map[chain[j]] << std::endl;
				// this chain represents the most critical incoming edge to this node.
				id_node_map[chain[j]].trigger_node = chain[j-1];

				if (id_node_map[chain[j-1]].most_critical_se == -1)
				{
					std::cout << "Node " << id_name_map[chain[j-1]] << "'s most critical successor is " << id_name_map[chain[j]] << std::endl; 
					// chain j is the most critical out of all nodes triggered by chain j-1.
					id_node_map[chain[j-1]].most_critical_se = chain[j];
				}
			}
		}
	}
}

void DAG::assign_publishing_rates()
{
	std::cout << "##### STARTING Step3 : assign_publishing_rates" << std::endl;
	// iterate over all nodes in DAG
	for (std::map<int, DAGNode>::iterator it = id_node_map.begin(); it != id_node_map.end(); it++)
	{
		// for each node, iterate over all nodes that this publishes to:
		DAGNode dn = it->second;
		int dn_id = it->first;

		int curr_count = 0; // to keep track of the number of unique f variables we shall define:
		for (std::map<int, int>::iterator ot = dn.out_edges.begin(); ot != dn.out_edges.end(); ot++)
		{
			int out_edge_id = ot->first;
			if (id_node_map[out_edge_id].trigger_node != dn_id)
				dn.out_edges[out_edge_id] = 0; // this node is asynchronous wrt dn.
			else if (out_edge_id == dn.most_critical_se)
				dn.out_edges[out_edge_id] = 0; // this node is the most critical out of all nodes triggered by dn.
			else
			{
				curr_count += 1;
				dn.out_edges[out_edge_id] = curr_count; // each +ve integer denotes a new fractional variable.
			}
			std::cout << "For edge " << dn.name << " -> " << id_name_map[out_edge_id] << ", rate assigned : " << dn.out_edges[out_edge_id] << ", curr_count : " << curr_count << std::endl;	
		}	
	}
}

void DAG::assign_src_rates()
{
	std::cout << "##### STARTING Step4 : assign_src_rates" << std::endl;
	// look at the src node for all the chains : most critical has rate r, all other src nodes have rate fi*r.
	int most_critical_src = (std::get<1>(all_chains[0]))[0];
	int curr_count = 0;
	std::cout << "Most critical src node [which has the highest period] : " << id_name_map[most_critical_src] << std::endl;
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::vector<int>& chain = std::get<1>(all_chains[i]);
		if (chain[0] == most_critical_src)
			id_node_map[ chain[0] ].pub_rate = 0;
		else
		{
			// each new src node gets a new var to denote its fractional rate wrt most_critical_src
			curr_count += 1;
			id_node_map[ chain[0] ].pub_rate = curr_count;
			id_node_map[ chain[0] ].pub_rate_frac_var = mosek_model->variable(1);	
		}
		std::cout << "For src node " << id_name_map[ chain[0] ] << ", rate : " << id_node_map[ chain[0] ].pub_rate << std::endl;
	}
} 
