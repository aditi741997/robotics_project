#include <dag.h>

// This function is picked up from the example gp code in mosek fusion examples.
// Models log(sum(exp(Ax+b))) <= 0.
// Each row of [A b] describes one of the exp-terms
void logsumexp(Model::t                             M,
               std::shared_ptr<ndarray<double, 2>>  A,
               Variable::t                          x,
               std::shared_ptr<ndarray<double, 1>>  b)
{
  int k = A->size(0);
  auto u = M->variable(k);
  M->constraint(Expr::sum(u), Domain::equalsTo(1.0));
  M->constraint(Expr::hstack(u,
                             Expr::constTerm(k, 1.0),
                             Expr::add(Expr::mul(A, x), b)), Domain::inPExpCone());
}

std::string int_vec_to_str(std::vector<int>& v)
{
	std::string ans = "";
	for (int i = 0; i < v.size(); i++)
		ans += std::to_string(v[i]) + ",";
	return ans;
}

bool is_a_subset_of_b(std::vector<int>& a, std::vector<int>& b)
{
	for (int i = 0; i < a.size(); i++)
	{
		if (std::find(b.begin(), b.end(), a[i]) == b.end() )
			return false; // a[i] not in b.
	}
	return true;
}

void test_solver_multicore()
{
	// Testing kcore solver on micaela's DAG
	Model::t testM = new Model("test_kCore");
	auto _testM = finally([&]() { testM->dispose(); });
	int total_vars = 7;
	Variable::t ki = testM->variable(total_vars);

	std::vector<double> a (total_vars, 0.0);
        a[total_vars-1] = 1; 
	
	testM->objective("Objective", ObjectiveSense::Minimize, Expr::dot(new_array_ptr<double> (a), ki) );

	double crit_ci = 56;
	double cdf_ci = 55;
	double gps_ci = 7;
	
	double crit_cons = 120;
	double cdf_cons = 150;
	double gps_cons = 120;

	double crit_mi = 28, cdf_mi = 28, gps_mi = 7;

	logsumexp(testM,
			new_array_ptr<double,2> ( { {0, 0, 0, -1, 0, 0, 0} } ),
			ki,
			new_array_ptr<double,1> ( {log(crit_mi)} ) );
	logsumexp(testM,
			new_array_ptr<double,2> ( { {-1, 0, 0, -1, 0, 0, 0} } ),
                        ki,
                        new_array_ptr<double,1> ( {log(crit_ci)} ) );
	logsumexp(testM,
                        new_array_ptr<double,2> ( { {0, 0, 0, 0, -1, 0, 0} } ),
                        ki,
                        new_array_ptr<double,1> ( {log(cdf_mi)} ) );
	logsumexp(testM,
                        new_array_ptr<double,2> ( { {0, -1, 0, 0, -1, 0, 0} } ),
                        ki,
                        new_array_ptr<double,1> ( {log(cdf_ci)} ) );
	logsumexp(testM,
                        new_array_ptr<double,2> ( { {0, 0, 0, 0, 0, -1, 0} } ),
                        ki,
                        new_array_ptr<double,1> ( {log(gps_mi)} ) );
        logsumexp(testM,
                        new_array_ptr<double,2> ( { {0, 0, -1, 0, 0, -1, 0} } ),
                        ki,
                        new_array_ptr<double,1> ( {log(gps_ci)} ) );

	// sum ki <= t
	logsumexp(testM,
			new_array_ptr<double,2> ( { {1,0,0,0,0,0,-1}, {0,1,0,0,0,0,-1}, {0,0,1,0,0,0,-1} } ),
			ki, 
			new_array_ptr<double,1> ( {0,0,0} ) );
	// si + ti <= xi i=1
	logsumexp(testM,
                        new_array_ptr<double,2> ( { {0,0,0,0,0,0,0}, {0,0,0,1,0,0,0} } ),
                        ki,
                        new_array_ptr<double,1> ( {log(crit_ci/crit_cons),log(1.0/crit_cons)} ) );
	// si + ti <= xi i=2
	logsumexp(testM,
                        new_array_ptr<double,2> ( { {0,0,0,0,0,0,0}, {0,0,0,1,0,0,0}, {0,0,0,0,1,0,0} } ), 
			ki,
			new_array_ptr<double,1> ( { log((9.5+cdf_ci)/cdf_cons), log(1.0/cdf_cons), log(1.0/cdf_cons) } ) );
	// si + ti <= xi i=3
        logsumexp(testM,
                        new_array_ptr<double,2> ( { {0,0,0,0,0,0,0}, {0,0,0,1,0,0,0}, {0,0,0,0,0,1,0} } ), 
                        ki,
                        new_array_ptr<double,1> ( { log((9.5+gps_ci)/gps_cons), log(1.0/gps_cons), log(1.0/gps_cons) } ) );
	/* Constraint that ki's > 1 : 
	for (int i = 0; i < 3; i++)
	{
		std::vector<double> a (total_vars, 0.0);
                a[i] = 1;
		testM->constraint( Expr::dot(new_array_ptr<double>(a) , ki) , Domain::greaterThan(0.001) );
	}
	*/
	clock_t solve_start_rt = clock();
	testM->setLogHandler([](const std::string & msg) { std::cout << msg << std::flush; } );
	testM->solve();
        auto opt_ans = std::make_shared<ndarray<double, 1>>(shape(total_vars), [ki](ptrdiff_t i) { return exp((*(ki->level()))[i]); });
	std::cout << "~~~~~~~ OPTIMAL ANSWER : " << (*opt_ans)[0] << ", " << (*opt_ans)[1] << ", " << (*opt_ans)[2] << ", " << (*opt_ans)[3] << std::endl;
	double solve_time = (double)(clock() - solve_start_rt)/CLOCKS_PER_SEC;
	std::cout << "~~~~ Solve time for ki's for micaela : " << solve_time << std::endl;
};

Monomial::Monomial()
{
}

// Power for all vars in the vector is pow. 0 for others.
Monomial::Monomial(double x, std::vector<int> vars_in_mono, int len, int pow)
{
	c = x;
	powers = std::vector<int> (len, 0);
	for (int i = 0; i < vars_in_mono.size(); i++)
		powers[vars_in_mono[i]] = pow;

	powers_str = int_vec_to_str(powers);
}

void Monomial::print()
{
	std::cout << "I am a mono. Name : " << powers_str << ", c : " << c << ", vec: " << int_vec_to_str(powers) << std::endl;
}

MaxMonomial::MaxMonomial()
{
}

MaxMonomial::MaxMonomial(int id)
{
	gvc_id = id;
	add_to_period = true;
}

MaxMonomial::MaxMonomial(int id, bool atp)
{
	gvc_id = id;
	add_to_period = atp;
}


void MaxMonomial::insert_mono(Monomial m)
{
	if (m.c != 1.0)
		std::cout << "ERRORRRRRRRR!!!!! Mono being added to MaxMono has const other than 1.0!!!!! \n";
	// NOTE that we assume that all the monos in max mono will have const equal to 1.
	if (monos.find(m.powers_str) == monos.end())
		monos[m.powers_str] = m;
}

void MaxMonomial::print()
{
	std::cout << "I am a MaxMono. Id : " << gvc_id << ", atp : " << add_to_period << ", Here are the monos I maximize over :" << std::endl;
	for (std::map<std::string, Monomial>::iterator it = monos.begin(); it != monos.end(); it++)
		it->second.print();
	
}

MinMonomial::MinMonomial()
{
}

MinMonomial::MinMonomial(int id)
{
        gvc_id = id;
}

void MinMonomial::insert_mono(Monomial m)
{
        if (m.c != 1.0)
                std::cout << "ERRORRRRRRRR!!!!! Mono being added to MinMono has const other than 1.0!!!!! \n";
	// NOTE that we assume that all the monos in min mono will have const equal to 1.
        if (monos.find(m.powers_str) == monos.end())
                monos[m.powers_str] = m;
}

void MinMonomial::insert_maxmono(MaxMonomial mm)
{
	maxm_gvc_ids.insert(mm.gvc_id);
}

void MinMonomial::print()
{
	std::cout << "I am a MinMono, Id : " << gvc_id << ", Here are the monos I minimize over : " << std::endl;
	for (std::map<std::string, Monomial>::iterator it = monos.begin(); it != monos.end(); it++)
                it->second.print();
	std::cout << "Here are the maxmonos I minimize over : " << std::endl;
	for (std::set<int>::iterator it = maxm_gvc_ids.begin(); it != maxm_gvc_ids.end(); it++)
		std::cout << (*it) << " ";
	std::cout << std::endl;
}

DAG::DAG()
{
}

DAG::DAG(std::string fname)
{
	global_var_count = 0;
	// Read out DAG structure from file.
	std::cout << "IN DAG Constructor!! \n";
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
				float ci, fixed_per;
				ss >> name >> ci >> fixed_per;

				n.name = name;
				n.id = node_count;
				n.compute = ci;
				n.fixed_period = fixed_per;

				std::cout << "Adding node, name : " << name << ", id : " << n.id << ", ci :" << ci << ", fixed_period: " << fixed_per << std::endl;

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
					na.out_edges[name_id_map[b]] = -1; // -1 denotes na sending all outputs to b.
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
	// test_solver_multicore();
	
}

void DAG::print_vec(std::vector<int>& v, std::string s)
{
	std::cout << s;
	for (int i = 0; i < v.size(); i++)
		std::cout << " " << v[i];
	std::cout << "\n";
}

void DAG::print_dvec(std::vector<double>& v, std::string s)
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
	
	// Oct: commenting sorting if the input chains are already ordered based on criticality.
	// std::sort(chains.begin(), chains.end(), [](const std::tuple<float, std::vector<int>, float, int >& c1, const std::tuple<float, std::vector<int>, float, int >& c2) {return ( std::get<2>(c1) < std::get<2>(c2) ); });
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

// returns true of the edge a->b is in the CC.
bool DAG::is_edge_in_cc(int a, int b)
{
	return ( ( nodes_in_most_critical_chain.find(a) != nodes_in_most_critical_chain.end() ) && ( nodes_in_most_critical_chain.find(b) != nodes_in_most_critical_chain.end() ) );
}

void DAG::assign_publishing_rates()
{
	std::cout << "##### STARTING Step3 : assign_publishing_rates" << std::endl;
	// iterate over all nodes in DAG
	for (std::map<int, DAGNode>::iterator it = id_node_map.begin(); it != id_node_map.end(); it++)
	{
		// for each node, iterate over all nodes that this publishes to:
		DAGNode& dn = it->second;
		int dn_id = it->first;

		int curr_count = 0; // to keep track of the number of unique f variables we shall define:
		for (std::map<int, int>::iterator ot = dn.out_edges.begin(); ot != dn.out_edges.end(); ot++)
		{
			int out_edge_id = ot->first;
			if (id_node_map[out_edge_id].trigger_node != dn_id)
				dn.out_edges[out_edge_id] = -1; // this node is asynchronous wrt dn.
			// Oct: modification : A->B runs at same rate only if B is triggered by A and (is the only one using A's outputs || A-B in CC).
			else if ( (out_edge_id == dn.most_critical_se) && ( ( dn.out_edges.size() == 1 )  || (is_edge_in_cc(dn_id, out_edge_id)) ) )
				dn.out_edges[out_edge_id] = -1; // this node is the most critical out of all nodes triggered by dn.
			else
			{
				curr_count += 1;
				dn.out_edges[out_edge_id] = curr_count; // each +ve integer denotes a new fractional variable.
				
				// Aug23 : USING global VAR count here.
				dn.out_edges[out_edge_id] = global_var_count; 
				global_var_count += 1;
				global_var_desc.push_back(std::make_tuple(dn.name, dn.id, id_node_map[out_edge_id].name, out_edge_id));
				
			}
			std::cout << "For edge " << dn.name << " -> " << id_name_map[out_edge_id] << ", rate assigned : " << dn.out_edges[out_edge_id] << ", curr_count : " << curr_count << ", global_var_count : " << global_var_count << std::endl;	
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
			if (id_node_map[ chain[0] ].pub_rate == -1)
			{
				curr_count += 1;
				id_node_map[ chain[0] ].pub_rate = curr_count;
			
				// Aug23 : USING global VAR count here.
				id_node_map[ chain[0] ].pub_rate_frac_var_id = global_var_count;
				global_var_count += 1;
			        global_var_desc.push_back(std::make_tuple(id_node_map[ chain[0] ].name, id_node_map[ chain[0] ].id, "OUT", -1 ));

			}
			
		}
		std::cout << "For src node " << id_name_map[ chain[0] ] << ", rate (as gvc var_id) : " << id_node_map[ chain[0] ].pub_rate_frac_var_id << std::endl;
	}
}

std::vector<std::vector<int> > DAG::get_exec_order()
{
	printf("IN DAG::get_exec_order!!! \n");
	std::vector<std::vector<int> > exec_order;
	std::set<int> covered_nodes;
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::vector<int>& chain = std::get<1>(all_chains[i]);
		std::vector<int> curr_subchain;
		if (covered_nodes.find(chain[0]) == covered_nodes.end())
		{
			std::cout << "Adding chain[0] : " << chain[0] << " to covered nodes \n";
			curr_subchain.push_back(chain[0]);
			covered_nodes.insert(chain[0]);
		}
		for (int j = 1; j < chain.size(); j++)
		{
			if ( (id_node_map[chain[j]].trigger_node == chain[j-1]) && (id_node_map[chain[j-1]].out_edges[chain[j]] == -1) )
			{
				if (covered_nodes.find(chain[j]) == covered_nodes.end())
				{
					std::cout << "Adding chain[j] : " << chain[j] << " to curr subchain & to covered nodes \n";
					curr_subchain.push_back(chain[j]);
				}
			}
			else
			{
				if ( curr_subchain.size() > 0 )
					exec_order.push_back(curr_subchain);
				print_vec(curr_subchain, "Here's the current subchain, About to clear it. ");
				if (covered_nodes.find(chain[j]) == covered_nodes.end())
					curr_subchain = std::vector<int> (1, chain[j]);
				else
					curr_subchain = std::vector<int> (0);
			}
			covered_nodes.insert(chain[j]);
		}

		if ( curr_subchain.size() > 0 )
		{
                	exec_order.push_back(curr_subchain);
			print_vec(curr_subchain, "Here's the current subchain, About to clear it, since chain has been traversed. ");
		}
	}
	return exec_order;
}

std::map<int, std::vector<int>> DAG::get_period()
{
	// Should return a vector of length gvc.
	std::cout << "##### STARTING Step5.1 : Getting period" << std::endl;
	std::map<int, std::vector<int>> covered_nodes; // all nodes that've been covered already. id->vec(var ids to multiply)
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::cout << "Starting chain #" << i << std::endl;
		std::vector<int>& chain = std::get<1>(all_chains[i]);
		std::set<int> current_frac; // probbaly not needed!
		for (int j = 0; j < chain.size(); j++)
		{
			 if (covered_nodes.find(chain[j]) == covered_nodes.end() )
			{
				std::cout << "New node : " << chain[j];
				// this node needs to be added to covered_nodes.
				if (nodes_in_most_critical_chain.find(chain[j]) != nodes_in_most_critical_chain.end())
				{
					std::cout << " In critical chain!! ";
					covered_nodes[chain[j]] = std::vector<int> (0); // fraction=1!
				}
				else if (j == 0)
				{
					covered_nodes[chain[j]] = std::vector<int> (1, id_node_map[chain[j]].pub_rate_frac_var_id); // fraction=its own pub rate!
					current_frac.insert(id_node_map[chain[j]].pub_rate_frac_var_id);
				}
				else if (id_node_map[chain[j]].trigger_node == chain[j-1])
				{
					std::cout << id_node_map[chain[j-1]].id << id_node_map[chain[j-1]].name << id_node_map[chain[j-1]].out_edges[chain[j]] << " -> " << id_node_map[chain[j]].trigger_node;
					// Oct-Dec: Trying a modification where all subchains are independent, and have a separae fi variable
					// i.e. A->B they're either at the same rate or A:fi, B: fj.
					if (id_node_map[chain[j-1]].out_edges[chain[j]] == -1)
					{
						// A,B at same rate.
						std::vector<int> a = std::vector<int> ( covered_nodes[chain[j-1]].begin(), covered_nodes[chain[j-1]].end() );
						// current_frac.insert( id_node_map[chain[j-1]].out_edges[chain[j]] ); // this is the frac. for the rate of publish
						covered_nodes[chain[j]] = std::vector<int> ( a );
					}
					else
					{
						// just one new variable.
						std::vector<int> a (1, id_node_map[chain[j-1]].out_edges[chain[j]]);
						covered_nodes[chain[j]] = std::vector<int> ( a );
					}
				}
				else
					std::cout << "DAMN, this is ERROR!!!" << id_node_map[chain[j-1]].id << id_node_map[chain[j-1]].name << id_node_map[chain[j-1]].out_edges[chain[j]] << " \n";
				print_vec(covered_nodes[chain[j]], " fraction: ");
			}
		}
	}

	for (std::map<int, std::vector<int>>::iterator it = covered_nodes.begin(); it != covered_nodes.end(); it++)
		print_vec(it->second, "fraction of node " + std::to_string(it->first) );
	
	return covered_nodes; // will convert to monomials later.	
}

void DAG::add_mono_to_map(std::map<std::string, Monomial>& monos, Monomial m)
{
	if (m.c == 0.0)
		std::cout << "Empty mono! Not adding :) \n";
	else if ( monos.find(m.powers_str) != monos.end() )
		monos[m.powers_str].c += m.c;
	else
		monos[m.powers_str] = m;
}

void DAG::update_chain_min_rate(std::vector<int>& a, std::vector<std::vector<int> >& x)
{
	if (x.size() == 0)
		x.push_back(a);
	else if (is_a_subset_of_b(x[x.size()-1], a) )
		x[x.size()-1] = a;
	else if ( is_a_subset_of_b(a, x[x.size()-1]) )
		std::cout << "Update chain min rate : Doing nothing \n";
	else
		x.push_back(a);
	std::cout << "New len of chain_min_rate :" << x.size() << std::endl;
}

void DAG::compute_rt_chain_wc(int i, std::map<int, std::vector<int>>& period_map)
{
	// computing RT of a chain
	// using w.c. latency formula similar to davare et al.
	// RT = approx tput + p0 + sum 2*pi.
	// TODO: Handle fixed_period nodes/subchains.
	std::cout << "STARTING TRAVERSING CHAIN id " << i << std::endl;

	std::map<std::string, Monomial> rt_periods;
	std::map<int, MaxMonomial> rt_maxmono_periods;
	std::map<int, MinMonomial> rt_minmono_periods;

	std::vector<int>& ith_chain = std::get<1>(all_chains[i]);

	int mono_len = global_var_count;

	Monomial m1 (1.0, period_map[ith_chain[0]], mono_len, -1);
	add_mono_to_map(rt_periods, m1);

	if (i == 0)
		std::cout << "Most critical chain!! Only adding const 1 to the monomial set. \n";
	else
	{
		MaxMonomial tput (global_var_count, true);
		tput.insert_mono(m1);
		
		std::vector<std::vector<int> > min_rate_tput; // add a new set whenever there's asynchrony.
		global_var_count += 1;
		global_var_desc.push_back( std::make_tuple("chain_approx_tput", i, "", -1) );

		for (int j = 0; j < (ith_chain.size()-1); j++)
		{
			std::cout << "Processing Edge from " << ith_chain[j] << std::endl;
			Monomial mt (1.0, period_map[ith_chain[j+1]], mono_len, -1);
			tput.insert_mono(mt);
			update_chain_min_rate(period_map[ith_chain[j]], min_rate_tput);

			DAGNode& nj = id_node_map[ith_chain[j]]; // node j
			DAGNode& nj1 = id_node_map[ith_chain[j+1]]; // node j+1

			if ( (nj1.trigger_node == nj.id) && (nj.out_edges[nj1.id] == -1) )
			{
				// Running at same rate, do nothing
				std::cout << "Case C1 : Do nothing! Wohoo! \n";
			}
			else
			{
				// Add 2*(#periods in nj1) to rt_periods.
				double mt_const = 2.0;
				if ( nodes_in_most_critical_chain.find(nj1.id) != nodes_in_most_critical_chain.end() )
					mt_const = 1.0; // i.e. no waiting time if CC.
				Monomial mt2 (mt_const, period_map[ith_chain[j+1]], mono_len, -1);
				add_mono_to_map(rt_periods, mt2);
			}
		}
		rt_maxmono_periods[tput.gvc_id] = tput;
	}

	// For RT, we have latency as #Periods + Tput.
	std::cout << "Printing rt_periods : " << std::endl;
	print_mono_map(rt_periods);

	std::cout << "Printing rt_maxmono_periods : " << std::endl;
	for (std::map<int, MaxMonomial>::iterator it = rt_maxmono_periods.begin(); it != rt_maxmono_periods.end(); it++)
		it->second.print();

	all_rt_periods.push_back(rt_periods);
	all_rt_maxmono_periods.push_back(rt_maxmono_periods);
	all_rt_minmono_periods.push_back(rt_minmono_periods);
}

/* Inputs : index of chain in array, map of node id -> its fraction of execution. e.g. f1*f2 will be [1,2].
   This function computes the RT as the number of periods, in the form of a set of monomials.
   Which can be multiplied with the set of monomials representing the period to get the RT expression.
*/
void DAG::compute_rt_chain(int i, std::map<int, std::vector<int>>& period_map)
{
	// computing RT of a chain
	std::cout << "STARTING TRAVERSING CHAIN id " << i << std::endl;		

	std::map<std::string, Monomial> rt_periods;
	std::map<int, MaxMonomial> rt_maxmono_periods; // Need to store then separately for now.
	std::map<int, MinMonomial> rt_minmono_periods;

	std::vector<int>& ith_chain = std::get<1>(all_chains[i]);

	// Add fraction of first var
	// Saving mono_len to ensure that ALL monos for a particular chain have same powers_len.
	int mono_len = global_var_count;

	Monomial m1 (1.0, period_map[ith_chain[0]], mono_len, -1);
	add_mono_to_map(rt_periods, m1);

	if (i == 0)
	{
		std::cout << "Most critical chain!! Only adding const 1 to the monomial set. \n";
	}

	else
	{
		// TODO:Later Use min_rate_tput to reduce the #monos in tput.
		// TODO:Later dont make a new variable for maxMono if same maxMono already exists..
		MaxMonomial tput (global_var_count, true);
		std::vector<std::vector<int> > min_rate_tput; // add a new set whenever there's asynchrony. 
		global_var_count += 1;
		global_var_desc.push_back( std::make_tuple("chain_approx_tput", i, "", -1) );

		double curr_ci = 0; // mostly not needed.

		for (int j = 0; j < (ith_chain.size()-1); j++)
		{
			std::cout << "Processing Edge from " << ith_chain[j] << std::endl;
			Monomial mt (1.0, period_map[ith_chain[j]], mono_len, -1);
			tput.insert_mono(mt);
			update_chain_min_rate(period_map[ith_chain[j]], min_rate_tput);

			DAGNode& nj = id_node_map[ith_chain[j]]; // node j
			DAGNode& nj1 = id_node_map[ith_chain[j+1]]; // node j+1
			
			if ( (nj1.trigger_node == nj.id) && (nj.out_edges[nj1.id] == -1) )
			{
				// Case C1: [Run at the same rate]
				std::cout << "Case C1 : Do nothing! Wohoo! \n";
			}
			else if ( nodes_in_most_critical_chain.find(nj1.id) != nodes_in_most_critical_chain.end() )
			{
				// Case C2:
				// Note that this assumes that CC has f=1 and is on same core as node nj.
				std::cout << "Case C2 \n";
				Monomial m (1.0, std::vector<int> (0), mono_len, -1);
				add_mono_to_map(rt_periods, m);
			}
			// TODO: If we go with all subchains-having-independent-fi-variable, we might not need to check subset, just checking equality would be enough.
			else if ( is_a_subset_of_b( period_map[nj1.id], period_map[nj.id] ) )
			{
				std::cout << "Case C3 ";
				Monomial m(1.0, period_map[nj1.id], mono_len, -1);
				add_mono_to_map(rt_periods, m);
			}
			else if ( nodes_in_most_critical_chain.find(nj.id) != nodes_in_most_critical_chain.end() )
			{
				std::cout << "Case C4 \n";
				if ( (min_rate_tput.size() == 1) && (min_rate_tput[0].size() == 0) )
				{
					std::cout << ": 4.1 1/f1 - 1...";
					Monomial m1 (1.0, period_map[nj1.id], mono_len, -1 );
					Monomial m2 (-1.0, std::vector<int> (0), mono_len, 0);
					add_mono_to_map(rt_periods, m1);
					add_mono_to_map(rt_periods, m2);
				}
				else if ( (min_rate_tput.size() == 1) && (is_a_subset_of_b(period_map[nj1.id], min_rate_tput[0]) ) )
				{
					std::cout << ": 4.2 2/f1 - 1";
					Monomial m1 (2.0, period_map[nj1.id], mono_len, -1 );
					Monomial m2 (-1.0, std::vector<int> (0), mono_len, 0);
					add_mono_to_map(rt_periods, m1);
					add_mono_to_map(rt_periods, m2);
				}
				else
				{
					std::cout << ": 4.3 async!!";
					add_monos_async_edge(period_map[nj1.id], mono_len, rt_periods, rt_maxmono_periods, rt_minmono_periods, nj.id, nj1.id, tput);
				}
			}
			else if ( is_a_subset_of_b(period_map[nj.id], period_map[nj1.id] ) )
			{
				std::cout << "Case C5 \n";
				if ( (min_rate_tput.size() == 1) && (is_a_subset_of_b(min_rate_tput[0], period_map[nj.id]) ) && (is_a_subset_of_b(period_map[nj.id], min_rate_tput[0]) ) )
				{
					std::cout << " case 5.1 ...";
					Monomial m1 (1.0, period_map[nj1.id], mono_len, -1 );
					add_mono_to_map(rt_periods, m1);
				}
				else if ( (min_rate_tput.size() == 1) && (is_a_subset_of_b(period_map[nj1.id], min_rate_tput[0]) ) )
				{
					std::cout << ": 5.2 2/B - 1";
					Monomial m1 (2.0, period_map[nj1.id], mono_len, -1 );
					Monomial m2 (-1.0, std::vector<int> (0), mono_len, 0);
					add_mono_to_map(rt_periods, m1);
					add_mono_to_map(rt_periods, m2);
				}
				else
				{
					std::cout << " : 5.3 Async!! ";
					add_monos_async_edge(period_map[nj1.id], mono_len, rt_periods, rt_maxmono_periods, rt_minmono_periods, nj.id, nj1.id, tput);
				}
			}
			else
			{
				std::cout << "Case C6 : \n";
				add_monos_async_edge(period_map[nj1.id], mono_len, rt_periods, rt_maxmono_periods, rt_minmono_periods, nj.id, nj1.id, tput);
			}
		}
		
		// Add last node's period in tput maxMono.
		Monomial mt (1.0, period_map[ith_chain[(ith_chain.size()-1)]], mono_len, -1);
		tput.insert_mono(mt);
		rt_maxmono_periods[tput.gvc_id] = tput;

	}
	// For RT, we have latency as #Periods + Tput.
	std::cout << "Printing rt_periods : " << std::endl;
	print_mono_map(rt_periods);

	std::cout << "Printing rt_maxmono_periods : " << std::endl;
	for (std::map<int, MaxMonomial>::iterator it = rt_maxmono_periods.begin(); it != rt_maxmono_periods.end(); it++)
                it->second.print();

	std::cout << "Printing rt_minmono_periods : " << std::endl;
        for (std::map<int, MinMonomial>::iterator it = rt_minmono_periods.begin(); it != rt_minmono_periods.end(); it++)
		it->second.print();

	all_rt_periods.push_back(rt_periods);
	all_rt_maxmono_periods.push_back(rt_maxmono_periods);
	all_rt_minmono_periods.push_back(rt_minmono_periods);
}

void DAG::add_monos_async_edge(std::vector<int>& b_frac_set, int mono_len, std::map<std::string, Monomial>& rt_ps, std::map<int, MaxMonomial>& rt_maxm_ps, std::map<int, MinMonomial>& rt_minm_ps, int a_id, int b_id, MaxMonomial curr_tput)
{
	// Exec time term : 
	Monomial m1 (1.0, b_frac_set, mono_len, -1 );
        add_mono_to_map(rt_ps, m1);

	// Wait time term :
	MinMonomial mm(global_var_count);
	global_var_count += 1;
	global_var_desc.push_back(std::make_tuple("min_" + id_node_map[a_id].name, a_id, id_node_map[b_id].name, b_id) );
	// This MinMono should have current tput MaxMono, along with b's period Mono.
	// We make a new maxmono since the curr_tput will be updated for the rest of the chain.
	MaxMonomial tput(global_var_count, false);
	global_var_count += 1;
        global_var_desc.push_back(std::make_tuple("min_max_" + id_node_map[a_id].name, a_id, id_node_map[b_id].name, b_id) );
	
	tput.monos = curr_tput.monos; // copying all monos.
	// We've set add_to_period to false to ensure that this doesnt get added to #periods.
	rt_maxm_ps[tput.gvc_id] = tput;	

	mm.insert_maxmono(tput);
	mm.insert_mono(m1);
	rt_minm_ps[mm.gvc_id] = mm;
}

// After all the Monos and MaxMonos are ready, 
// make a mono for each MM, and update length of all monos to 
void DAG::update_monos(int total_vars)
{
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::map<int, MaxMonomial>& rt_mm_ps = all_rt_maxmono_periods[i];
		std::map<std::string, Monomial>& rt_ps = all_rt_periods[i];
		std::map<int, MinMonomial>& rt_minm_ps = all_rt_minmono_periods[i];
		
		std::map<std::string, Monomial> new_rt_ps;	
	
		std::cout << "$$$$$ Update_monos for chain " << i << std::endl;

		for (std::map<std::string, Monomial>::iterator it = rt_ps.begin(); it != rt_ps.end(); it++)
		{
			std::vector<int> a (total_vars - it->second.powers.size(), 0);
			it->second.powers.insert(it->second.powers.end(), a.begin(), a.end());
			
			Monomial m;
			m.c = it->second.c;
			m.powers = it->second.powers;
			m.powers_str = int_vec_to_str(m.powers);

			add_mono_to_map(new_rt_ps, m);
		}

		for (std::map<int, MaxMonomial>::iterator it = rt_mm_ps.begin(); it != rt_mm_ps.end(); it++)
		{
			if (it->second.add_to_period)
			{	
				std::vector<int> a (1, it->second.gvc_id);
				Monomial mm (1.0, a, total_vars, -1); // 1/var_gvc_id = max (1/f1, 1/f2.... etc).
				add_mono_to_map(new_rt_ps, mm);
			}
		}

		for (std::map<int, MinMonomial>::iterator it = rt_minm_ps.begin(); it != rt_minm_ps.end(); it++)
		{
			std::vector<int> a (1, it->second.gvc_id);
                        Monomial minm (1.0, a, total_vars, -1);
			add_mono_to_map(new_rt_ps, minm);
		}
		
		rt_ps.clear();
		all_rt_periods[i] = new_rt_ps;
		std::cout << "Here's updated monoMAP for chain " << i << std::endl;
		print_mono_map(rt_ps);
	}
}

std::map<std::string, Monomial> DAG::convert_period_to_monos(std::map<int, std::vector<int>>& period_map, int total_vars)
{
	std::map<std::string, Monomial> period_mono_set;
	for (std::map<int, std::vector<int>>::iterator it = period_map.begin(); it != period_map.end(); it++)
	{
		Monomial m (id_node_map[it->first].compute, it->second, total_vars, 1); // product of ci * all fractions in period_map[ni]
		add_mono_to_map(period_mono_set, m);
	}
	std::cout << "Here's the period mono set : " << std::endl;
	print_mono_map(period_mono_set);
	return period_mono_set;
}

void DAG::print_global_vars_desc()
{
	std::cout << "Printing all vars desc : \n";
        for (int i = 0; i < global_var_count; i++)
                std::cout << std::get<0>(global_var_desc[i]) << std::get<1>(global_var_desc[i]) << std::get<2>(global_var_desc[i]) << std::get<3>(global_var_desc[i]) << ", ";
        std::cout << "\n";
}

void DAG::print_mono_map( std::map<std::string, Monomial>& s )
{
	for (std::map<std::string, Monomial>::iterator it = s.begin(); it != s.end(); it++)
		it->second.print();
}

Monomial DAG::multiply_monos(Monomial m1, Monomial m2)
{
	Monomial m;
	if (m1.powers.size() != m2.powers.size())
		std::cout << "ERRORRRRRR!!!!! m1, m2 monomials' power vec are of different sizes!!!! \n";
	else
	{
		m.c = m1.c * m2.c;
		m.powers = std::vector<int> (m1.powers.size());
		for (int i = 0; i < m1.powers.size(); i++)
			m.powers[i] = m1.powers[i] + m2.powers[i];
		m.powers_str = int_vec_to_str(m.powers);
	}
	// std::cout << "Multiplying the following monos : ";
	// std::cout << "Output : ";

	assert(m1.powers_str.compare(int_vec_to_str(m1.powers)) == 0);
	assert(m2.powers_str.compare(int_vec_to_str(m2.powers)) == 0);
	assert(m.powers_str.compare(int_vec_to_str(m.powers)) == 0);
	
	m.print();
	return m;
}

void DAG::multiply_monos_with_period(std::map<std::string, Monomial>& period_mono_set)
{
	int total_vars = 0;
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::cout << "Starting chain " << i << std::endl;
		std::map<std::string, Monomial> new_mono_set_i;
		std::map<std::string, Monomial>& rt_mono_periods = all_rt_periods[i];
		// Multiply all monos in rt_mono_periods to period_mono_Set.
		for (std::map<std::string, Monomial>::iterator it = rt_mono_periods.begin(); it != rt_mono_periods.end(); it++)
		{
			// std::cout << "Picking up mono from rt_mono_periods ";
			// it->second.print();
			for (std::map<std::string, Monomial>::iterator itp = period_mono_set.begin(); itp != period_mono_set.end(); itp++)
			{
				// std::cout << "Multiplying to : ";
				// itp->second.print();
				Monomial m = multiply_monos(it->second, itp->second);
				add_mono_to_map(new_mono_set_i, m);
				total_vars = m.powers.size();
			}
		}
		rt_mono_periods.clear();
		all_rt_periods[i] = new_mono_set_i;
		std::cout << "Here's the final RT mono set [rt_per_mono*period_mono] for chain " << i << std::endl;
		print_mono_map(all_rt_periods[i]);
	}
}

void DAG::add_constraints_for_max_monos(int total_vars, Variable::t all_lfrac_vars)
{
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::map<int, MaxMonomial>& rt_mm_ps = all_rt_maxmono_periods[i];
		for (std::map<int, MaxMonomial>::iterator it = rt_mm_ps.begin(); it != rt_mm_ps.end(); it++)
		{
			MaxMonomial& mm = it->second;
			std::cout << "The maxMono : " << std::to_string(mm.gvc_id);
			mm.print();
			for (std::map<std::string, Monomial>::iterator itm = mm.monos.begin(); itm != mm.monos.end(); itm++)
			{
				Monomial& m = itm->second;
				// e^gvc_id+[m.powers DOT variables vec] <= 1.
				std::vector<std::vector<double>> a (1, std::vector<double> (total_vars, 0.0) );
				a[0][mm.gvc_id] = 1;
				for (int j = 0; j < m.powers.size(); j++)
					a[0][j] = m.powers[j];
				m.print();
				print_dvec(a[0], "Here is the vector for MaxMono " + std::to_string(mm.gvc_id));
				logsumexp(mosek_model,
						new_array_ptr<double>( a ),
						all_lfrac_vars,
						new_array_ptr<double,1> ( { 0 } ) ); 
			}
		}
	}
}

void DAG::add_constraints_for_min_monos(int total_vars, Variable::t all_lfrac_vars)
{
	for (int i = 0; i < all_chains.size(); i++)
        {
                std::map<int, MinMonomial>& rt_minm_ps = all_rt_minmono_periods[i];
		for (std::map<int, MinMonomial>::iterator it = rt_minm_ps.begin(); it != rt_minm_ps.end(); it++)
		{
			MinMonomial& mm = it->second;
			std::cout << "The minMono : " << std::to_string(mm.gvc_id);
			mm.print();
			for (std::map<std::string, Monomial>::iterator itm = mm.monos.begin(); itm != mm.monos.end(); itm++)
                        {
				Monomial& m = itm->second;
				// e^ - gvc_id - [m.powers DOT variables vec] <= 1.
				std::vector<std::vector<double>> a (1, std::vector<double> (total_vars, 0.0) );
                                a[0][mm.gvc_id] = -1;
				for (int j = 0; j < m.powers.size(); j++)
                                        a[0][j] = -1*(m.powers[j]);
				m.print();
                                print_dvec(a[0], "Here is the vector for MinMono " + std::to_string(mm.gvc_id));
				logsumexp(mosek_model,
                                                new_array_ptr<double>( a ),
                                                all_lfrac_vars,
                                                new_array_ptr<double,1> ( { 0 } ) );
			}

			std::cout << "Now iterating over MaxMonos for minmomo " << mm.gvc_id << std::endl;
			for (std::set<int>::iterator its = mm.maxm_gvc_ids.begin(); its != mm.maxm_gvc_ids.end(); its++)
			{
				int maxmono_gvc_id = (*its);
				std::vector<std::vector<double>> a (1, std::vector<double> (total_vars, 0.0) );
				a[0][maxmono_gvc_id] = 1;
				a[0][mm.gvc_id] = -1;
                                print_dvec(a[0], "Here is the vector for MinMono " + std::to_string(mm.gvc_id) + ", maxmono id : " + std::to_string(maxmono_gvc_id) );
				logsumexp(mosek_model,
                                                new_array_ptr<double>( a ),
                                                all_lfrac_vars,
                                                new_array_ptr<double,1> ( { 0 } ) );
			}
		}
	}
}

void DAG::clear_old_data(int frac_var_count)
{
	global_var_count = frac_var_count;
	global_var_desc.resize(frac_var_count);
	std::cout << "CLEARED THE global_vars. New set: " << std::endl;
	print_global_vars_desc();

	all_rt_periods.clear();
	all_rt_maxmono_periods.clear();
	all_rt_minmono_periods.clear();
}

void DAG::update_cis(std::map<std::string, boost::circular_buffer<double> >& node_ci_arr)
{
	for (auto const& x: node_ci_arr)
	{
		std::vector<double> cis;
		if (x.second.size() > 0)
		{
			for (auto const& y: x.second)
				cis.push_back(y);
			std::sort(cis.begin(), cis.end());
			std::cout << "UPDATED Compute time of node " << x.first << " from " << id_node_map [ name_id_map [ x.first ] ].compute << " TO " << cis[(75*cis.size())/100]*1000.0 << std::endl;
			id_node_map [ name_id_map [ x.first ] ].compute = cis[(75*cis.size())/100]*1000.0; // Nodes publish time, DAG operates in ms.
		}
	}
}

// returns the value of each fi variable as rounded off (1/fi).
std::vector<int> DAG::compute_rt_solve()
{
	std::cout << "##### STARTING Step5 : computing RT for each chain" << std::endl;
	print_global_vars_desc();
	period_map = get_period();

	// To measure cpu time used by this func:
	struct timespec solve_start, solve_end;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_start);

	// What we essentially need, is to break down the wi*RTi into a sum of monomials, and we need the powers of each frac_var in each monomial term.
	// Then, we need to add a row to Ax+B thing for EACH monomial in the RT expr for EACH chain.
	
	for (int j = 0; j < all_chains.size(); j++)
		compute_rt_chain_wc(j, period_map); // Jan2: Trying WC RT formula.
	
	int total_vars = global_var_count+1;
	std::cout << "Total variables : " << total_vars << std::endl;
	print_global_vars_desc();

	// Done: Update each max mono into a new mono & update len of each Mono powers.
	update_monos(total_vars);

	clock_t ci_start_rt = clock();
	// Get set of Monos for the period.
	// This is another place where compute times are used.
	std::map<std::string, Monomial> period_mono_set = convert_period_to_monos(period_map, total_vars);

	// Store each chain's RT separately [rt_periods*period_mono_set]
	multiply_monos_with_period(period_mono_set);
	
	// TODO:Later - Subtract sum cifi from the RT expression, to account for ordering.

	// Done: Add sum ci to critical chain, after multiplying with period.
	Monomial m (0.0, std::vector<int> (0), total_vars, 0);
	for (std::map<int, int>::iterator it = nodes_in_most_critical_chain.begin(); it != nodes_in_most_critical_chain.end(); it++)
		m.c += id_node_map[it->first].compute;
	add_mono_to_map(all_rt_periods[0], m);	
	std::cout << "Here's the final (added sum ci) set of monos for chain0 : ";
	print_mono_map(all_rt_periods[0]);

	std::cout << "DONE with computing rt for each chain. Starting to make variables now \n";	
	
	// Initialize mosek solver:
	mosek_model = new Model("single_core_scheduler_algo");
	auto _mosek_model = finally([&]() { mosek_model->dispose(); });
	mosek_model->setSolverParam("numThreads", 1);
	mosek_model->setSolverParam("intpntMultiThread", "off");

	Variable::t all_l_vars = mosek_model->variable(total_vars);
	std::cout << "Initialized variable!!" << std::endl;

	// Constraint 1 : All fi's are <= 1, i.e. all log fi's are < 0.
	for (int i = 0; i < global_var_count; i++)
	{
		std::vector<double> a (total_vars, 0.0);
		a[i] = 1;
		mosek_model->constraint( Expr::dot(new_array_ptr<double>(a) , all_l_vars) , Domain::lessThan(-0.001) );
		std::cout << "ADDED vi < 0 constraint for var " << i << std::endl;
	}

	// Done: Constraints for all Max Monos:
	add_constraints_for_max_monos(total_vars, all_l_vars);

	// Done : Constraints for all Min monos:
	add_constraints_for_min_monos(total_vars, all_l_vars);

	clock_t solve_start_rt = clock();

	// Objective function is ALWAYS last variable.
	std::vector<double> a (total_vars, 0.0);
	a[total_vars-1] = 1;
	mosek_model->objective("Objective", ObjectiveSense::Minimize, Expr::dot(new_array_ptr<double> (a), all_l_vars) );

	//This last part is where compute times are used : 
	if (!is_constraint)
	{
		std::cout << "WEIGHT FORMULATION. ABOUT TO MERGE ALL RTs AND ADD CONSTRAINT \n";
		// Done: One constraint : sum wi*Ri <= last_variable.
		// i.e. sum mono*corresponding_wt/last_variable <= 1.
		std::vector< std::vector<double>> allA;
                std::vector<double> allB;
		for (int i = 0; i < all_chains.size(); i++)
		{
			double wt = std::get<0>(all_chains[i]);
			std::map<std::string, Monomial>& all_rt_monos = all_rt_periods[i];
			for (std::map<std::string, Monomial>::iterator cit = all_rt_monos.begin(); cit != all_rt_monos.end(); cit++)
			{
				Monomial& m = cit->second;
				m.print();
				allB.push_back(log(wt*m.c));
				std::vector<double> v (m.powers.begin(), m.powers.end());
                        	v[total_vars-1] = -1;
				allA.push_back(v);
				std::cout << "Added mono with B: " << allB[allB.size()-1] << " wt : " << wt;
				print_dvec(allA[allA.size()-1], " and here's the A :");
			}
		}
		logsumexp(mosek_model,
				new_array_ptr<double> (allA),
				all_l_vars,
				new_array_ptr<double> (allB));
	}
	else
	{
		std::cout << "CONSTRAINT FORMULATION. Part-I : MINIMIZING RT0 + 0.001*other RTs \n";
		// Done: minimize the RT for most critical chain + constraints for all chains.
		// minimize CC RT + very_small_numer*(sum of RT of all other chains)
		// Constraint A : R0 <= last_variable
		std::vector< std::vector<double>> Az;
                std::vector<double> Bz;
                
		for (int i = 0; i < all_chains.size(); i++)
		{
			double wt = 1.0;
			if (i > 1.0)
				wt = 0.001;
			std::map<std::string, Monomial>& all_rt_monos = all_rt_periods[i];
			for (std::map<std::string, Monomial>::iterator cit = all_rt_monos.begin(); cit != all_rt_monos.end(); cit++)
			{
				Monomial& m = cit->second;
				m.print();
				Bz.push_back(log(wt*m.c));
				std::vector<double> v (m.powers.begin(), m.powers.end());
				v[total_vars-1] = -1;
				Az.push_back(v);
				std::cout << "Added mono with B: " << Bz[Bz.size()-1];
				print_dvec(Az[Az.size()-1], " and here's the A :");
			}
		}
		
		/*
		std::map<std::string, Monomial>& all_rt_monos_z = all_rt_periods[0];

		for (std::map<std::string, Monomial>::iterator zit = all_rt_monos_z.begin(); zit != all_rt_monos_z.end(); zit++)
		{
			Monomial& m = zit->second;
			m.print();
			// Constraint : sum (monos)/last_var <= 1.
			Bz.push_back(log(m.c));
			std::vector<double> v (m.powers.begin(), m.powers.end());
			v[total_vars-1] = -1;
			Az.push_back(v);
			std::cout << "Added mono with B: " << Bz[Bz.size()-1];
			print_dvec(Az[Az.size()-1], " and here's the A :");
		}
		*/
		logsumexp(mosek_model,
                                new_array_ptr<double> (Az), 
                                all_l_vars,
                                new_array_ptr<double> (Bz));

		std::cout << "CONSTRAINT FORMULATION. Part-II : CONSTRAINT FOR each chain \n";
		// Constraint B : for all chains, ri <= Ci.
		for (int i = 0; i < all_chains.size(); i++)
		{
			std::cout << "STARTING CHAIN " << i << std::endl;
			// Constraint : sum (monos)/cons <= 1.
			std::vector< std::vector<double>> A;
			std::vector<double> B;
			std::map<std::string, Monomial>& all_rt_monos = all_rt_periods[i];
			double cons = std::get<0>(all_chains[i]);
			for (std::map<std::string, Monomial>::iterator cit = all_rt_monos.begin(); cit != all_rt_monos.end(); cit++)
			{
				Monomial& m = cit->second;
				m.print();
				// Add an elem to a,b for this mono.
				B.push_back(log(m.c/cons));
				A.push_back(std::vector<double>(m.powers.begin(), m.powers.end()));
				std::cout << "Added mono with B: " << B[B.size()-1];
				print_dvec(A[A.size()-1], " and here's the A :");
			}
			logsumexp(mosek_model,
					new_array_ptr<double> (A),
					all_l_vars,
					new_array_ptr<double> (B));
		}
	}

	std::vector<int> all_frac_vals = std::vector<int> (total_vars, 0.0);
	
	try
	{
		mosek_model->setLogHandler([](const std::string & msg) { std::cout << msg << std::flush; } );
		mosek_model->solve();
		mosek_model->acceptedSolutionStatus(AccSolutionStatus::Optimal);
		auto opt_ans = std::make_shared<ndarray<double, 1>>(shape(total_vars), [all_l_vars](ptrdiff_t i) { return exp((*(all_l_vars->level()))[i]); });
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_end);

		// Done: round off to closest integer.
		for (int i = 0; i < total_vars; i++)
		{
			all_frac_vals[i] = (int) round( 1.0/((*opt_ans)[i]) );
			printf("var %i : %f", i, (*opt_ans)[i]);
		}
		std::cout << "OPTIMAL ANSWER : " << (*opt_ans)[0] << ", " << (*opt_ans)[1] << ", " << (*opt_ans)[2] << ", " << (*opt_ans)[3] << std::endl;
		print_vec(all_frac_vals, "Here are all the variables [1/fi] rounded to closest integer ");

	}
	catch (const OptimizeError& e)
	{
		std::cout << "DAG::compute_rt_solve: Optimization failed. Error: " << e.what() << "\n";
	}
	catch (const SolutionError& e)
	{
		std::cout << "DAG::compute_rt_solve: Requested solution was not available.\n";
		auto prosta = mosek_model->getProblemStatus();
		switch(prosta)
		{
			case ProblemStatus::DualInfeasible:
				std::cout << "Dual infeasibility certificate found.\n";
				break;
			case ProblemStatus::PrimalInfeasible:
				std::cout << "Primal infeasibility certificate found.\n";
				break;
			case ProblemStatus::Unknown:
				std::cout << "The solution status is unknown.\n";
				char symname[MSK_MAX_STR_LEN];
				char desc[MSK_MAX_STR_LEN];
				MSK_getcodedesc((MSKrescodee)(mosek_model->getSolverIntInfo("optimizeResponse")), symname, desc);
				std::cout << "  Termination code: " << symname << " " << desc << "\n";
				break;
			default:
				std::cout << "Another unexpected problem status: " << prosta << "\n";
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "DAG::compute_rt_solve: Unexpected error: " << e.what() << "\n";
	}
	if (all_frac_vals[0] == 0)
	{
		for (int i = 1; i < all_chains.size(); i++)
		{
			std::get<0>(all_chains[i]) = 1.5*(std::get<0>(all_chains[i]));
			printf("New constraint for chain %i is : %f", i, std::get<0>(all_chains[i])); 
		}
		printf("Solver unable to satisfy constraints!! Loosening all non-CC constraints by 1.5x. \n");
	}
		// std::cout << (int) round(2.3) << ", " << (int) round(2.7) << ", " << (int) round(1.11) << ", " << std::endl;
	double total_time_ci = (double)(clock() - ci_start_rt)/CLOCKS_PER_SEC;
	double solve_time = (double)(clock() - solve_start_rt)/CLOCKS_PER_SEC;	
	std::cout << "TOTAL time including stuff that uses ci : " << total_time_ci << ", solve time : " << solve_time << ", CPUTime to solve: " << ( (solve_end.tv_sec + solve_end.tv_nsec*1e-9) - (solve_start.tv_sec + 1e-9*solve_start.tv_nsec) ) << std::endl;
	mosek_model->dispose();
	return all_frac_vals;
} 
