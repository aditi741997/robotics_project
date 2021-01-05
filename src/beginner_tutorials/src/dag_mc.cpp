#include <dag_mc.h>
#include <assert.h>

DAGMultiCore::DAGMultiCore()
{
}

void DAGMultiCore::set_params(int numc, std::vector< std::vector<int> >& sc_core_assgt)
{
	printf("IN DAGMultiCore::set_params, numc: %i, len of sc_core_assgt: %i, len of sc_core_assgt[0]: %i \n", numc, sc_core_assgt.size(), sc_core_assgt[0].size());
	num_cores = numc;
	core_sc_assgt = sc_core_assgt;
}

void DAGMultiCore::assign_fixed_periods()
{
	printf("DAGMultiCore ABOUT TO assign_fixed_periods!! WILL clear global_var info first! \n");
	global_var_count = 0;
	global_var_desc.clear();

	for (int i = 0; i < num_cores; i++)
		core_sc_list[i] = std::vector<int> ();

	std::vector<std::vector<int> > exec_order = get_exec_order();
	for (int i = 0; i < exec_order.size(); i++)
	{
		for (int j = 0; j < core_sc_assgt[i].size(); j++)
			core_sc_list[ core_sc_assgt[i][j] ].push_back(i);
	}
	std::cout << "Filled the list of subchains for each core. \n";
	for (int i = 0; i < num_cores; i++)
		print_vec(core_sc_list[i], "List of subchain ids on core"+std::to_string(i));

	// for subchains alone on >=1 cores
	for (int i = 0; i < exec_order.size(); i++)
	{
		bool alone = true;
		for (int j = 0; j < core_sc_assgt[i].size(); j++)
			alone = alone && (core_sc_list[ core_sc_assgt[i][j] ].size() == 1);
		printf("FOR subchain id %i, value of alone: %i \n", i, alone);
		if (alone)
		{
			// get fixed period for all nodes in ith subchain
			double sum_ci = 0.0;
			double max_ci = 0.0;
			for (int n = 0; n < exec_order[i].size(); n++ )
			{
				double ci = id_node_map[ exec_order[i][n] ].compute;
				max_ci = std::max(max_ci, ci);
				sum_ci += ci;
			}
			int k = core_sc_assgt[i].size();
			double per = std::max(max_ci, sum_ci/k);
			
			printf("SUBCHAIN id %i ALONE! Assigning fixed_period: %f \n", i, per);

			// modify fixed_period only if its not set currently.
			for (int n = 0; n < exec_order[i].size(); n++ )
			{
				if (id_node_map[ exec_order[i][n] ].fixed_period == 0)
					id_node_map[ exec_order[i][n] ].fixed_period = per;
				
				printf("SUBCHAIN id %i, node_id %i has fixed_period: %f \n", i, exec_order[i][n], id_node_map[ exec_order[i][n] ].fixed_period );
				node_id_sc_id[ exec_order[i][n] ] = i;
			}
		}
		else
		{
			// make a new variable for its fraction.
			std::string sc_name = std::to_string(i);
			for (int n = 0; n < exec_order[i].size(); n++ )
			{
				sc_name += "_" + id_node_map[ exec_order[i][n] ].name;
				node_id_sc_id[ exec_order[i][n] ] = i;
			}
			
			printf("FOR Subchain %i, Made a frac variable id = %i \n", i, global_var_count);

			sc_id_frac_id[i] = global_var_count;
			global_var_count += 1;
			global_var_desc.push_back( std::make_tuple( "frac_"+ sc_name, 0, "", 0) );
		}
	}
}

/*
void DAGMultiCore::compute_rt_chain_wc_mc(int i, int num_vars)
{
	// traverse each chain node by node. get subchain id for each [node_id_sc_id].
	// do nothing if same subchain and fixed_period = 0.0
	std::cout << "STARTING TRAVERSING CHAIN id " << i << std::endl;

	std::map<std::string, Monomial> rt_periods;
	std::map<int, MaxMonomial> rt_maxmono_periods; // maxMono wont work cuz here, tput = max(posynomials)

	std::vector<int>& ith_chain = std::get<1>(all_chains[i]);

	int mono_len = global_var_count;

	// Period of node0 is either fixed_period or a set of monos = 1/f0 * (sum fi*ci for same core)
	double m0_c = () ? : ;
	std::vector<int> m0_p = ;
	Monomial m1 ();
	 
	for (int j = 0; j < (ith_chain.size()-1); j++)
	{
		DAGNode& nj = id_node_map[ith_chain[j]]; // node j
		DAGNode& nj1 = id_node_map[ith_chain[j+1]]; // node j+1

		if ( (nj1.fixed_period > 0) && (nj1.fixed_period == nj.fixed_period) || (node_id_sc_id[nj1.id] == node_id_sc_id[nj.id]) )
			// actually, add nj1's compute.
			// std::cout << "Case MC.WC1 - DO Nothing!!" << std::endl;
		else
		{
			// Case MC.WC2 : Add P (wait) + sum ci (exec)
			// Case MC.WC3 : Add 2P
		}
	}
}
*/

void DAGMultiCore::get_period_map_core(int i, int total_vars, std::vector<std::vector<int> >& exec_order)
{
	printf("IN DAGMultiCore::get_period_map_core for core %i, total_vars %i, with exec_order input \n", i, total_vars);
	std::vector<std::vector<double> > mono_powers;
	std::vector<double> mono_const;
	if (core_sc_list[i].size() <= 1)
		return;

	for (int sci = 0; sci < core_sc_list[i].size(); sci++ )
	{
		int scid = core_sc_list[i][sci];

		// get sum ci for sci
		double sumci = 0.0;

		// NOTE that we're also adding the ci for nodes like imu,int with fixed period.
		// i.e. we're accounting for CCfreq*their ci in our schedule, will only need to care about (200Hz-CCfreq)*ci separately.
		for (int j = 0; j < exec_order[scid].size(); j++)
			sumci += id_node_map[ exec_order[scid][j] ].compute;
		mono_const.push_back(log(sumci));
		
		printf("ADDING ci for subchain id %i, sum=%f, frac var id = %i \n", scid, sumci, sc_id_frac_id[scid]);

		// add a vec to ans : sumci*fi :
		std::vector<double> ai (total_vars, 0.0);
		ai[ sc_id_frac_id[scid] ] = 1.0;
		mono_powers.push_back(ai);

		print_dvec(ai, "ADDED this vec to period for core " + std::to_string(i));
	}
	printf("We have the mono arrs ready for core %i, len: %i %i \n", i, mono_powers.size(), mono_const.size());
	per_core_period_mono_powers[i] = mono_powers;
	per_core_period_mono_const[i] = mono_const;
}

std::pair< std::vector<std::vector<double> >, std::vector<double> > DAGMultiCore::get_period_node(int node_id)
{
	assert( core_sc_assgt[node_id_sc_id[node_id]].size() == 1 );
	std::vector<std::vector<double> >& cvp = per_core_period_mono_powers[ core_sc_assgt[node_id_sc_id[node_id]][0] ];
	std::vector<double>& cvc = per_core_period_mono_const[ core_sc_assgt[node_id_sc_id[node_id]][0] ];

	std::vector<std::vector<double> > ap (cvp.begin(), cvp.end());
	std::vector<double> ac (cvc.begin(), cvc.end());
	printf("Node %i belongs to subchain %i, core %i, len of ap,ac: %i %i \n", node_id, node_id_sc_id[node_id], core_sc_assgt[node_id_sc_id[node_id]][0], ap.size(), ac.size());

	// multiply by 1/f to get period
	for (int i = 0; i < ap.size(); i++)
	{
		ap[i][ sc_id_frac_id[ node_id_sc_id[node_id] ] ] -= 1.0;
		print_dvec(ap[i], "SUBchain Period_posynomial "+ std::to_string(i)+"th term power, const: " + std::to_string(ac[i]) );
	}

	return std::make_pair(ap, ac);
}

std::vector<int> DAGMultiCore::compute_rt_solve()
{
	std::cout << "IN DAGMultiCore: ##### STARTING Step5 : computing RT for each chain" << std::endl;
	struct timespec solve_start, solve_end;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &solve_start);
	
	// RT for each chain:
	int frac_var_ct = global_var_count;
	for (int j = 0; j < all_chains.size(); j++)
	{
		// compute_rt_chain_wc_mc(j); // mc: multi core, wc: worst case
		printf("For chain id %i, approx_period varid %i \n", j, global_var_count);
		global_var_count += 1;
		global_var_desc.push_back(std::make_tuple("approx_period_chain",j,"",0));
	}
	print_global_vars_desc();

	mosek_model = new Model("multi_core_scheduler_algo");
	auto _mosek_model = finally([&]() { mosek_model->dispose(); });
	mosek_model->setSolverParam("numThreads", 1);
	mosek_model->setSolverParam("intpntMultiThread", "off");

	// num_vars: curr_count + num_chains (one approx_tput for each chain)
	int total_vars = global_var_count+1;
	Variable::t all_l_vars = mosek_model->variable(total_vars);
	if (sc_id_frac_id.find(0) != sc_id_frac_id.end())
	{
		// set CC's fraction=1. log=0
		assert( sc_id_frac_id[0] == 0 );
		auto sc0_sol = new_array_ptr<double,1>({-0.0001});
		all_l_vars->slice(0,1)->setLevel(sc0_sol);
	}

	// Constraint 1 : All fi's are <= 1, i.e. all log fi's are < 0.
	for (int i = 0; i < frac_var_ct; i++)
	{
		std::vector<double> a (total_vars, 0.0);
		a[i] = 1;
		mosek_model->constraint( Expr::dot(new_array_ptr<double>(a) , all_l_vars) , Domain::lessThan(0.0) );
		std::cout << "ADDED vi < 0 constraint for var " << i << std::endl;
	}

	// hyper-period of each core as vec of vecs...
	std::vector<std::vector<int> > exec_order = get_exec_order();
	for (int i = 0; i < num_cores; i++)
	// need a period map for each core with >1 subchains.
		if (core_sc_list[i].size() > 1)
			get_period_map_core(i, total_vars, exec_order);

	// approx_per chain i >= period of all nodes.
	for (int i = 0; i < all_chains.size(); i++)
	{
		printf("About to put constraints for approx period for chain %i \n", i);

		std::vector<int>& ith_chain = std::get<1>(all_chains[i]);
		for (int j = 0; j < ith_chain.size(); j++)
		{
			DAGNode& jnode = id_node_map[ ith_chain[j] ];
			printf("Traversing chain %i, %ith node id: %i, fixed_per: %f \n", i, j, jnode.id, jnode.fixed_period);
			if ( jnode.fixed_period > 0 )
			{
				// ch_i_period >= this fixed period
				// 1/ch_i_period <= 1/fixed_period
				/*
				std::vector<std::vector<double>> a (1, std::vector<double>(total_vars, 0.0) );
				a[0][frac_var_ct + i] = -1.0;

				print_dvec(a[0], "Adding this vec for constr with fixed_per "+std::to_string(id_node_map[ ith_chain[j] ].fixed_period)+" on approx_period of chain"+std::to_string(i)+", var id "+std::to_string(frac_var_ct + i) );
				
				logsumexp(mosek_model,
						new_array_ptr<double>( a ),
						all_l_vars,
						new_array_ptr<double,1> ( { log(1.0/id_node_map[ ith_chain[j] ].fixed_period) } ) );
				*/
				std::vector<double> a (total_vars, 0.0);
				a[frac_var_ct + i] = -1.0;
				// log (approx_per) >= log(fixed_per)
				mosek_model->constraint( Expr::dot(new_array_ptr<double>(a), all_l_vars), Domain::lessThan( log(1.0/id_node_map[ ith_chain[j] ].fixed_period) ) ); 
			}
			else
			{
				// get period of the node's SC.
				// 1/fi * period of that core <= ch_i_period
				assert(core_sc_assgt[node_id_sc_id[jnode.id]].size() == 1);
				
				auto& cp_p = per_core_period_mono_powers[ core_sc_assgt[node_id_sc_id[jnode.id]][0] ];
				auto& cp_c = per_core_period_mono_const[ core_sc_assgt[node_id_sc_id[jnode.id]][0] ];
				std::vector<std::vector<double>> ap (cp_p.begin(), cp_p.end());
				std::vector<double> ac (cp_c.begin(), cp_c.end());

				printf("ABOUT to put constraint on approx_period of chain %i, var id %i using period of subchain %i, its on core %i, #monos in this core's period: %i %i \n", i, frac_var_ct + i, node_id_sc_id[jnode.id], core_sc_assgt[node_id_sc_id[jnode.id]][0], ap.size(), ac.size() );

				// Divide all terms by sc_frac and by ch_i_period. <= 1.
				for (int api = 0; api < ap.size(); api++)
				{
					ap[api][ sc_id_frac_id[ node_id_sc_id[jnode.id] ] ] -= 1.0;
					ap[api][ frac_var_ct + i ] -= 1.0;
				
					print_dvec(ap[api], "In period_posynomial for sc"+std::to_string(i)+std::to_string(api)+"ith term, const: " + std::to_string(ac[api]) );
				}

				logsumexp(mosek_model,
						new_array_ptr<double>( ap ),
						all_l_vars,
						new_array_ptr<double>( ac ));
			}
		}
	}

	// Objective: 
	std::vector<std::vector<double>> obj_ap;
	std::vector<double> obj_ac;

	// RTs for each chain:
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::vector<std::vector<double>> ith_ap;
		std::vector<double> ith_ac;
		std::cout << "STARTING adding RT constr for chain " << i << std::endl;
		std::vector<int>& ith_chain = std::get<1>(all_chains[i]);

		// Add chain0's exec time.
		if ( (id_node_map[ith_chain[0]].fixed_period > 0) || (node_id_sc_id[ith_chain[0]] == 0) )
		{
			// add ci.
			std::cout << "ADDING ci for node 0 in chain " << i << std::endl;
			ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
			ith_ac.push_back( log( id_node_map[ith_chain[0]].compute ) );
		}
		else
		{
			std::pair< std::vector<std::vector<double> >, std::vector<double> > n0_period = get_period_node(ith_chain[0]);
			ith_ap.insert(ith_ap.end(), n0_period.first.begin(), n0_period.first.end());
			ith_ac.insert(ith_ac.end(), n0_period.second.begin(), n0_period.second.end());
		}
		
		// Iterate over nodes in chain
		for (int j = 0; j < (ith_chain.size()-1); j++)
		{
			DAGNode& nj = id_node_map[ith_chain[j]];
			DAGNode& nj1 = id_node_map[ith_chain[j+1]];

			bool add_ci = false;
			double p_mult_factor = -1.0; // -1: dont add period, o.w. multiply period by this.

			if ( (nj1.fixed_period == nj.fixed_period) && (node_id_sc_id[nj1.id] == node_id_sc_id[nj.id]) )
			{
				if ( (nj1.fixed_period > 0) || (node_id_sc_id[nj1.id] == 0) )
				{
					std::cout << "CaseMC.RT 1.a Adding ci=" << nj1.compute << std::endl;
					add_ci = true;
				}
				else
				{
					std::cout << "CaseMC.RT 1.b DOING Nothing!" << std::endl;
				}
			}
			else if ( (nj1.fixed_period != nj.fixed_period) && (node_id_sc_id[nj1.id] == node_id_sc_id[nj.id]) )
			{

				if ( (nj1.fixed_period > 0) || (node_id_sc_id[nj1.id] == 0) )
				{
					// Add P+C1
					std::cout << "CaseMC.RT 2.a Adding ci=" << nj1.compute << "+Period" << std::endl;
					add_ci = true;
					p_mult_factor = 1.0;
				}
				else
				{
					// Add 2P.
					std::cout << "CaseMC.RT 2.b Adding " << "2*Period" << std::endl;
					p_mult_factor = 2.0;
				}
			}
			else if ( (nj1.fixed_period == nj.fixed_period) && (node_id_sc_id[nj1.id] != node_id_sc_id[nj.id]) )
			{
				// std::cout << "THIS SHOULD NOT HAPPEN!! fixed_periods are equal but separate subchains!! " << std::endl;
				// this can happen if both are 0.
				p_mult_factor = 2.0;
				std::cout << "CaseMC.RT 3. ADDING 2*Period\n";
			}
			else
			{
				if ( (nj1.fixed_period > 0) || (node_id_sc_id[nj1.id] == 0) )
				{
					// Add P+c1
					std::cout << "CaseMC.RT 4.a Adding ci=" << nj1.compute << " + Period" << std::endl;
					add_ci = true;
					p_mult_factor = 1.0;
				}
				else
				{
					// Add 2P
					std::cout << "CaseMC.RT 4.b Adding " << "2*Period" << std::endl;
					p_mult_factor = 2.0;
				}
			}

			// Finally adding stuff:
			if (add_ci)
			{
				printf("Add_ci was true. Adding ci = %f \n", nj1.compute);
				ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
				ith_ac.push_back( log( nj1.compute ) );
			}

			if (p_mult_factor > 0.0)
			{
				std::cout << "ADDING Period with factor=" << p_mult_factor << std::endl;

				if (nj1.fixed_period > 0)
				{
					ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
					ith_ac.push_back( log( p_mult_factor*nj1.fixed_period ) );
					printf("Fixed period! = %f \n", nj1.fixed_period);
				}
				else
				{
					std::pair< std::vector<std::vector<double> >, std::vector<double> > nj1_period = get_period_node(nj1.id);
					for (int pmfi = 0; pmfi < nj1_period.second.size(); pmfi++)
						nj1_period.second[pmfi] += log(p_mult_factor);
					printf("Adding period posynomial, num_terms: %i %i \n", nj1_period.first.size(), nj1_period.second.size() );

					ith_ap.insert(ith_ap.end(), nj1_period.first.begin(), nj1_period.first.end() );
					ith_ac.insert(ith_ac.end(), nj1_period.second.begin(), nj1_period.second.end() );
				}
			}

			/*
			if ( ( (nj1.fixed_period > 0) && (nj1.fixed_period == nj.fixed_period) ) || ( (nj.fixed_period == 0.0) && (node_id_sc_id[nj1.id] == node_id_sc_id[nj.id]) && (node_id_sc_id[nj1.id] == 0) ) )
			{
				// just add ci.
				std::cout << "CaseMC.RT 1. Adding ci=" << nj1.compute << std::endl;
				ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
				ith_ac.push_back( log( nj1.compute ) );
			}
			else
			{
				if ( (node_id_sc_id[nj1.id] == node_id_sc_id[nj.id]) )
				{
				// same period NC, non-fixed period: do nothing
					assert( node_id_sc_id[nj1.id]>0 );
					std::cout << "CaseMC.RT 2. Do nothing!" << std::endl;
				}
				else
				{
					// period map:
					std::pair< std::vector<std::vector<double> >, std::vector<double> > nj1_period = get_period_node(nj1.id);
					
					if (node_id_sc_id[nj1.id] == 0)
					{
				// Add period + ci.
						std::cout << "CaseMC.RT 3. Adding period + ci"  << std::endl;
						ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
						ith_ac.push_back( log( nj1.compute ) );

						ith_ap.insert(ith_ap.end(), nj1_period.first.begin(), nj1_period.first.end() );
						ith_ac.insert(ith_ac.end(), nj1_period.second.begin(), nj1_period.second.end() );
					}
					else
					{
				// Add 2*period.
						
					}
				}
			} */
		}

		// Add approx tput of this chain.
		std::vector<double> app_per_ap = std::vector<double>(total_vars, 0.0);
		app_per_ap[frac_var_ct+i] = 1.0;
		print_dvec(app_per_ap, "ADDING approx period for chain");

		ith_ap.push_back(app_per_ap);
		ith_ac.push_back(0.0);

		// Add to the obj_ arrs.
		obj_ap.insert(obj_ap.end(), ith_ap.begin(), ith_ap.end() );
		std::vector<double> ith_ac_copy (ith_ac.begin(), ith_ac.end());
		if (i > 0)
			for (int ic = 0; ic < ith_ac.size(); ic++)
				ith_ac_copy[ic] += log(0.001); // NC nodes' RT*0.001 in obj.
		obj_ac.insert(obj_ac.end(), ith_ac_copy.begin(), ith_ac_copy.end());

		double cons = std::get<0>(all_chains[i]);
		printf("MODIFYing the const vector for chain %i, dividing by constr %f", i, cons);
		for (int ic = 0; ic < ith_ac.size(); ic++)
			ith_ac[ic] += (log(1.0/cons) );
		print_dvec(ith_ac, "New const vec");

		// Constraint:
		logsumexp(mosek_model,
			new_array_ptr<double>(ith_ap),
			all_l_vars,
			new_array_ptr<double>(ith_ac) );
	}

	// Objective Function: CC RT + 0.001* (other RTs)
	// divide all terms in obj_ap by last var [obj]
	for (int oapi=0; oapi<obj_ap.size(); oapi++)
	{
		obj_ap[oapi][total_vars-1] -=1;
		print_dvec(obj_ap[oapi], "IN Objective, "+std::to_string(oapi)+"th term ki powers & const: " + std::to_string(obj_ac[oapi]) );
	}
	
	// Objective func: last variable
	logsumexp(mosek_model,
		new_array_ptr<double>(obj_ap),
		all_l_vars,
		new_array_ptr<double>(obj_ac));

	std::vector<double> a (total_vars, 0.0);
	a[total_vars-1] = 1;
	mosek_model->objective("Objective", ObjectiveSense::Minimize, Expr::dot(new_array_ptr<double> (a), all_l_vars) );	
	
	mosek_model->setLogHandler([](const std::string & msg) { std::cout << msg << std::flush; } );
	mosek_model->solve();
	auto opt_ans = std::make_shared<ndarray<double, 1>>(shape(total_vars), [all_l_vars](ptrdiff_t i) { return exp((*(all_l_vars->level()))[i]); });

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &solve_end);

	std::vector<int> all_frac_vals = std::vector<int> (total_vars, 1);
	for (int i = 0; i < total_vars; i++)
	{
		all_frac_vals[i] = (int) round( 1.0/((*opt_ans)[i]) );
	        printf("var %i : %f", i, (*opt_ans)[i]);
	}
	printf("cputIME TAKEN TO solve: %f", (solve_end.tv_sec + solve_end.tv_nsec*1e-9) - (solve_start.tv_sec + 1e-9*solve_start.tv_nsec));
}
