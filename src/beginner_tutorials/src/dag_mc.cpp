#include <dag_mc.h>
#include <assert.h>

DAGMultiCore::DAGMultiCore()
{
}

void DAGMultiCore::set_params(int numc, std::vector< std::vector<int> >& sc_core_a)
{
	printf("IN DAGMultiCore::set_params, numc: %i, len of sc_core_assgt: %lu, len of sc_core_assgt[0]: %lu \n", numc, sc_core_a.size(), sc_core_a[0].size());
	num_cores = numc;
	sc_core_assgt = sc_core_a;
}

// see if atl one of the nodes in SC is streaming type.
bool DAGMultiCore::is_sc_streaming(std::vector<int>& sc)
{
	for (auto &x : sc)
		if (id_node_map[x].node_type.find("S") != std::string::npos )
			return true;
	return false;
}

void DAGMultiCore::assign_fixed_periods()
{
	printf("DAGMultiCore ABOUT TO assign_fixed_periods!! WILL clear global_var info first! \n");
	global_var_count = 0;
	global_var_desc.clear();

	sc_id_frac_id.clear(); // frac variable for non-alone subchains
	sc_id_alone_per.clear(); // fixed per for alone subchains

	for (int i = 0; i < num_cores; i++)
		core_sc_list[i] = std::vector<int> ();

	std::vector<std::vector<int> > exec_order = get_exec_order();
	for (int i = 0; i < exec_order.size(); i++)
	{
		for (int j = 0; j < sc_core_assgt[i].size(); j++)
			core_sc_list[ sc_core_assgt[i][j] ].push_back(i);
	}
	std::cout << "Filled the list of subchains for each core. \n";
	for (int i = 0; i < num_cores; i++)
		print_vec<int>(core_sc_list[i], "List of subchain ids on core"+std::to_string(i));

	// for subchains alone on >=1 cores
	for (int i = 0; i < exec_order.size(); i++)
	{
		bool alone = true;
		for (int j = 0; j < sc_core_assgt[i].size(); j++)
			alone = alone && (core_sc_list[ sc_core_assgt[i][j] ].size() == 1);
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
			int k = sc_core_assgt[i].size();
			double per = std::max(max_ci, sum_ci/k);
			
			// TODO: Adjust this based on #bins variable for multi threaded scenario.
			printf("SUBCHAIN id %i ALONE! Assigning fixed_period: %f \n", i, per);
				
			// Done: dont overload fixed_period, cuz how will we identify when we re-solve and get a diff core assgt?
			sc_id_alone_per[ i ] = per;

			// modify fixed_period only if its not set currently.
			for (int n = 0; n < exec_order[i].size(); n++ )
			{
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


void DAGMultiCore::get_period_map_core(int i, int total_vars, std::vector<std::vector<int> >& exec_order)
{
	printf("IN DAGMultiCore::get_period_map_core for core %i, total_vars %i, with exec_order input \n", i, total_vars);
	std::vector<std::vector<double> > mono_powers;
	std::vector<double> mono_const;
	if (core_sc_list[i].size() <= 1)
		return;

	double cpu_util_mul = 1.05;
	for (int sci = 0; sci < core_sc_list[i].size(); sci++ )
	{
		int scid = core_sc_list[i][sci];

		// get sum ci for sci
		double sumci = 0.0;

		// NOTE that we're also adding the ci for nodes like imu,int with fixed period.
		// i.e. we're accounting for CCfreq*their ci in our schedule, will only need to care about (200Hz-CCfreq)*ci separately.
		for (int j = 0; j < exec_order[scid].size(); j++)
			sumci += id_node_map[ exec_order[scid][j] ].compute;
		mono_const.push_back(log(sumci) + log(cpu_util_mul) );
		
		printf("ADDING ci for subchain id %i, sum=%f, frac var id = %i \n", scid, sumci, sc_id_frac_id[scid]);

		// add a vec to ans : sumci*fi :
		// CHECK CC FRAC1 Assumption:NOT Assuming f=1 for SCid=0 i.e. CC,-> will just normalize all fracs.
		std::vector<double> ai (total_vars, 0.0);
		// if (scid > 0)
			ai[ sc_id_frac_id[scid] ] = 1.0;
		mono_powers.push_back(ai);

		print_dvec(ai, "ADDED this vec to period for core " + std::to_string(i));
	}
	printf("We have the mono arrs ready for core %i, len: %lu %lu \n", i, mono_powers.size(), mono_const.size());
	per_core_period_mono_powers[i] = mono_powers;
	per_core_period_mono_const[i] = mono_const;
}

std::pair< std::vector<std::vector<double> >, std::vector<double> > DAGMultiCore::get_period_node(int node_id)
{
	assert( sc_core_assgt[node_id_sc_id[node_id]].size() == 1 );
	std::vector<std::vector<double> >& cvp = per_core_period_mono_powers[ sc_core_assgt[node_id_sc_id[node_id]][0] ];
	std::vector<double>& cvc = per_core_period_mono_const[ sc_core_assgt[node_id_sc_id[node_id]][0] ];

	std::vector<std::vector<double> > ap (cvp.begin(), cvp.end());
	std::vector<double> ac (cvc.begin(), cvc.end());
	printf("Node %i belongs to subchain %i, core %i, len of ap,ac: %lu %lu \n", node_id, node_id_sc_id[node_id], sc_core_assgt[node_id_sc_id[node_id]][0], ap.size(), ac.size());

	// multiply by 1/f to get period
	for (int i = 0; i < ap.size(); i++)
	{
		// CHECK CC FRAC1 Assumption: NOT assuming cc frac=1. 
		// if (node_id_sc_id[node_id] > 0)
			ap[i][ sc_id_frac_id[ node_id_sc_id[node_id] ] ] -= 1.0;
		print_dvec(ap[i], "SUBchain Period_posynomial "+ std::to_string(i)+"th term power, const: " + std::to_string(ac[i]) );
	}

	return std::make_pair(ap, ac);
}

double DAGMultiCore::get_fixed_period_node(int nid)
{
	double fixed_per = 0.0;
	if (id_node_map[nid].fixed_period > 0)
		fixed_per = id_node_map[nid].fixed_period;
	else if ( sc_id_alone_per.find( node_id_sc_id[nid] ) != sc_id_alone_per.end() )
		fixed_per = sc_id_alone_per[ node_id_sc_id[nid] ];
	return fixed_per;
}

std::vector<float> DAGMultiCore::compute_rt_solve()
{
	std::cout << "IN DAGMultiCore: ##### STARTING Step5 : computing RT for each chain" << std::endl;
	struct timespec solve_start, solve_end;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_start);

	std::vector<std::vector<int> > exec_order = get_exec_order();
	int frac_var_ct = global_var_count;
	// Period Pni of each subchain:
	for (int i = 0; i < exec_order.size(); i++)
	{
		printf("For subchain id %i, period Pni varid: %i", i, global_var_count);
		global_var_count += 1;
		global_var_desc.push_back( std::make_tuple("periodPni_subchain", i, "", 0) );
	}

	int frac_sc_var_ct = global_var_count;	
	// Approx period for each chain:
	for (int j = 0; j < all_chains.size(); j++)
	{
		printf("For chain id %i, approx_period varid %i \n", j, global_var_count);
		global_var_count += 1;
		global_var_desc.push_back(std::make_tuple("approx_period_chain",j,"",0));
	}

	// Period Pi of each subchain:
	int frac_sc_c_var_ct = global_var_count;
	for (int i = 0; i < exec_order.size(); i++)
	{
		printf("For subchain id %i, period STREam varid: %i", i, global_var_count);
		global_var_count += 1;
		global_var_desc.push_back( std::make_tuple("periodPi_subchain", i, "", 0) );
	}
	print_global_vars_desc();

	mosek_model = new Model("multi_core_scheduler_algo");
	auto _mosek_model = finally([&]() { mosek_model->dispose(); });
	mosek_model->setSolverParam("numThreads", 1);
	mosek_model->setSolverParam("intpntMultiThread", "off");

	int total_vars = global_var_count+1;
	Variable::t all_l_vars = mosek_model->variable(total_vars);

	// Constraint 1 : All fi's are <= 1, i.e. all log fi's are < 0. EXCEPT for streaming nodes!
	for (int i = 0; i < exec_order.size(); i++)
		if (sc_id_frac_id.find(i) != sc_id_frac_id.end())
		{
			if (!is_sc_streaming(exec_order[i]))
			{
				std::vector<double> a (total_vars, 0.0);
				a[ sc_id_frac_id[i] ]  =1;
				mosek_model->constraint( Expr::dot(new_array_ptr<double>(a) , all_l_vars) , Domain::lessThan(0.0) );
				std::cout << "ADDED fi < 0 constraint for var " << sc_id_frac_id[i] << ", SC ID: " << i << std::endl;
			}
			else
				printf("NOT adding fi<1 constraint for sc id %i, var id %i cuz STREAMM \n", i, sc_id_frac_id[i]);
		}
	/* for (int i = 0; i < frac_var_ct; i++)
	{
		std::vector<double> a (total_vars, 0.0);
		a[i] = 1;
		mosek_model->constraint( Expr::dot(new_array_ptr<double>(a) , all_l_vars) , Domain::lessThan(0.0) );
		std::cout << "ADDED fi < 0 constraint for var " << i << std::endl;
	} */
	
	
	// ci*fi > 1ms for all. ci: sum of ith subchain compute.
	// To minimize overhead and keep sleep time error < 10%. [sleep time error is ~0.12ms]
	for (int i = 0; i < exec_order.size(); i++)
		if (sc_id_frac_id.find(i) != sc_id_frac_id.end())
		{
			std::vector<double> a (total_vars, 0.0);
			a[ sc_id_frac_id[i] ] = -1;

			double sumci = 0.0;
			for (int ei = 0; ei < exec_order[i].size(); ei++)
				sumci += id_node_map[ exec_order[i][ei] ].compute;

			if ( sumci > 1.0 )
			{
				mosek_model->constraint( Expr::dot(new_array_ptr<double>(a) , all_l_vars) , Domain::lessThan( log(sumci) ) );
				print_dvec(a, "Adding ci*fi>=1 constr, sumci="+std::to_string(sumci) );
			}
		}

	// hyper-period of each core as vec of vecs...
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
			double fixed_per = get_fixed_period_node(jnode.id); 
			printf("Traversing chain %i, %ith node id: %i, fixed_period: %f, fixed_per: %f \n", i, j, jnode.id, jnode.fixed_period, fixed_per);
			
			// period of SC containing jnode >= fixed_per / period_map_node AND >= min_period.
			if (jnode.min_period > 0)
			{
				std::vector<double> a (total_vars, 0.0);
				a[frac_var_ct + node_id_sc_id[jnode.id] ] = -1.0;
				printf("ADDED constraint on Pni of SC id %i, >= min_per: %f ", node_id_sc_id[jnode.id], jnode.min_period);
				print_dvec(a, "");
				mosek_model->constraint( Expr::dot(new_array_ptr<double>(a), all_l_vars), Domain::lessThan( log(1.0/jnode.min_period) ) );
			}
			// period of SC containing jnode <= max_period.
			if (jnode.max_period > 0)
			{
				std::vector<double> a (total_vars, 0.0);
				a[frac_var_ct + node_id_sc_id[jnode.id] ] = 1.0;
				printf("ADDED constraint on Pni of SC id %i, <= max_per: %f ", node_id_sc_id[jnode.id], jnode.max_period);
				print_dvec(a, "");
				mosek_model->constraint( Expr::dot(new_array_ptr<double>(a), all_l_vars), Domain::lessThan( log(jnode.max_period) ) );
			}

			if (jnode.stream_minper > 0)
			{
				std::vector<double> a (total_vars, 0.0);
				a[frac_sc_c_var_ct + node_id_sc_id[jnode.id] ] = -1.0;
				printf("ADDED constraint on stream per of SC id %i, >= min_per: %f ", node_id_sc_id[jnode.id], jnode.stream_minper);
				print_dvec(a, "");
				mosek_model->constraint( Expr::dot(new_array_ptr<double>(a), all_l_vars), Domain::lessThan( log(1.0/jnode.stream_minper) ) );
			}

			if (jnode.tput_slower_than.size() > 0)
			{
				for (auto &st: jnode.tput_slower_than)
				{
					std::vector<double> a (total_vars, 0.0);
					a[frac_var_ct + node_id_sc_id[jnode.id] ] = -1.0;
					// Pni jnode_sc >= Pni st_sc
					a[frac_var_ct + node_id_sc_id[st] ] = 1.0;
					print_dvec(a, "ADDED SlowerThan: constraint on Pni of " + std::to_string(node_id_sc_id[jnode.id]) + " to be atleast Pni of " + std::to_string(node_id_sc_id[st]) );
					mosek_model->constraint( Expr::dot(new_array_ptr<double>(a), all_l_vars), Domain::lessThan(0.0));
				}
			}

			if ( fixed_per > 0 )
			{
				// subch_i_period >= this fixed period
				// 1/subch_i_period <= 1/fixed_period
				
				std::vector<double> a (total_vars, 0.0);
				a[frac_var_ct + node_id_sc_id[jnode.id] ] = -1.0;
				// log (approx_per) >= log(fixed_per)
				mosek_model->constraint( Expr::dot(new_array_ptr<double>(a), all_l_vars), Domain::lessThan( log(1.0/fixed_per) ) ); 
			}
			else
			{
				// get period of the node's SC.
				// 1/fi * period of that core <= subch_i_period
				assert(sc_core_assgt[node_id_sc_id[jnode.id]].size() == 1);
				
				std::pair< std::vector<std::vector<double>>, std::vector<double> > a_p_c = get_period_node(jnode.id);
				std::vector<std::vector<double>> ap (a_p_c.first.begin(), a_p_c.first.end());
				std::vector<double> ac (a_p_c.second.begin(), a_p_c.second.end() );

				printf("ABOUT to put constraint on period Pi of subchain %i, var id %i using period sum_mono, its on core %i, #monos in this core's period: %lu %lu \n", node_id_sc_id[jnode.id], frac_var_ct+node_id_sc_id[jnode.id] , sc_core_assgt[node_id_sc_id[jnode.id]][0], ap.size(), ac.size() );

				// Divide all terms by sc_frac and by subch_i_period. <= 1.
				for (int api = 0; api < ap.size(); api++)
				{
					ap[api][ frac_sc_c_var_ct + node_id_sc_id[jnode.id] ] -= 1.0;
					print_dvec(ap[api], "In period_posy [Pi] for sc"+std::to_string(node_id_sc_id[jnode.id])+std::to_string(api)+"ith term, const: " + std::to_string(ac[api]) );
				}

				logsumexp(mosek_model,
						new_array_ptr<double>( ap ),
						all_l_vars,
						new_array_ptr<double>( ac ));
			}

			// NOW, approx_period of chain i >= period of subchain node_id_sc_id[jnode.id].
			std::vector<double> a (total_vars, 0.0);
			a[frac_var_ct + node_id_sc_id[jnode.id] ] = 1.0;
			a[ frac_sc_var_ct + i ] = -1.0;
			print_dvec(a, "Putting constraint Period of chain " + std::to_string(i)+" to be atleast Pni of SC"+std::to_string(node_id_sc_id[jnode.id]));
			mosek_model->constraint( Expr::dot(new_array_ptr<double>(a), all_l_vars), Domain::lessThan(0.0) );
		
			std::vector<double> anew (total_vars, 0.0);
			anew[frac_var_ct + node_id_sc_id[jnode.id] ] = -1;
			anew[frac_sc_c_var_ct + node_id_sc_id[jnode.id] ] = 1;
			print_dvec(anew, "Putting constraint Period Pi less than Pni of SC "+std::to_string(node_id_sc_id[jnode.id]));
			mosek_model->constraint( Expr::dot(new_array_ptr<double>(anew), all_l_vars), Domain::lessThan(0.0) );
		}
	}

	// Objective: 
	std::vector<std::vector<double>> obj_ap;
	std::vector<double> obj_ac;

	double cons0 = std::get<0>(all_chains[0]);

	// RTs for each chain:
	for (int i = 0; i < all_chains.size(); i++)
	{
		std::vector<std::vector<double>> ith_ap;
		std::vector<double> ith_ac;
		std::cout << "STARTING adding RT constr for chain " << i << std::endl;
		std::vector<int>& ith_chain = std::get<1>(all_chains[i]);

		// Add chain[0]'s exec time.
		double n0_fixed_per = get_fixed_period_node( ith_chain[0] );
		if ( (n0_fixed_per > 0) ) // || (node_id_sc_id[ith_chain[0]] == 0) 
		{
			// add ci.
			std::cout << "ADDING ci for node 0 in chain " << i << std::endl;
			ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
			ith_ac.push_back( log( id_node_map[ith_chain[0]].compute ) );
		}
		else
		{
			/*std::pair< std::vector<std::vector<double> >, std::vector<double> > n0_period = get_period_node(ith_chain[0]);
			ith_ap.insert(ith_ap.end(), n0_period.first.begin(), n0_period.first.end());
			ith_ac.insert(ith_ac.end(), n0_period.second.begin(), n0_period.second.end());*/
			// Using period of the corresponding subchain, in the new formulation [Jan26]
			std::vector<double> a (total_vars, 0.0);
			a[frac_var_ct +  node_id_sc_id[ ith_chain[0] ] ] = 1.0;
			ith_ap.push_back(a);
			ith_ac.push_back(0.0);
			print_dvec(a, "ADDING period of SC"+std::to_string(node_id_sc_id[ ith_chain[0] ])+" for RT of chain"+std::to_string(i) );
		}

		// Iterate over nodes in chain
		for (int j = 0; j < (ith_chain.size()-1); j++)
		{
			DAGNode& nj = id_node_map[ith_chain[j]];
			DAGNode& nj1 = id_node_map[ith_chain[j+1]];

			// TODO: Differentiate bw cases where Add subchain's sum ci vs node's ci [for illixr]
			// TODO: execi = pi, so execi can now be less than pni. e.g. mcb. use exectime+Waittime.

			bool add_ci = false;
			double p_mult_factor = -1.0; // -1: dont add period, o.w. multiply period by this.

			double nj_fixed_per = get_fixed_period_node(nj.id);
			double nj1_fixed_per = get_fixed_period_node(nj1.id);

			if ( (nj1_fixed_per == nj_fixed_per) && (node_id_sc_id[nj1.id] == node_id_sc_id[nj.id]) )
			{
				if ( (nj1_fixed_per > 0) ) // || (node_id_sc_id[nj1.id] == 0) 
				{
					std::cout << "CaseMC.RT 1.a Adding ci=" << nj1.compute << std::endl;
					add_ci = true;
				}
				else
				{
					std::cout << "CaseMC.RT 1.b DOING Nothing!" << std::endl;
				}
			}
			else if ( (nj1_fixed_per != nj_fixed_per) && (node_id_sc_id[nj1.id] == node_id_sc_id[nj.id]) )
			{

				// CHECK CC FRAC1 Assumption: for illixr
				if ( (nj1_fixed_per > 0) ) // || (node_id_sc_id[nj1.id] == 0)  
				{
					// Add P+C1
					// Todo:maybe, add nj fixed period if nj1 fp=0 & CC : assuming that nj period<nj1 period.
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
			else if ( (nj1_fixed_per == nj_fixed_per) && (node_id_sc_id[nj1.id] != node_id_sc_id[nj.id]) )
			{
				// this can happen if both are 0.
				// if nj1==CC, add P+CC_sumci else 2P.
				// CHECK CC FRAC1 Assumption: for illixr
				/*
				if ( node_id_sc_id[nj1.id] == 0 )
				{
					p_mult_factor = 1.0;
					add_ci = true;
				}
				else */
					p_mult_factor = 2.0;
				std::cout << "CaseMC.RT 3. ADDING 2*Period\n";
			}
			else
			{
				if ( (nj1_fixed_per > 0)  ) // || (node_id_sc_id[nj1.id] == 0)
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
				int sc_id = node_id_sc_id[nj1.id];

				printf("Add_ci was true. Adding ci = %f \n", nj1.compute);
				ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
				ith_ac.push_back( log( nj1.compute ) );
			}

			if (p_mult_factor > 0.0)
			{
				std::cout << "ADDING Period with factor=" << p_mult_factor << std::endl;
				// Directly using node's SC's period variable in Jan26 formulation.
				std::vector<double> a (total_vars, 0.0);
				a[frac_var_ct + node_id_sc_id[nj1.id] ] = 1.0;
				print_dvec(a, "Adding period of subchain"+std::to_string(node_id_sc_id[nj1.id]) );
				ith_ap.push_back(a);
				ith_ac.push_back( log(p_mult_factor) );

				/*
				if (nj1_fixed_per > 0)
				{
					ith_ap.push_back( std::vector<double>(total_vars, 0.0) );
					ith_ac.push_back( log( p_mult_factor*nj1_fixed_per ) );
					printf("Fixed period! = %f \n", nj1_fixed_per);
				}
				else
				{
					std::pair< std::vector<std::vector<double> >, std::vector<double> > nj1_period = get_period_node(nj1.id);
					for (int pmfi = 0; pmfi < nj1_period.second.size(); pmfi++)
						nj1_period.second[pmfi] += log(p_mult_factor);
					printf("Adding period posynomial, num_terms: %lu %lu \n", nj1_period.first.size(), nj1_period.second.size() );

					ith_ap.insert(ith_ap.end(), nj1_period.first.begin(), nj1_period.first.end() );
					ith_ac.insert(ith_ac.end(), nj1_period.second.begin(), nj1_period.second.end() );
				} */
			}

			
		}

		// Add approx tput of this chain.
		std::vector<double> app_per_ap = std::vector<double>(total_vars, 0.0);
		app_per_ap[frac_sc_var_ct+i] = 1.0;
		print_dvec(app_per_ap, "ADDING approx period for chain");

		ith_ap.push_back(app_per_ap);
		ith_ac.push_back(0.0);

		double cons = std::get<0>(all_chains[i]);
		
		// Add to the obj_ arrs, in ratio of constr.
		obj_ap.insert(obj_ap.end(), ith_ap.begin(), ith_ap.end() );
		std::vector<double> ith_ac_copy (ith_ac.begin(), ith_ac.end());
		
		for (int ic = 0; ic < ith_ac.size(); ic++)
			ith_ac_copy[ic] += log( all_chains_rel_weights[i] ); // NC nodes' RT*relative_constr in obj.
		
		obj_ac.insert(obj_ac.end(), ith_ac_copy.begin(), ith_ac_copy.end());

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

	// add 0.5*pi for streaming node[SC]s 
	for (int i = 0; i < exec_order.size(); i++)
		if (is_sc_streaming(exec_order[i]))
		{
			std::vector<double> p_sci (total_vars, 0.0);
			p_sci[frac_sc_c_var_ct + i] = 1;
			double strc = log(0.00000001); 
				strc = log(0.5);
			obj_ap.push_back(p_sci);
			obj_ac.push_back(strc);
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
	mosek_model->writeTask("/home/ubuntu/DagMCFractionalSolver.gp");

	std::vector<float> all_frac_vals = std::vector<float> (total_vars, 0.0);
	try
	{
		mosek_model->setLogHandler([](const std::string & msg) { std::cout << msg << std::flush; } );
		mosek_model->solve();
		// mosek_model->acceptedSolutionStatus(AccSolutionStatus::Optimal);
		std::cout << "Solved! " << mosek_model->getAcceptedSolutionStatus();
		auto opt_ans = std::make_shared<ndarray<double, 1>>(shape(total_vars), [all_l_vars](ptrdiff_t i) { return exp((*(all_l_vars->level()))[i]); });
		std::cout << "Extracted opt ans!\n";

		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_end);
		std::cout << "DAGMC:: got solve_end, exec order sz:" << exec_order.size() << ",total vars: " << total_vars << ",sz of opt_ans: " << opt_ans->size() << ",sz of return arr:" << all_frac_vals.size() << std::endl;
		for (int i = 0; i < total_vars; i++)
		{
			std::cout << "i=" << i;
			printf("|var %i : %f", i, (*opt_ans)[i]);
		}
		for (int i = 0; i < num_cores; i++)
		{
			if (sc_id_frac_id.find( core_sc_list[i][0] ) != sc_id_frac_id.end() )
			{
				double max_frac = 0.0;
				for (int si = 0; si < core_sc_list[i].size(); si++ )
					if ( (*opt_ans)[ sc_id_frac_id[ core_sc_list[i][si] ] ] < 1.0 ) 
						max_frac = std::max( max_frac, (*opt_ans)[ sc_id_frac_id[ core_sc_list[i][si] ] ] );
				printf("MAX_FRAC val for core %i : %f [ignoring fi>1s] || ", i, max_frac);
				for (int si = 0; si < core_sc_list[i].size(); si++ )
				{
					auto& sid = core_sc_list[i][si];
					double f_norm = max_frac/( (*opt_ans)[ sc_id_frac_id[sid] ] );
					
					float rf_norm = (f_norm > 1.0) ? round(f_norm) : (f_norm) ;
					printf("SC ID %i, orig frac: %f, f_norm: %f, frac val (round): %f [can be<1 for S nodes] ", sid, (*opt_ans)[ sc_id_frac_id[sid] ], f_norm, rf_norm);
					all_frac_vals[sid] = rf_norm;
				}

			}
			else
				all_frac_vals[ core_sc_list[i][0] ] = 1.0;
		}
		/*
		for (int sci = 0; sci < exec_order.size(); sci++)
		{
			if ( (sc_id_frac_id.find(sci) != sc_id_frac_id.end()) && (sci>0) )
			{
				double fr = (*opt_ans)[ sc_id_frac_id[sci] ];
				double fr1 = (double)1.0/fr;
				int fr_r = (int) round(fr1);
				printf("SC ID %i HAS a frac var id %i! opt_ans : %f, 1/ans: %f round:  %i", sci, sc_id_frac_id[sci], fr, fr1, fr_r );
				all_frac_vals[sci] = fr_r;
			}
			else
				all_frac_vals[sci] = 1;
		}*/
		
		printf("cputIME TAKEN TO solve: %f", (solve_end.tv_sec + solve_end.tv_nsec*1e-9) - (solve_start.tv_sec + 1e-9*solve_start.tv_nsec));

		print_vec<float>(all_frac_vals, "final FRAC Answers from DAGMC");
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
				{	
					std::cout << "The solution status is unknown.\n";
					char symname[MSK_MAX_STR_LEN];
					char desc[MSK_MAX_STR_LEN];
					MSK_getcodedesc((MSKrescodee)(mosek_model->getSolverIntInfo("optimizeResponse")), symname, desc);
					std::cout << "  Termination code: " << symname << " " << desc << "\n";
					/* mosek_model->selectedSolution(SolutionType::Interior);
					auto opt_ans = std::make_shared<ndarray<double, 1>>(shape(total_vars), [all_l_vars](ptrdiff_t i) { return exp((*(all_l_vars->level()))[i]); });
					for (int i = 0; i < total_vars; i++)
						printf("var %i : %f", i, (*opt_ans)[i]); */
					break;

				}
			default:
				std::cout << "Another unexpected problem status: " << prosta << "\n";
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "DAG::compute_rt_solve: Unexpected error: " << e.what() << "\n";
	}
	
	mosek_model->dispose();
	return all_frac_vals;
}
