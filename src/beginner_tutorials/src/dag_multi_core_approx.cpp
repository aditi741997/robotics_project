#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/condition_variable.hpp>
#include <bits/stdc++.h>
#include <map>

#include <sys/types.h>
#include <signal.h>
#include <sched.h>
#include <pthread.h>

#include <dag_multi_core_approx.h>

MultiCoreApproxSolver::MultiCoreApproxSolver()
{
}

MultiCoreApproxSolver::MultiCoreApproxSolver(DAG* ndag, int k)
{
	num_cores = k;
	node_dag = ndag;
}

// exec_order_id -> cores
std::vector< std::vector<int> > MultiCoreApproxSolver::solve()
{
	struct timespec full_start, solve_start, full_end;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &full_start);

	std::vector<std::vector<int> > list_subchains = node_dag->get_exec_order();
	int num_subchains = list_subchains.size();

	std::map<int, int> node_id_exec_order_id;
	for (int nsi = 0; nsi < num_subchains; nsi++)
	// if (node_id_exec_order_id.find(ith_chain[n]) == node_id_exec_order_id.end())
	{
		for (int eo = 0; eo < list_subchains[nsi].size(); eo++)
		{
			node_id_exec_order_id[ list_subchains[nsi][eo] ] = nsi;
			printf("\n Found node %i in exec_order at index %i", list_subchains[nsi][eo], node_id_exec_order_id[ list_subchains[nsi][eo] ]);
		}
			//if (find(list_subchains[eo].begin(), list_subchains[eo].end(), ith_chain[n]) != list_subchains[eo].end())
	}
	
	std::vector<std::vector<int>> core_assgt (num_subchains, std::vector<int>() );
	
	if (num_cores>1)
	{
		Model::t mosek_model = new Model("multi_core_solver");
		auto _M = finally([&]() { mosek_model->dispose(); });
		mosek_model->setSolverParam("numThreads", 1);
		mosek_model->setSolverParam("intpntMultiThread", "off");

		int large_n = 50000;

		// aij vars: boolean. ij: i*num_cores+j
		Variable::t aij = mosek_model->variable("aij", (num_cores*num_subchains), Domain::binary());
		
		// setting a00 = 1.
		auto a00_sol = new_array_ptr<double,1>({1});
		aij->slice(0,1)->setLevel(a00_sol);

		printf("STARTING adding constraints, num_subchains: %i, num_cores: %i \n", num_subchains, num_cores);

		float eps = 0.01;

		// sum aij for all subchains is atleast 1.
		for (int i = 0; i < num_subchains; i++)
		{
			std::vector<double> aij_c1 (num_cores*num_subchains, 0);
			for (int j = 0; j < num_cores; j++)
				aij_c1[i*num_cores + j] = 1;
			node_dag->print_dvec(aij_c1, "\n Constraint on aij : sum aij>0.99 for i="+std::to_string(i));

			auto ac1 = new_array_ptr<double> (aij_c1);
			mosek_model->constraint("aij_c1"+std::to_string(i), Expr::dot(ac1, aij), Domain::greaterThan(1.0 - eps) );
		}

		// sum aij for all cores is atleast 1.
		for (int j = 0; j < num_cores; j++)
		{
			std::vector<double> aij_c2 (num_cores*num_subchains, 0);
			for (int i = 0; i < num_subchains; i++)
				aij_c2[i*num_cores + j] = 1;
			node_dag->print_dvec(aij_c2, "\n Constraint on aij : sum aij>0.99 for j="+std::to_string(j) );
			auto ac2 = new_array_ptr<double> (aij_c2);
			mosek_model->constraint("aij_c2"+std::to_string(j), Expr::dot(ac2, aij), Domain::greaterThan(0.99) );
		}

		// this denotes if subchain i has more than 1cores.
		Variable::t xi = mosek_model->variable("xi", num_subchains, Domain::binary());

		for (int i = 0; i < num_subchains; i++)
		{
			std::vector<double> aij_c1 (num_cores*num_subchains, 0);
			for (int j = 0; j < num_cores; j++)
				aij_c1[i*num_cores + j] = 1;
			node_dag->print_dvec(aij_c1, "\n Constraint on xi, aij for i= "+std::to_string(i));
			auto ac1 = new_array_ptr<double> (aij_c1);
			
			std::vector<double> xi_i (num_subchains, 0);
			xi_i[i] = -1.0*large_n;
			node_dag->print_dvec(xi_i, "\n Constraint on xi,i= "+std::to_string(i) );
			auto xi_ic = new_array_ptr<double> (xi_i);

			printf("\n Constr1: aij-Mxi > 1-M AND Constr2: aij-Mxi < 1");
			mosek_model->constraint("aij_xi_1"+std::to_string(i), Expr::add( Expr::dot(ac1, aij), Expr::dot(xi_ic, xi) ), Domain::greaterThan(1.01 - large_n) );

			mosek_model->constraint("aij_xi_2"+std::to_string(i), Expr::add( Expr::dot(ac1, aij), Expr::dot(xi_ic, xi) ), Domain::lessThan(1.01) );
		}

		// yij = aij && xi, i.e. whether subchain i is on core j and is on >1 cores.
		Variable::t yij = mosek_model->variable("yij", num_cores*num_subchains, Domain::binary());
		for (int i = 0; i < num_subchains; i++)
		{
			for (int j = 0; j < num_cores; j++)
			{
				std::string ij_str = std::to_string(i)+"_"+std::to_string(j);
				
				std::vector<double> aij_c1 (num_cores*num_subchains, 0);
				aij_c1[i*num_cores + j] = -1;
				node_dag->print_dvec(aij_c1, "\n ADDING constraints on yij, aij for i,j:"+ij_str);
				auto ac1 = new_array_ptr<double> (aij_c1);

				std::vector<double> xi_i (num_subchains, 0);
				xi_i[i] = -1.0;
				node_dag->print_dvec(xi_i, "\n ADDING constraints on yij, xi for i,j:"+ij_str);
				auto xi_ic = new_array_ptr<double> (xi_i);

				std::vector<double> yij_c (num_cores*num_subchains, 0);
				yij_c[i*num_cores + j] = 1;
				node_dag->print_dvec(yij_c, "\n ADDING constraints on yij, yij for "+ij_str);
				auto yij_cc = new_array_ptr<double>(yij_c);

				printf("\n For ij: %s, C1: yij-aij <= 0 AND C2: yij-xi <= 0 AND C3: yij-xi-aij >= -1 \n", ij_str.c_str());
				mosek_model->constraint("yij_c1_"+ij_str, Expr::add( Expr::dot(ac1, aij), Expr::dot(yij, yij_cc) ), Domain::lessThan(0.0+0.01) );

				mosek_model->constraint("yij_c2_"+ij_str, Expr::add( Expr::dot(xi_ic, xi), Expr::dot(yij, yij_cc) ), Domain::lessThan(0.0+0.01) );
			
				mosek_model->constraint("yij_c3_"+ij_str, Expr::add( Expr::add( Expr::dot(xi_ic, xi), Expr::dot(yij, yij_cc) ) , Expr::dot(ac1, aij)), Domain::greaterThan(-1.0-0.01) );
			
				// constraint that if yij=1 then there should be only one subchain on core j.
				std::vector<double> aij_c2 (num_cores*num_subchains, 0);
				for (int l = 0; l < num_subchains; l++)
					aij_c2[l*num_cores+j] = 1;
				node_dag->print_dvec(aij_c2, "Constr on yij_aij, aij for ij"+ij_str);
				auto ac2 = new_array_ptr<double> (aij_c2);

				std::vector<double> yij_c2 (num_cores*num_subchains, 0);
				yij_c2[i*num_cores + j] = 1.0*large_n; // -1.0*large_n;
				auto yij_cc2 = new_array_ptr<double> (yij_c2);
				node_dag->print_dvec(yij_c2, "ConstrC1 on yij_aij, for ij"+ij_str);
			
				std::vector<double> yij_c22 (num_cores*num_subchains, 0);
				yij_c22[i*num_cores + j] = -1.0*large_n;
				auto yij_cc22 = new_array_ptr<double> (yij_c22);
				node_dag->print_dvec(yij_c22, "ConstrC2 on yij_aij, for ij"+ij_str);

				// printf("\n ADDING constr C1. aij-Myij < 1+M AND C2. aij-yij > 1-M");
				printf("\n MODIFIED constr C1. aij < 1+M(1-yij) AND same C2. aij-yij > 1-M");
				mosek_model->constraint("aij_yij1_"+ij_str, Expr::add( Expr::dot(aij, ac2), Expr::dot(yij_cc2, yij) ), Domain::lessThan(1.01+large_n) );
				mosek_model->constraint("aij_yij2_"+ij_str, Expr::add( Expr::dot(aij, ac2), Expr::dot(yij_cc22, yij) ), Domain::greaterThan(0.99-large_n) );
			}
		}

		// period constraints for all subchains:
		Variable::t pi = mosek_model->variable("pi", num_subchains, Domain::greaterThan(0.0) );
		Variable::t execi = mosek_model->variable("execi", num_subchains, Domain::greaterThan(0.0) ); // represents exectime of a SC.
		Variable::t zij = mosek_model->variable("zij", (num_cores*num_subchains), Domain::greaterThan(0.0));
		Variable::t wijl = mosek_model->variable("Wijl", (num_cores*num_subchains*num_subchains), Domain::binary() );
		Variable::t pni = mosek_model->variable("pni", num_subchains, Domain::greaterThan(0.0) );

		// pi will represent streaming period for streaming nodes.
		// pni >= pi, minPer constr on pni. pi represents streaming so pi >= stream minper.
		// Use pni in lat/tput/rt. Add pi*small_wt for all streaming nodes in Obj.

		// Put min_period constraint from nodes:
		for (int i = 0; i < num_subchains; i++)
		{
			std::vector<double> pni_cm (num_subchains, 0.0);
			pni_cm[i] = 1.0;
			auto pni_cm1 = new_array_ptr<double> (pni_cm);

			std::vector<double> pi_cm (num_subchains, 0.0);
			pi_cm[i] = -1.0;
			auto pi_cm1 = new_array_ptr<double> (pi_cm);

			for (int j = 0; j < list_subchains[i].size(); j++)
			{
				double min_per = node_dag->id_node_map[ list_subchains[i][j] ].min_period;
				if (min_per > 0 )
				{
					printf("ADDING Constraint for period PNI of subchain %i, with min period %f of node %i \n", i, min_per, list_subchains[i][j] );
					// pni >= min_per
					mosek_model->constraint("pni_minper_"+std::to_string(j)+"_"+std::to_string(i), Expr::dot(pni, pni_cm1), Domain::greaterThan(min_per) );

				}

				double stream_minper = node_dag->id_node_map[ list_subchains[i][j] ].stream_minper;
				if (stream_minper > 0)
				{
					printf("ADDING Constraint for streaming period PI of subchain %i, with min period %f of node %i \n", i, stream_minper, list_subchains[i][j] );
					mosek_model->constraint("pi_minper"+std::to_string(i), Expr::dot(pi, pi_cm1) , Domain::lessThan(-1.0*stream_minper) );
				}
				
				std::vector<int> slower_thans = node_dag->id_node_map[ list_subchains[i][j] ].tput_slower_than;
				if (slower_thans.size() > 0)
				{
					for (auto &sti : slower_thans)
					{
						int scid = node_id_exec_order_id[sti];
						// pni [i] >= pni [scid]
						std::vector<double> starr (num_subchains, 0.0);
						starr[scid] = -1.0;
			                        auto starr1 = new_array_ptr<double> (starr);
						node_dag->print_dvec(starr, "Putting constraint that period of sc"+std::to_string(i)+"atleast per of "+std::to_string(scid));
						mosek_model->constraint("pni_"+std::to_string(i)+"atleast_"+std::to_string(scid), Expr::add(Expr::dot(pni, pni_cm1), Expr::dot( starr1, pni ) ), Domain::greaterThan(-0.01) );
					}
					// find which SC is this node in.
				}
			}
			// pni >= pi
			printf("ADDING Constraint that pni >= pi subchain %i \n", i);
			mosek_model->constraint("pni_pi_"+std::to_string(i), Expr::add(Expr::dot(pni, pni_cm1), Expr::dot(pi, pi_cm1)), Domain::greaterThan(-0.01) );
		}

		std::vector<double> subchain_sum_cis (num_subchains, 0.0);
		for (int i = 0; i < num_subchains; i++)
		{
			std::string i_str = std::to_string(i);
			double max_ci = 0.0;
			double sum_ci = 0.0;
			for (int j = 0; j < list_subchains[i].size(); j++)
			{
				double c = node_dag->id_node_map[ list_subchains[i][j] ].compute;
				max_ci = std::max(max_ci, c );
				sum_ci += c;
			}
			subchain_sum_cis[i] = sum_ci;
			
			printf("\n Subchain %i, maxci %f, sumci %f", i, max_ci, sum_ci);

		// pi >=  max ci of subchain
			std::vector<double> pi_cm (num_subchains, 0.0);
			pi_cm[i] = 1.0;
			// node_dag->print_dvec(pi_cm, "ADDING Constraint that period>max_ci, for subchain"+i_str);
			auto pi_cm1 = new_array_ptr<double> (pi_cm);

			mosek_model->constraint("pi_cm"+i_str, Expr::dot(pi, pi_cm1), Domain::greaterThan(max_ci-0.01) );

		// execi >= sum ci of subchain
			std::vector<double> execi_c1 (num_subchains, 0.0);
			execi_c1[i] = 1.0;
			auto execi_c11 = new_array_ptr<double> (execi_c1);
			mosek_model->constraint("execi_sumci_"+i_str, Expr::dot(execi_c11, execi), Domain::greaterThan(sum_ci-0.01) );

		// pi >= sum ci of subchain/ #cores assigned, define zij
			std::vector<double> zij_sum_c (num_cores*num_subchains, 0.0);
			for (int j = 0; j < num_cores; j++)
			{
				std::string ij_str = i_str + "_" + std::to_string(j);
				zij_sum_c[i*num_cores+j] = 1.0;
			// constraints on zij
				std::vector<double> zij_c (num_cores*num_subchains, 0.0);
				zij_c[i*num_cores+j] = -1.0;
				// node_dag->print_dvec(zij_c, "ADDING c1,c2,c3 constr on zij for"+ij_str);
				auto zij_c1 = new_array_ptr<double> (zij_c);

				std::vector<double> aij_c (num_cores*num_subchains, 0.0);
				aij_c[i*num_cores+j] = large_n;
				// node_dag->print_dvec(aij_c, "ADDING c1,c2,c3 constr on zij:aij for "+ij_str);
				auto aij_c1 = new_array_ptr<double> (aij_c);

				printf("\n ADDING C1. Maij-zij > 0.0, C2. pi-zij > 0.0, C3. pi+Maij-zij < M");
				mosek_model->constraint("zij_c1_"+ij_str, Expr::add( Expr::dot(zij, zij_c1), Expr::dot(aij_c1, aij) ), Domain::greaterThan(0.0) ); // -0.005
				mosek_model->constraint("zij_c2_"+ij_str, Expr::add( Expr::dot(pi, pi_cm1), Expr::dot(zij, zij_c1) ), Domain::greaterThan(0.0) ); // -0.005
				mosek_model->constraint("zij_c3_"+ij_str, Expr::add( Expr::dot(pi, pi_cm1), Expr::add( Expr::dot(zij, zij_c1), Expr::dot(aij_c1, aij) ) ), Domain::lessThan(large_n) ); // +0.005

			}
			auto zij_sum_c1 = new_array_ptr<double>(zij_sum_c);
			// node_dag->print_dvec(zij_sum_c, "\n ADDING constr that sum zij>sum_ci for i"+i_str);
			mosek_model->constraint("pi_cs_k"+i_str, Expr::dot(zij,zij_sum_c1 ), Domain::greaterThan(sum_ci-0.005) );

		// pi >= sum ci * [no of subchains sharing same core]
			std::vector<double> wijl_sum_c (num_cores*num_subchains*num_subchains, 0.0);
			for (int l = 0; l < num_subchains; l++)
			{
				for (int j = 0; j < num_cores; j++)
				{
					std::string ijl_str = i_str + "_" + std::to_string(j) + "_" + std::to_string(l);
					wijl_sum_c[ num_cores*num_subchains*i + num_cores*l + j ] = -1.0*sum_ci;

					std::vector<double> wijl_c1 (num_cores*num_subchains*num_subchains, 0.0);
					wijl_c1[ num_cores*num_subchains*i + num_cores*l + j ] = 1.0;
					// node_dag->print_dvec(wijl_c1, "\n ADDING constr on wijl, wijl vec for ijl:"+ijl_str);
					auto wij_c11 = new_array_ptr<double>(wijl_c1);
				
					std::vector<double> aij_c(num_cores*num_subchains, 0.0);
					aij_c[i*num_cores+j] = -1.0;
					// node_dag->print_dvec(aij_c, "\n ADDING constr on wijl, aij vec for ijl:"+ijl_str);
					auto aij_c1 = new_array_ptr<double>(aij_c);

					std::vector<double> alj_c(num_cores*num_subchains, 0.0);
					alj_c[l*num_cores+j] = -1.0;
					// node_dag->print_dvec(alj_c, "\n ADDING constr on wijl, alj vec for ijl:"+ijl_str);
					auto alj_c1 = new_array_ptr<double>(alj_c);

					printf("\n ADDED C1. wijl-aij < 0, C2. wijl-alj < 0, C3. wijl-aij-alj > -1");
					mosek_model->constraint("wijl_c1_"+ijl_str, Expr::add( Expr::dot(aij_c1, aij), Expr::dot(wij_c11, wijl) ), Domain::lessThan(0.0+0.01) );
					mosek_model->constraint("wijl_c2_"+ijl_str, Expr::add( Expr::dot(alj_c1, aij), Expr::dot(wij_c11, wijl)), Domain::lessThan(0.0+0.01) );
					mosek_model->constraint("wijl_c3_"+ijl_str, Expr::add( Expr::add( Expr::dot(alj_c1, aij), Expr::dot(wij_c11, wijl)), Expr::dot(aij_c1, aij) ), Domain::greaterThan(-1.0-0.01) );
				}
			}

			auto wijl_sum_c1 = new_array_ptr<double>(wijl_sum_c);
			
			std::vector<double> mxi_term (num_subchains, 0.0);
			mxi_term[i] = +1.0 * large_n;
			auto mxi_term1 = new_array_ptr<double>(mxi_term);
			
			printf("ADDING CONSTRAINT ON exec time of subchain: execi - pi + Mxi >= 0 \n");
			std::vector<double> neg_pi_term(num_subchains, 0.0);
			neg_pi_term[i] = -1.0;
			auto neg_pi_term1 = new_array_ptr<double>(neg_pi_term);
			mosek_model->constraint("execi_pi_"+i_str, Expr::add( Expr::add( Expr::dot(neg_pi_term1, pi), Expr::dot(execi, execi_c11) ), Expr::dot(mxi_term1, xi) ) , Domain::greaterThan(0.0-0.01) );

			// node_dag->print_dvec(wijl_sum_c, "\n ADDING constr pi - sum_ci*(sum wijl) > 0.0 for i"+i_str);
			node_dag->print_dvec(wijl_sum_c, "\n MODIFIED constr pi - sum_ci*(sum wijl) + M*xi > 0.0 for i"+i_str);
			mosek_model->constraint("pi_core_equal_share"+i_str, Expr::add( Expr::add(Expr::dot(wijl_sum_c1, wijl), Expr::dot(mxi_term1, xi) ), Expr::dot(pi, pi_cm1) ), Domain::greaterThan(0.0-0.01) );
		}

		// TODO: Handle multi threaded nodes with the perfect scaling assumption.
		// Make variables bi, i.e. #bins for sc i alone. 1<=bi<=sum aij.
		// Replace constraint Pi >= Mi BY : Pi * sum aij >= Mi * bi I.E. sum zij >= Mi*bi.
		// More constraints on bi : a. if xi = 0 then bi = 1 [1-Mxi <= bi <= 1+Mxi] 
		// b. We add this only if SC is not ||alizable [which is known at the time of adding constr here in code], then bi = sum aij.

		// Apr2021: Replacing pi by pni in all chain-level metrics [cuz pni is the actual period wrt chains [=pi for normal and >= pi for stream nodes]]
		// constraint on RT of all chains. [p0 + sum 2*pi + max(all pi)] = RT <= constr.
		int num_chains = node_dag->all_chains.size();
		Variable::t chains_pers =  mosek_model->variable("chains_pers", num_chains, Domain::greaterThan(0.0) );
		printf("\n MAKING chains_approx_period variable, sz: %i", num_chains);

		// FOR Objective func:
		std::vector<double> sum_rts_ps (num_subchains, 0.0); // multiply with pni
		std::vector<double> chain_pers_sum (num_chains, 0.0);
		std::vector<double> sum_rts_execs (num_subchains, 0.0);

		for (int ch = 0; ch < num_chains; ch++)
		{
			double cons = std::get<0>(node_dag->all_chains[ch]);
			double non_crit_rt_mult = node_dag->all_chains_rel_weights[ch];
			
			printf("\n For chain %i, constraint on RT: %f, weight: %f", ch, cons, non_crit_rt_mult);
			// ch0: sum_ci + pi
			/* if (ch == 0)
			{
				std::vector<double> p0 (num_subchains, 0.0);
				p0[0] = 1.0;
				node_dag->print_dvec(p0, "\n ADDing constr on p0 and obj=p0");
				auto p0_c = new_array_ptr<double>(p0);
				mosek_model->constraint("ch0_rt", Expr::dot(pi, p0_c), Domain::lessThan(cons - subchain_sum_cis[0]) );
				
				// Objective func: Add p0
				sum_rts_ps[0] += 1.0;
			} */
			// other chains: formula above.
			// else
			{
				std::vector<double> ith_chain_per (num_chains, 0.0);
				ith_chain_per[ch] = 1.0;
				node_dag->print_dvec(ith_chain_per, "\n Adding constr on ith chain period, vec for i="+std::to_string(ch) );
				auto ith_ch_per_c = new_array_ptr<double>(ith_chain_per);

				std::vector<int>& ith_chain = std::get<1>(node_dag->all_chains[ch]);
				std::vector<double> ps (num_subchains, 0.0); // all period terms in RT.
				std::vector<double> execs (num_subchains, 0.0); // all exectime terms in RT.

				for (int n = 0; n < ith_chain.size(); n++)
				{
					// need index of a node in the exec_order [cuz thats the order of pi variables.]
					//Note that node_id_exec_order_id is same as node_id_sc_id in node_dag_mc.
					
					int pid = node_id_exec_order_id[ith_chain[n]];
					if (n == 0)
						execs[pid] += 1.0;
					// note that if n, n-1 are same subchain, 
					// then we've already added P (max wait) +P (exec time for the whole subchain) when we were processing node n-1.
					else if (pid != node_id_exec_order_id[ith_chain[n-1]])
					{
						// add exec time (Period or sum ci if xi=1) of subchain and wait time (period) of subchain.
						ps[pid] += 1.0; 
						execs[pid] += 1.0;
					}

					std::vector<double> chain_per_const (num_subchains, 0.0);
					chain_per_const[pid] = -1.0;
					node_dag->print_dvec(chain_per_const, "\n Adding constr ith_chain_per > subchain_period PNI!!!");
					auto ch_per_tput_c = new_array_ptr<double>(chain_per_const);

					mosek_model->constraint("chain"+std::to_string(ch)+"_n_"+std::to_string( n )+"_cons", Expr::add( Expr::dot(ch_per_tput_c, pni), Expr::dot(ith_ch_per_c, chains_pers) ), Domain::greaterThan(0.0-0.01) );
				}

				node_dag->print_dvec(ps, "\n ADDING constr RT <= cons, RT me subchain_periods formula:");
				auto rt_ps = new_array_ptr<double> (ps);
				auto rt_execs = new_array_ptr<double> (execs);
				// RT : ps.pi + execs.execi + ith_ch_per_c.chains_pers <= cons
				if (node_dag->is_constraint)
				{
					mosek_model->constraint("chain"+std::to_string(ch)+"_rt_cons", Expr::add( Expr::add( Expr::dot(ith_ch_per_c, chains_pers), Expr::dot(execi, rt_execs) ) , Expr::dot(pni, rt_ps) ) , Domain::lessThan(cons) );
					printf("DAGMULTICORE: ADDING CONSTRAINT ON CHAIN RT!!! \n");
				}
				else
					printf("DAGMULTICORE: NOT ADDING CONSTRAINT ON CHAIN RT!!! NOT is_constraint \n");
				
				// For objective: [since ch>0 here]
				for (int i = 0; i < num_subchains; i++)
				{
					sum_rts_ps[i] += non_crit_rt_mult*ps[i];
					sum_rts_execs[i] += non_crit_rt_mult*execs[i];
				}
				node_dag->print_dvec(sum_rts_ps, "Sum RT Ps (subchain Per) after adding ch"+std::to_string(ch) );
				node_dag->print_dvec(sum_rts_execs, "SUM RT Execs (subchain exectime) after adding ch"+std::to_string(ch) );

				chain_pers_sum[ch] = non_crit_rt_mult;

			}
		}

		// Done: Objective is CC RT + 0.001*(sum of other three RTs)
		// rt_sum_ps DOT pi + chains approx_period : objective func.
		auto chain_pers_sum1 = new_array_ptr<double>(chain_pers_sum);

		node_dag->print_dvec(chain_pers_sum, "ADDING chains approx Per to ObjFunc!");

		auto sum_rts_ps1 = new_array_ptr<double> (sum_rts_ps);
		auto sum_rts_execs1 = new_array_ptr<double> (sum_rts_execs);

		// Adding streaming pi's term:
		std::vector<double> stream_pis (num_subchains, 0.0);
		for (int i = 0; i < num_subchains; i++)
		{
			bool is_stream_sc = false;
			for (int j = 0; j < list_subchains[i].size(); j++)
			{
				is_stream_sc = ( (is_stream_sc) || ( (node_dag->id_node_map[ list_subchains[i][j] ].node_type).find("S") != std::string::npos ) );
			}
			stream_pis[i] = (is_stream_sc) ? 0.5 : 0.0;
		}
		node_dag->print_dvec(stream_pis, "ADDING stream nodes' periods to ObjFunc!!");
		auto stream_pis1 = new_array_ptr<double> (stream_pis);
		
		mosek_model->objective("Objective", ObjectiveSense::Minimize, Expr::add( Expr::add(Expr::dot(sum_rts_ps1, pni), Expr::dot(stream_pis1, pi)), Expr::add(Expr::dot(chain_pers_sum1, chains_pers), Expr::dot(sum_rts_execs1, execi) ) ));
		mosek_model->writeTask("/home/ubuntu/DagMultiCoreApproxSolver.lp");
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_start);

		try
		{
			mosek_model->setLogHandler([](const std::string & msg) { std::cout << msg << std::flush; } );
			mosek_model->solve();
			
			clock_gettime(CLOCK_THREAD_CPUTIME_ID, &full_end);
			auto opt_aij = std::make_shared<ndarray<double, 1>>(shape(num_subchains*num_cores), [aij](ptrdiff_t i) { return ((*(aij->level()))[i]); });
			for (int i = 0; i < num_subchains; i++)
				for (int j = 0; j < num_cores; j++)
					printf("Value of a[%i][%i] : %f, ", i, j, (*opt_aij)[i*num_cores+j] );
			std::cout << std::endl;

			auto opt_xi = std::make_shared<ndarray<double, 1>>(shape(num_subchains), [xi](ptrdiff_t i) {return ((*(xi->level()))[i]); } );
			for (int i = 0; i < num_subchains; i++)
				printf("Value of xi[%i]: %f, ", i, (*opt_xi)[i]);
			std::cout << std::endl;
			
			auto opt_yij = std::make_shared<ndarray<double, 1>>(shape(num_subchains*num_cores), [yij](ptrdiff_t i) { return ((*(yij->level()))[i]); });
			for (int i = 0; i < num_subchains; i++)
				for (int j = 0; j < num_cores; j++)
					printf("Value of y[%i][%i] : %f, ", i, j, (*opt_yij)[i*num_cores+j] );
			std::cout << std::endl;
			
			auto opt_pi = std::make_shared<ndarray<double, 1>>(shape(num_subchains), [pi](ptrdiff_t i) {return ((*(pi->level()))[i]); } );
			for (int i = 0; i < num_subchains; i++)
				printf("Value of pi[%i]: %f, ", i, (*opt_pi)[i]);
			std::cout << std::endl;

			auto opt_pni = std::make_shared<ndarray<double, 1>>(shape(num_subchains), [pni](ptrdiff_t i) {return ((*(pni->level()))[i]); } );
			for (int i = 0; i < num_subchains; i++)
				printf("Value of pni[%i]: %f, ", i, (*opt_pni)[i]);
			std::cout << std::endl;


			auto opt_exi = std::make_shared<ndarray<double, 1>>(shape(num_subchains), [execi](ptrdiff_t i) {return ((*(execi->level()))[i]); } );
			for (int i = 0; i < num_subchains; i++)
				printf("Value of execi[%i]: %f, ", i, (*opt_exi)[i]);
			std::cout << std::endl;

			auto opt_zij = std::make_shared<ndarray<double, 1>>(shape(num_subchains*num_cores), [zij](ptrdiff_t i) { return ((*(zij->level()))[i]); });
			for (int i = 0; i < num_subchains; i++)
				for (int j = 0; j < num_cores; j++)
					printf("Value of z[%i][%i] : %f, ", i, j, (*opt_zij)[i*num_cores+j] );
			std::cout << std::endl;

			auto opt_wijl = std::make_shared<ndarray<double, 1>>(shape(num_subchains*num_cores*num_subchains), [wijl](ptrdiff_t i) { return ((*(wijl->level()))[i]); });
			for (int i = 0; i < num_subchains; i++)
				for (int l = 0; l < num_subchains; l++)
					for (int j = 0; j < num_cores; j++)
						printf("Value of w[%i][%i][%i] : %f, ", i, j, l, (*opt_wijl)[i*num_cores*num_subchains + l*num_cores + j] );
			std::cout << std::endl;

			std::cout << "Time Full: " << ( (full_end.tv_sec + full_end.tv_nsec*1e-9) - (full_start.tv_sec + 1e-9*full_start.tv_nsec) ) << std::endl;
			
			for (int i = 0; i < num_subchains; i++)
				for (int j = 0; j < num_cores; j++)
					if ( (*opt_aij)[i*num_cores+j] > 0.9 )
						core_assgt[i].push_back(j); // = j;
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
						/* This solution shouldnt be used cuz it can be garbage.
						mosek_model->selectedSolution(SolutionType::Interior);
						auto opt_ans = std::make_shared<ndarray<double, 1>>(shape(total_vars), [aij](ptrdiff_t i) { return exp((*(aij->level()))[i]); });
						for (int i = 0; i < num_subchains*num_cores; i++)
							printf("var %i : %f", i, (*opt_ans)[i]);
						*/
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

	}
	else
		for (int i = 0; i < num_subchains; i++)
			core_assgt[i].push_back(0);
	
	return core_assgt;
}
