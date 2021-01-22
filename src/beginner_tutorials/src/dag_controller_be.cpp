#define _GNU_SOURCE
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
#include <fstream>
#include <utility>
#include <math.h>

#include <sys/types.h>
#include <signal.h>
#include <sched.h>
#include <pthread.h>
#include <linux/sched.h>

#include <dag_controller_be.h>

int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
	        return syscall(__NR_sched_setattr, pid, attr, flags);
}

double get_monotime_now()
{
	struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec + 1e-9*ts.tv_nsec);
}

double get_realtime_now()
{
	struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
	return (ts.tv_sec + 1e-9*ts.tv_nsec - 1605000000.0);
}

	void set_other_policy_pthr(pthread_t& pt)
	{
		struct sched_param params;
		params.sched_priority = 0;
		int ret = pthread_setschedparam(pt, SCHED_OTHER, &params);
		printf("Monotime %f, realtime %f, retval for changing dyn_reopt THREAD to OTHER: %i |", get_monotime_now(), get_realtime_now(), ret);
	}

	void set_other_policy(int tid, int nice_val)
	{
		struct sched_attr attr;
		attr.size = sizeof (attr);
		attr.sched_policy = SCHED_OTHER;
		attr.sched_nice = nice_val;
		
		int ret = sched_setattr(tid, &attr, 0);
		printf("Monotime %f, realtime %f, retval for changing dynamic_reoptimize_func THREAD to OTHER, nice: %i: %i | tid: %i", get_monotime_now(), get_realtime_now(), nice_val, ret, tid);
	}

DAGControllerBE::DAGControllerBE()
	{
	}

// todo:Later: maybe a lock for reset_count.
DAGControllerBE::DAGControllerBE(std::string dag_file, DAGControllerFE* fe, bool dyn_resolve, std::string use_td, std::string fifo, int f_mc, int f_mu, int f_nc, int f_np, int p_s, int p_lc, int p_lp, int num_c)
	{
		// Note that the last 4 inputs are just for the offline stage.
		node_dag = DAG(dag_file);
		trigger_log.open("TriggerLog.csv");
		cc_completion_log.open("CCCompletionLog.csv");
		dag_name = dag_file;

		timer_thread_running = false;
		timer_thread = NULL;

		dynamic_reoptimize = dyn_resolve;

		node_dag.fill_trigger_nodes();
                node_dag.assign_publishing_rates();
                node_dag.assign_src_rates();

		num_cores = num_c;
		// /*
		// TODO: When doing dynamic, use node_dag_mc as input to MCApproxSolver:
		multi_core_solver = MultiCoreApproxSolver(&node_dag, num_cores);
		std::vector< std::vector<int> > sc_core_assgt = multi_core_solver.solve();
		node_dag_mc = DAGMultiCore(dag_file);
		// we will call these 3 functions again after every core-assgt re-solve.
		printf("INITIALIZED node_dag_mc. ABOUT to call set_params \n");
		node_dag_mc.set_params(num_cores, sc_core_assgt);
		node_dag_mc.assign_fixed_periods();
		all_frac_values = node_dag_mc.compute_rt_solve();
		// */

		frac_var_count = node_dag.global_var_count;
		// Nov: Solving for fi's commented for offline stage:
		// all_frac_values = node_dag.compute_rt_solve();
		// period_map = node_dag.period_map;

		exec_order = node_dag.get_exec_order();

		// Nov: Assigning fractions for offline stage
		offline_use_td = (use_td.find("yes") != std::string::npos);
		if (dag_name.find("nav") != std::string::npos)
		{
			offline_fracs["s"] = 1.0;
			offline_fracs["mapcb"] = 1.0/f_mc;
			offline_fracs["mapupd"] = 1.0/f_mu;
			offline_fracs["navc"] = 1.0/f_nc;
			offline_fracs["navp"] = 1.0/f_np;
		}
		else if (dag_name.find("illixr") != std::string::npos)
		{
			offline_fracs["8"] = 1.0;
			offline_fracs["9"] = 1.0/f_mu; // cam-slam : 3,4
			offline_fracs["5"] = 1.0/f_mc; // render : 7
		}

		/* This was needed only when we were not using optimal core assgt for expts:
		if (!offline_use_td)
		{
			if (use_td.find("2c") != std::string::npos )
			{
				offline_node_core_map["s"] = 0;
				offline_node_core_map["navc"] = 0;
				offline_node_core_map["navp"] = 0;

				offline_node_core_map["mapcb"] = 1;
				offline_node_core_map["mapupd"] = 1;
			}
			else if (use_td.find("3c") != std::string::npos)
			{
				offline_node_core_map["s"] = 0;

				offline_node_core_map["navc"] = 1;
				offline_node_core_map["navp"] = 1;

				offline_node_core_map["mapcb"] = 2;
				offline_node_core_map["mapupd"] = 2;
			}
			else
			{
				offline_node_core_map["navc"] = 0;
				offline_node_core_map["navp"] = 0;
				offline_node_core_map["mapupd"] = 0;
				offline_node_core_map["mapcb"] = 0;
				offline_node_core_map["s"] = 0;
			}
		} */

		if (fifo.find("yes") != std::string::npos)
		{
			std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", FIFO MODE!!!! Priorities: " << p_s << p_lc << p_lp << f_mc << f_mu << f_nc << f_np << std::endl;
			if (fifo.find("1c") != std::string::npos)
				fifo_nc = 1;
			else if (fifo.find("3c") != std::string::npos)
				fifo_nc = 3;
		
			if (dag_name.find("nav") != std::string::npos)
			{
				fifo_prio["s"] = p_s;
				fifo_prio["lc"] = p_lc;
				fifo_prio["lp"] = p_lp;
				fifo_prio["mapcb"] = f_mc;
				fifo_prio["mapupd"] = f_mu;
				fifo_prio["navc"] = f_nc;
				fifo_prio["navp"] = f_np;
			}
		}
		else
			fifo_nc = -1;

		printf("MonoTime: %f, RealTime: %f, fifo: %s, td: %s, f_mc: %i \n", get_monotime_now(), get_realtime_now(), fifo.c_str(), use_td.c_str(), f_mc);

		for (int i = 1; i < exec_order.size(); i++)
			reset_count.push_back(false);
	
		ready_sched = false;

		frontend = fe;
		
		// Initialize arrays for ci of all nodes:
		for (auto const& x : node_dag.id_name_map)
		{
			node_ci_arr[x.second] = boost::circular_buffer<double> (50);
			// node_extra_tids[x.second] = std::vector<int> ();
			last_trig_ts[x.second] = 0.0;
		}

		sched_started = false;
		shutdown_scheduler = false;
		// startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
	}

	void DAGControllerBE::start()
	{
		startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
	}

	DAGControllerBE::~DAGControllerBE()
	{
		shutdown_scheduler = true;
		if (sched_started)
			handle_sched_thread.join();
		else
			startup_thread->join();
		if (dynamic_reoptimize)
			reoptimize_thread.join();
	}

	void DAGControllerBE::startup_trigger_func()
	{
		set_high_priority("Startup trigger thread", 4, 0);
		double cc_period = 0.0; // in millisec.
		for (int i = 0; i < exec_order.size(); i++)
			cc_period += get_sum_ci_ith(exec_order[i])*offline_fracs[ node_dag.id_name_map[ exec_order[i][0] ] ];
		int oldstate;
		int ret_pct = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);
		printf("MonoTime: %f, RealTime: %f, STARTED startup_trigger thread TID: %li ! CC period: %f, sched_started: %i \n", get_monotime_now(), get_realtime_now(), ::gettid(), cc_period, sched_started.load() );

		curr_cc_period = cc_period;

		total_period_count = 0;
		per_core_period_counts[0] = 0;

		while ( (!sched_started) && (!shutdown_scheduler) )
		{
			long to = 1000*cc_period;
			std::this_thread::sleep_for( std::chrono::microseconds(to) );
			if (!sched_started)
			{
				total_period_count += 1;
				per_core_period_counts[0] += 1;
				printf("MonoTime: %f, RealTime: %f, Period ct: %li, checking Trigger for all, exec_order sz: %lu, curr_cc_period: %f", get_monotime_now(), get_realtime_now(), total_period_count, exec_order.size(), curr_cc_period);
				for (int i = 0; i < exec_order.size(); i++)
					checkTriggerExec(exec_order[i], 0); // just for startup, we keep triggering nodes at some rate.
			}
			
		}
	}

	// FE will call this for each msg from the CC.
	void DAGControllerBE::recv_critical_exec_end()
	{
                // total_period_count += 1;


		// Nov25: No need to do anything if we're using TD in offline version.
		if (!offline_use_td)
		{
			if (got_all_info())
			{
				if (!sched_started)
				{
					std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " $$$$$ DAGControllerBE:: STARTING SCHEDULING!!! \n";
					sched_started = true;
					
					// Lets kill the startup_thread!
					pthread_cancel(startup_thread->native_handle());
					pthread_join(startup_thread->native_handle(), NULL);
					startup_thread->detach();
					startup_thread = NULL;
					
					for (int i = 0; i < reset_count.size(); i++)
						reset_count[i] = true;
					
					// Moved to handle_Sched_main: total_period_count = 1;

					// Make the thread that handles scheduling.
					// Done: make multiple threads : 1 for each core with >1 sc, 1 for each alone subchain.
					for (int ic = 0; ic < num_cores; ic++)
						if (node_dag_mc.core_sc_list[ic].size() > 1)
							per_core_sched_threads[ic] = boost::thread(&DAGControllerBE::handle_sched_main, this, std::vector<int>(1,ic) );
					for (int is = 0; is < exec_order.size(); is++)
						if (node_dag_mc.sc_id_frac_id.find(is) == node_dag_mc.sc_id_frac_id.end() )
						{
							int is_c = node_dag_mc.sc_core_assgt[is][0];
							printf("Monotime %f, Realtime %f, SC id %i is ALONE!! Making a thread! Core list[0]: %i", get_monotime_now(), get_realtime_now(), is, is_c);
							per_core_sched_threads[is_c] = boost::thread(&DAGControllerBE::handle_sched_main, this, node_dag_mc.sc_core_assgt[is] );
						}
					std::this_thread::sleep_for (std::chrono::milliseconds(2));
				}
				else
				{
					// Notify the handle_sched_main : it waits for CC to end before moving on to NC nodes.	
					boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
					cc_end = true;
					ready_sched = true;
					// printf("ABOUT to NOTIFY the main thread, cc_End: TRUE! \n");
					cv_sched_thread.notify_all();
				}

				
			}
		}
			
	}

	void DAGControllerBE::handle_sched_main(std::vector<int> core_ids)
	{
		set_high_priority("MAIN sched Thread", 4, 0);
		for (auto &i : node_dag.name_id_map)
			printf("#EXTRA THREADS for node %s : %lu", i.first.c_str(), node_extra_tids[i.first].size() );
		
		per_core_period_counts[core_ids[0] ] = 1;
		total_period_count = 1;
		
		// Done: Make core-wise exec_order: [exec_list is the list of sc ids on a particular core.]
		// Done: Set all nodes + extras on this core & this thread to CPUSet core_id.
		std::vector<int>& core_exec_list = node_dag_mc.core_sc_list[ core_ids[0] ]; 
		std::vector<std::vector<int> > core_exec_order;
		for (int i = 0; i < core_exec_list.size(); i++)
		{
			std::vector<int> a (exec_order[ core_exec_list[i] ].begin(), exec_order[ core_exec_list[i] ].end());
			for (int na = 0; na < a.size(); na++)
			{
				std::string nname = node_dag_mc.id_name_map[ a[na] ];
				changeAffinityThread("thr for node "+nname , node_tid[ nname ], core_ids);

				if (node_extra_tids.find(nname) != node_extra_tids.end())
					for (auto nei: node_extra_tids[nname])
						changeAffinityThread("Extras thr for node "+nname , nei, core_ids);
			}

			core_exec_order.push_back( a );
		}
		changeAffinityThread("handle_Sched_main for core="+std::to_string(core_ids[0]), 0, core_ids );
		
		printf("STARTING MAinSched thread!! tid: %li Len of exec_order: %zu, curr_cc_period: %f, #cores: %zu, core0: %i \n", ::gettid(), core_exec_order.size(), curr_cc_period, core_ids.size(), core_ids[0] );
		
		while (!shutdown_scheduler)
		{
			for (int i = 0; ( (i < core_exec_order.size()) && (!shutdown_scheduler) ); i++)
			{
				// printf("Monotime %f, realtime %f, TIME to run subchain #%i, Timeout: %f \n", get_monotime_now(), get_realtime_now(), i,  );
				// prio(i) = 2, all others = 1.
				offline_fracs_mtx.lock();
				
				long i_to = 1000*get_timeout(core_exec_order[i], core_ids);
				changePriority(core_exec_order, i, core_ids[0]); // handles priority & trigger.
				auto trig_cc = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count();
				
				offline_fracs_mtx.unlock();

				ready_sched = false;

				// cv wait only if CC, i.e. sc id == 0 & only 1core.
				if ( (core_exec_list[i] == 0) && (core_ids.size() == 1) )
				{
					// printf("Monotime %f, realtime %f, TID %i WILL wait for CC-end. \n", get_monotime_now(), get_realtime_now(), ::gettid() );
					boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
					int ct = 0;

					while ( (!ready_sched) && (ct<12) && (!shutdown_scheduler) )
					{
						cv_sched_thread.wait_for(lock, boost::chrono::microseconds(i_to*5) );
						ct += 1;
					}
					if (ct > 4)
						printf("Monotime %f, realtime %f, Waited for CC's completion! ct %i ready_sched %i [if 0, CC hasnt ended!] \n", get_monotime_now(), get_realtime_now(), ct, (bool)(ready_sched.load()) );
					
		cc_completion_log << trig_cc << ", " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << "\n";
				}
				else
					std::this_thread::sleep_for( std::chrono::microseconds( i_to ) );
			
			}

			total_period_count += 1;
			per_core_period_counts[core_ids[0]] += 1;
			// inc prio of our dynamic_resolve thread for sometime.	
			// TODO: what is curr_cc_period in multi core case? Maybe only core0 runs re-solve thr, use its period.
			int one_in_k_per = ceil(2000/(float)(40*curr_cc_period));
			if (total_period_count%( one_in_k_per ) == 0 && (dynamic_reoptimize))
			{
				printf("WANNA run dyn_reopt-2ms now!! 1in?Per: %i \n", one_in_k_per);
				changePriority(core_exec_order, -1, core_ids[0]);
				set_high_priority("Dyn Reopt thread!", 2, reoptimize_thread_id);
				std::this_thread::sleep_for( std::chrono::milliseconds(2) );
				// change policy back to other. 
				set_other_policy_pthr(reoptimize_thread_p);
			}
			else
				// Just some slack time in each period. 5% of this core's period.
				std::this_thread::sleep_for( std::chrono::microseconds( (int) round(1000*0.05*curr_cc_period) ) );
		}
		printf("shutdown_scheduler is %i, handle_Sched_main is EXITING!!! \n", shutdown_scheduler.load());
	}

	void DAGControllerBE::update_per_core_periods()
	{
		// for each core, its period is sum ci*fi. 
	}

	std::string DAGControllerBE::get_last_node_cc_name()
	{
		int id = exec_order[0][exec_order[0].size()-1];
		return node_dag.id_name_map[id];
	}

	void DAGControllerBE::set_high_priority(std::string thread_name, int prio, int tid)
	{
		// set prio to prio, with FIFO.
		struct sched_param sp = { .sched_priority = prio,};
		int ret = sched_setscheduler(tid, SCHED_FIFO, &sp);
		if (ret != 0) 
			printf("MonoTime: %f, RealTime: %f, SETTING priority to %i, for thread: %s, retval: %i \n", get_monotime_now(), get_realtime_now(), prio, thread_name.c_str(), ret);
		// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " SETTING priority to " << prio << " retval: " << ret << thread_name << std::endl;
	 }

	
	// Changes subchain[ind] prio = 2, all others = 1.
	void DAGControllerBE::changePriority(std::vector<std::vector<int> >& iexec_order, int ind, int core_id)
	{
		for (int i = 0; i < iexec_order.size(); i++)
                {
                        int ret = 7;
			if (i == ind)
                                ret = changePrioritySubChain(iexec_order[i], 2);
                        else
                                ret = changePrioritySubChain(iexec_order[i], 1);
		}
		if (ind>=0)
			checkTriggerExec(iexec_order[ind], core_id);
	}

	int DAGControllerBE::changePrioritySubChain(std::vector<int>& sci, int prio)
	{
		bool ret = true;
		struct sched_param sp = { .sched_priority = prio,};
		// Nov: Change priorities in reverse order, so that if any later node in subchain is preempted,
		// it executes first. [since it'll be ahead in the list of same prio.]
		for (int j = sci.size()-1 ; j >= 0; j-- )
		{
			if (node_tid.find( node_dag.id_name_map[sci[j]] ) != node_tid.end())
			{
				if ( (node_curr_prio.find( node_dag.id_name_map[sci[j]] ) == node_curr_prio.end()) || (node_curr_prio[ node_dag.id_name_map[sci[j]] ] != prio) )
				{
					// printf("CHANGING prio of node %s & its extras to %i", node_dag.id_name_map[sci[j]].c_str(), prio);
					std::set<int>& n_extras = node_extra_tids[ node_dag.id_name_map[sci[j]] ];
					for (auto ne = n_extras.begin(); ne != n_extras.end(); ne++)
					{
						int rete = changePriorityThread( node_dag.id_name_map[sci[j]]+"_extra", *ne, prio );
					}
					
					int n_tid = node_tid[node_dag.id_name_map[sci[j]] ];
					int ret = ( changePriorityThread( node_dag.id_name_map[sci[j]], n_tid, prio ) );
					if (ret == 0)
						node_curr_prio[ node_dag.id_name_map[sci[j]] ] = prio;
					
				}
			}
			else
				printf("ERROR!!! Monotime %f, realtime %f, No threadID for %s !!! \n", get_monotime_now(), get_realtime_now(), node_dag.id_name_map[sci[j]].c_str() );
		}
		return ret;
	}

	int DAGControllerBE::changeAffinityThread(std::string nname, int tid, std::vector<int> cores)
	{
		cpu_set_t set;
		CPU_ZERO(&set);
		for (int i = 0; i < cores.size(); i++)
			CPU_SET(cores[i], &set);
		int reta = sched_setaffinity( tid, sizeof(set), &set );
		printf("Monotime %f, realtime %f, SET %s tid: %i CPUSET to :, retval: %i", get_monotime_now(), get_realtime_now(), nname.c_str(), tid, reta);
		node_dag.print_vec(cores, "");
		return reta;
	}

	bool DAGControllerBE::changePriorityThread(std::string nname, int tid, int prio)
	{
		if (tid == 0)
		{
			printf("ERROR!!! Monotime %f, realtime %f, Node %s tid=0!!! \n", get_monotime_now(), get_realtime_now(), nname.c_str() );
			return 1;
		}
		struct sched_param sp = { .sched_priority = prio,};
		int ret = sched_setscheduler(tid, SCHED_FIFO, &sp);
		if (ret != 0)
			std::cerr << "WEIRD-" << nname << " Changing prio to " << prio << ", tid: " << tid << ", RETVAL: " << ret << std::endl;
		return (ret); // ret=0 : successful.
	}

	// checks if all nodes' tid/pid info has been received.
	bool DAGControllerBE::got_all_info()
	{
		if (dag_name.find("nav") != std::string::npos )
			return (node_tid.find("navc") != node_tid.end());
		else if (dag_name.find("ill") != std::string::npos)
		{
			bool ans = true;
			std::vector<std::string> node_ids {"6", "9", "3", "2", "5", "8"};
			for (int i = 0; i < node_ids.size(); i++)
				ans = ans && ( node_tid.find(node_ids[i]) != node_tid.end() );
			return ans;
		}
	}

	// Called by FE when tid/pid info is received for any node.
	void DAGControllerBE::recv_node_info(std::string node_name, int tid, int pid)
	{
		if (node_name.find("extra") != std::string::npos)
		{
			// add it to list of extra tids for node.
			std::string nname = node_name.substr( 0, node_name.find("_") );
			node_extra_tids[nname].insert(tid);
			printf("Monotime %f, realtime %f, RECEIVED EXTRA TID for %s, nodename: %s, tid: %i, NEW setsize: %i", get_monotime_now(), get_realtime_now(), node_name.c_str(), nname.c_str(), tid, node_extra_tids[nname].size());
			return;
		}
		node_tid[node_name] = tid;
		node_pid[node_name] = pid;
		// todo: do we need to inc priority of CC here?
		// not if it starts with p>=2 already : True for ROS.

		if (offline_use_td && got_all_info() )
		{
			if (fifo_nc == -1)
			{
				for (int i = 0; i < exec_order.size(); i++)
				{
					// Assign all nodes in ith set to core i.
					for (int j = 0; j < exec_order[i].size(); j++)
					{
						changeAffinityThread( node_dag.id_name_map[exec_order[i][j]], node_tid[node_dag.id_name_map[exec_order[i][j]] ], std::vector<int>(1,i) );
						// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning core " << i << " to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << reta << ", cpucount: " << CPU_COUNT(&set) << std::endl;
						struct sched_param sp = { .sched_priority = 1,};
						int retp = sched_setscheduler( node_tid[node_dag.id_name_map[exec_order[i][j]] ] , SCHED_FIFO, &sp );
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning FIFO,prio=1 to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << retp << std::endl;
					}	
				}
			}
			else if (fifo_nc == 1)
			{
				
				for (auto const& x: node_dag.id_name_map)
				{
					// Assign cpu0 to all
					int reta = changeAffinityThread( x.second, node_tid[ x.second ], std::vector<int>(1,0) );
					struct sched_param sp = { .sched_priority = fifo_prio[x.second],};
					int retp = sched_setscheduler( node_tid[x.second], SCHED_FIFO, &sp );
					std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning core0 and prio" << fifo_prio[x.second] << " to " << x.second << ", tid:" << node_tid[x.second] << ", retvals: " << reta << " " << retp << std::endl;
				}
			}

			
		}
		else if (!offline_use_td && got_all_info() )
		{
			for (int i = 0; i < exec_order.size(); i++)
			{
				// if 0th node in ith subchain is in the offline_node_core_map
				if ( offline_node_core_map.find( node_dag.id_name_map[ exec_order[i][0] ] ) != offline_node_core_map.end() )
				{
					for (int j = 0; j < exec_order[i].size(); j++)
					{
						int reta = changeAffinityThread( node_dag.id_name_map[exec_order[i][j]], node_tid[node_dag.id_name_map[exec_order[i][j]] ], std::vector<int>(1,offline_node_core_map[ node_dag.id_name_map[ exec_order[i][0] ] ]) );
						
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning core " << offline_node_core_map[ node_dag.id_name_map[ exec_order[i][0] ] ] << " to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << reta  << std::endl;
						
						struct sched_param sp = { .sched_priority = 1,};
						int retp = sched_setscheduler( node_tid[node_dag.id_name_map[exec_order[i][j]] ] , SCHED_FIFO, &sp );
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning FIFO,prio=1 to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << retp << std::endl;

					}
					
				}
			}
			if (dynamic_reoptimize)
				reoptimize_thread = boost::thread(&DAGControllerBE::dynamic_reoptimize_func, this);
		}
	}

	
	void DAGControllerBE::dynamic_reoptimize_func()
	{
		std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " STARTING Dynamic Reoptimize thread!" << std::endl;
		reoptimize_thread_id = ::gettid();
		// set_other_policy(reoptimize_thread_id, 1);
		// set_high_priority("Dynamic Reoptimize thread!", 1, 0);
		pthread_t this_thread = pthread_self();
		reoptimize_thread_p = this_thread;
		struct sched_param params;
		params.sched_priority = 0;
		set_other_policy_pthr(reoptimize_thread_p); // pthread_setschedparam(this_thread, SCHED_OTHER, &params);
		
		printf("Monotime %f, realtime %f, dynamic_reoptimize_func THREAD pthread tid: %i, tid: %i \n", get_monotime_now(), get_realtime_now(), reoptimize_thread_id, ::gettid() );

		int sleep_time = 5000;

		while (!shutdown_scheduler)
		{
			std::this_thread::sleep_for (std::chrono::milliseconds(sleep_time));
			printf("Reoptimize thread WOKEN UP [Slept for %i]!! Starting to re-solve... \n", sleep_time);

			// Not needed for dag_mc: clear old data
			// node_dag.clear_old_data(frac_var_count);
			// Done: Update node_dag_mc ci, re-solve using that and update offline_fracs. 
			// TODO: Re-solve for core assgt every 7.5s or 10s.
			
			try
			{
			// update compute times to be 75ile of recent data.
			node_dag_mc.update_cis(node_ci_arr);
			printf("DONE Updating all cis! Will start solving now: ");
				double solve_start = get_monotime_now();
				node_dag_mc.assign_fixed_periods();
				all_frac_values = node_dag_mc.compute_rt_solve();
				printf("Monotime %f, realtime %f, Mono-time taken to solve GP: %f \n", get_monotime_now(), get_realtime_now(), (get_monotime_now()-solve_start) );

				if (all_frac_values[0] > 0)
				{
					// [node_dag:] get period map node_id -> set of f_ids.
					// period_map = node_dag.period_map; 
					// For each NC subchain, fraction = 1.0/[multiply all_frac_vals[i] for all i in period_map[subchain]]
					double period = 0.0;

					// take a lock to change offline_fracs: [i.e. the handle_Sched thread might have to wait a little bit sometimes before getting timeout value.] 
					offline_fracs_mtx.lock();
					/*
					for (auto const& x : period_map)
					{
						double frac = 1.0;
						for (int i = 0; i < x.second.size(); i++)
							frac *= 1.0/all_frac_values[x.second[i]];
						printf("MonoTime: %f, RealTime: %f, In new soln, Fraction for node %s is %f \n", get_monotime_now(), get_realtime_now(), node_dag.id_name_map[x.first].c_str(), frac);
						period += node_dag.id_node_map[x.first].compute * frac;
						offline_fracs [ node_dag.id_name_map[x.first] ] = frac;
					}
					*/
					// With new dag_mc, all_frac_vals directly gives frac of each subchain.
					for (int i = 0; i < exec_order.size(); i++)
					{
						printf("MonoTime: %f, RealTime: %f, In new soln, Fraction for node %s is %i \n", get_monotime_now(), get_realtime_now(), node_dag_mc.id_name_map[ exec_order[i][0] ].c_str(), all_frac_values[i]);
						offline_fracs[ node_dag_mc.id_name_map[ exec_order[i][0] ] ] = 1.0/all_frac_values[i];
						
						for (int ei = 0; ei < exec_order[i].size(); ei++)
							period += node_dag_mc.id_node_map[exec_order[i][ei]].compute / (double)all_frac_values[i];
					}
					curr_cc_period = period;
					offline_fracs_mtx.unlock();

					printf("MonoTime: %f, RealTime: %f, In new soln, CC period is %f \n", get_monotime_now() , get_realtime_now(), period);
					sleep_time = 5000;
				
					// For DFracV2 expts, where we re-solve only once:
					// dynamic_reoptimize = false;
					// return;
				}
				else
				{
					// node_dag.scale_nc_constraints(1.5);
					node_dag_mc.scale_nc_constraints(1.5);
					sleep_time = 1000;
				}
			}
			catch (const std::exception& e)
			{
				printf("MonoTime: %f, RealTime: %f, EXCEPTION in trying to solve GP ", get_monotime_now() , get_realtime_now() );
				std::cout << e.what() << std::endl;
			}
			catch (int e)
			{
				printf("MonoTime: %f, RealTime: %f, EXCEPTION in trying to solve GP %i", get_monotime_now() , get_realtime_now(), e);
			}
		}
		printf("MonoTime: %f, RealTime: %f, OUT OF THE reoptimize LOOP!!! shutdown_scheduler: %i \n", get_monotime_now() , get_realtime_now(), shutdown_scheduler.load());
	
	}

	void DAGControllerBE::update_ci(std::string node_name, double ci)
	{
		// Done: Add some extra time based on #Extra threads.
		double e_ci = 0.001*0.5*(node_extra_tids[node_name].size()); // in seconds.
		// printf("Updating ci for node %s, adding %f to ci, #extra tids: %i", node_name, e_ci,  node_extra_tids[node_name].size() );
		node_ci_arr[node_name].push_back(ci + e_ci );
	}

	// Returns sum ci of subchain ind, for current ci vals.
	double DAGControllerBE::get_sum_ci_ith(std::vector<int>& sci)
	{
		double tot = 0.0;
                for (int i = 0; i < sci.size(); i++)
                        tot += node_dag_mc.id_node_map[ sci[i] ].compute;
                return tot;
	}

	double DAGControllerBE::get_max_ci_ith(std::vector<int>& sci)
	{
		float max = 0.0;
		for (int i = 0; i < sci.size(); i++)
			max = std::max(max, node_dag.id_node_map[ sci[i] ].compute);
		return max;
	}

	// returns the timeout for ind subchain, for current ci,fi vals. This is in millisec.
	double DAGControllerBE::get_timeout(std::vector<int>& sci, std::vector<int>& cores)
	{
		// TODO: If this subchain is alone on >=1 cores, use max (sum ci / k, max ci). Also acct for multi threading.
		double tot = get_sum_ci_ith(sci);
		double max = get_max_ci_ith(sci);
		// Assuming that f is set to 1 for an alone subchain.
		tot *= offline_fracs[node_dag_mc.id_name_map[ sci[0] ] ];
	
		double ans = tot;
		if (cores.size()>1)	
			ans = std::max( max, ( tot/cores.size() ) );
		return ans;
	}

	// Func to just sleep for timeout millisec and then notify cv.
	void DAGControllerBE::timer_thread_func(double timeout)
	{
		set_high_priority("Timer thread", 4, 0);
		// set canceltype to asynchronous.
		int oldstate;
		int ret_pct = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);

		// printf("MonoTime: %f, RealTime: %f, making TIMER for to %f \n", get_monotime_now(), get_realtime_now(), timeout);
		// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " | making TIMER for to " << timeout << std::endl;
		long to = timeout*1000;
		timer_thread_running = true;
		std::this_thread::sleep_for (std::chrono::microseconds(to));
		boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
		// cc_end = false;
		ready_sched = true;
		printf("MonoTime %f, RealTime %f ABOUT to NOTIFY the main thread, cc_end is false! [Had timer for %f] [ret val for settinf canceltype %i, oldstate %i] \n", get_monotime_now(), get_realtime_now(), timeout, ret_pct, oldstate);
		cv_sched_thread.notify_all();
		timer_thread_running = false;
	}

	// checks if SC ind needs to be triggered, if yes, calls the FE func.
	// Done: use per core period_counts for this!!
	void DAGControllerBE::checkTriggerExec(std::vector<int>& sci, int core_id)
	{
		int id = sci[0];
                int ind_p = round(1.0/offline_fracs[node_dag.id_name_map[id] ]);
		std::string name = node_dag.id_name_map[id];
		if ( (ind_p == 1 || ( (per_core_period_counts[core_id]) %ind_p == 1) ) ) // && (node_tid.find(name) != node_tid.end() ) ) // (ind > 0) && : Removing cuz CC now triggered by scheduler. 
		{
			printf("MonoTime: %f, RealTime: %f, trigger node: %s, frac: %i, period_count: %li \n", get_monotime_now(), get_realtime_now(), name.c_str(), ind_p, per_core_period_counts[core_id]);
		
                	// we always want the node to run exactly once whenever it wakes up, and hence only a bool is needed.	
			frontend->trigger_node(name, true);
			// For Illixr Dag, need to trigger TW along with Imu, cuz Imu is at fixed freq for now. 
			if ( (name.find("8") != std::string::npos) && (dag_name.find("ill") != std::string::npos) )
				frontend->trigger_node("6", true);

			if (last_trig_ts[name] > 0)
				trigger_log << name << ", " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << "\n";
			last_trig_ts[name] = get_realtime_now();

			
		}
	}
