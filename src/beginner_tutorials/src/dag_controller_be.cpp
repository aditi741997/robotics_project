#define _GNU_SOURCE
#include <sstream>
#include <fstream>
#include <numeric>
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

#define DEBUG2(x1, x2) std::cerr << __FILE__ << ':' << __LINE__ << ": " << #x1 << "=" << x1 << << ", " << #x2 << "=" << x2 << std::endl;
#define DEBUG(x) std::cerr << __FILE__ << ':' << __LINE__ << ": " << #x << "=" << x << std::endl;

#define TW_PLUGIN_NAME "6"
#define IMU_PLUGIN_NAME "7"
#define CAM_PLUGIN_NAME "8"
#define INT_PLUGIN_NAME "3"

const int scheduler_priority = 6;
#define RENDER_PLUGIN_NAME "5"

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
		printf("Mt %f, Rt %f, retval for changing dyn_reopt THREAD to OTHER: %i |", get_monotime_now(), get_realtime_now(), ret);
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
		// node_dag = DAG(dag_file);
		trigger_log.open("TriggerLog.csv");
		cc_completion_log.open("CCCompletionLog.csv");
		dag_name = dag_file;

		timer_thread_running = false;
		timer_thread = NULL;

		dynamic_reoptimize = dyn_resolve;

		// node_dag.fill_trigger_nodes();
                // node_dag.assign_publishing_rates();
                // node_dag.assign_src_rates();

		num_cores = num_c;
		// /*
		// Done: When doing dynamic, use node_dag_mc as input to MCApproxSolver:
		node_dag_mc = DAGMultiCore(dag_file);
		multi_core_solver = MultiCoreApproxSolver(&node_dag_mc, num_cores);
		std::vector< std::vector<int> > sc_core_assgt = multi_core_solver.solve();
		curr_sc_core_assgt = sc_core_assgt;
		last_mc_reopt_ts = get_monotime_now();

		// we will call these 3 functions again after every core-assgt re-solve.
		printf("INITIALIZED node_dag_mc. ABOUT to call set_params \n");
		node_dag_mc.set_params(num_cores, sc_core_assgt);
		node_dag_mc.assign_fixed_periods();
		all_frac_values = node_dag_mc.compute_rt_solve();
		// */

		frac_var_count = node_dag_mc.global_var_count;
		// Nov: Solving for fi's commented for offline stage:
		// all_frac_values = node_dag.compute_rt_solve();
		// period_map = node_dag.period_map;

		exec_order = node_dag_mc.get_exec_order();

		// Nov: Assigning fractions for offline stage
		offline_use_td = (use_td.find("yes") != std::string::npos);
		if (dag_name.find("nav") != std::string::npos)
		{
			offline_fracs["s"] = 1.0;
			offline_fracs["mapcb"] = 1.0/f_mc; //CHECK FOR QNO4 expts. 1.0/f_mc;
			offline_fracs["mapupd"] = 1.0/f_mu;
			offline_fracs["navc"] = 1.0/f_nc;
			offline_fracs["navp"] = 1.0/f_np;
				
			offline_fracs["ppcam"] = 1.0;
			/* node_max_skips["navc"] = 5;
			node_max_skips["navp"] = 3;
			node_max_skips["mapupd"] = 2; */
		}
		else if (dag_name.find("illixr") != std::string::npos)
		{
			offline_fracs[IMU_PLUGIN_NAME] = 1.0; // imu is start of cc, tw
			offline_fracs[CAM_PLUGIN_NAME] = 1.0/f_mu; // cam-slam
			offline_fracs[RENDER_PLUGIN_NAME] = 1.0/f_mc; // render
		}
		update_per_core_period_map(); // uses offline_fracs,nodeDagMC's sc_core_Assgt. 

		offline_fracs["fakenode"] = 1.0/1000.0; // assuming ci[fakenode] = 1000.0

		
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
		for (auto const& x : node_dag_mc.id_name_map)
		{
			node_ci_arr[x.second] = boost::circular_buffer<double> (50);
			node_ci_mode_arr[x.second] = boost::circular_buffer<int> (250); // store ci mode for past 250vals. for wtd sum. 
			node_ci2_arr[x.second] = boost::circular_buffer<double> (50);
			// node_extra_tids[x.second] = std::vector<int> ();
			last_trig_ts[x.second] = 0.0;
		
			node_finished[ x.first ] = false;		
			wait_for_logs[x.first].open("/home/ubuntu/n"+x.second+"_wf.csv");
		}

		sched_started = false;
		shutdown_scheduler = false;
		// startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &controller_start_ts);	
	
		// fakenode:
		fakenode_thread = boost::thread(&DAGControllerBE::fakenode_work, this);
	
		reoptimize_thread_id = 0; // initialize with 0
	}

	void DAGControllerBE::start()
	{
		startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
	}

	void DAGControllerBE::fakenode_work()
	{
		// save thread id : to be used for changing priorities.
		node_tid["fakenode"] = ::gettid();
		node_pid["fakenode"] = ::getpid();
		printf("Controller CREATED A FAKENODE thread, pid: %i, tid: %i", node_pid["fakenode"], node_tid["fakenode"]);
		while (!shutdown_fake)
		{
			thread_custom_sleep_for(10*1000);
		}
	}

	void DAGControllerBE::thread_custom_sleep_for(int microsec)
        {
               // sleep for upto 5ms each time to allow quick exit at shutdown.
                if ( (microsec > 10000) && (total_period_count%100 == 7) )
			printf("Mt: %f, RT: %f, custom_sleep_for was called!! with sleeptime: %i, #cores: %i, dyn_reopt: %i \n", get_monotime_now(), get_realtime_now(), microsec, num_cores, dynamic_reoptimize);
		if ( (num_cores > 1) && (dynamic_reoptimize) )
                {
                        int slept_for = 0;
                        while ( (slept_for < microsec) && (!shutdown_scheduler) )
                        {
                                int sleep_for = std::min(5000, (microsec-slept_for) );
                                std::this_thread::sleep_for( std::chrono::microseconds(sleep_for) );
                                slept_for += sleep_for;
                        }												
		}
                else
                    	 std::this_thread::sleep_for( std::chrono::microseconds(microsec) );

        }

	DAGControllerBE::~DAGControllerBE()
	{
		std::cout << "DAGControllerBE::~DAGControllerBE" << std::endl;
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
		pthread_setname_np(pthread_self(), "startup_trig");
		set_high_priority("Startup trigger thread", scheduler_priority, 0);

		double cc_period = 0.0; // in millisec.
		for (int i = 0; i < exec_order.size(); i++)
			cc_period += get_sum_ci_ith(exec_order[i])*offline_fracs[ node_dag_mc.id_name_map[ exec_order[i][0] ] ];
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
				// printf("MT: %f, RealTime: %f, Period ct: %li, checking Trigger for all, exec_order sz: %lu, curr_cc_period: %f", get_monotime_now(), get_realtime_now(), total_period_count, exec_order.size(), curr_cc_period);
				for (int i = 0; i < exec_order.size() && !sched_started; i++)
					checkTriggerExec(exec_order[i], 0, 1.0); // just for startup, we keep triggering nodes at some rate.
			}
			
		}
		printf("MonoTime: %f, RealTime: %f, SIGNING OFF startup_trigger thread!!! \n");
	}

	// FE will call this for each msg from the CC.
	void DAGControllerBE::recv_critical_exec_end()
	{
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
					// if (startup_thread->joinable())
					// {
					//	pthread_cancel(startup_thread->native_handle());
					//	pthread_join(startup_thread->native_handle(), NULL);
					//	startup_thread->detach();
					//	startup_thread = NULL;
					// }
					
					for (int i = 0; i < reset_count.size(); i++)
						reset_count[i] = true;
					
					// Make the threads that handles scheduling.
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
					notify_node_exec_end( node_dag_mc.id_name_map[ exec_order[0][ exec_order[0].size()-1 ] ] );
				}

				
			}
		}
			
	}

	void DAGControllerBE::notify_node_exec_end(std::string nname)
	{
		int nid = node_dag_mc.name_id_map[nname];
		boost::unique_lock<boost::mutex> lock(node_sched_thread_mutex[nid]);
                node_finished[nid] = true;
                node_cv_sched_thread[nid].notify_all();
	}

	void DAGControllerBE::handle_sched_main(std::vector<int> core_ids)
	{
		set_high_priority("MAIN sched Thread", 4, 0);
		std::string name = "hand_sched_" + std::to_string(core_ids.at(0));
		pthread_setname_np(pthread_self(), name.c_str());
		set_high_priority("MAIN sched Thread", scheduler_priority, 0);

		for (auto &i : node_dag_mc.name_id_map)
			printf("#EXTRA THREADS for node %s : %lu", i.first.c_str(), node_extra_tids[i.first].size() );
	
		if ( core_ids.size() == 0 )
		{
			printf("ERRORR!!! WEIRD!!!! Empty core list to sched_main thread!! \n");
			return;
		}
		per_core_period_counts[core_ids[0] ] = 1;
		per_core_sc_last_trigger_ct[ core_ids[0] ].clear();
		total_period_count = 1;
		
		// Done: Make core-wise exec_order: [exec_list is the list of sc ids on a particular core.]
		// Done: Set all nodes + extras on this core & this thread to CPUSet core_id.
		std::vector<int> core_exec_list = node_dag_mc.core_sc_list[ core_ids[0] ]; 
		std::vector<std::vector<int> > core_exec_order;
		std::map<int, int> core_node_exec_order_id;
		for (int i = 0; i < core_exec_list.size(); i++)
		{
			std::vector<int> a (exec_order[ core_exec_list[i] ].begin(), exec_order[ core_exec_list[i] ].end());
			for (int na = 0; na < a.size(); na++)
			{
				std::string nname = node_dag_mc.id_name_map[ a[na] ];
				changeAffinityThread("thr for node "+nname , node_tid[ nname ], core_ids);
				core_node_exec_order_id[ a[na] ] = i; // node na is in ith subchain of this core.

				if (node_extra_tids.find(nname) != node_extra_tids.end())
					for (auto nei: node_extra_tids[nname])
						changeAffinityThread("Extras thr for node "+nname , nei, core_ids);
			}

			core_exec_order.push_back( a );
			per_core_sc_last_trigger_ct[ core_ids[0] ][ a[0] ] = -1;
		}

		// static bool swap_order = std::getenv("ILLIXR_SWAP_ORDER") && (strcmp(std::getenv("ILLIXR_SWAP_ORDER"), "y") == 0);
		// if (swap_order) {
		// 	std::cout << "Swapping (added on 2021-05-01)" << std::endl;
		// 	auto tmp = core_exec_order[1];
		// 	core_exec_order[1] = core_exec_order[2];
		// 	core_exec_order[2] = tmp;
		// }

		changeAffinityThread("handle_Sched_main for core="+std::to_string(core_ids[0]), 0, core_ids );
	
		// if ( dag_name.find("illixr") != std::string::npos && SWAP) : flip core_exec_order 1,2.

		printf("STARTING MAinSched thread!! tid: %li Len of exec_order: %zu, curr_cc_period: %f, #cores: %zu, core0: %i \n", ::gettid(), core_exec_order.size(), per_core_period_map[ core_ids[0] ], core_ids.size(), core_ids[0] );
	
		double core_period = 1.0;
		std::vector<long> sc_to (core_exec_order.size() );
                std::vector<double> sc_fracs ( core_exec_order.size() ) ;
		std::vector<double> sc_trigger_fracs ( core_exec_order.size() ) ; // To enforce a lower bound on period, a node can be sent triggers at a frequency lower than what was assigned by the solver. It still gets resources equal to solver's O/P, but they can be used by other nodes as well.

		while (!shutdown_scheduler)
		{
			set_high_priority("MAIN sched Thread", 4, 0);
			offline_fracs_mtx.lock();
			// Using lock to protect offline_fracs, node_dag_mc. core_period = per_core_period_map[ core_ids[0] ];
			core_period = 0.0;
			for (int i = 0; i < core_exec_order.size(); i++)
			{
				sc_fracs[i] = offline_fracs[ node_dag_mc.id_name_map[ core_exec_order[i][0] ] ] ;
				sc_to[i] = (long) (1000*get_timeout(core_exec_order[i], core_ids) );
				core_period += (double)sc_to[i]/1000.0;
			}
			offline_fracs_mtx.unlock();
			
			for (int i = 0; i < core_exec_order.size(); i++)
			{
				double min_per = node_dag_mc.get_subchain_min_per(core_exec_order[i]);
				if (min_per > 0.001)
				{
					sc_trigger_fracs[i] = std::min(sc_fracs[i], (1.05*core_period)/min_per);
					// printf("min per for sc id %i is %f, core per: %f, curr frac: %f, new frac: %f, trigger_Fracs: %f", core_exec_order[i][0], min_per, core_period, sc_fracs[i], (core_period)/min_per, sc_trigger_fracs[i]);
				}
				else
					sc_trigger_fracs[i] = sc_fracs[i];
			}
			
			if (total_period_count % 100 == 1) {
				DEBUG(core_period);
				for (size_t i = 0; i < sc_fracs.size(); ++i) {
					DEBUG(get_monotime_now());
					DEBUG(i);
					DEBUG(sc_fracs[i]);
					DEBUG(sc_trigger_fracs[i]);
				}
			}

			for (int i = 0; ( (i < core_exec_order.size()) && (!shutdown_scheduler) ); i++)
			{
				// prio(i) = 2, all others = 1.
				long i_to = sc_to[i];
				// CHECK: For DynNoSC 
				checkTriggerExec( core_exec_order[i], core_ids[0], sc_trigger_fracs[i] ); 
				
				if (! checkWaitFor(core_exec_order, i, i_to, core_ids[0], core_node_exec_order_id, sc_fracs, core_period) )
				{
					changePriority(core_exec_order, i, core_ids[0]); // handles changing priority
					
					auto trig_cc = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count();

					ready_sched = false;

					int lastnode_id = core_exec_order[i][ core_exec_order[i].size()-1 ]; // last node of ith SC.
					std::string lastnodename = node_dag_mc.id_name_map[ lastnode_id ];

					// CHECK: For DynNoSC
					// cv wait only if CC, i.e. sc id == 0 & only 1core.
					if ( (core_exec_list.at(i) == 0) && (core_ids.size() == 1) )
					// if ( (core_ids.size() == 1) && ( (core_exec_list.at(i) == 0) || lastnodename.find(RENDER_PLUGIN_NAME) != std::string::npos ) )
					{
						boost::unique_lock<boost::mutex> lock(node_sched_thread_mutex[lastnode_id]);
						int ct = 0;

						while ( (!node_finished[ lastnode_id ]) && (ct<10) && (!shutdown_scheduler) )
						{
							node_cv_sched_thread[lastnode_id].wait_for(lock, boost::chrono::microseconds(i_to*2) );
							ct += 1;
						}
						if (ct > 5)
							printf("Monotime %f, realtime %f, Waited for CC's completion! ct %i node finished:  %i [if 0, CC hasnt ended!] \n", get_monotime_now(), get_realtime_now(), ct, (bool)(node_finished[ lastnode_id ]) );
						
			cc_completion_log << trig_cc << ", " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << "\n";
						// clear node_finished bool.
		                                node_finished[ lastnode_id ] = false;
					}
					else 
					{
						// std::this_thread::sleep_for( std::chrono::microseconds( i_to ) );
						thread_custom_sleep_for(i_to);
					}
				}
				// else
				// {
				// 	if (swap_order) {
				// 		thread_custom_sleep_for(i_to);
				// 	} else {
				// 		// sleep for remaining time in budget.
				// 		long render_time = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() - render_trigger_ts  );
				// 		static_assert(sizeof(long) >= 7);
				// 		thread_custom_sleep_for( illixr_noncrit_to_budget - render_time );
				// 		cc_completion_log << render_trigger_ts << ", " << render_time << ", " << lastnodename << ", " << illixr_noncrit_to_budget << "\n";
				// 	}
				// }
			
			}

			total_period_count += 1;
			per_core_period_counts[core_ids[0]] += 1;
			// inc prio of our dynamic_resolve thread for sometime.	
			int one_in_k_per = ceil(2000/(float)(20* core_period ));
			
			// CHECK: For DynNoSC
			if (per_core_period_counts[core_ids[0]] %( one_in_k_per ) == 0 && (reoptimize_thread_id != 0))
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
				thread_custom_sleep_for( (int) round(1000*0.05*core_period) );
			
			if (total_period_count%500 == 7)
			{
				struct timespec contr_endi;
				clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &contr_endi);
				printf("CPU Time used by scheduler: %f | ", (contr_endi.tv_sec + 1e-9*contr_endi.tv_nsec) - (controller_start_ts.tv_sec + 1e-9*controller_start_ts.tv_nsec) );
			}
		}
		printf("Monotime %f, realtime %f,  shutdown_scheduler is %i, handle_Sched_main is EXITING!!! core id0: %i \n", get_monotime_now(), get_realtime_now(), shutdown_scheduler.load(), core_ids[0]);
	}


	std::string DAGControllerBE::get_last_node_cc_name()
	{
		int id = exec_order[0][exec_order[0].size()-1];
		return node_dag_mc.id_name_map[id];
	}

	void DAGControllerBE::set_high_priority(std::string thread_name, int prio, int tid)
	{
		// set prio to prio, with FIFO.
		struct sched_param sp = { .sched_priority = prio,};
		int ret = sched_setscheduler(tid, SCHED_FIFO, &sp);
		if (ret != 0) 
			printf("MonoTime: %f, RealTime: %f, SETTING priority to %i, for thread: %s, retval: %i \n", get_monotime_now(), get_realtime_now(), prio, thread_name.c_str(), ret);
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
	}

	int DAGControllerBE::changePrioritySubChain(std::vector<int>& sci, int prio)
	{
		bool ret = true;
		struct sched_param sp = { .sched_priority = prio,};
		// Nov: Change priorities in reverse order, so that if any later node in subchain is preempted,
		// it executes first. [since it'll be ahead in the list of same prio.]
		for (int j = sci.size()-1 ; j >= 0; j-- )
		{
			if (node_tid.find( node_dag_mc.id_name_map[sci[j]] ) != node_tid.end())
			{
				if ( (node_curr_prio.find( node_dag_mc.id_name_map[sci[j]] ) == node_curr_prio.end()) || (node_curr_prio[ node_dag_mc.id_name_map[sci[j]] ] != prio) )
				{
					if (total_period_count % 500 == 11)
						printf("CHANGING prio of node %s & its extras to %i", node_dag_mc.id_name_map[sci[j]].c_str(), prio);
					std::set<int>& n_extras = node_extra_tids[ node_dag_mc.id_name_map[sci[j]] ];
					for (auto ne = n_extras.begin(); ne != n_extras.end(); ne++)
					{
						changePriorityThread( node_dag_mc.id_name_map[sci[j]]+"_extra", *ne, prio );
					}
					
					int n_tid = node_tid[node_dag_mc.id_name_map[sci[j]] ];
					int ret = ( changePriorityThread( node_dag_mc.id_name_map[sci[j]], n_tid, prio ) );
					if (ret == 0)
						node_curr_prio[ node_dag_mc.id_name_map[sci[j]] ] = prio;
					
				}
			}
			else
				printf("ERROR!!! Monotime %f, realtime %f, No threadID for %s !!! \n", get_monotime_now(), get_realtime_now(), node_dag_mc.id_name_map[sci[j]].c_str() );
		}
		return ret;
	}

	int DAGControllerBE::changeAffinityThread(std::string nname, int tid, std::vector<int> cores)
	{
		// return 0; // CHECK: For DynNoSC
		cpu_set_t set;
		CPU_ZERO(&set);
		for (int i = 0; i < cores.size(); i++)
			CPU_SET(cores[i], &set);
		int reta = sched_setaffinity( tid, sizeof(set), &set );
		printf("Monotime %f, realtime %f, SET %s tid: %i CPUSET to :, retval: %i", get_monotime_now(), get_realtime_now(), nname.c_str(), tid, reta);
		node_dag_mc.print_vec<int>(cores, "");
		return reta;
	}

	bool DAGControllerBE::changePriorityThread(std::string nname, int tid, int prio)
	{
		if (tid == 0)
		{
			printf("ERROR!!! Monotime %f, realtime %f, Node %s tid=0!!! \n", get_monotime_now(), get_realtime_now(), nname.c_str() );
			return 1;
		}
		if (nname.find(INT_PLUGIN_NAME) == std::string::npos && nname.find(IMU_PLUGIN_NAME) == std::string::npos )
		{
			struct sched_param sp = { .sched_priority = prio,};
			int ret = sched_setscheduler(tid, SCHED_FIFO, &sp);
			if (ret != 0)
				std::cout << "WEIRD-" << nname << " Changing prio to " << prio << ", tid: " << tid << ", RETVAL: " << ret << std::endl;
			return (ret); // ret=0 : successful.
		}
		else
			return 0;
	}

	// checks if all nodes' tid/pid info has been received.
	bool DAGControllerBE::got_all_info()
	{
		if (dag_name.find("nav") != std::string::npos )
			return (node_tid.find("navc") != node_tid.end() && ( (dag_name.find("yolo") != std::string::npos) ? node_tid.find("yolo") != node_tid.end() : true ) );
		else if (dag_name.find("ill") != std::string::npos)
		{
			bool ans = true;
			std::vector<std::string> node_ids {TW_PLUGIN_NAME, INT_PLUGIN_NAME, "2", RENDER_PLUGIN_NAME, CAM_PLUGIN_NAME, IMU_PLUGIN_NAME}; // wait for all 6 tids.
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
						changeAffinityThread( node_dag_mc.id_name_map[exec_order[i][j]], node_tid[node_dag_mc.id_name_map[exec_order[i][j]] ], std::vector<int>(1,i) );
						// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning core " << i << " to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << reta << ", cpucount: " << CPU_COUNT(&set) << std::endl;
						struct sched_param sp = { .sched_priority = 1,};
						int retp = sched_setscheduler( node_tid[node_dag_mc.id_name_map[exec_order[i][j]] ] , SCHED_FIFO, &sp );
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning FIFO,prio=1 to node " << node_dag_mc.id_name_map[exec_order[i][j]] << ", retval: " << retp << std::endl;
					}	
				}
			}
			else if (fifo_nc == 1)
			{
				
				for (auto const& x: node_dag_mc.id_name_map)
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
				if ( offline_node_core_map.find( node_dag_mc.id_name_map[ exec_order[i][0] ] ) != offline_node_core_map.end() )
				{
					for (int j = 0; j < exec_order[i].size(); j++)
					{
						int reta = changeAffinityThread( node_dag_mc.id_name_map[exec_order[i][j]], node_tid[node_dag_mc.id_name_map[exec_order[i][j]] ], std::vector<int>(1,offline_node_core_map[ node_dag_mc.id_name_map[ exec_order[i][0] ] ]) );
						
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning core " << offline_node_core_map[ node_dag_mc.id_name_map[ exec_order[i][0] ] ] << " to node " << node_dag_mc.id_name_map[exec_order[i][j]] << ", retval: " << reta  << std::endl;
						
						struct sched_param sp = { .sched_priority = 1,};
						int retp = sched_setscheduler( node_tid[node_dag_mc.id_name_map[exec_order[i][j]] ] , SCHED_FIFO, &sp );
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning FIFO,prio=1 to node " << node_dag_mc.id_name_map[exec_order[i][j]] << ", retval: " << retp << std::endl;

					}
					
				}
			}
			if (dynamic_reoptimize && (reoptimize_thread_id == 0) )
				reoptimize_thread = boost::thread(&DAGControllerBE::dynamic_reoptimize_func, this);
		}
	}

	// re-solves for core assgt and returns whether it was able to solve or not.
	bool DAGControllerBE::dynamic_reoptimize_mc()
	{
		printf("Monotime %f, Realtime %f : ABOUT TO re-solve for core assgt!! last_solve_ts: %f \n", get_monotime_now(), get_realtime_now(), last_mc_reopt_ts);
		// re-solve
		std::vector< std::vector<int> > sc_core_assgt = multi_core_solver.solve();
		
		if (sc_core_assgt[0].size() == 0)
			return true; // i.e. need to scale up constr.

		// update core assgt for node_dag_mc.
		node_dag_mc.set_params(num_cores, sc_core_assgt);

		last_mc_reopt_ts = get_monotime_now();
		return false;
	}

	// returns all scids which share a core with sci
	std::set<int> core_sharing_sc_set(std::vector<std::vector<int> > sc_core_assgt, int sci)
	{
		std::set<int> s;
		for (int ci = 0; ci < sc_core_assgt[sci].size(); ci++)
		{
			int coreid = sc_core_assgt[sci][ci];
			// find all sc's which have coreid in their array.
			for (int scj = 0; scj < sc_core_assgt.size(); scj++)
			{
				if ( std::find( sc_core_assgt[scj].begin(), sc_core_assgt[scj].end(), coreid ) != sc_core_assgt[scj].end() )
				{
					s.insert(scj);
					printf("FOR SCID %i SHARES CORE with SCJ %i", sci, scj);
				}
			}
		}
		return s;
	}
	
	void DAGControllerBE::update_per_core_threads()
	{
		// check if we need to do this.
		if (num_cores > 1)
		{
			bool need_to_update = false;
			// check if there's any subchain which needs to be moved.
			// check if 1. all subchains have same #cores
			// TODO:Later: 2. for all non-alone subchains, the set of subchains sharing a core is same.
			for (int i = 0; i < curr_sc_core_assgt.size(); i++)
				{
					/*
					std::set<int> s1 ( curr_sc_core_assgt[i].begin(), curr_sc_core_assgt[i].end() );
					std::set<int> s2 ( node_dag_mc.sc_core_assgt[i].begin(), node_dag_mc.sc_core_assgt[i].end() );
					*/
					std::set<int> s1 = core_sharing_sc_set( curr_sc_core_assgt, i );
					std::set<int> s2 = core_sharing_sc_set( node_dag_mc.sc_core_assgt, i );
					
					need_to_update = need_to_update || ( s1 != s2 );
					printf("need_to_upd after checking scid= %i: %i | ", i, need_to_update);
				}

			if (need_to_update)
			{
				set_high_priority("dyn_reopt - NEW SCHED THR", scheduler_priority, ::gettid() );

				printf("Monotime %f, Realtime %f :  NEED TO UPDATE was true!!! Starting to join old threads!!", get_monotime_now(), get_realtime_now());
				shutdown_scheduler = true;
				for (auto & x : per_core_sched_threads)
					x.second.join();
				per_core_sched_threads.clear();
				shutdown_scheduler = false;

			// make new threads:
				for (int ic = 0; ic < num_cores; ic++)
						if (node_dag_mc.core_sc_list[ic].size() > 1)
							per_core_sched_threads[ic] = boost::thread(&DAGControllerBE::handle_sched_main, this, std::vector<int>(1,ic) );
					for (int is = 0; is < exec_order.size(); is++)
						if (node_dag_mc.sc_id_frac_id.find(is) == node_dag_mc.sc_id_frac_id.end() )
						{
							printf("MTime %f, Realtime %f, SC id %i is ALONE!! Making a thread! Core list sz: %i", get_monotime_now(), get_realtime_now(), is, node_dag_mc.sc_core_assgt[is].size());
							if ( node_dag_mc.sc_core_assgt[is].size() > 0 )
							{
								int is_c = node_dag_mc.sc_core_assgt[is][0];
								per_core_sched_threads[is_c] = boost::thread(&DAGControllerBE::handle_sched_main, this, node_dag_mc.sc_core_assgt[is] );
							}
							else
								printf("WEIRD!!! ERROR!!! EMPTy core list for SC %i", is);
						}
				curr_sc_core_assgt = node_dag_mc.sc_core_assgt;
		
				// use reopt_thread_p since that thread calls this func.	
				set_other_policy_pthr(reoptimize_thread_p);
			}
		}
	}

	// the thread calling this func should take the lock on offline_fracs.
	void DAGControllerBE::update_stream_periods()
	{
		for (auto it = node_dag_mc.name_id_map.begin(); it != node_dag_mc.name_id_map.end(); it++)
		{
			// check if streaming node:
			if ( node_dag_mc.id_node_map[it->second].node_type.find("S") != std::string::npos)
			{
				// if yes, call FE func to update dropFraction.
				int scid = 0;
				for (int j = 0; j < exec_order.size(); j++)
					if (std::find( exec_order[j].begin(), exec_order[j].end(), it->second ) != exec_order[j].end() )
						scid = j;
				double new_per = per_core_period_map[ node_dag_mc.sc_core_assgt[scid][0] ] / offline_fracs[it->first];
				frontend->update_speriod(new_per, it->first); // needs to know HP, fi.
				printf("UPDATING PERIOD OF stream NODE %i, %s, NEW PERIOD: %f \n", it->second, it->first.c_str(), new_per);
			}
		}
	}

	void DAGControllerBE::update_per_core_period_map()
	{
		std::map<int, double> new_per_core_period_map;
		for (int i = 0; i < num_cores; i++)
			if ( node_dag_mc.sc_id_frac_id.find( node_dag_mc.core_sc_list[i][0] ) != node_dag_mc.sc_id_frac_id.end() )
			{
				double per = 0.0;
                                for (int si = 0; si < node_dag_mc.core_sc_list[i].size(); si++ )
				{
					auto& sid = node_dag_mc.core_sc_list[i][si];
                                        std::string sid0_nm = node_dag_mc.id_name_map[ exec_order[ sid ][0] ];
					for (int sni = 0; sni < exec_order[ sid ].size(); sni++)
                                        	per += node_dag_mc.id_node_map[ exec_order[sid][sni] ].compute * offline_fracs[ sid0_nm ] ;
				}
				new_per_core_period_map[i] = per;
				printf("MT: %f, RT: %f, For core %i, NEW HyperPeriod: %f \n", get_monotime_now() , get_realtime_now(), i, per);
			}
	
		for (int i = 0; i < exec_order.size(); i++)
			if (node_dag_mc.sc_id_alone_per.find(i) != node_dag_mc.sc_id_alone_per.end())
			{
				printf("For cores [0th core: %i] of sc id %i, HP: alone_per: %f \n", node_dag_mc.sc_core_assgt[i][0], i, node_dag_mc.sc_id_alone_per[i]);
				for (int ci = 0; ci < node_dag_mc.sc_core_assgt[i].size(); ci++ )
					new_per_core_period_map[ node_dag_mc.sc_core_assgt[i][ci] ] = node_dag_mc.sc_id_alone_per[i];
			}
		per_core_period_map = new_per_core_period_map;	
	}

	void DAGControllerBE::dynamic_reoptimize_func()
	{
		std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " STARTING Dynamic Reoptimize thread!" << std::endl;
		reoptimize_thread_id = ::gettid();
		pthread_t this_thread = pthread_self();
		reoptimize_thread_p = this_thread;
		struct sched_param params;
		params.sched_priority = 0;
		set_other_policy_pthr(reoptimize_thread_p); // pthread_setschedparam(this_thread, SCHED_OTHER, &params);
		
		printf("Mt %f, realtime %f, dynamic_reoptimize_func THREAD pthread tid: %i, tid: %i \n", get_monotime_now(), get_realtime_now(), reoptimize_thread_id, ::gettid() );

		int sleep_time = 2000;
		bool first_resolve = true;

		while (!shutdown_scheduler)
		{
			std::this_thread::sleep_for (std::chrono::milliseconds(sleep_time));
			printf("Reoptimize thread WOKEN UP [Slept for %i]!! Starting to re-solve... \n", sleep_time);

			// Not needed for dag_mc: clear old data
			// node_dag.clear_old_data(frac_var_count);
			// Done: Update node_dag_mc ci, re-solve using that and update offline_fracs. 
			
			bool scale_nc_constr = false; 

			try
			{
			offline_fracs_mtx.lock();
			// update compute times to be 95ile of recent data.
			node_dag_mc.update_cis(node_ci_arr, node_ci2_arr, node_ci_mode_arr);
			if (first_resolve)
			{
				// bootstrapping: if a node has 0 ci samples now, use max of other ci's as an estimate. [only for first re-solve.]
				std::vector<int> bootstrap_ci_nodes;
				float max_ci = 0.0;
				for (auto& kv : node_dag_mc.name_id_map)
				{
					if (node_ci_mode_arr[kv.first].size() == 0)
						bootstrap_ci_nodes.push_back(kv.second);
					else
						max_ci = std::max(max_ci, node_dag_mc.id_node_map[kv.second].compute);
				}
				for (auto &x: bootstrap_ci_nodes)
				{
					printf("CI ESTIMATE unavailable for node %i, using max of other ci as an estimate %f", x, max_ci);
					node_dag_mc.id_node_map[x].compute = max_ci;
				}
				
				// assign each SC frac = (#nodes)*5 / ci : so that we give each node 5ms per HP until first re-solve.
				for (int i = 0; i < exec_order.size(); i++)
				{
					offline_fracs[ node_dag_mc.id_name_map[exec_order[i][0]] ] = std::min(1.0, (5.0 * exec_order[i].size())/( get_sum_ci_ith(exec_order[i]) ) );
					printf("JUST BEFORE FIRST re-solve: SETTING offline fracs so as to give each node <=5ms per HP [just until first re-solve!!] frac for %s : %f", node_dag_mc.id_name_map[exec_order[i][0]].c_str(), offline_fracs[ node_dag_mc.id_name_map[exec_order[i][0]] ]);
				}
			
				first_resolve = false;
			}
			
			offline_fracs_mtx.unlock();
			printf("DONE Updating all cis! Will start solving now: ");
				
				// Done: Re-solve for core assgt every 10s or 20s.
				if ( (get_monotime_now() - last_mc_reopt_ts) > 20.0)
					scale_nc_constr = dynamic_reoptimize_mc();

				double solve_start = get_monotime_now();
				node_dag_mc.assign_fixed_periods();
				all_frac_values = node_dag_mc.compute_rt_solve();
				printf("Monotime %f, realtime %f, Mono-time taken to solve GP: %f \n", get_monotime_now(), get_realtime_now(), (get_monotime_now()-solve_start) );

				if (all_frac_values[0] > 0)
				{
					// take a lock to change offline_fracs: [i.e. the handle_Sched thread might have to wait a little bit sometimes before getting timeout value.] 
					offline_fracs_mtx.lock();
					// With new dag_mc, all_frac_vals directly gives frac of each subchain.
					for (int i = 0; i < exec_order.size(); i++)
					{
						offline_fracs[ node_dag_mc.id_name_map[exec_order[i][0]] ] = (double)(1.0)/all_frac_values[i];
						printf("MT: %f, RT: %f, In new soln, Fraction for node %s is %f , frac_val: %f \n", get_monotime_now(), get_realtime_now(), node_dag_mc.id_name_map[exec_order[i][0]].c_str(), offline_fracs[ node_dag_mc.id_name_map[exec_order[i][0]] ], all_frac_values[i]);
					}
					update_per_core_period_map();	
					
					update_stream_periods();
					
					offline_fracs_mtx.unlock();

					update_per_core_threads();

					// For DFracV2 expts, where we re-solve only once:
					// dynamic_reoptimize = false;
					// return;
				}
				else
				{
					scale_nc_constr = true;
					printf("SAD: all_frac_values[0]: %f", all_frac_values[0]);
				}
			}
			catch (const std::exception& e)
			{
				printf("MT: %f, RT: %f, EXCEPTION in trying to solve GP ", get_monotime_now() , get_realtime_now() );
				std::cout << e.what() << std::endl;
			}
			
			sleep_time = scale_nc_constr ? 1000 : 5000;
			/*
			if (scale_nc_constr)
			{
				node_dag_mc.scale_nc_constraints(1.5);
				scale_nc_constr = false;
			} */
		}
		printf("MT: %f, RT: %f, OUT OF THE reoptimize LOOP!!! shutdown_scheduler: %i \n", get_monotime_now() , get_realtime_now(), shutdown_scheduler.load());
	
	}

	void DAGControllerBE::update_ci(std::string node_name, double ci, int mode)
	{
		// Done: Add some extra time based on #Extra threads.
		double e_ci = 0.001*0.1*(node_extra_tids[node_name].size()); // in seconds.
		if (mode == 0)
			node_ci_arr[node_name].push_back(ci + e_ci );
		else
			node_ci2_arr[node_name].push_back(ci + e_ci );
		node_ci_mode_arr[node_name].push_back(mode);
	}

	void DAGControllerBE::update_latest_sensor_ts(std::string node_name, float sensor_ts)
	{
		node_latest_sensor_ts[node_name] = sensor_ts;
		if (total_period_count % 200 == 7)
			printf("MT: %f, RT: %f, GOT node %s latest sensor ts %i", get_monotime_now() , get_realtime_now(), node_name.c_str(), sensor_ts);
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
			max = std::max(max, node_dag_mc.id_node_map[ sci[i] ].compute);
		return max;
	}

	// the thread calling this function takes the lock on offline_fracs.
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
		set_high_priority("Timer thread", scheduler_priority, 0);
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
		printf("MT %f, RT %f ABOUT to NOTIFY the main thread, cc_end is false! [Had timer for %f] [ret val for settinf canceltype %i, oldstate %i] \n", get_monotime_now(), get_realtime_now(), timeout, ret_pct, oldstate);
		cv_sched_thread.notify_all();
		timer_thread_running = false;
	}

	bool DAGControllerBE::checkWaitFor(std::vector<std::vector<int>>& iexec_order, int ind, long i_to, int core_id, std::map<int, int>& core_node_exec_order_id, std::vector<double>& sc_fracs, double core_per)
	{
		std::vector<int>& sc = iexec_order[ind];
		int n0_sc = sc[0];

		bool need_to_wait = false;

		int wnid, wn_sc, wn_sc_n0;
		double timenow = get_monotime_now();
		double per_output_thresh, curr_out_ts;

		if (node_dag_mc.id_node_map[n0_sc].wait_for_outputs.size() > 0 )
		{
			wnid = node_dag_mc.id_node_map[n0_sc].wait_for_outputs[0];
			if (core_node_exec_order_id.find(wnid) != core_node_exec_order_id.end())
			{
				wn_sc = core_node_exec_order_id[wnid];
				wn_sc_n0 = iexec_order[wn_sc][0];
				// threshold for one output = nid's current period = SC[nid]'s current period * 1.05 [scaled for slack].
				// stream_minper or min_period if stream_minper is 0.
				per_output_thresh = 1.05 * 0.001 * std::max( core_per / sc_fracs[wn_sc], (double) node_dag_mc.id_node_map[wn_sc_n0].stream_minper );
				curr_out_ts = node_latest_sensor_ts[node_dag_mc.id_name_map[wnid]];
				if (per_core_period_counts[core_id]%50 == 17)
					printf("MT: %f NODE scid %i, waitfor node %i with per_output_thresh: %f, latest outTS %f", timenow, ind, wnid, per_output_thresh, curr_out_ts);
				need_to_wait = need_to_wait || ( curr_out_ts < (timenow - per_output_thresh) );

			}
		}

		if (need_to_wait)
		{
			long micro_start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count();
			changePriority(iexec_order, wn_sc, core_id);
			// per node cv,lock...
			boost::unique_lock<boost::mutex> lock (node_sched_thread_mutex[wnid]);
			node_cv_sched_thread[wnid].wait_for( lock, boost::chrono::microseconds(i_to) );

			long rem_time = i_to - (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() - micro_start);
			if (rem_time > 500)
			{
				changePriority(iexec_order, ind, core_id);
				thread_custom_sleep_for(rem_time);
			}
			wait_for_logs[n0_sc] << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << ", " << i_to << ", " << (i_to-rem_time) << ", " << per_output_thresh << ", " << (timenow-curr_out_ts) << "\n";
			if (per_core_period_counts[core_id]%50 == 17)
				printf("mt: %f, RT: %f, Node %i WAITFOR %i !! total TO: %i, rem_time: %i", get_monotime_now(), get_realtime_now(), ind, wn_sc, i_to, rem_time);
		}
		return need_to_wait;
	}

	// checks if SC ind needs to be triggered, if yes, calls the FE func.
	bool DAGControllerBE::checkTriggerExec(std::vector<int>& sci, int core_id, double frac)
	{
		int id = sci[0];
		// DEBUG(sci[0]);
		// DEBUG(frac);
		// DEBUG(core_id);
		int ind_p = round(1.0/frac);
		std::string name = node_dag_mc.id_name_map[id];
		bool is_streaming = (node_dag_mc.id_node_map[id].node_type.find("S") != std::string::npos );

		bool need_to_skip_trigger_and_wait = false;
		int wait_for_nid = 0;
		// March-2021: [old implementation of waitfor] Skip triggers/give resrc to previous node, if need to wait for new inputs.
		/*	
		if ( node_dag_mc.id_node_map[id].wait_for_outputs.size() > 0 )		
			{
				for (auto nid : node_dag_mc.id_node_map[id].wait_for_outputs)
					if ( (node_skip_ct[name] < node_max_skips[name]) && ( ( node_latest_sensor_ts[name] >= node_latest_sensor_ts[node_dag_mc.id_name_map[nid]] ) ||  (node_latest_sensor_ts[node_dag_mc.id_name_map[nid]] < (frontend->get_sim_time() - 100) ) ) )
					{
						need_to_skip_trigger_and_wait = true; // i.e. did not trigger & give resrc to wait_for nodes.
						wait_for_nid = nid;
					}
			}
		*/

		if (ind_p == 0)
			printf("WEIRDDDDDD - ind_p : %i, frac: %f, name: %s", ind_p, frac, name.c_str());

		// Mar-Apr:2021: Triggering mapcb each period since its a streaming node. 
		// if ( (ind_p == 1 || ( (ind_p>0) && ( (per_core_period_counts[core_id]) %ind_p == 1)) ) || (name.find("mapcb") != std::string::npos) ) // && (node_tid.find(name) != node_tid.end() ) )  
		
		// ind_p=1, is_streaming : trigger per HP, per_core_period_counts = 1 means first period, then check wrt last trigger.

		if ( (ind_p == 1) || (is_streaming) || (per_core_period_counts[core_id] == 1) || (per_core_sc_last_trigger_ct[core_id][id] <= (per_core_period_counts[core_id] - ind_p) ) )
		{
			/*
			if (need_to_skip_trigger_and_wait)
			{
				node_skip_ct[name] += 1;	
				if (per_core_period_counts[core_id]%50 == 7 || (name.find("navp") != std::string::npos) || (name.find("mapu") != std::string::npos) )
					printf("MT: %f, RT: %f, node: %s, skip ct: %i, latest ts: %i, WAITFOR %s with ts: %i simTime: %i ", get_monotime_now(), get_realtime_now(), name.c_str(), node_skip_ct[name].load(), node_latest_sensor_ts[name], node_dag_mc.id_name_map[wait_for_nid].c_str(), node_latest_sensor_ts[node_dag_mc.id_name_map[wait_for_nid]], frontend->get_sim_time() );
			}
			else */

			{
				if (per_core_period_counts[core_id]%12 == 3 || (name.find("ppcam") != std::string::npos) )
					printf("MT: %f, RT: %f, trigger node: %s, frac: %i, period_count: %li \n", get_monotime_now(), get_realtime_now(), name.c_str(), ind_p, per_core_period_counts[core_id]);
			
				frontend->trigger_node(name, true);
				// For Illixr Dag, need to trigger TW along with Imu, cuz Imu is at fixed freq for now. 
				if ( (name.find(IMU_PLUGIN_NAME) != std::string::npos) && (dag_name.find("ill") != std::string::npos) )
					frontend->trigger_node(TW_PLUGIN_NAME, true);

				if (last_trig_ts[name] > 0)
					trigger_log << name << ", " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << "\n";
				last_trig_ts[name] = get_realtime_now();
				// node_skip_ct[name] = 0; // reset skip ct at every trigger
				per_core_sc_last_trigger_ct[core_id][id] = per_core_period_counts[core_id];
			}
		}
		// return false only if need_to_skip_trig_and_wait is true.
		return (need_to_skip_trigger_and_wait);
	}
