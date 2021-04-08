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

#define DEBUG2(x1, x2) std::cerr << __FILE__ << ':' << __LINE__ << ": " << #x1 << "=" << x1 << << ", " << #x2 << "=" << x2 << std::endl;
#define DEBUG(x) std::cerr << __FILE__ << ':' << __LINE__ << ": " << #x << "=" << x << std::endl;

#define TW_PLUGIN_NAME "6"
#define RENDER_PLUGIN_NAME "5"
#define IMU_PLUGIN_NAME "7"
#define CAM_PLUGIN_NAME "8"

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
		cc_completion_log.open("IllixrNewLog.csv");
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
			offline_fracs["mapcb"] = 1.0/f_mc;
			offline_fracs["mapupd"] = 1.0/f_mu;
			offline_fracs["navc"] = 1.0/f_nc;
			offline_fracs["navp"] = 1.0/f_np;
		}
		else if (dag_name.find("illixr") != std::string::npos)
		{
			offline_fracs[IMU_PLUGIN_NAME] = 1.0; // imu is start of cc, tw
			offline_fracs[CAM_PLUGIN_NAME] = 1.0/f_mu; // cam-slam
			offline_fracs[RENDER_PLUGIN_NAME] = 1.0/f_mc; // render
		}
		update_per_core_period_map(); // uses offline_fracs,nodeDagMC's sc_core_Assgt. 

		// ILLIXR_SPECIAL_TESTING_LOGIC 
		for (int i = 0; i < exec_order.size(); i++)
		{
			node_finished[ exec_order[i][0] ] = false;
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
		{
			reset_count.push_back(false);
		}
	
		ready_sched = false;

		frontend = fe;
		
		// Initialize arrays for ci of all nodes:
		for (auto const& x : node_dag_mc.id_name_map)
		{
			node_ci_arr[x.second] = boost::circular_buffer<double> (50);
			// node_extra_tids[x.second] = std::vector<int> ();
			last_trig_ts[x.second] = 0.0;
		}

		sched_started = false;
		shutdown_scheduler = false;
		// startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &controller_start_ts);	
	}

	void DAGControllerBE::start()
	{
		startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
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
		set_high_priority("Startup trigger thread", 4, 0);
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
					
					// // Lets kill the startup_thread!
					// if (startup_thread->joinable())
					// {
					// 	pthread_cancel(startup_thread->native_handle());
					// 	pthread_join(startup_thread->native_handle(), NULL);
					// 	startup_thread->detach();
					// 	startup_thread = NULL;
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
					notify_node_exec_end( node_dag_mc.id_name_map[exec_order[0][0]] );
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
		std::string name = "hand_sched_" + std::to_string(core_ids.at(0));
		pthread_setname_np(pthread_self(), name.c_str());
		set_high_priority("MAIN sched Thread", 4, 0);
		for (auto &i : node_dag_mc.name_id_map)
			printf("#EXTRA THREADS for node %s : %lu", i.first.c_str(), node_extra_tids[i.first].size() );
	
		if ( core_ids.size() == 0 )
		{
			printf("ERRORR!!! WEIRD!!!! Empty core list to sched_main thread!! \n");
			return;
		}
		per_core_period_counts[core_ids[0] ] = 1;
		total_period_count = 1;
		
		// Done: Make core-wise exec_order: [exec_list is the list of sc ids on a particular core.]
		// Done: Set all nodes + extras on this core & this thread to CPUSet core_id.
		std::vector<int> core_exec_list = node_dag_mc.core_sc_list[ core_ids[0] ]; 
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
		
		printf("STARTING MAinSched thread!! tid: %li Len of exec_order: %zu, curr_cc_period: %f, #cores: %zu, core0: %i \n", ::gettid(), core_exec_order.size(), per_core_period_map[ core_ids[0] ], core_ids.size(), core_ids[0] );
	
		double core_period = 1.0;
		std::vector<long> sc_to (core_exec_order.size() );
                std::vector<double> sc_fracs ( core_exec_order.size() ) ;
		
		while (!shutdown_scheduler)
		{
			// ILLIXR_SPECIAL_TESTING_LOGIC : calculate render+CS budget (in us) = fr*cr + fcs*ccs.
			long illixr_noncrit_to_budget = 0; 
			offline_fracs_mtx.lock();
			// Using lock to protect offline_fracs, node_dag_mc. core_period = per_core_period_map[ core_ids[0] ];
			core_period = 0.0;
			for (int i = 0; i < core_exec_order.size(); i++)
			{
				std::string scname = node_dag_mc.id_name_map[ core_exec_order[i][0] ];
				sc_fracs[i] = offline_fracs[ scname ] ;
				sc_to[i] = (long) (1000*get_timeout(core_exec_order[i], core_ids) );
				core_period += (double)sc_to[i]/1000.0;

				if ( (scname.find(RENDER_PLUGIN_NAME) != std::string::npos) || (scname.find(CAM_PLUGIN_NAME) != std::string::npos) )
					illixr_noncrit_to_budget += sc_to[i];
			}
			if (total_period_count % 100 == 0) {
				DEBUG(core_period);
				for (size_t i = 0; i < sc_to.size(); ++i) {
					DEBUG(i);
					DEBUG(sc_to[i]);
				}
			}
			offline_fracs_mtx.unlock();

			/* ORIGINAL:
			for (int i = 0; ( (i < core_exec_order.size()) && (!shutdown_scheduler) ); i++)
			{
				// prio(i) = 2, all others = 1.
				long i_to = sc_to[i];
				// CHECK: For DynNoSC 
				changePriority(core_exec_order, i, core_ids[0]); // handles changing priority
				checkTriggerExec( core_exec_order[i], core_ids[0], sc_fracs[i] );
				auto trig_cc = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count();

				ready_sched = false;

				// CHECK: For DynNoSC
				// cv wait only if CC, i.e. sc id == 0 & only 1core.
				if ( (core_exec_list.at(i) == 0) && (core_ids.size() == 1) )
				{
					boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
					int ct = 0;

					while ( (!ready_sched) && (ct<100) && (!shutdown_scheduler) )
					{
						cv_sched_thread.wait_for(lock, boost::chrono::microseconds(i_to*2) );
						ct += 1;
					}
					if (ct > 5)
						printf("Monotime %f, realtime %f, Waited for CC's completion! ct %i ready_sched %i [if 0, CC hasnt ended!] \n", get_monotime_now(), get_realtime_now(), ct, (bool)(ready_sched.load()) );
					
		cc_completion_log << trig_cc << ", " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << "\n";
				}
				else 
				{
					// std::this_thread::sleep_for( std::chrono::microseconds( i_to ) );
					thread_custom_sleep_for(i_to);
				}
			
			}
			*/
			// NEW CODE FOR ILLIXR: [TW - f=1 wait for it, Reder - f=1 wait for it, CS: Rem time based on vsync freq]
			// ILLIXR_SPECIAL_TESTING_LOGIC specific to 1core, just for testing v1
			
			long render_trigger_ts = 0;
			for (int i = 0; ( (i < core_exec_order.size()) && (!shutdown_scheduler) ); i++)
			{
				long i_to = sc_to[i];
				changePriority(core_exec_order, i, core_ids[0]); // handles changing priority
				checkTriggerExec( core_exec_order[i], core_ids[0], sc_fracs[i] );


				// wait for finish for both TW, render [this assumes render f=1]
				std::string nodename = node_dag_mc.id_name_map[ core_exec_order[i][0] ];
				if (nodename.find(RENDER_PLUGIN_NAME) != std::string::npos)
					render_trigger_ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count();

				if ( ( nodename.find(IMU_PLUGIN_NAME) != std::string::npos ) || ( nodename.find(RENDER_PLUGIN_NAME) != std::string::npos ) )
				{
					boost::unique_lock<boost::mutex> lock(node_sched_thread_mutex[ core_exec_order[i][0] ]);
					int ct = 0;
					while (!node_finished[ core_exec_order[i][0] ] && (ct<100) && (!shutdown_scheduler))
					{
						ct += 1;
						node_cv_sched_thread[ core_exec_order[i][0] ].wait_for(lock, boost::chrono::microseconds(i_to*2));
					}
					if (ct > 10)
						printf("Monotime %f, realtime %f, Waited for %s completion, ct %i ready_sched %i [if 0, node hasnt ended!] \n", get_monotime_now(), get_realtime_now(), nodename.c_str(), ct, (bool)(node_finished[ core_exec_order[i][0] ].load()) );
					// clear node_finished bool.
					node_finished[ core_exec_order[i][0] ] = false;
				}
				else
				{
					// sleep for remaining time in budget.
					long render_time = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() - render_trigger_ts  );
					static_assert(sizeof(long) >= 7);
					thread_custom_sleep_for( illixr_noncrit_to_budget - render_time );
					cc_completion_log << render_trigger_ts << ", " << render_time << ", " << illixr_noncrit_to_budget << "\n";
				}
			}


			total_period_count += 1;
			per_core_period_counts[core_ids[0]] += 1;
			// inc prio of our dynamic_resolve thread for sometime.	
			int one_in_k_per = ceil(2000/(float)(20* core_period ));
			
			// CHECK: For DynNoSC
			if (per_core_period_counts[core_ids[0]] %( one_in_k_per ) == 0 && (dynamic_reoptimize))
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
		node_dag_mc.print_vec(cores, "");
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
			std::cout << "WEIRD-" << nname << " Changing prio to " << prio << ", tid: " << tid << ", RETVAL: " << ret << std::endl;
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
			std::vector<std::string> node_ids {TW_PLUGIN_NAME, "3", "2", RENDER_PLUGIN_NAME, "8", IMU_PLUGIN_NAME}; // wait for all 6 tids.
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
		std::cout << "GOT TID FOR " << node_name << ", " << tid << std::endl;
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
			if (dynamic_reoptimize)
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
					std::set<int> s1 ( curr_sc_core_assgt[i].begin(), curr_sc_core_assgt[i].end() );
					std::set<int> s2 ( node_dag_mc.sc_core_assgt[i].begin(), node_dag_mc.sc_core_assgt[i].end() );
					need_to_update = need_to_update || ( s1 != s2 );
					printf("need_to_upd after checking scid= %i: %i | ", i, need_to_update);
				}

			if (need_to_update)
			{
				set_high_priority("dyn_reopt - NEW SCHED THR", 4, ::gettid() );

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

		int sleep_time = 5000;

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
			// update compute times to be 75ile of recent data.
			node_dag_mc.update_cis(node_ci_arr);
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
						printf("MT: %f, RT: %f, In new soln, Fraction for node %s is %f , frac_val: %i \n", get_monotime_now(), get_realtime_now(), node_dag_mc.id_name_map[exec_order[i][0]].c_str(), offline_fracs[ node_dag_mc.id_name_map[exec_order[i][0]] ], all_frac_values[i]);
					}
					update_per_core_period_map();	
					
					offline_fracs_mtx.unlock();

					update_per_core_threads();

					// For DFracV2 expts, where we re-solve only once:
					// dynamic_reoptimize = false;
					// return;
				}
				else
					scale_nc_constr = true;
			}
			catch (const std::exception& e)
			{
				printf("MT: %f, RT: %f, EXCEPTION in trying to solve GP ", get_monotime_now() , get_realtime_now() );
				std::cout << e.what() << std::endl;
			}
			
			sleep_time = scale_nc_constr ? 1000 : 5000;
			if (scale_nc_constr)
			{
				node_dag_mc.scale_nc_constraints(1.5);
				scale_nc_constr = false;
			}
		}
		printf("MT: %f, RT: %f, OUT OF THE reoptimize LOOP!!! shutdown_scheduler: %i \n", get_monotime_now() , get_realtime_now(), shutdown_scheduler.load());
	
	}

	void DAGControllerBE::update_ci(std::string node_name, double ci)
	{
		// Done: Add some extra time based on #Extra threads.
		double e_ci = 0.001*0.5*(node_extra_tids[node_name].size()); // in seconds.
		node_ci_arr[node_name].push_back(ci + e_ci );
	}

	void DAGControllerBE::update_latest_sensor_ts(std::string node_name, int sensor_ts)
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
		printf("MT %f, RT %f ABOUT to NOTIFY the main thread, cc_end is false! [Had timer for %f] [ret val for settinf canceltype %i, oldstate %i] \n", get_monotime_now(), get_realtime_now(), timeout, ret_pct, oldstate);
		cv_sched_thread.notify_all();
		timer_thread_running = false;
	}

	// checks if SC ind needs to be triggered, if yes, calls the FE func.
	void DAGControllerBE::checkTriggerExec(std::vector<int>& sci, int core_id, double frac)
	{
		int id = sci[0];
                // int ind_p = round(1.0/offline_fracs[node_dag.id_name_map[id] ]);
		int ind_p = round(1.0/frac);
		std::string name = node_dag_mc.id_name_map[id];
		if ( (ind_p == 1 || ( (per_core_period_counts[core_id]) %ind_p == 1) ) ) // && (node_tid.find(name) != node_tid.end() ) ) // (ind > 0) && : Removing cuz CC now triggered by scheduler. 
		{
			if (per_core_period_counts[core_id]%50 == 7)
				printf("MT: %f, RT: %f, trigger node: %s, frac: %i, period_count: %li \n", get_monotime_now(), get_realtime_now(), name.c_str(), ind_p, per_core_period_counts[core_id]);
		
			frontend->trigger_node(name, true);
			// For Illixr Dag, need to trigger TW along with Imu, cuz Imu is at fixed freq for now. 
			if ( (name.find(IMU_PLUGIN_NAME) != std::string::npos) && (dag_name.find("ill") != std::string::npos) )
				frontend->trigger_node(TW_PLUGIN_NAME, true);

			if (last_trig_ts[name] > 0)
				trigger_log << name << ", " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << "\n";
			last_trig_ts[name] = get_realtime_now();

			
		}
	}
