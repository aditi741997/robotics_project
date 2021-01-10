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
		printf("Monotime %f, realtime %f, retval for changing prio of dynamic_reoptimize_func THREAD: %i |", get_monotime_now(), get_realtime_now(), ret);
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
DAGControllerBE::DAGControllerBE(std::string dag_file, DAGControllerFE* fe, bool dyn_resolve, std::string use_td, std::string fifo, int f_mc, int f_mu, int f_nc, int f_np, int p_s, int p_lc, int p_lp)
	{
		// Note that the last 4 inputs are just for the offline stage.
		node_dag = DAG(dag_file);
		dag_name = dag_file;

		timer_thread_running = false;
		timer_thread = NULL;

		dynamic_reoptimize = dyn_resolve;

		node_dag.fill_trigger_nodes();
                node_dag.assign_publishing_rates();
                node_dag.assign_src_rates();

		/*
		int num_cores = 1;
		multi_core_solver = MultiCoreApproxSolver(&node_dag, num_cores);
		std::vector< std::vector<int> > sc_core_assgt = multi_core_solver.solve();
		DAGMultiCore node_dag_mc = DAGMultiCore(dag_file);
		node_dag_mc.set_params(num_cores, sc_core_assgt);
		node_dag_mc.assign_fixed_periods();
		node_dag_mc.compute_rt_solve();
		*/

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
			offline_fracs["imu"] = 1.0;
			offline_fracs["2"] = 1.0/f_mu; // cam-slam : 2,3
			offline_fracs["6"] = 1.0/f_mc; // render : 6
		}

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
		}

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
	
		curr_exec_index = 0;
		ready_sched = false;

		frontend = fe;
		
		// Initialize arrays for ci of all nodes:
		for (auto const& x : node_dag.id_name_map)
		{
			node_ci_arr[x.second] = boost::circular_buffer<double> (50);
		}

		sched_started = false;
		// startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
	}

	void DAGControllerBE::start()
	{
		startup_thread = new boost::thread(&DAGControllerBE::startup_trigger_func, this);
	}

	void DAGControllerBE::startup_trigger_func()
	{
		double cc_period = 0.0; // in millisec.
		for (int i = 0; i < exec_order.size(); i++)
			cc_period += get_sum_ci_ith(exec_order[i])*offline_fracs[ node_dag.id_name_map[ exec_order[i][0] ] ];
		int oldstate;
		int ret_pct = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);
		printf("MonoTime: %f, RealTime: %f, STARTED startup_trigger thread! CC period: %f, sched_started: %i \n", get_monotime_now(), get_realtime_now(), cc_period, sched_started );
	
		curr_cc_period = cc_period;

		total_period_count = 0;

		while (!sched_started)
		{
			long to = 1000*cc_period;
			std::this_thread::sleep_for( std::chrono::microseconds(to) );
			if (!sched_started)
			{
				total_period_count += 1;
				printf("MonoTime: %f, RealTime: %f, Period ct: %i, checking Trigger for all, exec_order sz: %i, curr_cc_period: %f", get_monotime_now(), get_realtime_now(), total_period_count, exec_order.size(), curr_cc_period);
				for (int i = 0; i < exec_order.size(); i++)
					checkTriggerExec(exec_order[i]);
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
					handle_sched_thread = boost::thread(&DAGControllerBE::handle_sched_main, this);
					std::this_thread::sleep_for (std::chrono::milliseconds(2));
				}
				else
				{
					// Moved to handle_Sched_main: total_period_count += 1;
					
					// Notify the handle_sched_main : it waits for CC to end before moving on to NC nodes.	
					boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
					cc_end = true;
					ready_sched = true;
					printf("ABOUT to NOTIFY the main thread, cc_End: TRUE! \n");
					cv_sched_thread.notify_all();
				}

				
			}
			/* MOVED to startup thread.
			else
			{
				std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " | GOT exec_end msg. NOT making a timer,Will trigger nodes" << std::endl;
				for (int i = 1; i < exec_order.size(); i++)
					checkTriggerExec(i);
					//frontend->trigger_node(node_dag.id_name_map[exec_order[i][0]], false);
			}
			*/
		}
			
	}

	void DAGControllerBE::handle_sched_main()
	{
		set_high_priority("MAIN sched Thread", 4, 0);
		printf("Len of exec_order: %zu, curr_cc_period: %f, ceil 0.5: %f \n", exec_order.size(), curr_cc_period, ceil(0.5) );
		total_period_count = 1;
		while (true)
		{
			for (int i = 0; i < exec_order.size(); i++)
			{
				// printf("Monotime %f, realtime %f, TIME to run subchain #%i, Timeout: %f \n", get_monotime_now(), get_realtime_now(), i, get_timeout(exec_order[i]) );
				// prio(i) = 2, all others = 1.
				offline_fracs_mtx.lock();
				long i_to = 1000*get_timeout(exec_order[i]);
				changePriority(exec_order, i); // handles priority & trigger.
				offline_fracs_mtx.unlock();

				ready_sched = false;

				// cv.wait (i=0), sleep for all others
				if (i == 0)
				{
					
					boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
					int ct = 0;

					while ( (!ready_sched) && (ct<12) )
					{
						cv_sched_thread.wait_for(lock, boost::chrono::microseconds(i_to*5) );
						ct += 1;
					}
					if (ct > 4)
						printf("Monotime %f, realtime %f, Waited for CC's completion! ct %i ready_sched %i [if 0, CC hasnt ended!] \n", get_monotime_now(), get_realtime_now(), ct, (bool)(ready_sched.load()) );
				}
				else
					std::this_thread::sleep_for( std::chrono::microseconds( i_to ) );
			}

			total_period_count += 1;
			// inc prio of our dynamic_resolve thread for sometime. to fifo,p2, the whole exec_order to p1 or just the last guy to p1.
			// assuming 20ms -> 40ms for LowCF needed by reopt thread in 1s : 
			int one_in_k_per = ceil(1000/(float)(40*curr_cc_period));
			if (total_period_count%( one_in_k_per ) == 0 && (dynamic_reoptimize))
			{
				printf("WANNA run dyn_reopt for 1ms now!!");
				changePriority(exec_order, -1);
				set_high_priority("Dyn Reopt thread!", 2, reoptimize_thread_id);
				std::this_thread::sleep_for( std::chrono::milliseconds(1) );
				// change policy back to rr, p1. RR wont work!!
				set_other_policy_pthr(reoptimize_thread_p);
				// set_high_priority("Dyn Reopt thread!", 1, reoptimize_thread_id);
			}
			// max(1ms, 0.05*curr_per) slack time.
		}
	}

	std::string DAGControllerBE::get_last_node_cc_name()
	{
		int id = exec_order[0][exec_order[0].size()-1];
		return node_dag.id_name_map[id];
	}

	void DAGControllerBE::set_high_priority(std::string thread_name, int prio, int tid)
	{
		// set prio to 4, with FIFO.
		struct sched_param sp = { .sched_priority = prio,};
		int ret = sched_setscheduler(tid, SCHED_FIFO, &sp);
		 printf("MonoTime: %f, RealTime: %f, SETTING priority to %i, for thread: %s, retval: %i \n", get_monotime_now(), get_realtime_now(), prio, thread_name.c_str(), ret);
		// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " SETTING priority to " << prio << " retval: " << ret << thread_name << std::endl;
	 }

	void DAGControllerBE::handle_noncritical_loop()
	{
		set_high_priority("NonCritical Loop", 4, 0);
		while (true)
		{
			boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
			while (!ready_sched)
			{
				cv_sched_thread.wait(lock);
				printf("ABOUT to run noncritical_loop!!!");
			}
			bool do_work = true;	

			if (cc_end)
				curr_exec_index = 1;
			else if (curr_exec_index > 0)
				curr_exec_index = (curr_exec_index+1)%(exec_order.size());
			else
				do_work = false;
			std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " NC Loop: curr_exec_index: " << curr_exec_index << ", do_work: " << do_work << std::endl;

			if (do_work)
			{
				changePriority(exec_order, curr_exec_index);
				double timeout = get_timeout(exec_order[curr_exec_index]);
				if ( timer_thread_running )
				{
					pthread_cancel(timer_thread->native_handle());
					// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << "ABOUT to join timer thread!!!" << std::endl;
					printf("Monotime %f, realtime %f, ABOUT to join timer thread!", get_monotime_now(), get_realtime_now());
					// pthread_kill(timer_thread.native_handle(),9);
					pthread_join(timer_thread->native_handle(), NULL);
					timer_thread_running = false;
					timer_thread->detach();
					timer_thread = NULL;
					printf("Monotime %f, realtime %f, JOINED with timer thread! \n", get_monotime_now(), get_realtime_now());
				}
				else if (timer_thread != NULL)
				{
					pthread_join(timer_thread->native_handle(), NULL);
					timer_thread->detach();
					timer_thread = NULL;
					printf("Monotime %f, realtime %f, Destructed timer thread! \n", get_monotime_now(), get_realtime_now());
				}
				timer_thread = new boost::thread(&DAGControllerBE::timer_thread_func, this, timeout);
			}
			
			cc_end = false;
			ready_sched = false;
		}
	}


	// Changes subchain[ind] prio = 2, all others = 1.
	void DAGControllerBE::changePriority(std::vector<std::vector<int> >& iexec_order, int ind)
	{
		// curr_exec_index = ind;
		for (int i = 0; i < iexec_order.size(); i++)
                {
                        int ret = 7;
			if (i == ind)
                                ret = changePrioritySubChain(iexec_order[i], 2);
                        else
                                ret = changePrioritySubChain(iexec_order[i], 1);
			// if (ret != 0)
				// std::cerr << "WEIRD, Ret value " << ret << " for setting prio of SC node " << node_dag.id_name_map[exec_order[i][0]] << " curr_exec_index: " << curr_exec_index << std::endl;
                                // ROS_WARN("Weird: Ret value %i for setting priority of subchain node %s, ind to be exec : %i", ret, node_dag.id_name_map[exec_order[i][0]].c_str(), curr_exec_index);
		}
		if (ind>=0)
			checkTriggerExec(iexec_order[ind]);
	}

	int DAGControllerBE::changePrioritySubChain(std::vector<int>& sci, int prio)
	{
		bool ret = true;
		struct sched_param sp = { .sched_priority = prio,};
		// Nov: Change priorities in reverse order, so that if any later node in subchain is preempted,
		// it executes first. [since it'll be ahead in the list of same prio.]
		for (int j = sci.size()-1 ; j >= 0; j-- )
		{
			// printf("Monotime %f, realtime %f, ABOUT TO change prio for node %s to %i, tid %i \n", get_monotime_now(), get_realtime_now(), node_dag.id_name_map[exec_order[ind][j]].c_str(), prio, node_tid[node_dag.id_name_map[exec_order[ind][j]] ] );
			if (node_tid.find( node_dag.id_name_map[sci[j]] ) != node_tid.end())
			{
				int n_tid = node_tid[node_dag.id_name_map[sci[j]] ];
				if (n_tid == 0)
					std::cerr << "WUTT!!!!" << sci[0] << ", " << node_dag.id_name_map[sci[j]] << ", " << (node_tid.find( node_dag.id_name_map[sci[j]] ) == node_tid.end()) << std::endl;
				ret = ( ret && ( sched_setscheduler( node_tid[node_dag.id_name_map[sci[j]] ] , SCHED_FIFO, &sp) ) );
				if (ret != 0)
					std::cerr << "WEIRD!!! Changing prio for SC with 0th node:" << sci[0] << "-" << j << " to " << prio << std::endl;
			}
			else
				std::cerr << "NO THREAD ID FOR " << node_dag.id_name_map[sci[j]] << std::endl;
		}
		return ret;
	}

	// checks if all nodes' tid/pid info has been received.
	bool DAGControllerBE::got_all_info()
	{
		if (dag_name.find("nav") != std::string::npos )
			return (node_tid.find("navc") != node_tid.end());
		else if (dag_name.find("ill") != std::string::npos)
		{
			bool ans = true;
			std::vector<std::string> node_ids {"2", "3", "6", "7"};
			for (int i = 0; i < node_ids.size(); i++)
				ans = ans && ( node_tid.find(node_ids[i]) != node_tid.end() );
			return ans;
		}
	}

	// Called by FE when tid/pid info is received for any node.
	void DAGControllerBE::recv_node_info(std::string node_name, int tid, int pid)
	{
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
						cpu_set_t set;
						CPU_ZERO(&set);
						CPU_SET(i, &set);
						int reta = sched_setaffinity( node_tid[node_dag.id_name_map[exec_order[i][j]] ], sizeof(set), &set );
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning core " << i << " to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << reta << ", cpucount: " << CPU_COUNT(&set) << std::endl;
						struct sched_param sp = { .sched_priority = 1,};
						int retp = sched_setscheduler( node_tid[node_dag.id_name_map[exec_order[i][j]] ] , SCHED_FIFO, &sp );
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning FIFO,prio=1 to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << retp << std::endl;
					}	
				}
			}
			else if (fifo_nc == 1)
			{
				// Assign cpu0 to all
				cpu_set_t set;
				CPU_ZERO(&set);
				CPU_SET(0, &set);
				
				for (auto const& x: node_dag.id_name_map)
				{
					int reta = sched_setaffinity( node_tid[ x.second ], sizeof(set), &set );
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
						cpu_set_t set;
						CPU_ZERO(&set);
						CPU_SET( offline_node_core_map[ node_dag.id_name_map[ exec_order[i][0] ] ] , &set);
						int reta = sched_setaffinity( node_tid[node_dag.id_name_map[exec_order[i][j]] ], sizeof(set), &set );
						std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << ", Assigning core " << offline_node_core_map[ node_dag.id_name_map[ exec_order[i][0] ] ] << " to node " << node_dag.id_name_map[exec_order[i][j]] << ", retval: " << reta << ", cpucount: " << CPU_COUNT(&set) << std::endl;
						
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
		int ret = pthread_setschedparam(this_thread, SCHED_OTHER, &params);
		// RR doesnt work cuz timeslice=100ms. Setting nice prio highest, will inc its priority for 1ms per period.
		/*
		struct sched_param sp = { .sched_priority = 1,};
		int ret = sched_setscheduler(0, SCHED_OTHER, &sp);
		*/
		printf("Monotime %f, realtime %f, retval for changing prio of dynamic_reoptimize_func THREAD: %i | tid: %i", get_monotime_now(), get_realtime_now(), ret, reoptimize_thread_id);

		while (true)
		{
			// 5s period
			std::this_thread::sleep_for (std::chrono::milliseconds(5000));
			// clear old data
			node_dag.clear_old_data(frac_var_count);
			// update compute times to be 75ile of recent data.
			node_dag.update_cis(node_ci_arr);
			try
			{
				// call compute_rt_solve
				double solve_start = get_monotime_now();
				all_frac_values = node_dag.compute_rt_solve();
				printf("Monotime %f, realtime %f, Mono-time taken to solve GP: %f \n", get_monotime_now(), get_realtime_now(), (get_monotime_now()-solve_start) );

				if (all_frac_values[0] > 0)
				{
					// get period map node_id -> set of f_ids.
					period_map = node_dag.period_map; 
					// update CC freq:
					// Update fractions.
					// For each NC subchain, fraction = 1.0/[multiply all_frac_vals[i] for all i in period_map[subchain]]
					double period = 0.0;

					// take a lock to change offline_fracs: [i.e. the handle_Sched thread might have to wait a little bit sometimes before getting timeout value.] 
					offline_fracs_mtx.lock();
					for (auto const& x : period_map)
					{
						double frac = 1.0;
						for (int i = 0; i < x.second.size(); i++)
							frac *= 1.0/all_frac_values[x.second[i]];
						printf("MonoTime: %f, RealTime: %f, In new soln, Fraction for node %s is %f \n", get_monotime_now(), get_realtime_now(), node_dag.id_name_map[x.first].c_str(), frac);
						// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << "In new soln, Fraction for node " << node_dag.id_name_map[x.first] << " is : " << frac << std::endl;
						period += node_dag.id_node_map[x.first].compute * frac;
						offline_fracs [ node_dag.id_name_map[x.first] ] = frac;
					}
					curr_cc_period = period;
					offline_fracs_mtx.unlock();

					printf("MonoTime: %f, RealTime: %f, In new soln, CC period is %f \n", get_monotime_now() , get_realtime_now(), period);
					// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << "In new soln, CC period is " << period << std::endl;
					std::string cmd = "rosrun dynamic_reconfigure dynparam set /shim_freq_node cc_freq " + std::to_string(1000.0/(period) ); // since period is in millisec.
					system(cmd.c_str());


				}
			}
			catch (int e)
			{
				printf("MonoTime: %f, RealTime: %f, EXCEPTION in trying to solve GP %i", get_monotime_now() , get_realtime_now(), e);
			}
		}
	
	}

	void DAGControllerBE::update_ci(std::string node_name, double ci)
	{
		node_ci_arr[node_name].push_back(ci);
	}

	// Returns sum ci of subchain ind, for current ci vals.
	double DAGControllerBE::get_sum_ci_ith(std::vector<int>& sci)
	{
		double tot = 0.0;
                for (int i = 0; i < sci.size(); i++)
                        tot += node_dag.id_node_map[ sci[i] ].compute;
                return tot;
	}

	// returns the timeout for ind subchain, for current ci,fi vals.
	double DAGControllerBE::get_timeout(std::vector<int>& sci)
	{
		double tot = get_sum_ci_ith(sci);
		tot *= offline_fracs[node_dag.id_name_map[ sci[0] ] ];

                return tot;
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
	// TODO: use per core period_counts for this!!
	void DAGControllerBE::checkTriggerExec(std::vector<int>& sci)
	{
		int id = sci[0];
                int ind_p = round(1.0/offline_fracs[node_dag.id_name_map[id] ]);
		std::string name = node_dag.id_name_map[id];
		if ( (ind_p == 1 || (total_period_count%ind_p == 1) ) ) // && (node_tid.find(name) != node_tid.end() ) ) // (ind > 0) && : Removing cuz CC now triggered by scheduler. 
		{
			printf("MonoTime: %f, RealTime: %f, About to trigger node: %s, frac: %i, period_count: %li \n", get_monotime_now(), get_realtime_now(), name.c_str(), ind_p, total_period_count);
		
			//Nov: We just want there to be a bool at the nodes: Never a trigger_count.
                	// This is because we always want the node to run exactly once whenever it wakes up, and hence only a bool is needed.	
			// frontend->trigger_node(name, reset_count[ind-1]);
			frontend->trigger_node(name, true);
			if ( (name.find("imu") != std::string::npos) && (dag_name.find("ill") != std::string::npos) )
				frontend->trigger_node("7", true);

			// reset_count[ind-1] = false;
			
			// For Illixr Dag, need to trigger TW along with Imu, cuz Imu is at fixed freq for now. TODO.
			
		}
	}
