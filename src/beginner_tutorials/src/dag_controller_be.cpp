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

#include <dag_controller_be.h>

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

DAGControllerBE::DAGControllerBE()
	{
	}

// todo:Later: maybe a lock for reset_count.
// use_td: Whether nodes should be timer driven
// fifo: denotes if using fifo with static priorities & the #cores for the same
// if fifo, f_nodeName params will be used as priority,
// also, for 3cores with navigation: S,LC,LP | MCB,MU | NP,NC. 
DAGControllerBE::DAGControllerBE(std::string dag_file, DAGControllerFE* fe, std::string use_td, std::string fifo, int f_mc, int f_mu, int f_nc, int f_np, int p_s, int p_lc, int p_lp)
	{
		// Note that the last 4 inputs are just for the offline stage.
		node_dag = DAG(dag_file);
		dag_name = dag_file;

		node_dag.fill_trigger_nodes();
                node_dag.assign_publishing_rates();
                node_dag.assign_src_rates();
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
		else if (dag_name.find("ill") != std::string::npos)
		{
			offline_fracs["imu"] = 1.0;
			// just need render and CS fractions.
			offline_fracs["4"] = 1.0/f_mc;
			offline_fracs["0"] = 1.0/f_mu;
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

		for (int i = 1; i < exec_order.size(); i++)
			reset_count.push_back(false);
	
		curr_exec_index = 0;
		ready_sched = false;

		frontend = fe;
	}

	// FE will call this for each msg from the CC.
	void DAGControllerBE::recv_critical_exec_end()
	{
                total_period_count += 1;

		// Nov25: No need to do anything if we're using TD in offline version.
		if (!offline_use_td)
		{
			if (got_all_info())
			{
				if (!sched_started)
				{
					std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " $$$$$ DAGControllerBE:: STARTING SCHEDULING!!! \n";
					sched_started = true;
					for (int i = 0; i < reset_count.size(); i++)
						reset_count[i] = true;
					total_period_count = 1;

					// Make the thread that handles scheduling.
					handle_sched_thread = boost::thread(&DAGControllerBE::handle_noncritical_loop, this);
					std::this_thread::sleep_for (std::chrono::milliseconds(2));
				}
			
				// Notify the thread.	
				boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
				cc_end = true;
				ready_sched = true;
				printf("ABOUT to NOTIFY the main thread, cc_End: TRUE! \n");
				cv_sched_thread.notify_all();
			}
			else
			{
				std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " | GOT exec_end msg. NOT making a timer,Will trigger nodes" << std::endl;
				for (int i = 1; i < exec_order.size(); i++)
					checkTriggerExec(i);
					//frontend->trigger_node(node_dag.id_name_map[exec_order[i][0]], false);
			}
		}
			
	}

	// This will be called once for each NC node, in each period : 
	// makes another timer, changes priorities, triggers nodes if needed.
	void DAGControllerBE::handle_noncritical_exec()
	{
	}

	std::string DAGControllerBE::get_last_node_cc_name()
	{
		int id = exec_order[0][exec_order[0].size()-1];
		return node_dag.id_name_map[id];
	}

	void DAGControllerBE::handle_noncritical_loop()
	{
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
				changePriority(curr_exec_index);
				double timeout = get_timeout(curr_exec_index);
				pthread_cancel(timer_thread.native_handle());
				timer_thread = boost::thread(&DAGControllerBE::timer_thread_func, this, timeout);
			}
			
			cc_end = false;
			ready_sched = false;
		}
	}

	// Changes subchain[ind] prio = 2, all others = 1.
	void DAGControllerBE::changePriority(int ind)
	{
		// curr_exec_index = ind;
		for (int i = 0; i < exec_order.size(); i++)
                {
                        int ret = 7;
			if (i == curr_exec_index)
                                ret = changePrioritySubChain(i, 2);
                        else
                                ret = changePrioritySubChain(i, 1);
			if (ret != 0)
				std::cerr << "WEIRD, Ret value " << ret << " for setting prio of SC node " << node_dag.id_name_map[exec_order[i][0]] << " curr_exec_index: " << curr_exec_index << std::endl;
                                // ROS_WARN("Weird: Ret value %i for setting priority of subchain node %s, ind to be exec : %i", ret, node_dag.id_name_map[exec_order[i][0]].c_str(), curr_exec_index);
		}
		checkTriggerExec(ind);
	}

	int DAGControllerBE::changePrioritySubChain(int ind, int prio)
	{
		bool ret = true;
		struct sched_param sp = { .sched_priority = prio,};
		// Nov: Change priorities in reverse order, so that if any later node in subchain is preempted,
		// it executes first. [since it'll be ahead in the list of same prio.]
		for (int j = exec_order[ind].size()-1 ; j >= 0; j-- )
		{
			std::cerr << "Changing prio for node exec_order" << ind << "-" << j << " to " << prio << std::endl;
			ret = ( ret && ( sched_setscheduler( node_tid[node_dag.id_name_map[exec_order[ind][j]] ] , SCHED_FIFO, &sp) ) );
		}
		return ret;
	}

	// checks if all nodes' tid/pid info has been received.
	bool DAGControllerBE::got_all_info()
	{
		if (dag_name.find("nav") != std::string::npos )
			return (node_tid.find("navc") != node_tid.end());
		else
			return true; // modify for illixr
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
	}

	// Returns sum ci of subchain ind, for current ci vals.
	double DAGControllerBE::get_sum_ci_ith(int isc)
	{
		double tot = 0.0;
                for (int i = 0; i < exec_order[isc].size(); i++)
                        tot += node_dag.id_node_map[exec_order[isc][i]].compute;
                return tot;
	}

	// returns the timeout for ind subchain, for current ci,fi vals.
	double DAGControllerBE::get_timeout(int ind)
	{
		double tot = get_sum_ci_ith(ind);
		tot *= offline_fracs[node_dag.id_name_map[ exec_order[ind][0] ] ];

                return tot;
	}

	// Func to just sleep for timeout millisec and then notify cv.
	void DAGControllerBE::timer_thread_func(double timeout)
	{
		printf("MonoTime: %f, RealTime: %f, making TIMER for to %f \n", get_monotime_now(), get_realtime_now(), timeout);
		// std::cout << "MonoTime: " << get_monotime_now() << " RealTime: " << get_realtime_now() << " | making TIMER for to " << timeout << std::endl;
		long to = timeout*1000;
		std::this_thread::sleep_for (std::chrono::microseconds(to));
		boost::unique_lock<boost::mutex> lock(sched_thread_mutex);
		// cc_end = false;
		ready_sched = true;
		printf("MonoTime %f, RealTime %f ABOUT to NOTIFY the main thread, cc_end is false! \n");
		cv_sched_thread.notify_all();
	}

	// checks if SC ind needs to be triggered, if yes, calls the FE func.
	void DAGControllerBE::checkTriggerExec(int ind)
	{
		int id = exec_order[ind][0];
                int ind_p = round(1.0/offline_fracs[node_dag.id_name_map[id] ]);
		std::string name = node_dag.id_name_map[id];
		if ( (ind > 0) && (ind_p == 1 || (total_period_count%ind_p == 1) ) && (node_tid.find(name) != node_tid.end() ) )
		{
			printf("MonoTime: %f, RealTime: %f, About to trigger node: %s, frac: %i, period_count: %i \n", get_monotime_now(), get_realtime_now(), name.c_str(), ind_p, total_period_count);
		
			//Nov: We just want there to be a bool at the nodes: Never a trigger_count.
                	// This is because we always want the node to run exactly once whenever it wakes up, and hence only a bool is needed.	
			// frontend->trigger_node(name, reset_count[ind-1]);
			frontend->trigger_node(name, true);
			reset_count[ind-1] = false;
		}
	}
