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

DAGControllerBE::DAGControllerBE()
	{
	}

DAGControllerBE::DAGControllerBE(std::string dag_file, int f_mc, int f_mu, int f_nc, int f_np)
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
		if (dag_name.find("nav") != std::string::npos)
		{
			offline_fracs["s"] = 1.0;
			offline_fracs["mapcb"] = 1.0/f_mc;
			offline_fracs["mapupd"] = 1.0/f_mu;
			offline_fracs["navc"] = 1.0/f_nc;
			offline_fracs["navp"] = 1.0/f_np;
		}
		
		for (int i = 1; i < exec_order.size(); i++)
			reset_count.push_back(false);
	}

	// FE will call this for each msg from the CC.
	void DAGControllerBE::recv_critical_exec_end()
	{
		int ind = 1;
		double to = get_timeout(ind);
                total_period_count += 1;

		if (got_all_info())
		{
			if (!sched_started)
			{
				std::cout << "$$$$$ DAGControllerBE:: STARTING SCHEDULING!!! \n";
				sched_started = true;
				for (int i = 0; i < reset_count.size(); i++)
					reset_count[i] = true;
				total_period_count = 1;
			}
			// ROS_WARN("GOT exec_end msg. Making timer for time : %f", timeout);
			changePriority(ind);
			// todo: timer...
		}
		else
		{
			// todo
		}
			
	}

	// This will be called once for each NC node, in each period : 
	// makes another timer, changes priorities, triggers nodes if needed.
	void DAGControllerBE::handle_noncritical_exec()
	{
	}

	// Changes subchain[ind] prio = 2, all others = 1.
	void DAGControllerBE::changePriority(int ind)
	{
		curr_exec_index = ind;
		for (int i = 0; i < exec_order.size(); i++)
                {
                        int ret = 7;
			if (i == curr_exec_index)
                                ret = changePrioritySubChain(i, 2);
                        else
                                ret = changePrioritySubChain(i, 1);
			if (ret != 0)
				0;
                                // ROS_WARN("Weird: Ret value %i for setting priority of subchain node %s, ind to be exec : %i", ret, node_dag.id_name_map[exec_order[i][0]].c_str(), curr_exec_index);
		}
		checkTriggerExec(ind);
	}

int DAGControllerBE::changePrioritySubChain(int ind, int prio)
	{
		return 0;
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
	void DAGControllerBE::recv_node_info()
	{
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

	// checks if SC ind needs to be triggered, if yes, calls the FE func.
	void DAGControllerBE::checkTriggerExec(int ind)
	{
		// todo
	}
