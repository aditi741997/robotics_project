#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
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

#include <dag.h>

class sendSignal
{
	public:
		sendSignal (int pid) : node_pid( pid ) {}
		void operator()(const ros::TimerEvent& event)
		{
			ROS_INFO("Sending SIGUSR1 to pid %i", node_pid);
			kill(node_pid, SIGUSR1);
		}

private:
	int node_pid;
};

class DAGController
{
        ros::NodeHandle nh;
	// Store tid, pid for each node:
        std::map<std::string, int> node_pid;
        std::map<std::string, int> node_tid; // id of the thread which executes the main cb for a node. 
        // DAG data structure : to be used by the scheduling algorithm
	std::map<std::string, int> node_ci, node_fi;
        std::vector<ros::Subscriber> exec_start_subs;
	std::map<std::string, ros::Timer> exec_yield_timer; // we will only use 1 timer at a time in 1c scenario, but in multicore case, there can be multiple nodes running on different cores. 
	// std::vector<std::string> exec_order; // will be an output from the sched algo
	std::vector<std::vector<int> > exec_order; // vector of subchains.
	std::map<int, std::vector<int>> period_map;
	std::vector<int> all_frac_values;
	int curr_exec_index;

	ros::Subscriber critical_exec_end_sub;
	ros::Timer exec_prio_timer;

	DAG node_dag;

public:
        DAGController(int x, std::string dag_file)
        {
		ROS_INFO("Initializing DAGController class");
		node_dag = DAG(dag_file);
		
		clock_t cb_start_rt = clock();
		node_dag.fill_trigger_nodes();
		node_dag.assign_publishing_rates();
		node_dag.assign_src_rates();
		all_frac_values = node_dag.compute_rt_solve();
		
		period_map = node_dag.period_map;	
		
		double total_time = (double)(clock() - cb_start_rt)/CLOCKS_PER_SEC;
		std::cout << "TOTAL TIME TAKEN to do everything : " << total_time << std::endl;

		// We will use dag.period_map [node id -> set of frac vars] and dag.get_exec_order() 
		// Also compute_rt_solve returns the value of each frac. variable.
		exec_order = node_dag.get_exec_order();
		
                /* hard coding for testing : aug15
                node_ci.insert({"globalcmp", 205});
                node_ci.insert({"nncmp", 57});

                node_fi.insert({"globalcmp", 10});
                node_fi.insert({"nncmp", 4});

		exec_order.push_back("globalcmp");
		exec_order.push_back("nncmp");
		*/
	
		int last_node_cc_id = exec_order[0][ exec_order[0].size() - 1 ];
		critical_exec_end_sub = nh.subscribe<std_msgs::Header>("/exec_end_" + node_dag.id_name_map[last_node_cc_id], 1, &DAGController::critical_exec_end_cb, this, ros::TransportHints().tcpNoDelay());

		/*
                ROS_INFO("DAGController : Subscribe to 'exec_start' topics for ALL nodes");
                for(std::map<std::string,int>::iterator it = node_ci.begin(); it != node_ci.end(); ++it)
                {
                        std::string topic = "/exec_start_" + it->first;
			ROS_INFO("Subscribing to %s", topic.c_str());
			ros::Subscriber si = nh.subscribe<std_msgs::Header>(topic, 1, &DAGController::exec_start_cb, this, ros::TransportHints().tcpNoDelay());
			exec_start_subs.push_back(si);
                }
		*/
        }

	// TODO: Test - The actual scheduling code hasnt been tested since updated on Sept2 [after plugigng with the 1core Algo.]
	// TODO: Crit chain will be executed like RTC, i.e. scheduler pings Sensor when to start.

	void critical_exec_end_cb(const std_msgs::Header::ConstPtr& msg)
	{
		// when crit exec ends, always start with ind=0.
		int ind = 0;
		changePriority(ind);
		double timeout = get_timeout(ind);
		ROS_INFO("GOT exec_end msg. Making timer with to : %f", timeout);
		exec_prio_timer = nh.createTimer(ros::Duration(0.001*timeout), &DAGController::exec_nc, this, true);
	}

	void exec_nc(const ros::TimerEvent& event)
	{
		// probably dont need to do this for ind = last elem in crit_nc_exec_order. But no harm.
		int ind = (curr_exec_index+1)%(exec_order.size());
		changePriority(ind);
		double timeout = get_timeout(ind);
		if (ind == ( exec_order.size() - 1) )
			ROS_INFO("Dont need timer for last elem in exec_order");
		else
		{
			ROS_INFO("Making timer with to : %f", timeout);
			exec_prio_timer = nh.createTimer(ros::Duration(0.001*timeout), &DAGController::exec_nc, this, true);
		}
		
	}

	float get_timeout(int isc)
	{
		// add ci of all nodes in subchain isc.
		double tot = 0.0;
		for (int i = 0; i < exec_order[isc].size(); i++)
			tot += node_dag.id_node_map[exec_order[isc][i]].compute;

		// multiply by fractions in period_map[subchain] 
		std::vector<int>& frac_set = period_map[exec_order[isc][0]];
		for (int i = 0; i < frac_set.size(); i++)
			tot /= all_frac_values[frac_set[i]];
		return tot;
	}

	void changePriority(int ind)
	{
		curr_exec_index = ind;
		ROS_INFO("Changing priority : ind to be executed %i : name of 1st node in subchain : %s", ind, node_dag.id_name_map[exec_order[ind][0]].c_str());
                for (int i = 0; i < exec_order.size(); i++)
                {
			int ret = 7;
			// No need to change to tid instead of pid, pid=tid for main cb thread!!
			if (i == curr_exec_index)
				ret = changePrioritySubChain(i, 2);
			else
				ret = changePrioritySubChain(i, 1);
			// ret = sched_setscheduler(node_pid[exec_order[i]], SCHED_FIFO, &sp_other);
			ROS_INFO("Ret value %i for setting priority of subchain node %s, exec id : %i", ret, node_dag.id_name_map[exec_order[i][0]].c_str(), curr_exec_index);
                }
	}

	int changePrioritySubChain(int i, int prio)
	{
		bool ret = true;
		struct sched_param sp = { .sched_priority = prio,};
		for (int j = 0; j < exec_order[i].size(); j++)
			ret = ( (ret) && (sched_setscheduler( node_pid[node_dag.id_node_map[exec_order[i][j]].name] , SCHED_FIFO, &sp) ) );
		return ret;
	}

	void exec_start_cb(const std_msgs::Header::ConstPtr& msg)
        {
                // get node name : from topic? or msg?
                std::string node_name;
                int pid, tid;
                std::stringstream ss;
                ss << msg->frame_id;
                ss >> pid >> node_name >> tid;
                ROS_INFO("Got exec_start msg from node %s, pid : %i, tid : %i", node_name.c_str(), pid, tid);
                // add tid, pid if not already there :
                node_pid[node_name] = pid;
		node_tid[node_name] = tid;

		float timeout = node_ci[node_name]/(float)(node_fi[node_name]);
                ROS_INFO("Will send signal to node %s after time %f", node_name.c_str(), timeout);
		// exec_yield_timer[node_name] = nh.createTimer(ros::Duration(0.001*timeout), sendSignal(node_pid[node_name]), true);
                // send_signal(node_name);
        }

        void send_signal(std::string node_name)
        {
                // send sigusr1 to node after time timeout
                kill(node_pid[node_name], SIGUSR1);
        }

};

int main (int argc, char **argv)
{
        std::string node_name = "dagcontroller";
        ROS_INFO("Init node name %s", node_name.c_str());
        ros::init(argc, argv, node_name);
	std::string dag_fname = argv[1];
        DAGController dagc(0, "/home/ubuntu/catkin_ws/" + dag_fname + "_dag.txt");
        ros::spin();

        return 0;
}
