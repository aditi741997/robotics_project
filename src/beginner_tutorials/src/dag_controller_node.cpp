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
	std::vector<std::string> exec_order; // will be an output from the sched algo
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
		node_dag.compute_rt();	
		double total_time = (double)(clock() - cb_start_rt)/CLOCKS_PER_SEC;
		std::cout << "TOTAL TIME TAKEN to do everything : " << total_time << std::endl;

                // hard coding for testing : aug15
                node_ci.insert({"globalcmp", 205});
                node_ci.insert({"nncmp", 57});

                node_fi.insert({"globalcmp", 10});
                node_fi.insert({"nncmp", 4});

		exec_order.push_back("globalcmp");
		exec_order.push_back("nncmp");

		critical_exec_end_sub = nh.subscribe<std_msgs::Header>("/exec_end_lplan", 1, &DAGController::critical_exec_end_cb, this, ros::TransportHints().tcpNoDelay());

                ROS_INFO("DAGController : Subscribe to 'exec_start' topics for ALL nodes");
                for(std::map<std::string,int>::iterator it = node_ci.begin(); it != node_ci.end(); ++it)
                {
                        std::string topic = "/exec_start_" + it->first;
			ROS_INFO("Subscribing to %s", topic.c_str());
			ros::Subscriber si = nh.subscribe<std_msgs::Header>(topic, 1, &DAGController::exec_start_cb, this, ros::TransportHints().tcpNoDelay());
			exec_start_subs.push_back(si);
                }
        }

	void critical_exec_end_cb(const std_msgs::Header::ConstPtr& msg)
	{
		// when crit exec ends, always start with ind=0.
		int ind = 0;
		changePriority(ind);
		float timeout = node_ci[exec_order[ind]]/(float)(node_fi[exec_order[ind]]);
		ROS_INFO("GOT exec_end msg. Making timer with to : %f", timeout);
		exec_prio_timer = nh.createTimer(ros::Duration(0.001*timeout), &DAGController::exec_nc, this, true);
	}

	void exec_nc(const ros::TimerEvent& event)
	{
		// probably dont need to do this for ind = last elem in crit_nc_exec_order. But no harm.
		int ind = (curr_exec_index+1)%(exec_order.size());
		changePriority(ind);
		float timeout = node_ci[exec_order[ind]]/(float)(node_fi[exec_order[ind]]);
		if (ind == ( exec_order.size() - 1) )
			ROS_INFO("Dont need timer for last elem in exec_order");
		else
		{
			ROS_INFO("Making timer with to : %f", timeout);
			exec_prio_timer = nh.createTimer(ros::Duration(0.001*timeout), &DAGController::exec_nc, this, true);
		}
		
	}

	void changePriority(int ind)
	{
		curr_exec_index = ind;
		ROS_INFO("Changing priority : ind to be exec %i : name : %s", ind, exec_order[ind].c_str());
                for (int i = 0; i < exec_order.size(); i++)
                {
			int ret = 7;
			struct sched_param sp_exec = { .sched_priority = 2,};
			struct sched_param sp_other = { .sched_priority = 1,};
			// No need to change to tid instead of pid, pid=tid for main cb thread!!
			if (i == curr_exec_index)
				ret = sched_setscheduler(node_pid[exec_order[i]], SCHED_FIFO, &sp_exec);
			else
				ret = sched_setscheduler(node_pid[exec_order[i]], SCHED_FIFO, &sp_other);
			ROS_INFO("Ret value %i for setting priority of node %s, exec id : %i", ret, exec_order[i].c_str(), curr_exec_index);
                }
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
