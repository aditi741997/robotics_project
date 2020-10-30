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
	std::vector<std::vector<int> > exec_order; // Output from sched algo: vector of subchains.
	std::map<int, std::vector<int>> period_map;
	std::vector<int> all_frac_values;
	int curr_exec_index;

	// Oct: just for offline expts:
	std::map<std::string, double> offline_fracs;

	ros::Subscriber critical_exec_end_sub;
	ros::WallTimer exec_prio_timer;

	DAG node_dag;

public:
        DAGController(int x, std::string dag_file)
        {
		ROS_INFO("Initializing DAGController class");
		// reads DAG from text file and sorts chains based on criticality. 
		node_dag = DAG(dag_file);
		
		clock_t cb_start_rt = clock();
		
		node_dag.fill_trigger_nodes();
		node_dag.assign_publishing_rates();
		node_dag.assign_src_rates();
		
		// Oct: dont need to solve for offline experiments.
		// all_frac_values = node_dag.compute_rt_solve();
		// period_map = node_dag.period_map;	
		
		double total_time = (double)(clock() - cb_start_rt)/CLOCKS_PER_SEC;
		std::cout << "TOTAL TIME TAKEN to do everything : " << total_time << std::endl;

		// We will use dag.period_map [node id -> set of frac vars] and dag.get_exec_order() 
		// Also compute_rt_solve returns the value of each frac. variable.
		// This gives us the order in which to execute the subchains in the DAG:
		// each elem in this array is a subchain (list of nodes).
		exec_order = node_dag.get_exec_order();
		
                /* hard coding for testing : aug15
                node_ci.insert({"globalcmp", 205});
                node_ci.insert({"nncmp", 57});

                node_fi.insert({"globalcmp", 10});
                node_fi.insert({"nncmp", 4});

		exec_order.push_back("globalcmp");
		exec_order.push_back("nncmp");
		*/

		// Oct: for offline : 
		// need to assign fraction to all NC nodes.
		offline_fracs["s"] = 1.0;
		offline_fracs["mapcb"] = 0.5;
		offline_fracs["mapupd"] = 0.125;
		offline_fracs["navc"] = 0.5;
		offline_fracs["navp"] = 0.125;
		// With wcet exec times, period for 1,2,4,2,4 : 154.25
		// [~collision once] period for 1,2,8,2,8 : 120.375
		// [more collisions?] period for 1,2,2,2,2 : 222ms.

		ROS_INFO("DAGController : Subscribe to 'exec_start' topics for ALL nodes, to get tid/pid.");
                for(std::map<std::string,int>::iterator it = node_dag.name_id_map.begin(); it != node_dag.name_id_map.end(); ++it)
                {
                        std::string topic = "/robot_0/exec_start_" + it->first;
			ROS_INFO("Subscribing to %s", topic.c_str());
			ros::Subscriber si = nh.subscribe<std_msgs::Header>(topic, 1, &DAGController::exec_start_cb, this, ros::TransportHints().tcpNoDelay());
			exec_start_subs.push_back(si);
                }
		
		int last_node_cc_id = exec_order[0][ exec_order[0].size() - 1 ];
		critical_exec_end_sub = nh.subscribe<std_msgs::Header>("/robot_0/exec_end_" + node_dag.id_name_map[last_node_cc_id], 1, &DAGController::critical_exec_end_cb, this, ros::TransportHints().tcpNoDelay());

        }

	// TODO: Test - The actual scheduling code hasnt been tested since updated on Sept2 [after plugigng with the 1core Algo.]
	// TODO: Crit chain will be executed like RTC, i.e. scheduler pings Sensor when to start.

	// Cb to handle end of critical chain exec. Need to start dynamic sched stuff:
	void critical_exec_end_cb(const std_msgs::Header::ConstPtr& msg)
	{
		// when crit exec ends, always start with ind=0.
		// Oct: shouldnt we start with ind=1, since ind=0 is the CC.
		int ind = 1;
		double timeout = get_timeout(ind);
		// Oct: Make timer with WallDuration so that it works fine even when simulation has speedup>1.
		if (node_tid.find("navc") != node_tid.end() )
		{
			ROS_WARN("GOT exec_end msg. Making timer for time : %f", timeout);
			changePriority(ind);
			exec_prio_timer = nh.createWallTimer(ros::WallDuration(0.001*timeout), &DAGController::exec_nc, this, true);
		}
		else
			ROS_WARN("GOT exec_end msg. NOT making a timer for now.");
	}

	void exec_nc(const ros::WallTimerEvent& event)
	{
		// Oct: Dont do anything if ind=0 : indicates waiting for CC's output.
		if (curr_exec_index > 0)
		{
			int ind = (curr_exec_index+1)%(exec_order.size());
			changePriority(ind);
			double timeout = get_timeout(ind);

			/*
			if (ind == ( exec_order.size() - 1) )
				ROS_INFO("Dont need timer for last elem in exec_order");
			else
			*/
			{
				ROS_INFO("Making timer with to : %f", timeout);
				exec_prio_timer = nh.createWallTimer(ros::WallDuration(0.001*timeout), &DAGController::exec_nc, this, true);
			}
		}
		else
			ROS_INFO("In exec_nc, curr_exec_index = 0. Not making a timer, Waiting for CC to finish.");
	}

	double get_sum_ci_ith(int isc)
	{
		double tot = 0.0;
                for (int i = 0; i < exec_order[isc].size(); i++)
                        tot += node_dag.id_node_map[exec_order[isc][i]].compute;
		return tot;
	}

	float get_timeout(int isc)
	{
		// add ci of all nodes in subchain isc.
		double tot = get_sum_ci_ith(isc);

		// Oct: commenting out the lines frac_Set and for loop just for the offline setup
		// multiply by fractions in period_map[subchain]. 
		// std::vector<int>& frac_set = period_map[exec_order[isc][0]];
		// for (int i = 0; i < frac_set.size(); i++)
			// tot /= all_frac_values[frac_set[i]];
		tot *= offline_fracs[node_dag.id_name_map[ exec_order[isc][0] ] ];

		return tot;
	}

	void changePriority(int ind)
	{
		curr_exec_index = ind;
		ROS_INFO("Changing priority : ind to be executed %i : name of 1st node in subchain : %s", ind, node_dag.id_name_map[exec_order[ind][0]].c_str());
                for (int i = 0; i < exec_order.size(); i++)
                {
			int ret = 7;
			if (i == curr_exec_index)
				ret = changePrioritySubChain(i, 2);
			else
				ret = changePrioritySubChain(i, 1);
			// ret = sched_setscheduler(node_pid[exec_order[i]], SCHED_FIFO, &sp_other);
			if (ret != 0)
				ROS_WARN("Weird: Ret value %i for setting priority of subchain node %s, ind to be exec : %i", ret, node_dag.id_name_map[exec_order[i][0]].c_str(), curr_exec_index);
                }
	}

	int changePrioritySubChain(int i, int prio)
	{
		bool ret = true;
		struct sched_param sp = { .sched_priority = prio,};
		// Oct: Using tid to change priorities, as each node is in its own thread, and not always in its own process.
		for (int j = 0; j < exec_order[i].size(); j++)
			ret = ( (ret) && (sched_setscheduler( node_tid[node_dag.id_node_map[exec_order[i][j]].name] , SCHED_FIFO, &sp) ) );
		return ret;
	}

	// This cb is just to get the tid/pid for all nodes.
	// Each node should publish a msg on this topic at start.
	void exec_start_cb(const std_msgs::Header::ConstPtr& msg)
        {
                // get node name : from topic? or msg?
                std::string node_name;
                int pid, tid;
                std::stringstream ss;
                ss << msg->frame_id;
                ss >> pid >> node_name >> tid;
                ROS_INFO("Got exec_start msg %s from node %s, pid : %i, tid : %i", msg->frame_id.c_str(), node_name.c_str(), pid, tid);
                // add tid, pid if not already there :
                node_pid[node_name] = pid;
		node_tid[node_name] = tid;

		// float timeout = node_ci[node_name]/(float)(node_fi[node_name]);
		// Oct: Do work for 1period, so as to avoid any other node to start before the CC.
		if (node_name.find("navc") != std::string::npos )
		{
			changePriority(0); // make prio CC = 2, all others = 1.
			ROS_WARN("GOT ALL nodes' tid, pid. Now will do dummy work for ~1period+eps.");
			float dummy_work_timeout = 0.0;
			//Oct: for offline, just use offline_fracs.
			for (int i = 0; i < exec_order.size(); i++)
			{
				dummy_work_timeout += get_sum_ci_ith(i) * offline_fracs[node_dag.id_name_map[ exec_order[i][0] ] ]; 
			}
			// do_sieve(dummy_work_timeout*0.001 + 0.002);
			ROS_WARN("Done with dummy sieve!!");
		}
		
        }

	double get_curr_time()
	{
		boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
                boost::chrono::system_clock::duration tse = now.time_since_epoch();
		unsigned long long ct = boost::chrono::duration_cast<boost::chrono::milliseconds>(tse).count() - (1603000000000);
                double start_time = ct / (double)(1000.0);
		return start_time;
	}

	void do_sieve(double t)
	{
		// run sieve for a large number, exit when time > t.
                double start_time = get_curr_time();
		ROS_WARN("DAGController starting dummy sieve!!! Timeout %f", t);

		int i, num = 1, primes = 0;
		int limit = 1000000;

        	while (num <= limit) {
            		i = 2;
			if ( (get_curr_time() - start_time) > t) return;

            		while (i <= num) {
                		if(num % i == 0)
                    			break;
                		if ( (get_curr_time() - start_time) > t) return;
				i++;
            		}
            		if (i == num)
                		primes++;
            		num++;
        	}

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
