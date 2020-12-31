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
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <stdlib.h> 
#include <netinet/in.h> 

#include <dag_controller_be.h>

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

class DAGController: public DAGControllerFE
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

	std::vector<double> reset_count;
	ros::Subscriber critical_exec_end_sub;
	std::map<std::string, ros::Publisher> trigger_nc_exec; // may want to trigger CC's first node too, in the future, for a pure RTC thing.
	int nav_client_fd, srv_fd, port_no; 
	bool nav_socket_connected; // for IPC with the Navigator node. 	
	boost::thread socket_conn_thread; // thread to accept client cnn.

	std::map<std::string, ros::Subscriber> exec_end_subs; // to get ci estimates from all nodes.
	std::string last_node_cc_name;

	long int total_period_count = 0; // increment each time we get a CC's end or at end of period.
	bool sched_started = false; // becomes true when we get tid for all nodes.
	ros::WallTimer exec_prio_timer;

	DAG node_dag; // Nov: not used now.
	DAGControllerBE* controller;

public:
        DAGController(int x, std::string dag_file, bool dyn_opt, std::string use_td, std::string fifo, int f_mc, int f_mu, int f_nc, int f_np, int p_s, int p_lc, int p_lp)
        {
		ROS_INFO("Initializing DAGController class, params: dyn_opt: %i, use_td: %s, fifo: %s, fmc: %i, fmu %i, fnc %i, fnp %i, ps %i, plc %i, plp %i", dyn_opt, use_td.c_str(), fifo.c_str(), f_mc, f_mu, f_nc, f_np, p_s, p_lc, p_lp);
		controller = new DAGControllerBE(dag_file, this, dyn_opt, use_td, fifo, f_mc, f_mu, f_nc, f_np, p_s, p_lc, p_lp);

		ROS_INFO("DAGController : Subscribe to 'exec_start' topics for ALL nodes, to get tid/pid.");
		last_node_cc_name = controller->get_last_node_cc_name();
		critical_exec_end_sub = nh.subscribe<std_msgs::Header>("/robot_0/exec_end_" + last_node_cc_name, 1, &DAGController::critical_exec_end_cb, this, ros::TransportHints().tcpNoDelay());
                exec_end_subs[last_node_cc_name] = critical_exec_end_sub;
		
		for (std::map<std::string,int>::iterator it = controller->node_dag.name_id_map.begin(); it != controller->node_dag.name_id_map.end(); ++it)
                {
                        std::string topic = "/robot_0/exec_start_" + it->first;
			ROS_INFO("Subscribing to %s", topic.c_str());
			ros::Subscriber si = nh.subscribe<std_msgs::Header>(topic, 1, &DAGController::exec_start_cb, this, ros::TransportHints().tcpNoDelay());
			exec_start_subs.push_back(si);

			std::string pubtopic = "/robot_0/trigger_exec_" + it->first;
			ROS_INFO("Publishing TRIGGER stuff to %s", pubtopic.c_str());
			ros::Publisher tpub = nh.advertise <std_msgs::Header> (pubtopic, 1, false);
			trigger_nc_exec[it->first] = (tpub);

			if ( exec_end_subs.find( it->first ) == exec_end_subs.end() )
			{
				std::string end_subtopic = "/robot_0/exec_end_" + it->first;
				ROS_INFO("Subscribing to %s to get exec_end msgs from ALL nodes", end_subtopic.c_str());
				ros::Subscriber esi = nh.subscribe<std_msgs::Header>( end_subtopic, 1, &DAGController::exec_end_cb, this, ros::TransportHints().tcpNoDelay() );
				exec_end_subs[it->first] = esi;
			}	
		}
		
		// Nov: for IPC with navigator node:
		srv_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (srv_fd < 0) ROS_ERROR("Socket:: fd %i < 0", srv_fd);

		int opt = 1; 
		if ( setsockopt(srv_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt)) )
			ROS_ERROR("DAGControllerFE::Socket::  setsockopt failed!!");
		
		struct sockaddr_in saddr;
		memset(&saddr, 0, sizeof(saddr));
		saddr.sin_family = AF_INET;
		saddr.sin_addr.s_addr = INADDR_ANY;
		port_no = 7727;
		saddr.sin_port = htons(port_no);

		if (bind(srv_fd, (struct sockaddr *) &saddr, sizeof(saddr)) < 0) ROS_ERROR("Socket:: bind FAILED!!");

		// A thread to accept client's connection when it arrives.
		nav_socket_connected = false;
		socket_conn_thread = boost::thread(&DAGController::socket_conn, this);
        }

	~DAGController()
	{
		ROS_WARN("In DAGController Decontructor.");
		close(nav_client_fd);
	}

	void socket_conn()
	{
		if (listen(srv_fd, 2) < 0) ROS_ERROR("Socket:: LISTEN failed!!!");
		ROS_INFO("DAGC: Socket:: Listening on port %i for clients...\n", port_no);

		// while (true)
		{
			struct sockaddr_in caddr;
			socklen_t len = sizeof(caddr);
		
			// accept is a blocking call : does nothing till someone connects.	
			nav_client_fd = accept(srv_fd, (struct sockaddr*) &caddr, &len);
			if (nav_client_fd < 0) ROS_ERROR("DAGC: Socket:: nav_client_fd is <0!!!!");
			else nav_socket_connected = true;
			ROS_INFO("DAGC: Socket:: Done with socket connection Accept. !!!!!!");
		}
	}

	// This is called when any of the nodes finishes exec (except last node of CC)
	// Used to update ci estimates of all nodes.
	void exec_end_cb(const std_msgs::Header::ConstPtr& msg)
	{
		std::stringstream ss;
		ss << msg->frame_id;
		double ci;
		std::string n_name;
		ss >> ci >> n_name;
		controller->update_ci(n_name, ci);
	}


	// Cb to handle end of critical chain exec. Need to start dynamic sched stuff:
	void critical_exec_end_cb(const std_msgs::Header::ConstPtr& msg)
	{
		// ROS_WARN("GOT exec_end msg from cc!!! Calling the BE.");
		controller->recv_critical_exec_end();
		controller->update_ci(last_node_cc_name, stod(msg->frame_id) );
		/*
		// Oct: shouldnt we start with ind=1, since ind=0 is the CC.
		int ind = 1;
		double timeout = get_timeout(ind);
		total_period_count += 1;
		// Oct: Make timer with WallDuration so that it works fine even when simulation has speedup>1.
		if (node_tid.find("navc") != node_tid.end() )
		{
			if (!sched_started)
			{
				ROS_WARN("$$$$$ SCHEDULING Starts now!, Setting reset_count to TRUE");
				sched_started = true;
				for (int i = 0; i < reset_count.size(); i++)
					reset_count[i] = true; // reset trigger counts for all NCs.
				total_period_count = 1; // this is the 1st msg after getting all tids.
			}
			ROS_WARN("GOT exec_end msg. Making timer for time : %f", timeout);
			// TODO: might need to add trigger_ct=1 for the case when scheduling just starts: so as to remove the current trigger_ct
			// stored in the nodes.
			changePriority(ind);
			exec_prio_timer = nh.createWallTimer(ros::WallDuration(0.001*timeout), &DAGController::exec_nc, this, true);
		}
		else
		{
			ROS_WARN("GOT exec_end msg. NOT making a timer for now. Will trigger nodes if needed.");
			for (int i = 1; i < exec_order.size(); i++)
				trigger_exec(i);
		}
		*/

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

	// change priorities so that ind is the NC to run.
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
		
		// Send a trigger to the 1st node of ind's subchain, if count%(1/fi) == 1.
			trigger_exec(ind);
	}

	void trigger_exec(int ind)
	{
		int id = exec_order[ind][0];
                int ind_p = round(1.0/offline_fracs[node_dag.id_name_map[id] ]);
		std::string name = node_dag.id_name_map[id];
		// Nov: trigger if count divisible by period AND node has started i.e. tid is there.
		if ( (ind > 0) && (total_period_count%ind_p == 1) && (node_tid.find(name) != node_tid.end() ) )
		{
			ROS_WARN("About to trigger exec for node %i, %s 1/f is %i, count: %i", ind, name.c_str(), ind_p, total_period_count);
			std_msgs::Header hdr;
			hdr.frame_id = name;
			if (reset_count[ind-1])
			{
				hdr.frame_id += " RESETCOUNT";
				ROS_WARN("RESETTING trigger_ct for NC %s", name.c_str());
				reset_count[ind-1] = false; 
			}
			hdr.frame_id += "\n";
			if (name.find("nav") == std::string::npos)
				trigger_nc_exec[name].publish(hdr); // TODO:Later: ind-1 cuz CC does not have a trigger, so ind ka triggerPub is at ind-1.
			else
			{
				// Use socket here.
				if (nav_socket_connected)
				{
					char msg[1+hdr.frame_id.length()];
					strcpy(msg, hdr.frame_id.c_str());
					send(nav_client_fd, msg, strlen(msg), 0);
				}
				else
					ROS_ERROR("nav_socket_connected IS false!!! Nav-DAGController SOCKET NOT connected!!");
			}
		}
	}

	// this func will be called by BE to trigger NC nodes:
	void trigger_node(std::string name, bool reset)
	{
		std_msgs::Header hdr;
                hdr.frame_id = name;
		if (reset)
		{
			hdr.frame_id += " RESETCOUNT";
                        // ROS_WARN("RESETTING trigger_ct for NC %s", name.c_str());
		}
		hdr.frame_id += "\n";
                if (name.find("nav") == std::string::npos)
                	trigger_nc_exec[name].publish(hdr);
		else
		{
			if (nav_socket_connected)
                        {
				char msg[1+hdr.frame_id.length()];
                                strcpy(msg, hdr.frame_id.c_str());
                                send(nav_client_fd, msg, strlen(msg), 0);
			}
			else
				ROS_ERROR("nav_socket_connected IS false!!! Nav-DAGController SOCKET NOT connected!!");
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

		controller->recv_node_info(node_name, tid, pid);

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
	std::string use_td = argv[2];
	std::string fifo = argv[3];
	// for offline:
	int f_mc = atoi(argv[4]);
	int f_mu = atoi(argv[5]);
	int f_nc = atoi(argv[6]);
	int f_np = atoi(argv[7]);
	int p_s, p_lc, p_lp;
	if (fifo.find("yes") != std::string::npos)
	{
		p_s = atoi(argv[8]);
		p_lc = atoi(argv[9]);
		p_lp = atoi(argv[10]);	
	}
	bool dyn_opt = (atoi(argv[11]) == 1) ;
        DAGController dagc(0, "/home/ubuntu/catkin_ws/" + dag_fname + "_dag.txt", dyn_opt, use_td, fifo, f_mc, f_mu, f_nc, f_np, p_s, p_lc, p_lp);
        ros::spin();

        return 0;
}
