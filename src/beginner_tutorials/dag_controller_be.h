#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include<algorithm>
#include <set>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <atomic>
#include <mutex>

#include <dag_controller_fe.h>
#include <dag.h>

class DAGControllerBE
{
public:
	// Store tid, pid for each node:
        std::map<std::string, int> node_pid;
        std::map<std::string, int> node_tid; // id of the thread which executes the main cb for a node. 

        // DAG data structure : to be used by the scheduling algorithm
        std::map<std::string, int> node_ci, node_fi;

	std::map<std::string, boost::circular_buffer<double> > node_ci_arr;

        std::vector<std::vector<int> > exec_order; // Output from sched algo: vector of subchains.
        std::map<int, std::vector<int>> period_map;
        std::vector<int> all_frac_values;
        int curr_exec_index;

        // Oct: just for offline expts:
        std::map<std::string, double> offline_fracs;
	std::mutex offline_fracs_mtx;	

	bool offline_use_td; // if we're using the offline TD version. i.e. all nodes are TD.
	std::map<std::string, int> offline_node_core_map; // needed for assigning each node to the right core.
	int fifo_nc; // for testing Davare et al : denotes #cores for fifo, -1 o.w.
	std::map<std::string, int> fifo_prio; // for testing Davare et al

        long int total_period_count = 0; // increment each time we get a CC's end or at end of period.
        bool sched_started = false; // becomes true when we get tid for all nodes.
        std::vector<double> reset_count; // Nov: to reset counters of NC'.

        DAG node_dag;
	std::string dag_name;
	
	DAGControllerBE();
        DAGControllerBE(std::string dag_file, DAGControllerFE* fe, bool dyn_resolve, std::string use_td, std::string fifo, int f_mc, int f_mu, int f_nc, int f_np, int p_s, int p_lc, int p_lp);

	DAGControllerBE(const DAGControllerBE&) = delete;

	// FE will call this for each msg from the CC.
        void recv_critical_exec_end();

	void handle_noncritical_exec();

	void handle_noncritical_loop();

	void changePriority(int ind);

	int changePrioritySubChain(int ind, int prio);

	void recv_node_info(std::string node_name, int tid, int pid=0);

	std::string get_last_node_cc_name();
	void update_ci(std::string node_name, double ci);
private:
	void set_high_priority(std::string thread_name);
	double get_timeout(int ind);  
	double get_sum_ci_ith(int ind);
	bool got_all_info();
	void checkTriggerExec(int ind);

	boost::thread* timer_thread;
	void timer_thread_func(double timeout);
	std::atomic<bool> timer_thread_running;

	boost::thread handle_sched_thread;
	boost::mutex sched_thread_mutex;
	bool cc_end, ready_sched;
	boost::condition_variable cv_sched_thread;

	boost::thread* startup_thread; // triggers nodes until main scheduling starts
	void startup_trigger_func();

	void handle_sched_main();

	DAGControllerFE* frontend;
	
	boost::thread reoptimize_thread;
	bool dynamic_reoptimize;
	void dynamic_reoptimize_func(); // update the fraction of all nodes.
	int frac_var_count; // #vars make during fill_trigger, assign_publishing, assign_src.
};

