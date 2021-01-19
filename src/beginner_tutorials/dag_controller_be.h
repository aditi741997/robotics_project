#define _GNU_SOURCE
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
#include <dag_mc.h>

#include <sched.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>
#include <pthread.h>

#define gettid() syscall(__NR_gettid)
#define SCHED_OTHER 0

/* XXX use the proper syscall numbers */
#ifdef __x86_64__
#define __NR_sched_setattr 314
#define __NR_sched_getattr 315
#endif
#ifdef __i386__
#define __NR_sched_setattr 351
#define __NR_sched_getattr 352
#endif
#ifdef __arm__
#define __NR_sched_setattr 380
#define __NR_sched_getattr 381
#endif

struct sched_attr {
	__u32 size;
	__u32 sched_policy;
	__u64 sched_flags;
	/* SCHED_NORMAL, SCHED_BATCH */
	__s32 sched_nice;
	/* SCHED_FIFO, SCHED_RR */
	__u32 sched_priority;
	/* SCHED_DEADLINE (nsec) */
	__u64 sched_runtime;
	__u64 sched_deadline;
	__u64 sched_period;
};

int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags);

class DAGControllerBE
{
public:
	// Store tid, pid for each node:
        std::map<std::string, int> node_pid;
        std::map<std::string, int> node_tid; // id of the thread which executes the main cb for a node. 
	
	// includes under_the_hood threads such as PMT,CBT associated with each node.
	std::map<std::string, std::set<int> > node_extra_tids; 
	std::map<std::string, int> node_curr_prio;

	std::ofstream trigger_log;
	std::map<std::string, double> last_trig_ts;

	std::ofstream cc_completion_log;

        // DAG data structure : to be used by the scheduling algorithm
        std::map<std::string, int> node_ci, node_fi;

	std::map<std::string, boost::circular_buffer<double> > node_ci_arr;

        std::vector<std::vector<int> > exec_order; // Output from sched algo: vector of subchains.
	double curr_cc_period = 0.0;
        std::map<int, std::vector<int>> period_map;
        std::vector<int> all_frac_values;
        // int curr_exec_index;

        // Oct: just for offline expts:
        std::map<std::string, double> offline_fracs;
	std::mutex offline_fracs_mtx;	

	bool offline_use_td; // if we're using the offline TD version. i.e. all nodes are TD.
	std::map<std::string, int> offline_node_core_map; // needed for assigning each node to the right core.
	int fifo_nc; // for testing Davare et al : denotes #cores for fifo, -1 o.w.
	std::map<std::string, int> fifo_prio; // for testing Davare et al

        long int total_period_count = 0; // increment each time we get a CC's end or at end of period.
	std::atomic<bool> sched_started = ATOMIC_VAR_INIT(false); // becomes true when we get tid for all nodes.
        std::vector<double> reset_count; // Nov: to reset counters of NC'.

        DAG node_dag;
	MultiCoreApproxSolver multi_core_solver;
	DAGMultiCore node_dag_mc;
	std::string dag_name;
	int num_cores;

	DAGControllerBE();
        DAGControllerBE(std::string dag_file, DAGControllerFE* fe, bool dyn_resolve, std::string use_td, std::string fifo, int f_mc, int f_mu, int f_nc, int f_np, int p_s, int p_lc, int p_lp, int numc);

	DAGControllerBE(const DAGControllerBE&) = delete;
	~DAGControllerBE();


	void start();

	// FE will call this for each msg from the CC.
        void recv_critical_exec_end();

	void handle_noncritical_exec();

	// void changePriority(int ind); change ind of iexec_order to p2, others to p1
	void changePriority(std::vector<std::vector<int>>& iexec_order, int ind, int core_id = 0);

	// int changePrioritySubChain(int ind, int prio);
	int changePrioritySubChain(std::vector<int>& sc, int prio);


	void recv_node_info(std::string node_name, int tid, int pid=0);

	std::string get_last_node_cc_name();
	void update_ci(std::string node_name, double ci);

	// Helper functions:
	bool changePriorityThread(std::string nname, int tid, int prio);
	int changeAffinityThread(std::string nname, int tid, std::vector<int> cores);

private:
	void set_high_priority(std::string thread_name, int prio, int tid);

	// double get_timeout(int ind); New version for multi-core:  
	double get_timeout(std::vector<int>& sci, std::vector<int>& cores);
	double get_sum_ci_ith(std::vector<int>& sci);
	double get_max_ci_ith(std::vector<int>& sci);
	bool got_all_info();
	
	// void checkTriggerExec(int ind);
	void checkTriggerExec(std::vector<int>& sci, int core_id = 0);

	boost::thread* timer_thread;
	void timer_thread_func(double timeout);
	std::atomic<bool> timer_thread_running;

	boost::thread handle_sched_thread;
	boost::mutex sched_thread_mutex;
	std::atomic<bool> cc_end, ready_sched;
	boost::condition_variable cv_sched_thread; // this is just for the core with CC in it.

	// For multi-core scheduling:
	std::map<int, boost::thread> per_core_sched_threads;
	std::map<int, long int> per_core_period_counts;
	std::map<int, double> per_core_period; // HP of each core sched.
	// std::map<int, boost::condition_variable> cv_;
	// pass core # to each function to easily access period ct etc.

	boost::thread* startup_thread; // triggers nodes until main scheduling starts
	void startup_trigger_func();

	void handle_sched_main(std::vector<int> core_id);
	std::atomic<bool> shutdown_scheduler;

	DAGControllerFE* frontend;
	
	boost::thread reoptimize_thread;
	bool dynamic_reoptimize;
	void dynamic_reoptimize_func(); // update the fraction of all nodes.
	int frac_var_count; // #vars make during fill_trigger, assign_publishing, assign_src.
	int reoptimize_thread_id;
	pthread_t reoptimize_thread_p;	
};

