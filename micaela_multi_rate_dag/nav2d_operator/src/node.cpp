#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav2d_operator/RobotOperator.h>
#include <numeric>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <sstream>
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

#include <cerrno>

using namespace ros;

ros::Publisher thread_exec_pub;

void print_arr(std::vector<double> m_arr, std::string m)
{
        int perc7 = 75;
    int percentile = 95;
    int perc0 = 90;
    int perc2 = 99;
    float perc3 = 99.9;
    float perc4 = 99.99;

        int l = m_arr.size();
    if (l > 0)
    {
        std::sort(m_arr.begin(), m_arr.end());
        double avg = (std::accumulate(m_arr.begin(), m_arr.end(), 0.0))/l;
        double med = m_arr[l/2];
        double perc = m_arr[(l*percentile)/100];

        ROS_ERROR("Mean %f, Median, %i tail of %s is %f %f , arr sz %i , %i : %f, %i : %f, %i : %f, %f : %f, %f : %f #", avg, percentile, m.c_str(), med, perc, l, perc7, m_arr[(l*perc7)/100], perc0, m_arr[(l*perc0)/100], perc2, m_arr[(l*perc2)/100], perc3, m_arr[(l*perc3)/100], perc4, m_arr[(l*perc4)/100]);
    }
}

double get_time_now()
{
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double time_now = ts.tv_sec + 1e-9*ts.tv_nsec;

        return time_now;
}

void spinner_work()
{
	int started = 0;
        // this is the thread that execs LC scan Callback.
        // Needs to be sched_ddl so LC can exec CB as soon as scan is received & LP can execute /cmd CB.
	int ci = 1;
	// period = 100 since LC, LP both are running at 100.
	int ddl = 100;
	int period = 100;

	struct sched_attr attr;
            attr.size = sizeof(attr);
            attr.sched_flags = 0;
	    attr.sched_policy = SCHED_DEADLINE;
         attr.sched_runtime = ci*1000*1000; // nanosec
        attr.sched_period = period*1000*1000;
         attr.sched_deadline = ddl*1000*1000;
         unsigned int flags = 0;
        int a = sched_setattr(0, &attr, flags);
	ROS_ERROR("Output of setting Operator's CBThread to DDL policy %i", a);

	if (a < 0)
		std::cerr << "sched_setattr for Operator's CBT FAILED!! error:" << std::strerror(errno) << std::endl;

	while (started < 3)
        {
                ROS_ERROR("Publishing node LC tid %i, pid %i to controller.", ::gettid(), ::getpid() );
                std_msgs::Header hdr;

                std::stringstream ss_e;
                ss_e << ::getpid() << " lc "  << ::gettid();
                hdr.frame_id = ss_e.str();

                ros::NodeHandle nh;
                thread_exec_pub = nh.advertise<std_msgs::Header>("/robot_0/exec_start_lc", 1, true);
                thread_exec_pub.publish(hdr);
                started += 1;
                ros::Duration(0.2).sleep();
        }
        ros::spin();
}

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n("~/");

	double frequency;
	n.param("frequency", frequency, 10.0);
	ROS_INFO("Operator will run at %.2f Hz.", frequency);

	RobotOperator robOp;
	
	std::thread spinner_thr(spinner_work);

	double last_lp_out;
        long int total_lp_ct = 0;
        std::vector<double> tput_lp;

	WallRate loopRate(frequency);

	int ci = 2.3;
	int period = 100.0;
	int ddl = 2.7;
	struct sched_attr attr;
            attr.size = sizeof(attr);
            attr.sched_flags = 0;
            attr.sched_policy = SCHED_DEADLINE;
         attr.sched_runtime = ci*1000*1000; // nanosec
        attr.sched_period = period*1000*1000;
         attr.sched_deadline = ddl*1000*1000;
         unsigned int flags = 0;
        int a = sched_setattr(0, &attr, flags);
	ROS_ERROR("Output of setting Operator's LP to DDL policy %i", a);
        if (a < 0)
                std::cerr << "sched_setattr for Operator's LP FAILED!! error:" << std::strerror(errno) << std::endl;

	while(ok())
	{
		robOp.executeCommand();
		
		double time_now = get_time_now();
                total_lp_ct += 1;
                if (total_lp_ct > 1)
                        tput_lp.push_back(time_now - last_lp_out);
                last_lp_out = time_now;

                if (tput_lp.size()%500 == 177)
                        print_arr(tput_lp, "LP TPUT!!!");
		
		loopRate.sleep();
		// if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
			// ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());
	}
	spinner_thr.join();
	return 0;	
}
