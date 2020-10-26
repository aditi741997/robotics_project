#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav2d_operator/RobotOperator.h>

#include <mutex>
#include <condition_variable>
#include <thread>

#include <sstream>
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

using namespace ros;

int started = 0;
ros::Publisher thread_exec_pub;

void spinner_work()
{
	// this is the thread that execs LC stuff
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
	NodeHandle n;

	double frequency;
        n.param("frequency", frequency, 10.0);
        ROS_ERROR("NAV2D: RobotOperator %.2f Hz input frequency param.", frequency);

        std::mutex mutex_robot_op_lp;
        std::condition_variable cv_robot_op_lp;

        RobotOperator robOp (&cv_robot_op_lp);

	Rate loopRate(frequency);

	ros::Publisher cc_end_pub = n.advertise<std_msgs::Header>("/robot_0/exec_end_lp", 1, true);

	// this is the thread that executes LP.

	std::thread spinner_thr(spinner_work);
	
	ROS_ERROR("Publishing node LP tid %i, pid %i to controller.", ::gettid(), ::getpid());
	std_msgs::Header hdr;
        
	std::stringstream ss_e;
        ss_e << ::getpid() << " lp "  << ::gettid();
        hdr.frame_id = ss_e.str();

	ros::Publisher pub1 = n.advertise<std_msgs::Header>("/robot_0/exec_start_lp", 1, true);
        pub1.publish(hdr);
	
	while(ok())
	{
		robOp.executeCommand();
		// spinOnce();
		// loopRate.sleep();

		// For making critical chain ED
                // We need to wait on the CV.
		std::unique_lock<std::mutex> lk_mutex_robot_op_lp (mutex_robot_op_lp);
                cv_robot_op_lp.wait(lk_mutex_robot_op_lp);
		ROS_WARN("RobotOp notified. Will execute now!");
	}
	spinner_thr.join();
	return 0;	
}
