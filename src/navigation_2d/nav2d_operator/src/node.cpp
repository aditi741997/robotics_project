#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav2d_operator/RobotOperator.h>

#include <mutex>
#include <condition_variable>
#include <thread>

using namespace ros;

void spinner_work()
{
	ros::spin();
}

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n("~/");

	double frequency;
	n.param("frequency", frequency, 100.0);
	ROS_WARN("NAV2D: RobotOperator will run at %.2f Hz frequency.", frequency);

	std::mutex mutex_robot_op_lp;
	std::condition_variable cv_robot_op_lp;

	RobotOperator robOp (&cv_robot_op_lp);
	
	Rate loopRate(frequency);

	std::thread spinner_thr(spinner_work);

	while(ok())
	{
		// Commenting spinOnce in making this node ED:
		// spinOnce();
		robOp.executeCommand();

		// For making critical chain ED
		// We need to wait on the CV.
		// loopRate.sleep();
		// if(loopRate.cycleTime() > ros::Duration(1.0 / frequency))
		// 	ROS_WARN("Missed desired rate of %.2f Hz! Loop actually took %.4f seconds!",frequency, loopRate.cycleTime().toSec());

		std::unique_lock<std::mutex> lk_mutex_robot_op_lp (mutex_robot_op_lp);
		cv_robot_op_lp.wait(lk_mutex_robot_op_lp);

		// ROS_WARN("RobotOperator got notified, About to executeCommand!!!");
	}

	spinner_thr.join();
	return 0;	
}