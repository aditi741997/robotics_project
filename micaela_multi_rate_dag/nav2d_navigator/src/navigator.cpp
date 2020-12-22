#include <ros/ros.h>

#include <nav2d_navigator/RobotNavigator.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	
	RobotNavigator robNav;

	// Setting SCHED_DEADLINE policy:
        struct sched_attr attr;
        attr.sched_policy = SCHED_DEADLINE;
        float ci = 1;
        float period = 100;
        attr.sched_runtime = ci*1000*1000; // nanosec
        attr.sched_period = period*1000*1000;
        attr.sched_deadline = period*1000*1000;
        int oout = sched_setattr(0, &attr, 0);
        ROS_ERROR("Output of setting DDL policy for Navigator's CBT  %i", oout);
        if (oout < 0)
                std::cerr << "setting DDL policy for Navigator's CBT retVal=-1, error: " << std::strerror(errno) << std::endl;

	ros::spin();
	return 0;
}
