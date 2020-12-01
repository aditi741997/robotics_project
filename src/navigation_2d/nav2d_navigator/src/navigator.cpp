#include <ros/ros.h>

#include <nav2d_navigator/RobotNavigator.h>

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	
	RobotNavigator robNav;

	ROS_ERROR("in RobotNavigator Main thread TID: %i. Setting prio=2.", ::gettid() );

	struct sched_param sp = { .sched_priority = 2,};
	int ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);	
	ROS_ERROR("set PRIORITY of Navigator::Spin thread. retval: %i", ret);
	
	ros::spin();
	return 0;
}
