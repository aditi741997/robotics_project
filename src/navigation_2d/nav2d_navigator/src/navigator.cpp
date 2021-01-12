#include <ros/ros.h>

#include <nav2d_navigator/RobotNavigator.h>

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<std_msgs::Header>("/robot_0/exec_start_navc", 20, true);
	
	RobotNavigator robNav(pub1);

	/*
	ROS_ERROR("in RobotNavigator Main thread TID: %i. Setting prio=2.", ::gettid() );
	struct sched_param sp = { .sched_priority = 2,};
	int ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);	
	ROS_ERROR("set PRIORITY of Navigator::Spin thread. retval: %i", ret);
	*/

	// publishing tid as navc, navp_extra thread.
	ROS_ERROR("Publishing navigator_extra tid %i (cbt), %i (pmt) pid %i to controller", ::gettid(), n.getPMTId(), ::getpid() );
	std_msgs::Header hdr, hdr1;

	// CBT:
	publish_tid("navc_extra", ::gettid(), &pub1);
	publish_tid("navp_extra", ::gettid(), &pub1);

	// PMT:
	publish_tid("navc_extra", n.getPMTId(), &pub1);
	publish_tid("navp_extra", n.getPMTId(), &pub1);

	/*
	std::stringstream ss_e;
        ss_e << ::getpid() << " navc_extra " << ::gettid();
        hdr.frame_id = ss_e.str();
        std::stringstream ss_e1;
	ss_e1 << ::getpid() << " navp_extra " << ::gettid();
	hdr1.frame_id = ss_e1.str();

        pub1.publish(hdr);
        pub1.publish(hdr1);
	*/

	ros::spin();
	return 0;
}
