#include <ros/ros.h>

#include <nav2d_navigator/RobotNavigator.h>
#include <thread>

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

void publish_tids_loop(ros::Publisher* pub, std::vector<int> tids, std::vector<std::string> names)
{
        for (int pi = 0; pi < 9; pi++)
		for (int i = 0; i < tids.size(); i++)
                        for (int ni = 0; ni < names.size(); ni++)
                        {	
			        publish_tid(names[ni], tids[i], pub);
				std::this_thread::sleep_for( std::chrono::milliseconds(5) );
			}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<std_msgs::Header>("/robot_0/exec_start_navc", 90, true);
	
	RobotNavigator robNav(pub1);

	/*
	ROS_ERROR("in RobotNavigator Main thread TID: %i. Setting prio=2.", ::gettid() );
	struct sched_param sp = { .sched_priority = 2,};
	int ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);	
	ROS_ERROR("set PRIORITY of Navigator::Spin thread. retval: %i", ret);
	*/

	// publishing tid as navc, navp_extra thread.
	ROS_ERROR("Publishing navigator_extra tid %i (cbt), %i (pmt),  %i Internal CBQT, %i TF-CBT id, pid %i to controller", ::gettid(), n.getPMTId(), n.getInternalCBQTId(), robNav.mTfListener->getTFCBTid(), ::getpid() );

	std::thread pub_id_thr (publish_tids_loop, &pub1, std::vector<int> {::gettid(),n.getPMTId(),n.getInternalCBQTId(), robNav.mTfListener->getTFCBTid() }, std::vector<std::string> {"navc_extra", "navp_extra"} );

	/*
	// CBT:
	publish_tid("navc_extra", ::gettid(), &pub1);
	publish_tid("navp_extra", ::gettid(), &pub1);

	// PMT:
	publish_tid("navc_extra", n.getPMTId(), &pub1);
	publish_tid("navp_extra", n.getPMTId(), &pub1);

	publish_tid("navc_extra", n.getInternalCBQTId(), &pub1);
	publish_tid("navp_extra", n.getInternalCBQTId(), &pub1);
	*/

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
