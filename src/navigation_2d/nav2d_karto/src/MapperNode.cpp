#include <ros/ros.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>
#include <nav2d_karto/SpaSolver.h>
#include <thread>

#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

/*
void publish_tid(std::string name, int tid, ros::Publisher* pub)
{
	std_msgs::Header hdr;
	std::stringstream ss;
	ss << ::getpid() << " " << name << " " << tid;
	hdr.frame_id = ss.str();
	pub->publish(hdr);
}
*/

void publish_tids_loop(ros::Publisher* pub, std::vector<int> tids, std::vector<std::string> names)
{
	ROS_ERROR("tHREAD::Starting publish tids in loop!!");
	for (int pi = 0; pi < 10; pi++)
		for (int i = 0; i < tids.size(); i++)
			for (int ni = 0; ni < names.size(); ni++)
			{
				publish_tid(names[ni], tids[i], pub);
				std::this_thread::sleep_for( std::chrono::milliseconds(5) );
			}
}

void publishTfLoop(int milli_sleeptime, MultiMapper* mapper)
{
	// this thread sleep for some time
	// publish current tf with current simTS.
	// set high prio so this can run whenever needed.
	struct sched_param sp = { .sched_priority = 3,};
	int ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);
	ROS_ERROR("PublishTf LOOP Has started!! with sleeptime: %i", milli_sleeptime);
	while(ros::ok())
	{
		std::this_thread::sleep_for (std::chrono::milliseconds(milli_sleeptime));
		mapper->publishTransform();
	}
}


int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "MultiMapper");
	ros::NodeHandle node;

	// Create a scan-solver
	SpaSolver* solver = new SpaSolver();

	// Create the MultiMapper
	ros::Publisher pub1 = node.advertise<std_msgs::Header>("/robot_0/exec_start_mapcb", 100, true);
	MultiMapper mapper(pub1);
	mapper.setScanSolver(solver);
	
	ROS_ERROR("Publishing mapper_extra tid %i (CBT), pmt %i , InternalCBQT %i , pid %i to controller", ::gettid(), node.getPMTId(), node.getInternalCBQTId(), ::getpid() );
	std::thread pub_id_thr (publish_tids_loop, &pub1, std::vector<int> {::gettid(),node.getPMTId(),node.getInternalCBQTId()}, std::vector<std::string> {"mapcb_extra","mapupd_extra"} );

	// std::thread pub_tf_thr ( publishTfLoop, 20, &mapper); // 50Hz RT ~ 10Hz ST

	/*
	publish_tid("mapcb_extra", ::gettid(), &pub1);
	publish_tid("mapupd_extra", ::gettid(), &pub1);
	
	publish_tid("mapcb_extra", node.getPMTId(), &pub1);
	publish_tid("mapupd_extra", node.getPMTId(), &pub1);

	publish_tid("mapcb_extra", node.getInternalCBQTId(), &pub1);
	publish_tid("mapupd_extra", node.getInternalCBQTId(), &pub1);
	*/

	// Jan: This is the map CBT which processes latest scan msgs. [in original code, not now.]
	ros::spin();

	// For converting navigation2d to ED
	// tf will be published at the end of mapper Scan callback.
	// while(ros::ok())
	// {
	// 	mapper.publishTransform();
	// 	ros::spinOnce();
	// 	publishRate.sleep();
	// }

	// Quit
	return 0;
}
