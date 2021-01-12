#include <ros/ros.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>
#include <nav2d_karto/SpaSolver.h>

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

int main(int argc, char **argv)
{
	// Initialize ROS
	
	ros::init(argc, argv, "MultiMapper");

	ros::NodeHandle node;

	// Create a scan-solver
	SpaSolver* solver = new SpaSolver();

	// Create the MultiMapper
	ros::Publisher pub1 = node.advertise<std_msgs::Header>("/robot_0/exec_start_mapcb", 20, true);
	MultiMapper mapper(pub1);
	mapper.setScanSolver(solver);

	
	ROS_ERROR("Publishing mapper_extra tid %i (CBT), pmt %i pid %i to controller", ::gettid(), node.getPMTId(), ::getpid() );
	std_msgs::Header hdr, hdr1;

	publish_tid("mapcb_extra", ::gettid(), &pub1);
	publish_tid("mapupd_extra", ::gettid(), &pub1);
	
	publish_tid("mapcb_extra", node.getPMTId(), &pub1);
	publish_tid("mapupd_extra", node.getPMTId(), &pub1);

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
