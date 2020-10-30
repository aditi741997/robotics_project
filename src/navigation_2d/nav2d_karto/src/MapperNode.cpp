#include <ros/ros.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>
#include <nav2d_karto/SpaSolver.h>

#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

int main(int argc, char **argv)
{
	// Initialize ROS
	
	ros::init(argc, argv, "MultiMapper");

	ros::NodeHandle node;

	// Create a scan-solver
	SpaSolver* solver = new SpaSolver();

	// Create the MultiMapper
	MultiMapper mapper;
	mapper.setScanSolver(solver);

	// Start main loop
	ros::Rate publishRate(10);

	ROS_ERROR("Publishing mapCB tid %i pid %i to controller", ::gettid(), ::getpid() );
	std_msgs::Header hdr;
	
	std::stringstream ss_e;
	ss_e << ::getpid() << " mapcb " << ::gettid();
	hdr.frame_id = ss_e.str();

	// ros::Publisher pub1 = node.advertise<std_msgs::Header>("/robot_0/exec_start_mapcb", 1, true);
	// pub1.publish(hdr);
	
	// this is the mapCB thread.
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
