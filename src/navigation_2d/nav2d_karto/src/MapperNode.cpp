#include <ros/ros.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>
#include <nav2d_karto/SpaSolver.h>

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
