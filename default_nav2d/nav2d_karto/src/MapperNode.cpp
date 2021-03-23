#include <ros/ros.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>
#include <nav2d_karto/SpaSolver.h>

#include <thread>

void publishTf(MultiMapper* mapper)
{
        ROS_ERROR("IN MAPPER NODE, STARTING PUBLISH TF THREAD!!");
        ros::Rate publishRate(10);
        while(ros::ok())
        {
                mapper->publishTransform();
                publishRate.sleep();
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
	MultiMapper mapper;
	mapper.setScanSolver(solver);

	// std::thread pub_tf_thr (publishTf, &mapper);

	// Start main loop
	ros::Rate publishRate(10);
	while(ros::ok())
	{
		mapper.publishTransform();
		ros::spinOnce();
		publishRate.sleep();
	}

	// Quit
	return 0;
}
