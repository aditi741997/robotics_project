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

	// Setting SCHED_DEADLINE policy:
        struct sched_attr attr;
        attr.sched_policy = SCHED_DEADLINE;
        int ci = 1;
        int period = 100;
        attr.sched_runtime = ci*1000*1000; // nanosec
        attr.sched_period = period*1000*1000;
        attr.sched_deadline = period*1000*1000;
        int oout = sched_setattr(0, &attr, 0);
        ROS_ERROR("Output of setting DDL policy for Mapper's CBT %i", oout);
        if (oout < 0)
                std::cerr << "setting DDL policy for Mapper's CBT retVal=-1, error: " << std::strerror(errno) << std::endl;

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
