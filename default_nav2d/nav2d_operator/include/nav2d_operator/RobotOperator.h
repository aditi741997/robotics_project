#ifndef OPERATOR_H
#define OPERATOR_H

#define NODE_NAME     "operator"
#define COMMAND_TOPIC "cmd"
#define CONTROL_TOPIC "cmd_vel"
#define ROUTE_TOPIC   "route"
#define PLAN_TOPIC    "desired"
#define LUT_RESOLUTION 100

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav2d_operator/cmd.h>

#include <string>
#include <fstream>
/**
 * @class RobotOperator
 * @author Sebastian Kasperski
 * @date 03/25/14
 * @file RobotOperator.h
 * @brief The core of a ROS node to provide purely reactive obstacle avoidance
 * The RobotOperator is supposed to be placed between a path planner and a mobile
 * robots hardware driver. It takes in motion commands defined in the robot's
 * coordinate frame and outputs a Twist-Message to directly control a robot.
 * The given command is continually checked against a local map updated from
 * collected laser range data and corrected to follow a safe path.
 */
class RobotOperator
{
public:
	// Default Constructor & Destructor
	RobotOperator();
	~RobotOperator();
	
	// Public Methods
	/**
	 * @brief Callback function to receive move commands
	 * @param msg Command-Message
	 * Direction [-1.0 .. 1.0]: -1(rotate left); 0(straight); 1(rotate right)
	 * Velocity  [-1.0 .. 1.0]: -1(full speed back); 0(stop); 1(full speed ahead)
	 * Mode: 0(Avoid obstacles); 1(Stop at obstacles)
	 */
	void receiveCommand(const nav2d_operator::cmd::ConstPtr& msg);

	/**
	 * @brief Generates and sends Twist-Message to Robot
	 * This is the Operator's core function and should be called periodically
	 */
	void executeCommand();

	 // For measuring RT

        // Need to store [the latest scan USED TS] in the last executeCmd, wrt each of the chains:
        double last_scan_lcmp_ts; // TS of [latest scan in latest LC] used by last executeCommand
        double last_scan_mapCB_navPlan_navCmd_ts; // TS of last scan used wrt S-MapCB-LC chain
        double last_scan_mapCB_navCmd_ts;
        double last_scan_mapCB_mapUpd_navPlan_navCmd_ts;

        double last_scan_mapCB_ts;

        // TS of the scan, based on which we have got the command.
        double last_scan_mapCB_mapUpd_navPlan_navCmd_recv;

        // TS of the scan, whose TF was used by navCmd.
        double last_scan_mapCB_navCmd_recv;

        // TS of scan, whose TF was used by navPlan, used by navCmd.
        double last_scan_mapCB_navPlan_navCmd_recv;

	// Need to store TS of [last new output] wrt each chain
        double last_scan_lcmp_out, last_scan_mapCB_navPlan_navCmd_out, last_scan_mapCB_navCmd_out, last_scan_mapCB_mapUpd_navPlan_navCmd_out;

        // Vector of latency for [every new output] wrt each chain
        std::vector<double> lat_scan_lcmp_arr, lat_scan_mapCB_navCmd_arr, lat_scan_mapCB_navPlan_navCmd_arr, lat_scan_mapCB_mapUpd_navPlan_navCmd_arr;

        // Vector for tput
        std::vector<double> tput_scan_lcmp_arr, tput_scan_mapCB_navPlan_navCmd_arr, tput_scan_mapCB_navCmd_arr, tput_scan_mapCB_mapUpd_navPlan_navCmd_arr;

        // Vector for RT
        std::vector<double> rt_scan_lcmp_arr, rt_scan_mapCB_navCmd_arr, rt_scan_mapCB_navPlan_navCmd_arr, rt_scan_mapCB_mapUpd_navPlan_navCmd_arr;

        // Storing the realTS of per chain outputs:
        std::vector<double> ts_scan_lcmp_arr, ts_scan_mapCB_navCmd_arr, ts_scan_mapCB_navPlan_navCmd_arr, ts_scan_mapCB_mapUpd_navPlan_navCmd_arr;

        double first_out_ts_scan_lcmp, first_out_ts_scan_mapCB_navCmd;
        long count_scan_lcmp, count_scan_mapCB_navCmd, count_scan_mapCB_navPlan_navCmd, count_scan_mapCB_mapUpd_navPlan_navCmd; // total #outputs.

private:
	// Internal Methods
	/**
	 * @brief Calculates the distance the robot can move following the given trajectory
	 * @param cloud PointCloud defining a trajectory
	 * @return Nmber of free cells
	 */
	int calculateFreeSpace(sensor_msgs::PointCloud* cloud);

	/**
	 * @brief Calculate the action value of a given command
	 * @param direction How to move the robot
	 * @param velocity Only used to distinguish forward and backward motion
	 * @param debug Publish result of evaluation functions on debug topic
	 * @return Weighted sum of all evaluation functions
	 * The given action is rated by 4 different evaluation functions:
	 * Free Space: How far can the robot move following this command
	 * Safety: How close will it get near obstacles
	 * Conformance: How good does it follow the commanded direction
	 * Continuity: (experimental!) How does it conform with the last issued command
	 */
	double evaluateAction(double direction, double velocity, bool debug = false);

	/**
	 * @brief Evaluates all possible directions with a fixed resolution
	 * @return Best evaluated direction
	 */
	double findBestDirection();

	/**
	 * @brief Initializes look-up tables
	 * This calculates the trajectories of all possible direction commands.
	 * Must be called once before the Operator is used
	 */
	void initTrajTable();
	
	/**
	 * @brief Get the trajectory defined by the given movement command
	 * @param direction How to move the robot
	 * @param velocity Only used to distinguish forward and backward motion
	 * @return A pointer to the PointCloud defined in the robot coordinate frame
	 */
	inline sensor_msgs::PointCloud* getPointCloud(double direction, double velocity);

	// Internal Storage
	costmap_2d::Costmap2DROS* mLocalMap;
	costmap_2d::Costmap2D* mCostmap;
	double mRasterSize;
	
	tf::TransformListener mTfListener;
	
	ros::Subscriber mCommandSubscriber;
	ros::Publisher mControlPublisher;
	ros::Publisher mTrajectoryPublisher;
	ros::Publisher mPlanPublisher;
	ros::Publisher mCostPublisher;
	
	double mDesiredVelocity;
	double mDesiredDirection;
	double mCurrentVelocity;
	double mCurrentDirection;
	int mDriveMode;
	
	sensor_msgs::PointCloud* mTrajTable[(LUT_RESOLUTION * 4) + 2];
	
	double mMaxVelocity;
	
	bool mPublishRoute;
	double mMaxFreeSpace;
	double mSafetyDecay;
	int mDistanceWeight;
	int mSafetyWeight;
	int mConformanceWeight;
	int mContinueWeight;
	int mEscapeWeight;

	std::string mOdometryFrame;
	std::string mRobotFrame;
	
	unsigned int mRecoverySteps;

	std::vector<double> operator_loop_times;
        std::vector<double> operator_loop_ts;
};

#endif
