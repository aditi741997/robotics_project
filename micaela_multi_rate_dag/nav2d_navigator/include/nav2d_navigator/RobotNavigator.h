#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>
#include <nav2d_navigator/MoveToPosition2DAction.h>
#include <nav2d_navigator/ExploreAction.h>
#include <nav2d_navigator/GetFirstMapAction.h>
#include <nav2d_navigator/LocalizeAction.h>

#include <nav2d_navigator/GridMap.h>
#include <nav2d_navigator/commands.h>
#include <nav2d_navigator/MapInflationTool.h>
#include <nav2d_navigator/ExplorationPlanner.h>

#include <queue>

typedef actionlib::SimpleActionServer<nav2d_navigator::MoveToPosition2DAction> MoveActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::ExploreAction> ExploreActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::GetFirstMapAction> GetMapActionServer;
typedef actionlib::SimpleActionServer<nav2d_navigator::LocalizeAction> LocalizeActionServer;
typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;

class RobotNavigator
{
public:
	RobotNavigator();
	~RobotNavigator();

	bool receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	void receiveMoveGoal(const nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal);
	void receiveExploreGoal(const nav2d_navigator::ExploreGoal::ConstPtr &goal);
	void receiveGetMapGoal(const nav2d_navigator::GetFirstMapGoal::ConstPtr &goal);
	void receiveLocalizeGoal(const nav2d_navigator::LocalizeGoal::ConstPtr &goal);

	// For moving Navigator cmd into a separate thread:
        boost::thread* nav_cmd_thread_;
        bool nav_cmd_thread_shutdown_;
        double mReplanningPeriod;
        void navGenerateCmdLoop();

        // For measuring tput of NavPlan and navCmd subchains:
        std::vector<double> tput_nav_plan, tput_nav_cmd;
        // TS of the inputs used when the node ran last time:
        double nav_cmd_last_tf_used, nav_cmd_last_plan_used;
        double nav_plan_last_tf_used, nav_plan_last_map_used;

        double last_nav_cmd_out, last_nav_plan_out;

        ros::Publisher navp_exec_info_pub, navc_exec_info_pub;

private:
	bool isLocalized();
	bool setCurrentPosition(int x=-1);
	bool getMap();
	void stop();
	bool correctGoalPose();
	bool generateCommand();
	bool preparePlan();
	bool createPlan();
	void publishPlan();

	// Everything related to ROS
	tf::TransformListener mTfListener;
	ros::ServiceClient mGetMapClient;
	ros::Subscriber mGoalSubscriber;
	ros::Publisher mPlanPublisher;
	ros::Publisher mCommandPublisher;
	ros::Publisher mMarkerPublisher;
	ros::ServiceServer mStopServer;
	ros::ServiceServer mPauseServer;

	std::string mMapFrame;
	std::string mRobotFrame;
	std::string mMoveActionTopic;
	std::string mExploreActionTopic;
	std::string mGetMapActionTopic;
	std::string mLocalizeActionTopic;

	MoveActionServer* mMoveActionServer;
	ExploreActionServer* mExploreActionServer;
	GetMapActionServer* mGetMapActionServer;
	LocalizeActionServer* mLocalizeActionServer;

	PlanLoader* mPlanLoader;

	// Current status and goals
	bool mHasNewMap;
	bool mIsPaused;
	bool mIsStopped;
	int mStatus;
	int mRobotID;
	unsigned int mGoalPoint;
	unsigned int mStartPoint;
	double mCurrentDirection;
	double mCurrentPositionX;
	double mCurrentPositionY;

	// Everything related to the global map and plan
	MapInflationTool mInflationTool;
	std::string mExplorationStrategy;
	boost::shared_ptr<ExplorationPlanner> mExplorationPlanner;
	GridMap mCurrentMap;
	double* mCurrentPlan;

	int currentPlanSize;
        boost::mutex currentPlanMutex; // need to take this lock when reading,writing on mCurrentPlan.

	double mFrequency;
	double mInflationRadius;
	double mRobotRadius;
	unsigned int mCellInflationRadius;
	unsigned int mCellRobotRadius;

	signed char mCostObstacle;
	signed char mCostLethal;

	double mCommandTargetDistance;
	double mNavigationGoalDistance;
	double mNavigationGoalAngle;
	double mNavigationHomingDistance;
	double mExplorationGoalDistance;
	double mMinReplanningPeriod;
	double mMaxReplanningPeriod;

	std::vector<double> explore_cb_plan_times;
        std::vector<double> explore_cb_plan_ts;

        std::vector<double> explore_cb_cmd_times;
        std::vector<double> explore_cb_cmd_ts;

        // For measuring RT:
        double current_plan_last_scan_mapCB_mapUpd_used_ts;
        double current_mapCB_tf_navCmd_scan_ts; // current Cmd - Transform's TS wrt latest scan used by mapper.
        double current_mapCB_tf_navPlan_scan_ts, latest_mapCB_tf_navPlan_scan_ts;

        ros::Subscriber mScanUsedTSTFSubscriber;
        double current_mapper_tf_scan_ts; // latest transform's - scan used TS.
        void updateMapperScanTSUsedTF(const std_msgs::Header& hdr);

	int navc_trigger_count, navp_trigger_count;
};