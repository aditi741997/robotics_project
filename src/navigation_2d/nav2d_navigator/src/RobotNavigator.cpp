#include <nav2d_operator/cmd.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>

#include <nav2d_navigator/RobotNavigator.h>
#include <nav2d_navigator/ExplorationPlanner.h>

#include <set>
#include <map>
#include <ctime>
#include <chrono>

#include <fstream>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

#include <unistd.h>
#include <sys/syscall.h>
#include <cmath>
#define gettid() syscall(SYS_gettid)

#define PI 3.14159265

using namespace ros;
using namespace tf;

double get_time_diff(timespec& a, timespec& b)
{
  return (( b.tv_sec + 1e-9*b.tv_nsec ) - ( a.tv_sec + 1e-9*a.tv_nsec ));
}

double get_time_now()
{
	/*
	boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
        boost::chrono::system_clock::duration tse = now.time_since_epoch();
        //using system_clock to measure latency:
        unsigned long long ct = boost::chrono::duration_cast<boost::chrono::milliseconds>(tse).count() - (1603000000000);
        double time_now = ct / (double)(1000.0);
	*/

	struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double time_now = ts.tv_sec + 1e-9*ts.tv_nsec;

	return time_now;
}

void write_arr_to_file(std::vector<double>& tput, std::string s)
{
        std::string ename;
    ros::NodeHandle nh;
    nh.param<std::string>("/expt_name", ename, "");

        int sz = tput.size();

	if (sz > 0)
	{
		std::ofstream of;
        	of.open( ("/home/ubuntu/robot_" + s + "_stats_" + ename + ".txt").c_str() , std::ios_base::app);

        	of << "\n" << s << " tput: ";
        	for (int i = 0; i < sz; i++)
                	of << tput[i] << " ";

        	std::sort(tput.begin(), tput.end());
        	ROS_ERROR("Nav2d Navigator %s, Median, 95ile TPUT %f %f", s.c_str(), tput[sz/2], tput[(95*sz)/100]);
        	tput.clear();
	}
}

RobotNavigator::RobotNavigator()
{	
	NodeHandle robotNode;

	std::string serviceName;
	robotNode.param("map_service", serviceName, std::string("get_map"));
	ROS_ERROR("INitializing NAV2D::Navigator Node. The map service for getMap is %s", serviceName.c_str());
	mGetMapClient = robotNode.serviceClient<nav_msgs::GetMap>(serviceName);

	mCommandPublisher = robotNode.advertise<nav2d_operator::cmd>("cmd", 1);
	mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE, &RobotNavigator::receiveStop, this);
	mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE, &RobotNavigator::receivePause, this);
	mCurrentPlan = NULL;
	currentPlanSize = 0;

	NodeHandle navigatorNode("~/");
	ROS_ERROR("INitializing NAV2D::Navigator Node. Publishing plan to plan topic and markers AND PUBLISHES nav2d_operator::cmd on cmd topic.");
	mPlanPublisher = navigatorNode.advertise<nav_msgs::GridCells>("plan", 1);
	mMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("markers", 1, true);
	
	// Get parameters
	navigatorNode.param("map_inflation_radius", mInflationRadius, 1.0);
	navigatorNode.param("robot_radius", mRobotRadius, 0.3);
	navigatorNode.param("exploration_strategy", mExplorationStrategy, std::string("NearestFrontierPlanner"));
	navigatorNode.param("navigation_goal_distance", mNavigationGoalDistance, 1.0);
	navigatorNode.param("navigation_goal_angle", mNavigationGoalAngle, 1.0);
	navigatorNode.param("exploration_goal_distance", mExplorationGoalDistance, 3.0);
	navigatorNode.param("navigation_homing_distance", mNavigationHomingDistance, 3.0);
	navigatorNode.param("min_replanning_period", mMinReplanningPeriod, 3.0);
	navigatorNode.param("max_replanning_period", mMaxReplanningPeriod, 1.0);
	navigatorNode.param("command_target_distance", mCommandTargetDistance, 1.0);
	navigatorNode.param("frequency", mFrequency, 10.0);
	mCostObstacle = 100;
	mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;

	navigatorNode.param("replanning_period", mReplanningPeriod, 3.0);

	robotNode.param("map_frame", mMapFrame, std::string("map"));
	robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
	robotNode.param("robot_id", mRobotID, 1);
	robotNode.param("move_action_topic", mMoveActionTopic, std::string(NAV_MOVE_ACTION));
	robotNode.param("explore_action_topic", mExploreActionTopic, std::string(NAV_EXPLORE_ACTION));
	robotNode.param("getmap_action_topic", mGetMapActionTopic, std::string(NAV_GETMAP_ACTION));
	robotNode.param("localize_action_topic", mLocalizeActionTopic, std::string(NAV_LOCALIZE_ACTION));

	// Apply tf_prefix to all used frame-id's
	mRobotFrame = mTfListener.resolve(mRobotFrame);
	mMapFrame = mTfListener.resolve(mMapFrame);

	ROS_ERROR("INitializing nav2d_navigator NODE. mRobotFrame : %s, mMapFrame : %s", mRobotFrame.c_str(), mMapFrame.c_str());
	ROS_ERROR("IN RobotNavigator NavCmd Freq : %f, NavP: min_replanning_period: %f, max_replanning_period: %f, NavPlanThread period: %f", mFrequency, mMinReplanningPeriod, mMaxReplanningPeriod, mReplanningPeriod);
	try
	{
		mPlanLoader = new PlanLoader("nav2d_navigator", "ExplorationPlanner");
		mExplorationPlanner = mPlanLoader->createInstance(mExplorationStrategy);
		ROS_ERROR("IN NAV2D::Navigator - Successfully loaded exploration strategy [%s].", mExplorationStrategy.c_str());

		mExploreActionServer = new ExploreActionServer(mExploreActionTopic, boost::bind(&RobotNavigator::receiveExploreGoal, this, _1), false);
		mExploreActionServer->start();
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("Failed to load exploration plugin! Error: %s", ex.what());
		mExploreActionServer = NULL;
		mPlanLoader = NULL;
	}

	// Create action servers
	ROS_ERROR("NOT making localize, moveGoal servers for now!!!");
	/*
	mMoveActionServer = new MoveActionServer(mMoveActionTopic, boost::bind(&RobotNavigator::receiveMoveGoal, this, _1), false);
	mMoveActionServer->start();
	
	mLocalizeActionServer = new LocalizeActionServer(mLocalizeActionTopic, boost::bind(&RobotNavigator::receiveLocalizeGoal, this, _1), false);
	mLocalizeActionServer->start();
	*/

	if(mRobotID == 1)
	{
		mGetMapActionServer = new GetMapActionServer(mGetMapActionTopic, boost::bind(&RobotNavigator::receiveGetMapGoal, this, _1), false);
		mGetMapActionServer->start();
	}else
	{
		mGetMapActionServer = NULL;
	}
	
	mHasNewMap = false;
	mIsStopped = false;
	mIsPaused = false;
	mStatus = NAV_ST_IDLE;
	mCellInflationRadius = 0;

	// For measuring RT:
	current_plan_last_scan_mapCB_mapUpd_used_ts = 0.0;
	// seting tcp nodelay to false...
	mScanUsedTSTFSubscriber = navigatorNode.subscribe("/robot_0/mapper_scan_ts_used_TF", 1, &RobotNavigator::updateMapperScanTSUsedTF, this, ros::TransportHints().tcpNoDelay());

	navp_exec_info_pub = navigatorNode.advertise<std_msgs::Header>("/robot_0/exec_start_navp", 1, true);
	navc_exec_info_pub = navigatorNode.advertise<std_msgs::Header>("/robot_0/exec_start_navc", 1, true);

	navc_exec_end_pub = navigatorNode.advertise<std_msgs::Header>("/robot_0/exec_end_navc", 1, true);
	navp_exec_end_pub = navigatorNode.advertise<std_msgs::Header>("/robot_0/exec_end_navp", 1, true);

	// For measuring tput of subchains :
	last_nav_cmd_out = 0.0;
	last_nav_plan_out = 0.0;

	nav_cmd_thread_ = NULL;
	nav_cmd_thread_shutdown_ = false;

	// For conencting socket to Controller.
	client_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (client_sock_fd < 0) ROS_ERROR("RobotNavigator:: SOCKET: client_sock_fd is NEGATIVE!!");

	struct sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(7727);

	int pton_ret = inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
	if ( pton_ret <= 0 )
		ROS_ERROR("RobotNavigator:: SOCKET: Error in inet_pton %i", pton_ret);

	if ( connect(client_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0 )
		ROS_ERROR("RobotNavigator:: SOCKET: ERROR in connecting!!!");

	// Nov: need a thread to continually listen on the socket.
	sock_recv_thread = boost::thread(&RobotNavigator::socket_recv, this);
	ROS_INFO("RobotNavigator:: SOCKET: DONE with socket stuff!");

	navc_trigger_count = 0;
	navp_trigger_count = 0;
}

void RobotNavigator::socket_recv()
{
	int read_size;
	char msg [2048];


	// Setting priority=2 so that this gets notified at the right time.
	int ret = 7;
	struct sched_param sp = { .sched_priority = 2,};
	ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);
	
	ROS_ERROR("RobotNavigator::socket_recv THREAD: tid %i, pid %i, Changed prio to 2, retval: %i", ::gettid(), ::getpid(), ret);	

	// recv call waits for a message, thats why this is in a separate thread.
	while ( (read_size = recv(client_sock_fd, msg, 2048, 0) ) > 0 )
	{
		// Got a message:
		std::string s_msg = msg;
		ROS_ERROR("RobotNavigator::socket_recv THREAD: GOT msg %s", s_msg.c_str());
		memset(msg, 0, 2048); // clear the arr
		std::stringstream ss(s_msg);
		std::string to;
		while(std::getline(ss,to,'\n'))
		{
			ROS_ERROR("Processing line : %s", to.c_str());
			if (to.find("navc") != std::string::npos)
			{
				boost::unique_lock<boost::mutex> lock(navc_trigger_mutex);
				ROS_ERROR("Got a trigger for navc, curr count %i", navc_trigger_count);
				if (to.find("RESETCOUNT") != std::string::npos)
				{
					navc_trigger_count = 1;
					ROS_ERROR("Resetting the navc count to 1!!!!");
				}
				else
					navc_trigger_count += 1;
				cv_navc.notify_all();
			}
			else
			{
				boost::unique_lock<boost::mutex> lock(navp_trigger_mutex);
				ROS_ERROR("Got a trigger for navp, curr_count %i", navp_trigger_count);
				if (to.find("RESETCOUNT") != std::string::npos)
                                {
					navp_trigger_count = 1;
					ROS_ERROR("Resetting the navp count to 1!!!!");
				}
				else
					navp_trigger_count += 1;
				cv_navp.notify_all();
			}
		}
	}
	if(read_size == 0)
		ROS_ERROR("RobotNavigator::socket_recv THREAD: Client disconnected!!");
	else if(read_size == -1)
		ROS_ERROR("RobotNavigator::socket_recv THREAD: Client read error!!");
}

RobotNavigator::~RobotNavigator()
{
	delete[] mCurrentPlan;
	delete mMoveActionServer;
	delete mExploreActionServer;
	delete mGetMapActionServer;
	mExplorationPlanner.reset();
	delete mPlanLoader;
	close(client_sock_fd);
}

void RobotNavigator::updateMapperScanTSUsedTF(const std_msgs::Header& hdr)
{
	current_mapper_tf_scan_ts = hdr.stamp.toSec();
	// ROS_WARN("In ROBOTNavigator::updateMapperScanTSUsedTF : Mapper's TF TS is now %f", current_mapper_tf_scan_ts);
}

bool RobotNavigator::getMap()
{	
	if(mHasNewMap) return true;
	
	if(!mGetMapClient.isValid())
	{
		ROS_ERROR("GetMap-Client is invalid!");
		return false;
	}
	
	nav_msgs::GetMap srv;
	// ROS_WARN("IN nav2d::RobotNavigator About to call getMap service!!");
	if(!mGetMapClient.call(srv))
	{
		ROS_ERROR("Could not get a map.");
		return false;
	}
	mCurrentMap.update(srv.response.map);

	currentPlanMutex.lock();	
	if(mCurrentPlan) delete[] mCurrentPlan;
	mCurrentPlan = new double[mCurrentMap.getSize()];
	currentPlanSize = mCurrentMap.getSize();
	currentPlanMutex.unlock();	

	if(mCellInflationRadius == 0)
	{
		ROS_INFO("Navigator is now initialized.");
		mCellInflationRadius = mInflationRadius / mCurrentMap.getResolution();
		mCellRobotRadius = mRobotRadius / mCurrentMap.getResolution();
		mInflationTool.computeCaches(mCellInflationRadius);
		mCurrentMap.setLethalCost(mCostLethal);
	}
	
	mHasNewMap = true;
	return true;
}

bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	mIsStopped = true;
	res.success = true;
	res.message = "Navigator received stop signal.";
	return true;
}

bool RobotNavigator::receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{	
	if(mIsPaused)
	{
		mIsPaused = false;
		res.success = false;
		res.message = "Navigator continues.";
	}else
	{
		mIsPaused = true;
		nav2d_operator::cmd stopMsg;
		stopMsg.Turn = 0;
		stopMsg.Velocity = 0;
		mCommandPublisher.publish(stopMsg);
		res.success = true;
		res.message = "Navigator pauses.";
	}
	return true;
}

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;

bool RobotNavigator::preparePlan()
{
	// ROS_WARN("IN nav2d::RobotNavigator preparePlan.. Will call getMap now.");
	// Get the current map
	if(!getMap()) // return false;
	{
		if(mCellInflationRadius == 0) return false;
		ROS_ERROR("Could not get a new map, trying to go with the old one...");
	}
	
	// Where am I?
	if(!setCurrentPosition(1) || (get_pos_tf_error_ct>1)) return false;
	
	// Clear robot footprint in map
	unsigned int x = 0, y = 0;
	if(mCurrentMap.getCoordinates(x, y, mStartPoint))
		for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
			for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
				mCurrentMap.setData(x+i, y+j, 0);
	
	mInflationTool.inflateMap(&mCurrentMap);
	return true;
}

bool RobotNavigator::createPlan()
{	
	// ROS_ERROR("In RobotNavigator::createPlan, Map-Value of goal point is %d, lethal threshold is %d.", mCurrentMap.getData(mGoalPoint), mCostLethal);
	unsigned int start_x = 0, start_y = 0;
	unsigned int goal_x = 0, goal_y = 0;
	if(mCurrentMap.getCoordinates(goal_x,goal_y,mGoalPoint))
	{
		mCurrentMap.getCoordinates(start_x, start_y, mStartPoint);
		ROS_ERROR("IN ROBOTNavigator:: createPlan Goal coordinates in map: %i %i | STartPOint : %i %i #", goal_x, goal_y, start_x, start_y);
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = mCurrentMap.getOriginX() + (((double)goal_x+0.5) * mCurrentMap.getResolution());
		marker.pose.position.y = mCurrentMap.getOriginY() + (((double)goal_y+0.5) * mCurrentMap.getResolution());
		marker.pose.position.z = 0.5;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = mCurrentMap.getResolution() * 3.0;
		marker.scale.y = mCurrentMap.getResolution() * 3.0;
		marker.scale.z = 1.0;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		mMarkerPublisher.publish(marker);
	}else
	{
		ROS_ERROR("Couldn't ressolve goal point coordinates!");
	}
	
	Queue queue;

	int mapSize = mCurrentMap.getSize();
	// Done: First make a new planObj, make it ready. Then, just copy to mCurrentPlan with lock.	
	double* newPlan = new double[mapSize];	

	// Reset the plan
	for(int i = 0; i < mapSize; i++)
	{
		// mCurrentPlan[i] = -1;
		newPlan[i] = -1;
	}

	if(mCurrentMap.isFree(mGoalPoint))
	{
		queue.insert(Entry(0.0, mGoalPoint));
		// mCurrentPlan[mGoalPoint] = 0;
		newPlan[mGoalPoint] = 0;
	}else
	{
		// Initialize the queue with area around the goal point
		int reach = mCellRobotRadius + (1.0 / mCurrentMap.getResolution());
		std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(mGoalPoint, reach);
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			queue.insert(Entry(0.0, neighbors[i]));
			// mCurrentPlan[neighbors[i]] = 0;
			newPlan[neighbors[i]] = 0;
		}
	}
	ROS_ERROR("IN ROBOTNavigator:: createPlan, added %i nodes (Goal/NearGoal) to queue.", queue.size());
	
	Queue::iterator next;
	double distance;
	unsigned int x, y, index;
	double linear = mCurrentMap.getResolution();
	double diagonal = std::sqrt(2.0) * linear;
	
	// Do full search with Dijkstra-Algorithm
	int count = 0;
	while(!queue.empty())
	{
		// Get the nearest cell from the queue
		next = queue.begin();
		distance = next->first;
		index = next->second;
		queue.erase(next);
		
		count += 1;

		// if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;
		if ( newPlan[index] >= 0 && newPlan[index] < distance ) continue;		

//		if(index == mStartPoint) break;
		
		// Add all adjacent cells
		if(!mCurrentMap.getCoordinates(x, y, index)) continue;
		std::vector<unsigned int> ind;
		ind.push_back(index - 1);
		ind.push_back(index + 1);
		ind.push_back(index - mCurrentMap.getWidth());
		ind.push_back(index + mCurrentMap.getWidth());
		ind.push_back(index - mCurrentMap.getWidth() - 1);
		ind.push_back(index - mCurrentMap.getWidth() + 1);
		ind.push_back(index + mCurrentMap.getWidth() - 1);
		ind.push_back(index + mCurrentMap.getWidth() + 1);
			
		for(unsigned int it = 0; it < ind.size(); it++)
		{
			unsigned int i = ind[it];
			if(mCurrentMap.isFree(i))
			{
				double delta = (it < 4) ? linear : diagonal;
				double newDistance = distance + delta + (10 * delta * (double)mCurrentMap.getData(i) / (double)mCostObstacle);
				// if(mCurrentPlan[i] == -1 || newDistance < mCurrentPlan[i])
				if(newPlan[i] == -1 || newDistance < newPlan[i])
				{
					queue.insert(Entry(newDistance, i));
					// mCurrentPlan[i] = newDistance;
					newPlan[i] = newDistance;
				}
			}
		}
		if (count%17500 == 77)
			ROS_ERROR("In navPlan createPlan, curr que sz : %i, count %i", queue.size(), count);
	}

	currentPlanMutex.lock();
	// copy newPlan to mCurrentPlan
	mCurrentPlan = newPlan;
	for (int i = 0; i < mapSize; i++)
		if (mCurrentPlan[i] != newPlan[i])
			ROS_ERROR("SHITTTTTTTTT Plan was NOT COPIED CORRECTLY!!! In createPlan()");
	currentPlanSize = mapSize;
	currentPlanMutex.unlock();
	
	if(mCurrentPlan[mStartPoint] < 0)
	{
		ROS_ERROR("No way between robot and goal!");
		return false;
	}
	
	publishPlan();

	// For measuring RT: 
	current_plan_last_scan_mapCB_mapUpd_used_ts = mCurrentMap.last_scan_mapCB_mapUpd_used_ts;
	// current_... denotes the TS for the current copy of navPlan
	// latest_... denotes the TS for the navPlan being made, i.e. latest_ >= current_.
	current_mapCB_tf_navPlan_scan_ts = latest_mapCB_tf_navPlan_scan_ts;
	return true;
}

void RobotNavigator::publishPlan()
{
	nav_msgs::GridCells plan_msg;
	plan_msg.header.frame_id = mMapFrame.c_str();
	plan_msg.header.stamp = Time::now();
	
	plan_msg.cell_width = mCurrentMap.getResolution();
	plan_msg.cell_height = mCurrentMap.getResolution();
	
	unsigned int index = mStartPoint;
	std::vector<std::pair<double, double> > points;
	while(true)
	{
		unsigned int x = 0, y = 0;
		if(mCurrentMap.getCoordinates(x,y,index))
			points.push_back(std::pair<double, double>(
				((x+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX(), 
				((y+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY()
			));

		if(mCurrentPlan[index] == 0) break;
		
		unsigned int next_index = index;
		
		std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(index);
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mCurrentPlan[neighbors[i]] >= 0 && mCurrentPlan[neighbors[i]] < mCurrentPlan[next_index])
				next_index = neighbors[i];
		}
		
		if(index == next_index) break;
		index = next_index;
	}
	
	plan_msg.cells.resize(points.size());
	for(unsigned int i = 0; i < points.size(); i++)
	{
		plan_msg.cells[i].x = points[i].first;
		plan_msg.cells[i].y = points[i].second;
		plan_msg.cells[i].z = 0.0;
	}
	mPlanPublisher.publish(plan_msg);
}

bool RobotNavigator::correctGoalPose()
{
	// Reset the plan
	int mapSize = mCurrentMap.getSize();
	for(int i = 0; i < mapSize; i++)
	{
		mCurrentPlan[i] = -1;
	}
	
	// Initialize the queue with the goal point
	Queue queue;
	Entry goal(0.0, mGoalPoint);
	queue.insert(goal);
	mCurrentPlan[mGoalPoint] = 0;
	
	Queue::iterator next;
	double linear = mCurrentMap.getResolution();
	
	// Do full search with Dijkstra-Algorithm
	while(!queue.empty())
	{
		// Get the nearest cell from the queue
		next = queue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		queue.erase(next);
		
		if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;
		
		// Add all adjacent cells
		std::vector<unsigned int> neighbors = mCurrentMap.getNeighbors(index);
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			if(mCurrentMap.isFree(neighbors[i]))
			{
				mGoalPoint = neighbors[i];
				return true;
			}else
			{
				double newDistance = distance + linear;
				if(mCurrentPlan[neighbors[i]] == -1)
				{
					queue.insert(Entry(newDistance, neighbors[i]));
					mCurrentPlan[neighbors[i]] = newDistance;
				}
			}
		}
	}
	return false;
}

void RobotNavigator::stop()
{
	nav2d_operator::cmd stopMsg;
	stopMsg.Turn = 0;
	stopMsg.Velocity = 0;
	mCommandPublisher.publish(stopMsg);
	mStatus = NAV_ST_IDLE;
	mIsPaused = false;
	mIsStopped = false;
}

bool RobotNavigator::generateCommand()
{
	// These 2 values are set by the planner node, 
	// since it can run in ||al with the NavCmd node, we shoudl save these at 
	double using_curr_plan_mapUpd_scan_ts = current_plan_last_scan_mapCB_mapUpd_used_ts;
	double using_curr_plan_mapCB_tf_scan_ts = current_mapCB_tf_navPlan_scan_ts;

	// ROS_WARN("IN RobotNavigator::generateCommand - WIll try to publish a command for the Operator to follow.");
	// Do nothing when paused
	if(mIsPaused)
	{
		ROS_INFO_THROTTLE(1.0, "Navigator is paused and will not move now.");
		return true;
	}
	
	if(mStatus != NAV_ST_NAVIGATING && mStatus != NAV_ST_EXPLORING)
	{
		ROS_WARN_THROTTLE(1.0, "Navigator has status %d when generateCommand() was called!", mStatus);
		return false;
	}
	
	// Generate direction command from plan
	unsigned int current_x = 0, current_y = 0;
	if(!mCurrentMap.getCoordinates(current_x, current_y, mStartPoint)) // || !mCurrentMap.isFree(mStartPoint))
	{
		ROS_ERROR("Plan execution failed, robot not in map!");
		return false;
	}

	// TODO: make a copy of currentPlan here
	//Note that currentMap is only updated when navPlan starts executing.
	// It might happen that we pick up a larger mapSize here, than the currentPlan size...Maybe also store currentSz with CurrentPlan.
	currentPlanMutex.lock();
	double* currentPlan_copy = new double[currentPlanSize];
	// std::memcpy(currentPlan_copy, mCurrentPlan, currentPlanSize*sizeof(double) );
	for (int i = 0; i < currentPlanSize; i++)
		currentPlan_copy[i] = mCurrentPlan[i];
	int wrong_count = 0;
	for (int i = 0; i < currentPlanSize; i++)
		if (std::abs(mCurrentPlan[i] - currentPlan_copy[i]) > 0.01)
			wrong_count += 1;
	if (wrong_count > 0)
		ROS_ERROR("SHITTTTT!!!!! Could not copy mCurrentPlan correctly!!!! In generateCmd call, wrong_count %i, total %i", wrong_count, currentPlanSize);
	// maybe just copy elem by elem if this doesnt work.
	currentPlanMutex.unlock();

	unsigned int target = mStartPoint;
	int steps = mCommandTargetDistance / mCurrentMap.getResolution();
	for(int i = 0; i < steps; i++)
	{
		// ROS_WARN("Iterating over #steps : i = %i, steps : %i", i, steps);
		unsigned int bestPoint = target;
		std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(target);
		for(unsigned int i = 0; i < neighbors.size(); i++)
		{
			// if(mCurrentPlan[neighbors[i]] >= (unsigned int)0 && mCurrentPlan[neighbors[i]] < mCurrentPlan[bestPoint])
			if (currentPlan_copy[neighbors[i]] >= (unsigned int)0 && currentPlan_copy[neighbors[i]] < currentPlan_copy[bestPoint])	
				bestPoint = neighbors[i];
		}	
		target = bestPoint;
	}
	
	// Head towards (x,y)
	unsigned int x = 0, y = 0;
	if(!mCurrentMap.getCoordinates(x, y, target))
	{
		ROS_ERROR("Plan execution failed, target pose not in map!");
		return false;
	}
	double map_angle = atan2((double)y - current_y, (double)x - current_x);
	
	double angle = map_angle - mCurrentDirection;
	if(angle < -PI) angle += 2*PI;
	if(angle > PI) angle -= 2*PI;
	
	// Create the command message
	nav2d_operator::cmd msg;
	msg.Turn = -2.0 * angle / PI;
	if(msg.Turn < -1) msg.Turn = -1;
	if(msg.Turn >  1) msg.Turn = 1;
	
	// if(mCurrentPlan[mStartPoint] > mNavigationHomingDistance || mStatus == NAV_ST_EXPLORING)
	if(currentPlan_copy[mStartPoint] > mNavigationHomingDistance || mStatus == NAV_ST_EXPLORING)
		msg.Mode = 0;
	else
		msg.Mode = 1;
		
	// if(mCurrentPlan[mStartPoint] > 1.0 || mCurrentPlan[mStartPoint] < 0)
	if(currentPlan_copy[mStartPoint] > 1.0 || currentPlan_copy[mStartPoint] < 0)
	{
		msg.Velocity = 1.0;
	}else
	{	
		msg.Velocity = 0.5 + (currentPlan_copy[mStartPoint] / 2.0);
		// msg.Velocity = 0.5 + (mCurrentPlan[mStartPoint] / 2.0);
	}

	// For measuring RT:
	msg.LastScanTSScanMapCBMapUpdNavPlanNavCmd = using_curr_plan_mapUpd_scan_ts;
	msg.LastScanTSScanMapCBNavCmd = current_mapCB_tf_navCmd_scan_ts;// dont need to store 'using' here since NavCmd runs in the order : setCurrentPosition, stuff, generateCommand.
	msg.LastScanTSScanMapCBNavPlanNavCmd = using_curr_plan_mapCB_tf_scan_ts;
	// ROS_WARN("IN NAV2D::ROBOTNavigator generateCommand - About to publish command for operator!!!");
	mCommandPublisher.publish(msg);
	return true;
}

void RobotNavigator::receiveGetMapGoal(const nav2d_navigator::GetFirstMapGoal::ConstPtr &goal)
{
	ROS_ERROR("IN NAV2D::ROBOTNavigator - Received STartMapping Goal.");
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_ERROR("MAPPING Failed. Navigator is busy!");
		mGetMapActionServer->setAborted();
		return;
	}
	
	// Move the robot slowly ahead
	mStatus = NAV_ST_RECOVERING;
	nav2d_operator::cmd msg;
	msg.Turn = 0;
	msg.Velocity = 1.0;
	msg.Mode = 0;
	
	nav2d_navigator::GetFirstMapFeedback f;
	
	Rate loopRate(mFrequency);
	unsigned int cycles = 0;
	while(true)
	{
		if(!ok() || mGetMapActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_ERROR("MAPPING Failed. GetFirstMap has been preempted externally.");
			mGetMapActionServer->setPreempted();
			stop();
			return;
		}
		
		if(cycles >= 4*mFrequency) break;
		cycles++;
		
		mGetMapActionServer->publishFeedback(f);
		mCommandPublisher.publish(msg);
		spinOnce();
		loopRate.sleep();
	}
	
	if(!getMap() || !setCurrentPosition() || (get_pos_tf_error_ct>0) )
	{
		mGetMapActionServer->setAborted();
		stop();
		ROS_ERROR("MAPPING Failed. getMap or setCurrentPosition failed.");
		return;
	}
	
	// Do a full turn to have a initial map
	msg.Turn = 1;
	msg.Mode = 1;
	double lastDirection = mCurrentDirection;
	double turn = 0;
	while(true)
	{
		if(!ok() || mGetMapActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_ERROR("MAPPING Failed. GetFirstMap has been preempted externally.");
			mGetMapActionServer->setPreempted();
			stop();
			return;
		}
		
		setCurrentPosition();
		double deltaTheta = mCurrentDirection - lastDirection;
		while(deltaTheta < -PI) deltaTheta += 2*PI;
		while(deltaTheta >  PI) deltaTheta -= 2*PI;
		turn += deltaTheta;
		lastDirection = mCurrentDirection;
		if(turn > 2*PI || turn < -2*PI)
		{
			break;
		}

		// Lets try doing a half turn : 
		// To avoid the weird rotation issue with faster simulation in stage
		// if(turn > PI || turn < PI)
		// {
		// 	break;
		// }

		mGetMapActionServer->publishFeedback(f);
		mCommandPublisher.publish(msg);
		spinOnce();
		loopRate.sleep();
	}
	
	stop();
	mHasNewMap = false;
	
	if(getMap() && setCurrentPosition())
	{
		ROS_ERROR("MAPPING Successful!!!!");
		mGetMapActionServer->setSucceeded();
	}else
	{
		ROS_ERROR("MAPPING Failed. Navigator could not be initialized!");
		mGetMapActionServer->setAborted();
	}
}

void RobotNavigator::receiveLocalizeGoal(const nav2d_navigator::LocalizeGoal::ConstPtr &goal)
{
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("[Localize] Action aborted, Navigator is busy!");
		mGetMapActionServer->setAborted();
		return;
	}
	ROS_WARN("In nav2d_navigator:: receiveLocalizeGoal FUNCTION. Weird!!");
	
	// Move the robot slowly ahead
	mStatus = NAV_ST_RECOVERING;
	nav2d_operator::cmd msg;
	msg.Turn = 0;
	msg.Velocity = goal->velocity;
	msg.Mode = 0;
	
	nav2d_navigator::LocalizeFeedback f;
	
	mHasNewMap = false;
	Rate loopRate(1);
	while(true)
	{
		// Check if we are asked to preempt
		if(!ok() || mLocalizeActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("[Localize] Action has been preempted externally.");
			mLocalizeActionServer->setPreempted();
			stop();
			return;
		}
		
		if(mHasNewMap)
		{
			mCommandPublisher.publish(msg);
		}else
		{
			getMap();
		}
		
		// Check if we are localized successfully
		if(isLocalized())
		{
			ROS_INFO("[Localize] Action succeeded.");
			mLocalizeActionServer->setSucceeded();
			stop();
			return;
		}
		
		mLocalizeActionServer->publishFeedback(f);
		spinOnce();
		loopRate.sleep();
	}
}

void RobotNavigator::receiveMoveGoal(const nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal)
{
	ROS_WARN("IN NAV2D::ROBOTNavigator - IN receiveMoveGoal.");
	if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("Navigator is busy!");
		mMoveActionServer->setAborted();
		return;
	}
	
	ROS_DEBUG("Received Goal: %.2f, %.2f (in frame '%s')", goal->target_pose.x, goal->target_pose.y, goal->header.frame_id.c_str());

	// Start navigating according to the generated plan
	// TODO: change from ros rate to wall time rate, cuz ros time is 10times slower than real time.
	Rate loopRate(mFrequency);
	unsigned int cycle = 0;
	bool reached = false;
	int recheckCycles = mMinReplanningPeriod * mFrequency;
	
	double targetDistance = (goal->target_distance > 0) ? goal->target_distance : mNavigationGoalDistance;
	double targetAngle = (goal->target_angle > 0) ? goal->target_angle : mNavigationGoalAngle;
	
	while(true)
	{
		// Check if we are asked to preempt
		if(!ok() || mMoveActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_INFO("Navigation has been preempted externally.");
			mMoveActionServer->setPreempted();
			stop();
			return;
		}
		
		// Constantly replan every 3 seconds
		if(cycle == 0 || (recheckCycles && cycle % recheckCycles == 0))
		{
			WallTime startTime = WallTime::now();
			mStatus = NAV_ST_NAVIGATING;
			
			// Create the plan for navigation
			mHasNewMap = false;
			if(!preparePlan())
			{
				ROS_ERROR("Prepare failed!");
				mMoveActionServer->setAborted();
				stop();
				return;
			}
			
			int mapX =  (double)(goal->target_pose.x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
			int mapY =  (double)(goal->target_pose.y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
			if(mapX < 0) mapX = 0;
			if(mapX >= (int)mCurrentMap.getWidth()) mapX = mCurrentMap.getWidth() - 1;
			if(mapY < 0) mapY = 0;
			if(mapY >= (int)mCurrentMap.getHeight()) mapY = mCurrentMap.getHeight() - 1;

			bool success = false;
			if(mCurrentMap.getIndex(mapX, mapY, mGoalPoint))
				success = createPlan();
				
			if(!success)
			{
				if(correctGoalPose())
					success = createPlan();
			}
			
			if(!success)
			{
				ROS_ERROR("Planning failed!");
				mMoveActionServer->setAborted();
				stop();
				return;
			}
			
			WallTime endTime = WallTime::now();
			WallDuration d = endTime - startTime;
			ROS_INFO("Path planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
		}
		
		// Where are we now
		mHasNewMap = false;
		if(!setCurrentPosition())
		{
			ROS_ERROR("Navigation failed, could not get current position.");
			mMoveActionServer->setAborted();
			stop();
			return;
		}
		
		// Are we already close enough?
		if(!reached && mCurrentPlan[mStartPoint] <= targetDistance && mCurrentPlan[mStartPoint] >= 0)
		{
			ROS_INFO("Reached target, now turning to desired direction.");
			reached = true;
		}
		
		if(reached)
		{
			// Are we also headed correctly?
			double deltaTheta = mCurrentDirection - goal->target_pose.theta;
			while(deltaTheta < -PI) deltaTheta += 2*PI;
			while(deltaTheta >  PI) deltaTheta -= 2*PI;
			
			double diff = (deltaTheta > 0) ? deltaTheta : -deltaTheta;
			ROS_INFO_THROTTLE(1.0,"Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal->target_pose.theta, diff, targetAngle);
			if(diff <= targetAngle)
			{
				ROS_INFO("Final Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal->target_pose.theta, diff, targetAngle);
				break;
			}
			
			nav2d_operator::cmd msg;
			if(deltaTheta > 0)
			{
				msg.Turn = 1;
				msg.Velocity = deltaTheta;
			}else
			{
				msg.Turn = -1;
				msg.Velocity = -deltaTheta;
			}
			if(msg.Velocity > 1) msg.Velocity = 1;
				msg.Mode = 1;
				
			mCommandPublisher.publish(msg);
		}else
		{
			generateCommand();
		}
		
		// Publish feedback via ActionServer
		if(cycle%10 == 0)
		{
			nav2d_navigator::MoveToPosition2DFeedback fb;
			fb.distance = mCurrentPlan[mStartPoint];
			mMoveActionServer->publishFeedback(fb);
		}

		// Sleep remaining time
		cycle++;
		spinOnce();
		loopRate.sleep();
		if(loopRate.cycleTime() > ros::Duration(1.0 / mFrequency))
			ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",mFrequency, loopRate.cycleTime().toSec());
	}
	
	// Set ActionServer suceeded
	ROS_INFO("Goal reached.");
	nav2d_navigator::MoveToPosition2DResult r;
	r.final_pose.x = mCurrentPositionX;
	r.final_pose.y = mCurrentPositionY;
	r.final_pose.theta = mCurrentDirection;
	r.final_distance = mCurrentPlan[mStartPoint];
	mMoveActionServer->setSucceeded(r);
	stop();

}

void write_arrs_to_file(std::vector<double>& times, std::vector<double>& ts, std::string s)
{
	std::ofstream of;
	std::string ename;
	NodeHandle nh;
	nh.param<std::string>("/expt_name", ename, "");
	of.open("/home/ubuntu/robot_" + s + "_stats_" + ename + ".txt", std::ios_base::app);
	of << "\n" << s << " times: ";
	int sz = times.size();
	for (int i = 0; i < sz; i++)
		of << times[i] << " ";
	of << "\n" << s << " ts: ";
	for (int i = 0; i < sz; i++)
		of << std::to_string(ts[i]) << " ";

	times.clear();
	ts.clear();
}

void RobotNavigator::receiveExploreGoal(const nav2d_navigator::ExploreGoal::ConstPtr &goal)
{

	if(mStatus != NAV_ST_IDLE)
	{
		ROS_ERROR("Exploration failed. Navigator is busy!");
		mExploreActionServer->setAborted();
		nav_cmd_thread_shutdown_ = true;
		return;
	}

	// Making a separate thread for navigator cmd:

	struct timespec explore_start, explore_end;
	clock_gettime(CLOCK_MONOTONIC, &explore_start);

	clock_t explore_start_clk, explore_end_clk;
	explore_start_clk = clock();
	
	mStatus = NAV_ST_EXPLORING;
	unsigned int cycle = 0;
	unsigned int lastCheck = 0;
	unsigned int recheckCycles = mMinReplanningPeriod * mFrequency;
	unsigned int recheckThrottle = mMaxReplanningPeriod * mFrequency;

	double explore_start_mono_rt = get_time_now();
	
	ROS_ERROR("IN NAV2D::NAVIGATOR, received StartExploration service action. Mono RT %f, recheckCycles %i, recheckThrottle %i, replanningPeriod %f", explore_start_mono_rt, recheckCycles, recheckThrottle, mReplanningPeriod);
	// Move to exploration target
	Rate loopRate(mFrequency);

	ros::WallRate planUpdateLoopRate(1.0/mReplanningPeriod);

	ROS_ERROR("Publishing node navPlan tid %i, pid %i to controller.", ::gettid(), ::getpid());
        std_msgs::Header hdr;

        std::stringstream ss_e;
        ss_e << ::getpid() << " navp " << ::gettid();
        hdr.frame_id = ss_e.str();

	navp_exec_info_pub.publish(hdr);

	ros::NodeHandle nh;
	std::string use_td;
        nh.param<std::string>("/use_td", use_td, "");

        bool use_timer = ( use_td.find("yes") != std::string::npos );
        // use_timer = true;
	ROS_ERROR("In navPlanLoop, use_timer is %i, use_td param is %s",use_timer, use_td.c_str() );

	while(true)
	{
		struct timespec cb_start, cb_end;
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_start);

		// ROS_WARN("In nav2d::RobotNavigator STARTING Explore Loop!");
		// Check if we are asked to preempt
		if(!ok() || mExploreActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_ERROR("Exploration failed. Exploration has been preempted externally.");
			mExploreActionServer->setPreempted();
			nav_cmd_thread_shutdown_ = true;
			stop();
			return;
		}
		
		// Where are we now : Uses the tf output from Mapper Node.
		mHasNewMap = false;
		if(!setCurrentPosition(1) || (get_pos_tf_error_ct > 1) )
		{
			ROS_ERROR("Exploration failed. could not get current position.");
			mExploreActionServer->setAborted();
			nav_cmd_thread_shutdown_ = true;
			stop();
			return;
		}

		// Making the navigator Plan a periodic node:
		// running at period mReplanningPeriod.

		// // Regularly recheck for exploration target
		// bool reCheck = lastCheck == 0 || (recheckCycles && (cycle - lastCheck > recheckCycles));
		// bool planOk = mCurrentPlan && mCurrentPlan[mStartPoint] >= 0;
		// bool nearGoal = planOk && ((cycle - lastCheck) > recheckThrottle && mCurrentPlan[mStartPoint] <= mExplorationGoalDistance);
		
		// if(reCheck || nearGoal)
		// {

		double using_map_ts, using_tf_scan_ts;

			WallTime startTime = WallTime::now();
			lastCheck = cycle;

			bool success = false;
			if(preparePlan())
			{
				// the TS of TF being used by navPlan is set when setCurrentPosition is called from within preparePlan
				// TS of map being used is also set when setCurrentPosition is called from within preparePlan
				using_map_ts = mCurrentMap.last_scan_mapCB_mapUpd_used_ts;
				using_tf_scan_ts = current_mapCB_tf_navPlan_scan_ts;
				
				int result = mExplorationPlanner->findExplorationTarget(&mCurrentMap, mStartPoint, mGoalPoint);
				switch(result)
				{
				case EXPL_TARGET_SET:
					success = createPlan();
					mStatus = NAV_ST_EXPLORING;
					break;
				case EXPL_FINISHED:
					{
						nav2d_navigator::ExploreResult r;
						r.final_pose.x = mCurrentPositionX;
						r.final_pose.y = mCurrentPositionY;
						r.final_pose.theta = mCurrentDirection;
						mExploreActionServer->setSucceeded(r);
						explore_end_clk = clock();
						double explore_end_real_ts = get_time_now();
						ROS_ERROR("Exploration has finished. Total_Time_Taken_Clk: %f, Time of finish : %f #", (double)(explore_end_clk - explore_start_clk)/CLOCKS_PER_SEC, explore_end_real_ts);
					}
					stop();
					clock_gettime(CLOCK_MONOTONIC, &explore_end);
					ROS_ERROR("Exploration has finished. Total_Time_Taken: %f", (double)get_time_diff(explore_start, explore_end) );
					nav_cmd_thread_shutdown_ = true;
					return;
				case EXPL_WAITING:
					mStatus = NAV_ST_WAITING;
					{
						nav2d_operator::cmd stopMsg;
						stopMsg.Turn = 0;
						stopMsg.Velocity = 0;
						mCommandPublisher.publish(stopMsg);
					}
					ROS_ERROR("Exploration is waiting.");
					break;
				case EXPL_FAILED:
					ROS_ERROR("Got EXPL_FAILED from findExplorationTarget");
					break;
				default:
					ROS_ERROR("Exploration planner returned invalid status code: %d!", result);
				}
			}
			else
				ROS_ERROR("PreparePlan was unsuccessful!!");
			
			if(mStatus == NAV_ST_EXPLORING)
			{
				if(success)
				{
					WallTime endTime = WallTime::now();
					WallDuration d = endTime - startTime;
					ROS_ERROR("Exploration planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
					// indicates a plan was made successfully. [preparePlan, findExplorationtarget, createPlan.]
					// if ( (using_map_ts > nav_plan_last_map_used) || (using_tf_scan_ts > nav_plan_last_tf_used) )
					{
						double time_now = get_time_now();
						if (last_nav_plan_out > 0.0)
						{
							tput_nav_plan.push_back(time_now - last_nav_plan_out);
							ROS_ERROR("MADE NEW NAV PLAN, Tput : %f , realtime: %f", (time_now - last_nav_plan_out), time_now );
						}

						last_nav_plan_out = time_now;
						nav_plan_last_map_used = using_map_ts;
						nav_plan_last_tf_used = using_tf_scan_ts;
					}
				}else
				{
					mExploreActionServer->setAborted();
					stop();
					explore_end_clk = clock();
					clock_gettime(CLOCK_MONOTONIC, &explore_end);
					double explore_end_ts = get_time_now();
					ROS_ERROR("Exploration has failed. Total_Time_Taken_Clk: %fi, Time of finish : %f #", (double)(explore_end_clk - explore_start_clk)/CLOCKS_PER_SEC, explore_end_ts);
					ROS_ERROR("Exploration has failed. Total_Time_Taken: %f", (double)get_time_diff(explore_start, explore_end) );
					nav_cmd_thread_shutdown_ = true;
					return;
				}
			}

			clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_end);
			double plan_t = get_time_diff(cb_start, cb_end);

			double exec_rt_end = get_time_now();

			explore_cb_plan_times.push_back(plan_t);
			explore_cb_plan_ts.push_back(exec_rt_end);
			ROS_WARN("Exploration planning took %f cputime using CLOCK_THREAD_CPUTIME_ID", plan_t);

			std_msgs::Header np_ee;
			np_ee.frame_id = std::to_string(plan_t) + " navp";
			navp_exec_end_pub.publish(np_ee);

			// Making a separate thread for Navigator generateCommand
			// It should start after a plan has been made.
			if (nav_cmd_thread_ == NULL)
			{
				nav_cmd_thread_shutdown_ = false;
				ROS_ERROR("IN RobotNavigator::receiveExploreGoal, STARTING navCmd thread!!");
				nav_cmd_thread_ = new boost::thread(boost::bind(&RobotNavigator::navGenerateCmdLoop, this));
			}

			if (nav_cmd_thread_shutdown_)
			{
				// if the navC shuts down due to tf error, fail expl and exit.
			}
		// }
	
		// Moving this code to the navGenerateCmdLoop function.
		// if(mStatus == NAV_ST_EXPLORING)
		// {
		// 	// Publish feedback via ActionServer
		// 	if(cycle%10 == 0)
		// 	{
		// 		nav2d_navigator::ExploreFeedback fb;
		// 		fb.distance = mCurrentPlan[mStartPoint];
		// 		fb.robot_pose.x = mCurrentPositionX;
		// 		fb.robot_pose.y = mCurrentPositionY;
		// 		fb.robot_pose.theta = mCurrentDirection;
		// 		mExploreActionServer->publishFeedback(fb);
		// 	}

		// 	// Create a new command and send it to Operator
		// 	generateCommand();
		// }

		// cb_cmd is in a separate thread now, measuring ci there.
		// if (!(reCheck || nearGoal))
		// {
		// 	// add to cmd tme arr
		// 	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_end);
		// 	double cmd_t = get_time_diff(cb_start, cb_end);

		// 	explore_cb_cmd_times.push_back(cmd_t);
		// 	explore_cb_cmd_ts.push_back(ros::Time::now().toSec());
		// }
		
		// Sleep remaining time
		cycle++;

		// Moved spinOnce to setCurrentPosition function.

	
		if (explore_cb_plan_ts.size()%20 == 7)
		{
			write_arrs_to_file(explore_cb_plan_times, explore_cb_plan_ts, "nav2d_navigator_plan");
			write_arr_to_file(tput_nav_plan, "nav2d_navigator_plan");
		}
		
		// Nov: Scheduler triggers navPlan.
		// planUpdateLoopRate.sleep();
		if (use_timer)
			planUpdateLoopRate.sleep();
		else
		{
			boost::unique_lock<boost::mutex> lock(navp_trigger_mutex);
			if (navp_trigger_count > 0)
				navp_trigger_count -= 1;
			while (navp_trigger_count == 0)
			{
				cv_navp.wait(lock);
				ROS_ERROR("About to run navPlan ct:%i", navp_trigger_count);
			}
		}
	
	}
	ROS_ERROR("Exploration failed. THIS IS VERY WEIRD!!! This is the last LoC in receiveExplore CB. Should never reach here!");
}

void RobotNavigator::navGenerateCmdLoop()
{
	ros::NodeHandle nh;
	ROS_ERROR("In RobotNavigator::navGenerateCmdLoop frequency : %f", mFrequency);
	ros::WallRate r(mFrequency);
	int cycle = 0;

	std::string use_td;
        nh.param<std::string>("/use_td", use_td, "");

        bool use_timer = ( use_td.find("yes") != std::string::npos );
        ROS_ERROR("In navCmdLoop, use_timer is %i, use_td param is %s",use_timer, use_td.c_str() );

	while (nh.ok() && !nav_cmd_thread_shutdown_)
	{
		struct timespec cb_start, cb_end;
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_start);

		// Need to do the same checks as in the receiveExploreGoal function loop.
		if(!ok() || mExploreActionServer->isPreemptRequested() || mIsStopped)
		{
			ROS_WARN("Exploration has been preempted externally. Stopping nav_cmd_thread_");
			nav_cmd_thread_shutdown_ = true;
		}

		// Need to get latest position.
		if (!setCurrentPosition(0) || (get_pos_tf_error_ct > 1))
		{
			ROS_WARN("Exploration failed, could not get current position. Stopping nav_cmd_thread_");
			nav_cmd_thread_shutdown_ = true;
		}

		double using_tf_scan_ts = current_mapCB_tf_navCmd_scan_ts;
		double using_plan_ts = last_nav_plan_out; // navP itself has multiple inputs, so we consider the TS of the latest new plan. 

		// The generateCommand work, moved from the receiveExploreGoal:
		if(mStatus == NAV_ST_EXPLORING)
		{

			// Publish feedback via ActionServer
			if(cycle%10 == 0)
			{
				nav2d_navigator::ExploreFeedback fb;
				fb.distance = mCurrentPlan[mStartPoint];
				fb.robot_pose.x = mCurrentPositionX;
				fb.robot_pose.y = mCurrentPositionY;
				fb.robot_pose.theta = mCurrentDirection;
				mExploreActionServer->publishFeedback(fb);
			}

			// Create a new command and send it to Operator
			generateCommand();
			// ROS_ERROR("Generated command!");

			clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_end);
			double cmd_t = get_time_diff(cb_start, cb_end);

			double exec_rt_end = get_time_now();

			explore_cb_cmd_times.push_back(cmd_t);
			explore_cb_cmd_ts.push_back(exec_rt_end);
		
			std_msgs::Header nc_ee;
			nc_ee.frame_id = std::to_string(cmd_t) + " navc";
			navc_exec_end_pub.publish(nc_ee);

			double time_now = exec_rt_end;		
	
			// Measuring NavC tput : check if used a new tf or a new plan?
			// if ( (using_plan_ts > nav_cmd_last_plan_used) || (using_tf_scan_ts > nav_cmd_last_tf_used) )
			{
				// add to tput array.
				if (last_nav_cmd_out > 0.0)
				{
					tput_nav_cmd.push_back(time_now - last_nav_cmd_out);
					ROS_ERROR("MADE NEW NAV CMD, tput %f", time_now - last_nav_cmd_out);
				}

				last_nav_cmd_out = time_now;	
				nav_cmd_last_plan_used = using_plan_ts;
				nav_cmd_last_tf_used = using_tf_scan_ts;
			}
		}
		cycle++;

		// if (explore_cb_cmd_ts.size() == 1)
		// This only needs to be published exactly once.
		if (cycle == 1)
		{
			// Publish tid after 1st comd has been generated. Indicates that at this TS, mC,mU, nC,nP all are waiting for triggers.
			ROS_ERROR("Publishing node navCmd tid %i, pid %i to controller.", ::gettid(), ::getpid());
			std_msgs::Header hdr;
			
			std::stringstream ss_e;
			ss_e << ::getpid() << " navc " << ::gettid();
			hdr.frame_id = ss_e.str();
			navc_exec_info_pub.publish(hdr);	
		}


		if (explore_cb_cmd_ts.size()%50 == 20)
		{
			write_arrs_to_file(explore_cb_cmd_times, explore_cb_cmd_ts, "nav2d_navigator_cmd");
			write_arr_to_file(tput_nav_cmd, "nav2d_navigator_cmd");
		}
		
		// Nov: NavC triggered by scheduler.
		// r.sleep();
		if (use_timer)
			r.sleep();
		else
		{
			boost::unique_lock<boost::mutex> lock(navc_trigger_mutex);
			if (navc_trigger_count > 0)
				navc_trigger_count -= 1;
			while (navc_trigger_count == 0)
			{
				cv_navc.wait(lock);
				ROS_ERROR("About to generateCmd NAVC ct:%i", navc_trigger_count);
			}
		}
	
	}
}

bool RobotNavigator::isLocalized()
{
	return mTfListener.waitForTransform(mMapFrame, mRobotFrame, Time::now(), Duration(0.1));
}

bool RobotNavigator::setCurrentPosition(int x)
{
	StampedTransform transform;
	try
	{
		// Spinning here to get the latest tf's TS.
		spinOnce();
		mTfListener.lookupTransform(mMapFrame, mRobotFrame, Time(0), transform);
		if (x == 0)
			current_mapCB_tf_navCmd_scan_ts = current_mapper_tf_scan_ts; // THis is the TS of the scan used by the mapper_TF used by the NavCmd.
		else if (x == 1)
			latest_mapCB_tf_navPlan_scan_ts = current_mapper_tf_scan_ts; // THis is the TS of the scan used by the mapper_TF used by the NavPlan.

		// ROS_WARN("IN ROBOTNavigator:: setCurrentPosition, TS of TF bw /map and /base_footp : %f, x: %i", current_mapper_tf_scan_ts, x);
	}catch(TransformException ex)
	{
		get_pos_tf_error_ct += 1;
		ROS_ERROR("In setCurrentPosition() : Could not get robot position: %s, TF_ERROR CT: %i", ex.what(), get_pos_tf_error_ct);
		return true; // Jan: Modifying so that we exit from exploration only after we've seen this error 2/3 times.
	}
	double world_x = transform.getOrigin().x();
	double world_y = transform.getOrigin().y();
	double world_theta = getYaw(transform.getRotation());

	unsigned int current_x = (world_x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
	unsigned int current_y = (world_y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
	unsigned int i;
	
	// getIndex just checks if current_x and current_y are within the current GridMap.
	if(!mCurrentMap.getIndex(current_x, current_y, i))
	{
		if(mHasNewMap || !getMap() || !mCurrentMap.getIndex(current_x, current_y, i))
		{
			ROS_ERROR("Is the robot out of the map? Called by %i navC:0,P:1", x);
			return false;
		}
	}
	mStartPoint = i;
	mCurrentDirection = world_theta;
	mCurrentPositionX = world_x;
	mCurrentPositionY = world_y;
	return true;
}
