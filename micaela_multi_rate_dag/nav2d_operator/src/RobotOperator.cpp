#include <nav_msgs/GridCells.h>
#include <math.h>
#include <numeric>
#include <fstream>

#include <nav2d_operator/RobotOperator.h>

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

#define PI 3.14159265

double get_time_diff(timespec& a, timespec& b)
{
  return (( b.tv_sec + 1e-9*b.tv_nsec ) - ( a.tv_sec + 1e-9*a.tv_nsec ));
}

double get_time_now()
{

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double time_now = ts.tv_sec + 1e-9*ts.tv_nsec;

        return time_now;
}

void print_arr(std::vector<double> m_arr, std::string m)
{
        int perc7 = 75;
    int percentile = 95;
    int perc0 = 90;
    int perc2 = 99;
    float perc3 = 99.9;
    float perc4 = 99.99;

        int l = m_arr.size();
    if (l > 0)
    {
        std::sort(m_arr.begin(), m_arr.end());
        double avg = (std::accumulate(m_arr.begin(), m_arr.end(), 0.0))/l;
        double med = m_arr[l/2];
        double perc = m_arr[(l*percentile)/100];

        ROS_ERROR("Median, %i tail of %s is %f %f , arr sz %i , %i : %f, %i : %f, %i : %f, %f : %f, %f : %f #", percentile, m.c_str(), med, perc, l, perc7, m_arr[(l*perc7)/100], perc0, m_arr[(l*perc0)/100], perc2, m_arr[(l*perc2)/100], perc3, m_arr[(l*perc3)/100], perc4, m_arr[(l*perc4)/100]);
    }
}

void write_arr_to_file(std::vector<double>& arr, std::string m)
{
        std::string ename;
        ros::NodeHandle nh;
        nh.param<std::string>("/expt_name", ename, "");

        std::ofstream of;
        ROS_ERROR("IN RobotOperator: write_arr_to_file CALLED. Ename %s", ename.c_str());
    of.open("/home/ubuntu/robot_nav2d_" + ename + "_rt_stats.txt", std::ios_base::app);
    of << m << ": ";
    for (int i = 0; i < arr.size(); i++)
        of << std::to_string(arr[i]) << " ";
    of << "\n";

    arr.clear();
}

RobotOperator::RobotOperator() : mTf2Buffer(), mTf2Listener(mTf2Buffer)
{
	// Create the local costmap
	mLocalMap = new costmap_2d::Costmap2DROS("local_map", mTf2Buffer);
	mRasterSize = mLocalMap->getCostmap()->getResolution();
	
	// Publish / subscribe to ROS topics
	ros::NodeHandle robotNode;
	robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
	robotNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));
	mCommandSubscriber = robotNode.subscribe(COMMAND_TOPIC, 1, &RobotOperator::receiveCommand, this);
	mControlPublisher = robotNode.advertise<geometry_msgs::Twist>(CONTROL_TOPIC, 1);
	mCostPublisher = robotNode.advertise<geometry_msgs::Vector3>("costs", 1);

	ROS_ERROR("Subscribing to COmmand_Topic : %s, Publishing COntrol %s, Publishing Cost at costs topic", COMMAND_TOPIC, CONTROL_TOPIC);

	// Get parameters from the parameter server
	ros::NodeHandle operatorNode("~/");
	operatorNode.param("publish_route", mPublishRoute, false);
	if(mPublishRoute)
	{
		ROS_INFO("Will publish desired direction on '%s' and control direction on '%s'.", ROUTE_TOPIC, PLAN_TOPIC);
		mTrajectoryPublisher = operatorNode.advertise<nav_msgs::GridCells>(ROUTE_TOPIC, 1);
		mPlanPublisher = operatorNode.advertise<nav_msgs::GridCells>(PLAN_TOPIC, 1);
	}
	operatorNode.param("max_free_space", mMaxFreeSpace, 5.0);
	operatorNode.param("safety_decay", mSafetyDecay, 0.95);
	operatorNode.param("safety_weight", mSafetyWeight, 1);
	operatorNode.param("conformance_weight", mConformanceWeight, 1);
	operatorNode.param("continue_weight", mContinueWeight, 1);
	operatorNode.param("escape_weight", mEscapeWeight, 1);
	operatorNode.param("max_velocity", mMaxVelocity, 1.0);

	ROS_ERROR("RobotOperator mMaxFreeSpace %f, mSafetyDecay %f, mSafetyWeight %i, mConformanceWeight %i, mContinueWeight %i, mMaxVelocity %f, mEscapeWeight %i", mMaxFreeSpace, mSafetyDecay, mSafetyWeight, mConformanceWeight, mContinueWeight, mMaxVelocity, mEscapeWeight);

	// Apply tf_prefix to all used frame-id's
	mRobotFrame = mTfListener.resolve(mRobotFrame);
	mOdometryFrame = mTfListener.resolve(mOdometryFrame);

	// Initialize the lookup table for the driving directions
	ROS_INFO("Initializing LUT...");
	initTrajTable();
	ROS_INFO("...done!");
	
	// Set internal parameters
	mDesiredDirection = 0;
	mDesiredVelocity = 0;
	mCurrentDirection = 0;
	mCurrentVelocity = 0;
	mDriveMode = 0;
	mRecoverySteps = 0;

	// For measuring RT:
        last_scan_lcmp_ts = -1.0;
        last_scan_mapCB_navCmd_ts = -1.0;
        last_scan_mapCB_navPlan_navCmd_ts = -1.0;
        last_scan_mapCB_mapUpd_navPlan_navCmd_ts = -1.0;

        last_scan_lcmp_out = 0.0;
        last_scan_mapCB_navCmd_out = 0.0;
        last_scan_mapCB_navPlan_navCmd_out = 0.0;
        last_scan_mapCB_mapUpd_navPlan_navCmd_out = 0.0;

        last_scan_mapCB_mapUpd_navPlan_navCmd_recv = -1.0;

	// For publishing end of CC exec to controller:
        // exec_end_cc_pub = robotNode.advertise<std_msgs::Header>("/robot_0/exec_end_lp", 1, false);

        count_scan_lcmp = 0;
        count_scan_mapCB_navCmd = 0;
        count_scan_mapCB_navPlan_navCmd = 0;
        count_scan_mapCB_mapUpd_navPlan_navCmd = 0;
}

RobotOperator::~RobotOperator()
{
	for(int i = 0; i < LUT_RESOLUTION; i++)
	{
		delete mTrajTable[i];
	}
}

void RobotOperator::initTrajTable()
{
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		mTrajTable[i] = NULL;
	}	
	for(int i = 1; i < LUT_RESOLUTION; i++)
	{
		double tw = -PI * i / LUT_RESOLUTION;
		double tx = cos(tw) + 1;
		double ty = -sin(tw);
		double tr = ((tx*tx)+(ty*ty))/(ty+ty);
		std::vector<geometry_msgs::Point32> points;
		double alpha = 0;
		while(alpha < PI)
		{
			double x = tr * sin(alpha);
			double y = tr * (1.0 - cos(alpha));
			geometry_msgs::Point32 p;
			p.x = x;
			p.y = y;
			p.z = 0;
			points.push_back(p);
			alpha += mRasterSize / tr;
		}
		// Add the PointCloud to the LUT
		// Circle in forward-left direction
		sensor_msgs::PointCloud* flcloud = new sensor_msgs::PointCloud();
		flcloud->header.stamp = ros::Time(0);
		flcloud->header.frame_id = mRobotFrame;
		flcloud->points.resize(points.size());
		
		// Circle in forward-right direction
		sensor_msgs::PointCloud* frcloud = new sensor_msgs::PointCloud();
		frcloud->header.stamp = ros::Time(0);
		frcloud->header.frame_id = mRobotFrame;
		frcloud->points.resize(points.size());
		
		// Circle in backward-left direction
		sensor_msgs::PointCloud* blcloud = new sensor_msgs::PointCloud();
		blcloud->header.stamp = ros::Time(0);
		blcloud->header.frame_id = mRobotFrame;
		blcloud->points.resize(points.size());
		
		// Circle in backward-right direction
		sensor_msgs::PointCloud* brcloud = new sensor_msgs::PointCloud();
		brcloud->header.stamp = ros::Time(0);
		brcloud->header.frame_id = mRobotFrame;
		brcloud->points.resize(points.size());
		
		for(unsigned int j = 0; j < points.size(); j++)
		{
			flcloud->points[j] = points[j];
			frcloud->points[j] = points[j];
			blcloud->points[j] = points[j];
			brcloud->points[j] = points[j];
			
			frcloud->points[j].y *= -1;
			blcloud->points[j].x *= -1;
			brcloud->points[j].x *= -1;
			brcloud->points[j].y *= -1;
		}
		mTrajTable[LUT_RESOLUTION - i] = flcloud;
		mTrajTable[LUT_RESOLUTION + i] = frcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) - i] = blcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) + i] = brcloud;
	}
	
	// Add First and Last LUT-element
	geometry_msgs::Point32 p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	
	sensor_msgs::PointCloud* turn = new sensor_msgs::PointCloud();
	turn->header.stamp = ros::Time(0);
	turn->header.frame_id = mRobotFrame;
	turn->points.resize(1);
	turn->points[0] = p;
	
	int straight_len = 5.0 / mRasterSize;
	
	sensor_msgs::PointCloud* fscloud = new sensor_msgs::PointCloud();
	fscloud->header.stamp = ros::Time(0);
	fscloud->header.frame_id = mRobotFrame;
	fscloud->points.resize(straight_len);
	
	sensor_msgs::PointCloud* bscloud = new sensor_msgs::PointCloud();
	bscloud->header.stamp = ros::Time(0);
	bscloud->header.frame_id = mRobotFrame;
	bscloud->points.resize(straight_len);
	
	for(int i = 0; i < straight_len; i++)
	{
		fscloud->points[i] = p;
		bscloud->points[i] = p;
		bscloud->points[i].x *= -1;
		p.x += mRasterSize;
	}
	
	mTrajTable[LUT_RESOLUTION] = fscloud;
	mTrajTable[LUT_RESOLUTION*3 + 1] = bscloud;
	
	mTrajTable[0] = turn;
	mTrajTable[LUT_RESOLUTION*2] = turn;
	mTrajTable[LUT_RESOLUTION*2 + 1] = turn;
	mTrajTable[LUT_RESOLUTION*4 + 1] = turn;
	
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		if(!mTrajTable[i])
		{
			ROS_ERROR("Table entry %d has not been initialized!", i);
		}
	}	
}

void RobotOperator::receiveCommand(const nav2d_operator::cmd::ConstPtr& msg)
{
	if(msg->Turn < -1 || msg->Turn > 1)
	{
		// The given direction is invalid.
		// Something is going wrong, so better stop the robot:
		mDesiredDirection = 0;
		mDesiredVelocity = 0;
		mCurrentDirection = 0;
		mCurrentVelocity = 0;
		ROS_ERROR("Invalid turn direction on topic '%s'!", COMMAND_TOPIC);
		return;
	}
	mDesiredDirection = msg->Turn;
	mDesiredVelocity = msg->Velocity * mMaxVelocity;
	mDriveMode = msg->Mode;

	last_scan_mapCB_mapUpd_navPlan_navCmd_recv = msg->LastScanTSScanMapCBMapUpdNavPlanNavCmd;
        last_scan_mapCB_navCmd_recv = msg->LastScanTSScanMapCBNavCmd;
        last_scan_mapCB_navPlan_navCmd_recv = msg->LastScanTSScanMapCBNavPlanNavCmd;

	ROS_ERROR("NAV2D : RobotOperator got command : direction %f, velocity %f, MODE : %i, with Scan input wrt Scan-MapCB-MapU-NavP-NavC-LP TS: %f, Scan input wrt S-MapCB-NavC-LP %f, Scan input wrt S-MapCB-NavP-NavC-LP %f", mDesiredDirection, mDesiredVelocity, mDriveMode, last_scan_mapCB_mapUpd_navPlan_navCmd_recv, last_scan_mapCB_navCmd_recv, last_scan_mapCB_navPlan_navCmd_recv);
}

void RobotOperator::executeCommand()
{
	struct timespec exec_start, exec_end;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &exec_start);

	// 1. Get a copy of the costmap to work on.
	mCostmap = mLocalMap->getCostmap();
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));
	double bestDirection, d;

	// For measuring RT:
        double using_scan_lcmp_ts = mCostmap->getLatestObsTSUsed();
        double using_scan_mapCB_mapUpd_navPlan_navCmd_ts = last_scan_mapCB_mapUpd_navPlan_navCmd_recv;
        double using_s_mapCB_navC_ts = last_scan_mapCB_navCmd_recv;
        double using_s_mapCB_navP_navC_ts = last_scan_mapCB_navPlan_navCmd_recv;


	// 2. Set velocity and direction depending on drive mode
	switch(mDriveMode)
	{
	case 0:
		bestDirection = findBestDirection();
		d = bestDirection - mCurrentDirection;
		if(d < -0.2) d = -0.2;
		if(d > 0.2) d = 0.2;
		mCurrentDirection += d;
		mCurrentVelocity = mDesiredVelocity;
		break;
	case 1:
		mCurrentDirection = mDesiredDirection;
		mCurrentVelocity = mDesiredVelocity;
		break;
	default:
		ROS_ERROR("Invalid drive mode!");
		mCurrentVelocity = 0.0;
	}
	
	// Create some Debug-Info
	evaluateAction(mCurrentDirection, mCurrentVelocity, true);
	
	sensor_msgs::PointCloud* originalCloud = getPointCloud(mCurrentDirection, mDesiredVelocity);
	sensor_msgs::PointCloud transformedCloud;

	try
	{
		mTfListener.transformPointCloud(mOdometryFrame,*originalCloud,transformedCloud);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
	
	// Determine maximum linear velocity
	int freeCells = calculateFreeSpace(&transformedCloud);
	double freeSpace = mRasterSize * freeCells;

	double safeVelocity = (freeSpace / mMaxFreeSpace) + 0.05;
	if(freeCells == transformedCloud.points.size() && safeVelocity < 0.5)
		safeVelocity = 0.5;
		
	if(freeSpace < 0.3 && freeCells < transformedCloud.points.size())
		safeVelocity = 0;

	if(safeVelocity > mMaxVelocity)
		safeVelocity = mMaxVelocity;

	// Check whether the robot is stuck
	if(mRecoverySteps > 0) mRecoverySteps--;
	if(safeVelocity < 0.1)
	{
		if(mDriveMode == 0)
		{
			mRecoverySteps = 30; // Recover for 3 seconds
			ROS_WARN_THROTTLE(1, "Robot is stuck! Trying to recover...");
		}else
		{
			mCurrentVelocity = 0;
			ROS_WARN_THROTTLE(1, "Robot cannot move further in this direction!");
		}
	}

	// Publish route via ROS (mainly for debugging)
	if(mPublishRoute)
	{
		nav_msgs::GridCells route_msg;
		route_msg.header.frame_id = mOdometryFrame;
		route_msg.header.stamp = ros::Time::now();
	
		route_msg.cell_width = mCostmap->getResolution();
		route_msg.cell_height = mCostmap->getResolution();
	
		route_msg.cells.resize(freeCells);
		for(int i = 0; i < freeCells; i++)
		{
			route_msg.cells[i].x = transformedCloud.points[i].x;
			route_msg.cells[i].y = transformedCloud.points[i].y;
			route_msg.cells[i].z = transformedCloud.points[i].z;
		}
		mTrajectoryPublisher.publish(route_msg);
	
		// Publish plan via ROS (mainly for debugging)
		sensor_msgs::PointCloud* originalPlanCloud = getPointCloud(mDesiredDirection, mDesiredVelocity);
		sensor_msgs::PointCloud transformedPlanCloud;

		try
		{
			mTfListener.transformPointCloud(mOdometryFrame,*originalPlanCloud,transformedPlanCloud);
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}
		nav_msgs::GridCells plan_msg;
		plan_msg.header = route_msg.header;
	
		plan_msg.cell_width = mCostmap->getResolution();
		plan_msg.cell_height = mCostmap->getResolution();
	
		int freeSpacePlan = calculateFreeSpace(&transformedPlanCloud);
		plan_msg.cells.resize(freeSpacePlan);
		for(int i = 0; i < freeSpacePlan; i++)
		{
			plan_msg.cells[i].x = transformedPlanCloud.points[i].x;
			plan_msg.cells[i].y = transformedPlanCloud.points[i].y;
			plan_msg.cells[i].z = transformedPlanCloud.points[i].z;
		}
		mPlanPublisher.publish(plan_msg);
	}
	
	// Publish result via Twist-Message
	geometry_msgs::Twist controlMsg;
	double velocity = mCurrentVelocity;
	if(mCurrentDirection == 0)
	{
		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = 0;
	}else if(mCurrentDirection == -1 || mCurrentDirection == 1)
	{
		controlMsg.linear.x = 0;
		controlMsg.angular.z = -1.0 * mCurrentDirection * velocity;
	}else
	{
		double x = sin(mCurrentDirection * PI);
		double y = (cos(mCurrentDirection * PI) + 1);
		double r = ((x*x) + (y*y)) / (2*x);
		double abs_r = (r > 0) ? r : -r;
		velocity /= (1 + (1.0/abs_r));
		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = -1.0 / r * controlMsg.linear.x;
	}
	mControlPublisher.publish(controlMsg);

	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &exec_end);
        double exec_rt_end = get_time_now();
        double t = get_time_diff(exec_start, exec_end);
        operator_loop_times.push_back(t);
        operator_loop_ts.push_back(exec_rt_end);

	double time_now = get_time_now();

        double lat_scan_lcmp = time_now - using_scan_lcmp_ts;
        double lat_scan_mapCB_navCmd = time_now - using_s_mapCB_navC_ts;
        double lat_scan_mapCB_navPlan_navCmd = time_now - using_s_mapCB_navP_navC_ts;
        double lat_scan_mapCB_mapUpd_navPlan_navCmd = time_now - using_scan_mapCB_mapUpd_navPlan_navCmd_ts;

        if (lat_scan_lcmp > 2.0)
                ROS_ERROR("Latency weird wrt Scan-LC-LP : using_scan_lcmp_ts : %f", using_scan_lcmp_ts);

	if ((using_scan_lcmp_ts > last_scan_lcmp_ts) && (lat_scan_lcmp < 100.0))
        {
                lat_scan_lcmp_arr.push_back(lat_scan_lcmp);
                ts_scan_lcmp_arr.push_back(time_now);
                count_scan_lcmp += 1;

                if (last_scan_lcmp_out > 0.0)
                {
                        double tput_scan_lcmp = time_now - last_scan_lcmp_out;
                        if (tput_scan_lcmp > 0.15)
                                ROS_WARN("RobotOperator: CC Tput mor than 0.15 : %f, last_out: %f", tput_scan_lcmp, last_scan_lcmp_out);
                        tput_scan_lcmp_arr.push_back(tput_scan_lcmp);
                }

                // if (tput_scan_lcmp_arr.size() > 2)
                if (count_scan_lcmp > 1)
                {
                        // calculate RT
                        double rt = time_now - last_scan_lcmp_ts;
                        rt_scan_lcmp_arr.push_back(rt);
                }

		last_scan_lcmp_ts = using_scan_lcmp_ts;
                // last_scan_lcmp_out = ros::Time::now().toSec();
                last_scan_lcmp_out = time_now;
        }

        if ( (using_scan_mapCB_mapUpd_navPlan_navCmd_ts > last_scan_mapCB_mapUpd_navPlan_navCmd_ts) && (lat_scan_mapCB_mapUpd_navPlan_navCmd < 100.0) )
        {

		lat_scan_mapCB_mapUpd_navPlan_navCmd_arr.push_back(lat_scan_mapCB_mapUpd_navPlan_navCmd);
                ts_scan_mapCB_mapUpd_navPlan_navCmd_arr.push_back(time_now);
                count_scan_mapCB_mapUpd_navPlan_navCmd += 1;

                ROS_ERROR("Adding to Latency wrt LONG chain!! Using scan TS %f, Lat: %f", using_scan_mapCB_mapUpd_navPlan_navCmd_ts, lat_scan_mapCB_mapUpd_navPlan_navCmd);

                if (last_scan_mapCB_mapUpd_navPlan_navCmd_out > 0.0)
                {
                        double tput_scan_mapCB_mapU_navP_navC = time_now - last_scan_mapCB_mapUpd_navPlan_navCmd_out;
                        tput_scan_mapCB_mapUpd_navPlan_navCmd_arr.push_back(tput_scan_mapCB_mapU_navP_navC);
                        ROS_ERROR("Adding to TPUT wrt LONG chain!! , Last out %f, Tput : %f", last_scan_mapCB_mapUpd_navPlan_navCmd_out, tput_scan_mapCB_mapU_navP_navC);
                }

		if (count_scan_mapCB_mapUpd_navPlan_navCmd > 1)
                {
                        // calculate RT
                        double rt = time_now - last_scan_mapCB_mapUpd_navPlan_navCmd_ts;
                        rt_scan_mapCB_mapUpd_navPlan_navCmd_arr.push_back(rt);
                }

                last_scan_mapCB_mapUpd_navPlan_navCmd_ts = using_scan_mapCB_mapUpd_navPlan_navCmd_ts;
                last_scan_mapCB_mapUpd_navPlan_navCmd_out = time_now;
        }

	if ( (using_s_mapCB_navC_ts > last_scan_mapCB_navCmd_ts) && (lat_scan_mapCB_navCmd < 100.0) )
        {
                lat_scan_mapCB_navCmd_arr.push_back(lat_scan_mapCB_navCmd);
                ts_scan_mapCB_navCmd_arr.push_back(time_now);
                count_scan_mapCB_navCmd += 1;

                ROS_ERROR("Adding Latency wrt S-MapCB-NavC-LP chain!! Using scanTS %f, Lat %f", using_s_mapCB_navC_ts, lat_scan_mapCB_navCmd);

                if (last_scan_mapCB_navCmd_out > 0.0)
                {
                        double tput = time_now - last_scan_mapCB_navCmd_out;
                        tput_scan_mapCB_navCmd_arr.push_back(tput);
                        ROS_ERROR("Adding tput wrt S-MapCB-NavC-LP!!, Last out %f, Tput %f", last_scan_mapCB_navCmd_out, tput);
                }

		if (count_scan_mapCB_navCmd > 1)
                {
                        // RT
                        double rt = time_now - last_scan_mapCB_navCmd_ts;
                        rt_scan_mapCB_navCmd_arr.push_back(rt);
                }

                last_scan_mapCB_navCmd_ts = using_s_mapCB_navC_ts;
                last_scan_mapCB_navCmd_out = time_now;
        }

        if ( (using_s_mapCB_navP_navC_ts > last_scan_mapCB_navPlan_navCmd_ts) & (lat_scan_mapCB_navPlan_navCmd < 100.0) )
	{
                lat_scan_mapCB_navPlan_navCmd_arr.push_back(lat_scan_mapCB_navPlan_navCmd);
                ts_scan_mapCB_navPlan_navCmd_arr.push_back(time_now);
                count_scan_mapCB_navPlan_navCmd += 1;

                ROS_ERROR("Adding Latency wrt S-MapCB-NavP-NavC-LP!! Using scanTS %f, Lat %f", using_s_mapCB_navP_navC_ts, lat_scan_mapCB_navPlan_navCmd);

                if (last_scan_mapCB_navPlan_navCmd_out > 0.0)
                {
                        double tput = time_now - last_scan_mapCB_navPlan_navCmd_out;
                        tput_scan_mapCB_navPlan_navCmd_arr.push_back(tput);
                        ROS_ERROR("Adding tput wrt S-MapCB-NavP-NavC-LP!!, Last out %f, Tput %f", last_scan_mapCB_navPlan_navCmd_out, tput);
                }

		if (count_scan_mapCB_navPlan_navCmd > 1)
                {
                        // RT
                        double rt = time_now - last_scan_mapCB_navPlan_navCmd_ts;
                        rt_scan_mapCB_navPlan_navCmd_arr.push_back(rt);
                }

                last_scan_mapCB_navPlan_navCmd_ts = using_s_mapCB_navP_navC_ts;
                last_scan_mapCB_navPlan_navCmd_out = time_now;
        }

        if (operator_loop_times.size() % 200 == 150)
        {
                // write to file
                std::string ss = "nav2d_operator_loop";
                write_arrs_to_file(operator_loop_times, operator_loop_ts, ss);
        }

	if (operator_loop_times.size() % 200 == 30)
        {
                print_arr(lat_scan_lcmp_arr, "Latency wrt Scan-LC-LP chain");
                print_arr(tput_scan_lcmp_arr, "Tput wrt Scan-LC-LP chain");
                print_arr(rt_scan_lcmp_arr, "RT wrt S-LC-LP chain");
                print_arr(ts_scan_lcmp_arr, "Checking TS...");

                write_arr_to_file(lat_scan_lcmp_arr, "Latency_Scan_LC_LP");
                write_arr_to_file(tput_scan_lcmp_arr, "Tput_Scan_LC_LP");
                write_arr_to_file(rt_scan_lcmp_arr, "RT_Scan_LC_LP");
                write_arr_to_file(ts_scan_lcmp_arr, "TS_Scan_LC_LP");

        }

	if (operator_loop_times.size() % 200 == 60)
        {
                print_arr(lat_scan_mapCB_mapUpd_navPlan_navCmd_arr, "Latency wrt Scan-MapCB-MapU-NavP-NavC-LP chain");
                print_arr(tput_scan_mapCB_mapUpd_navPlan_navCmd_arr, "Tput wrt Scan-MapCB-MapU-NavP-NavC-LP chain");
                print_arr(rt_scan_mapCB_mapUpd_navPlan_navCmd_arr, "RT wrt S-MapCB-MapU-NavP-NavC-LP chain");

                write_arr_to_file(lat_scan_mapCB_mapUpd_navPlan_navCmd_arr, "Latency_Scan_MapCB_MapU_NavP_NavC_LP");
                write_arr_to_file(tput_scan_mapCB_mapUpd_navPlan_navCmd_arr, "Tput_Scan_MapCB_MapU_NavP_NavC_LP");
                write_arr_to_file(rt_scan_mapCB_mapUpd_navPlan_navCmd_arr, "RT_Scan_MapCB_MapU_NavP_NavC_LP");
                write_arr_to_file(ts_scan_mapCB_mapUpd_navPlan_navCmd_arr, "TS_Scan_MapCB_MapU_NavP_NavC_LP");

        }

	if (operator_loop_times.size() % 200 == 90)
        {
                print_arr(lat_scan_mapCB_navCmd_arr, "Latency wrt Scan-MapCB-NavCmd-LP chain");
                print_arr(tput_scan_mapCB_navCmd_arr, "Tput wrt Scan-MapCB-NavCmd-LP");
                print_arr(rt_scan_mapCB_navCmd_arr, "RT wrt S-MapCB-NavC-LP chain");

                write_arr_to_file(lat_scan_mapCB_navCmd_arr, "Latency_Scan_MapCB_NavCmd_LP");
                write_arr_to_file(tput_scan_mapCB_navCmd_arr, "Tput_Scan_MapCB_NavCmd_LP");
                write_arr_to_file(rt_scan_mapCB_navCmd_arr, "RT_Scan_MapCB_NavCmd_LP");
                write_arr_to_file(ts_scan_mapCB_navCmd_arr, "TS_Scan_MapCB_NavCmd_LP");

        }

	if (operator_loop_times.size() % 200 == 120)
        {
                print_arr(lat_scan_mapCB_navPlan_navCmd_arr, "Latency wrt Scan-MapCB-NavP-NavC-LP chain");
                print_arr(tput_scan_mapCB_navPlan_navCmd_arr, "Tput wrt Scan-MapCB-NavP-NavC-LP chain");
                print_arr(rt_scan_mapCB_navPlan_navCmd_arr, "RT wrt S-MapCB-NavP-NavC-LP chain");

                write_arr_to_file(lat_scan_mapCB_navPlan_navCmd_arr, "Latency_Scan_MapCB_NavPlan_NavCmd_LP");
                write_arr_to_file(tput_scan_mapCB_navPlan_navCmd_arr, "Tput_Scan_MapCB_NavPlan_NavCmd_LP");
                write_arr_to_file(rt_scan_mapCB_navPlan_navCmd_arr, "RT_Scan_MapCB_NavPlan_NavCmd_LP");
                write_arr_to_file(ts_scan_mapCB_navPlan_navCmd_arr, "TS_Scan_MapCB_NavPlan_NavCmd_LP");
        }
}

void write_arrs_to_file(std::vector<double>& times, std::vector<double>& ts, std::string s)
{
        std::string ename;
        ros::NodeHandle nh;
        nh.param<std::string>("/expt_name", ename, "");

        int sz = times.size();
        if (sz > 0)
        {
                std::ofstream of;
                of.open("/home/ubuntu/robot_" + s + "_stats_" + ename + ".txt", std::ios_base::app);
                of << "\n" << s << " times: ";
                for (int i = 0; i < sz; i++)
                        of << times[i] << " ";
                of << "\n" << s << " ts: ";
                for (int i = 0; i < sz; i++)
                        of << std::to_string(ts[i]) << " ";

                times.clear();
                ts.clear();
        }
}

int RobotOperator::calculateFreeSpace(sensor_msgs::PointCloud* cloud)
{	
	unsigned int mx, my;
	int length = cloud->points.size();
	int freeSpace = 0;
	for(int i = 0; i < length; i++)
	{
		if(mCostmap->worldToMap(cloud->points[i].x, cloud->points[i].y, mx, my))
		{
			if(mCostmap->getCost(mx,my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				freeSpace++;
			}else
			{
				break;
			}
		}else
		{
			break;
		}
	}
	return freeSpace;
}

double RobotOperator::evaluateAction(double direction, double velocity, bool debug)
{
	sensor_msgs::PointCloud* originalCloud = getPointCloud(direction, velocity);
	sensor_msgs::PointCloud transformedCloud;
	try
	{
		mTfListener.transformPointCloud(mOdometryFrame, *originalCloud,transformedCloud);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return 1;
	}
	
	double valueSafety = 0.0;      // How safe is it to move in that direction?
	double valueEscape = 0.0;      // How much does the safety improve?
	double valueConformance = 0.0; // How conform is it with the desired direction?
	double valueContinue = 0.0;    // How conform is it with the previous command?
	
	double decay = 1.0;
	double safe_max = 0.0;
	double cost_max = 0.0;
	double cost_start = 1.0;
	
	// Calculate safety and escape value
	int length = transformedCloud.points.size();
	for(int i = 0; i < length; i++)
	{
		unsigned int mx, my;
		double cell_cost;
		if(mCostmap->worldToMap(transformedCloud.points[i].x, transformedCloud.points[i].y, mx, my))
		{
			cell_cost = (double)mCostmap->getCost(mx,my) / costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
			if(cell_cost >= 1.0)
			{
				// Trajectory hit an obstacle
				break;
			}
		}
		if(i == 0)
			cost_start = cell_cost;
		double cost = cell_cost * decay;
		double safe = (cost_start - cell_cost) * decay * 2.0;
		
		if(cost > cost_max) cost_max = cost;
		if(safe > safe_max) safe_max = safe;
		
		decay *= mSafetyDecay;
	}
	
	double action_value = 0.0;
	double normFactor = 0.0;
	
	// Add safety value
	valueSafety = 1.0 - cost_max;
	action_value += valueSafety * mSafetyWeight;
	normFactor += mSafetyWeight;
	
	// Add escape value
	valueEscape = safe_max;
	action_value += valueEscape * mEscapeWeight;
	normFactor += mEscapeWeight;

	if(mRecoverySteps == 0)
	{
		// Calculate continuety value
		valueContinue = (mCurrentDirection - direction) * 0.5;
		valueContinue = 1.0 - (valueContinue * valueContinue);
		
		// Calculate conformance value
		double corr = (mDesiredDirection - direction) * PI;
		valueConformance = 0.5 * cos(corr) + 0.5;
		
		// Add both to action value
		action_value += valueConformance * mConformanceWeight;
		action_value += valueContinue * mContinueWeight;
		normFactor += mConformanceWeight + mContinueWeight;
	}
	
	action_value /= normFactor;
	
	if(debug)
	{
		geometry_msgs::Vector3 cost_msg;
		cost_msg.x = valueSafety;
		cost_msg.y = valueEscape;
		cost_msg.z = valueConformance;
		mCostPublisher.publish(cost_msg); 
	}
	
	return action_value;
}

double diff(double v1, double v2)
{
	if(v1 > v2)
		return v1 - v2;
	else
		return v2 - v1;
}

double RobotOperator::findBestDirection()
{
	double best_dir = -1.0;
	double best_value = 0.0;
	double step = 0.01;
	double dir = -1.0;
	
	while(dir <= 1.0)
	{
		double value = evaluateAction(dir, mDesiredVelocity);
		if(value > best_value)
		{
			best_dir = dir;
			best_value = value;
		}
		dir += step;
	}
	return best_dir;
}

sensor_msgs::PointCloud* RobotOperator::getPointCloud(double direction, double velocity)
{
	if(direction < -1) direction = -1;
	if(direction > 1) direction = 1;
	int offset = (velocity >= 0) ? LUT_RESOLUTION : 3*LUT_RESOLUTION + 1;
	int table_index = (direction * LUT_RESOLUTION) + offset;
	return mTrajTable[table_index];
}
