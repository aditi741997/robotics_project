#include <visualization_msgs/Marker.h>
#include <nav2d_msgs/RobotPose.h>
#include <nav2d_karto/MultiMapper.h>

#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

#include <cerrno>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

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

MultiMapper::MultiMapper()
{
	ROS_ERROR("MultiMapper::MultiMapper empty cONSTRUCTOR CALLED!!");
}

MultiMapper::MultiMapper(ros::Publisher& mcb_pub)
{
	// Get parameters from the ROS parameter server
	ros::NodeHandle robotNode;
	robotNode.param("robot_id", mRobotID, 1);
	robotNode.param("scan_input_topic", mScanInputTopic, std::string("karto_in"));
	robotNode.param("scan_output_topic", mScanOutputTopic, std::string("karto_out"));
	robotNode.param("laser_frame", mLaserFrame, std::string("laser"));
	robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
	robotNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));
	robotNode.param("offset_frame", mOffsetFrame, std::string("odometry_offset"));
	robotNode.param("map_frame", mMapFrame, std::string("map"));
	robotNode.param("map_service", mMapService, std::string("get_map"));
	robotNode.param("laser_topic", mLaserTopic, std::string("base_scan"));
	robotNode.param("map_topic", mMapTopic, std::string("map"));
	
	ros::NodeHandle mapperNode("~/");
	mapperNode.param("grid_resolution", mMapResolution, 0.05);
	mapperNode.param("range_threshold", mRangeThreshold, 30.0);
	mapperNode.param("map_update_rate", mMapUpdateRate, 1.0);
	mapperNode.param("publish_pose_graph", mPublishPoseGraph, true);
	mapperNode.param("max_covariance", mMaxCovariance, 0.05);
	mapperNode.param("min_map_size", mMinMapSize, 50);

	// Oct: For making a thread for scancb:
        mapperNode.param("map_scan_period", mMapScanUpdateRate, 0.2);

	// Apply tf_prefix to all used frame-id's
	mLaserFrame = mTransformListener.resolve(mLaserFrame);
	mRobotFrame = mTransformListener.resolve(mRobotFrame);
	mOdometryFrame = mTransformListener.resolve(mOdometryFrame);
	mOffsetFrame = mTransformListener.resolve(mOffsetFrame);
	mMapFrame = mTransformListener.resolve(mMapFrame);

	// Initialize Publisher/Subscribers
	ROS_ERROR("IN NAV2D : Mapper Init : mMapUpdateRate %f, Subscribing to Scan topic %s, Publishing LocalizedScan at %s, Publishing Map at %s, Subscribing to Laser Topic %s, PUblishes its pose on others topic.", mMapUpdateRate, mScanInputTopic.c_str(), mScanOutputTopic.c_str(), mMapTopic.c_str(), mLaserTopic.c_str());
	ROS_ERROR("IN NAV2D : Mapper Init : mRobotFrame : %s, mOdometryFrame : %s", mRobotFrame.c_str(), mOdometryFrame.c_str());
	ROS_ERROR("IN NAV2D : Mapper INit : Publishing vertices, edges and localization_result. ALSO HAS mMapServer, name %s", mMapService.c_str());
	mScanSubscriber = robotNode.subscribe(mScanInputTopic, 100, &MultiMapper::receiveLocalizedScan, this, ros::TransportHints().tcpNoDelay());
	mScanPublisher = robotNode.advertise<nav2d_msgs::LocalizedScan>(mScanOutputTopic, 100, true);
	mMapServer = robotNode.advertiseService(mMapService, &MultiMapper::getMap, this);
	mMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>(mMapTopic, 1, true);
	mLaserSubscriber = robotNode.subscribe(mLaserTopic, 100, &MultiMapper::receiveLaserScan, this, ros::TransportHints().tcpNoDelay());

	mapperNode.param("drop_fraction", mDropFraction, 1);
	
	robotNode.changeDropFraction(mDropFraction);

	mInitialPoseSubscriber = robotNode.subscribe("initialpose", 1, &MultiMapper::receiveInitialPose, this);
	mOtherRobotsPublisher = robotNode.advertise<nav2d_msgs::RobotPose>("others", 10, true);

	mVerticesPublisher = mapperNode.advertise<visualization_msgs::Marker>("vertices", 1, true);
	mEdgesPublisher = mapperNode.advertise<visualization_msgs::Marker>("edges", 1, true);
	mPosePublisher = robotNode.advertise<geometry_msgs::PoseStamped>("localization_result", 1, true);

	// Initialize KARTO-Mapper
	// true : multi threaded!!!
	mMapper = new karto::OpenMapper(false);
	
	double param_d;
	bool param_b;
	int param_i;
	
	if(mapperNode.getParam("UseScanMatching", param_b))
		mMapper->SetParameters("UseScanMatching", param_b);
		
	if(mapperNode.getParam("UseScanBarycenter", param_b))
		mMapper->SetParameters("UseScanBarycenter", param_b);
		
	if(mapperNode.getParam("MinimumTravelDistance", param_d))
		mMapper->SetParameters("MinimumTravelDistance", param_d);
	
	if(mapperNode.getParam("MinimumTravelHeading", param_d))
		mMapper->SetParameters("MinimumTravelHeading", param_d);
		
	if(mapperNode.getParam("ScanBufferSize", param_i))
		mMapper->SetParameters("ScanBufferSize", param_i);
		
	if(mapperNode.getParam("ScanBufferMaximumScanDistance", param_d))
		mMapper->SetParameters("ScanBufferMaximumScanDistance", param_d);
		
	if(mapperNode.getParam("UseResponseExpansion", param_b))
		mMapper->SetParameters("UseResponseExpansion", param_b);
		
	if(mapperNode.getParam("DistanceVariancePenalty", param_d))
		mMapper->SetParameters("DistanceVariancePenalty", param_d);
		
	if(mapperNode.getParam("MinimumDistancePenalty", param_d))
		mMapper->SetParameters("MinimumDistancePenalty", param_d);
		
	if(mapperNode.getParam("AngleVariancePenalty", param_d))
		mMapper->SetParameters("AngleVariancePenalty", param_d);
		
	if(mapperNode.getParam("MinimumAnglePenalty", param_d))
		mMapper->SetParameters("MinimumAnglePenalty", param_d);
		
	if(mapperNode.getParam("LinkMatchMinimumResponseFine", param_d))
		mMapper->SetParameters("LinkMatchMinimumResponseFine", param_d);
		
	if(mapperNode.getParam("LinkScanMaximumDistance", param_d))
		mMapper->SetParameters("LinkScanMaximumDistance", param_d);
		
	if(mapperNode.getParam("CorrelationSearchSpaceDimension", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceDimension", param_d);
		
	if(mapperNode.getParam("CorrelationSearchSpaceResolution", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceResolution", param_d);
		
	if(mapperNode.getParam("CorrelationSearchSpaceSmearDeviation", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceSmearDeviation", param_d);
		
	if(mapperNode.getParam("CoarseSearchAngleOffset", param_d))
		mMapper->SetParameters("CoarseSearchAngleOffset", param_d);
		
	if(mapperNode.getParam("FineSearchAngleOffset", param_d))
		mMapper->SetParameters("FineSearchAngleOffset", param_d);
		
	if(mapperNode.getParam("CoarseAngleResolution", param_d))
		mMapper->SetParameters("CoarseAngleResolution", param_d);
		
	if(mapperNode.getParam("LoopSearchSpaceDimension", param_d))
		mMapper->SetParameters("LoopSearchSpaceDimension", param_d);
		
	if(mapperNode.getParam("LoopSearchSpaceResolution", param_d))
		mMapper->SetParameters("LoopSearchSpaceResolution", param_d);
		
	if(mapperNode.getParam("LoopSearchSpaceSmearDeviation", param_d))
		mMapper->SetParameters("LoopSearchSpaceSmearDeviation", param_d);
		
	if(mapperNode.getParam("LoopSearchMaximumDistance", param_d))
		mMapper->SetParameters("LoopSearchMaximumDistance", param_d);
		
	if(mapperNode.getParam("LoopMatchMinimumChainSize", param_i))
		mMapper->SetParameters("LoopMatchMinimumChainSize", param_i);
		
	if(mapperNode.getParam("LoopMatchMaximumVarianceCoarse", param_d))
		mMapper->SetParameters("LoopMatchMaximumVarianceCoarse", param_d);
		
	if(mapperNode.getParam("LoopMatchMinimumResponseCoarse", param_d))
		mMapper->SetParameters("LoopMatchMinimumResponseCoarse", param_d);
		
	if(mapperNode.getParam("LoopMatchMinimumResponseFine", param_d))
		mMapper->SetParameters("LoopMatchMinimumResponseFine", param_d);
	
	mMapper->Message += karto::delegate(this, &MultiMapper::onMessage);
	
	mLaser = NULL;
	
	// Initialize Variables
	mMapToOdometry.setIdentity();
	mOdometryOffset.setIdentity();
	mNodesAdded = 0;
	mMapChanged = true;
	mLastMapUpdate = ros::WallTime(0);
	
	if(mRobotID == 1)
	{
		// I am the number one, so start mapping right away.
		mState = ST_MAPPING;
		ROS_ERROR("IN nav2d::MAPPER Inititialized robot 1, starting to map now. mState is SET TO ST_MAPPING.");
		mSelfLocalizer = NULL;
		
		geometry_msgs::PoseStamped locResult;
		locResult.header.stamp = ros::Time::now();
		locResult.header.frame_id = mMapFrame.c_str();
		locResult.pose.position.x = 0;
		locResult.pose.position.y = 0;
		locResult.pose.position.z = 0;
		locResult.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		mPosePublisher.publish(locResult);
	}else
	{
		// I am not number one, so wait to receive a map from number one.
		mState = ST_WAITING_FOR_MAP;
		ROS_WARN("Initialized robot %d, waiting for map from robot 1 now.", mRobotID);
		mSelfLocalizer = new SelfLocalizer();
	}

	// For measuring RT:
	last_scan_mapCB_processed = 0.0;
	last_scan_mapCB_tf_processed = 0.0;
	mScanTSPublisher = robotNode.advertise<std_msgs::Header>("mapper_scan_ts_used_TF", 1, false);
	publishTransform();

	// For measuring tput of subchains MapCB and MapUpdate:
	// last_map_cb_out = 0.0;

	map_update_thread_shutdown_ = false;
        map_scan_cb_thread_shutdown_ = false;
        received_scans = false;
        processed_scans = false;

        total_mapcb_count = 0;
        total_mapupdate_count = 0;

        

        total_map_cb_trig_count = 0;

	// Nov5: subscribe to trigger_Exec topics.
	mapcb_trigger_count = 0;
	mapupd_trigger_count = 0;
	mapcb_trigger_sub = robotNode.subscribe("trigger_exec_mapcb", 1, &MultiMapper::recv_trigger_exec, this, ros::TransportHints().tcpNoDelay());
	mapupd_trigger_sub = robotNode.subscribe("trigger_exec_mapupd", 1, &MultiMapper::recv_trigger_exec, this, ros::TransportHints().tcpNoDelay());

	sock_recv_thread = boost::thread(&MultiMapper::socket_recv, this);

	ROS_ERROR("Mapper's TF CBT tid: %i", mTransformListener.getTFCBTid() );
	
	// For making a separate thread for mapCB:
        // map_cb_exec_info_pub = robotNode.advertise<std_msgs::Header>("exec_start_mapcb", 1, true);
        map_cb_exec_info_pub = mcb_pub;
	// publish_tid("mapcb_extra", mTransformListener.getTFCBTid(), &map_cb_exec_info_pub);
	map_scan_cb_thread_ = new boost::thread(boost::bind(&MultiMapper::mapScanCBLoop, this, mMapScanUpdateRate));

	map_upd_exec_info_pub = robotNode.advertise<std_msgs::Header>("exec_start_mapupd", 10, true);
	// publish_tid("mapupd_extra", mTransformListener.getTFCBTid(), &map_upd_exec_info_pub);
        map_update_thread_ = new boost::thread(boost::bind(&MultiMapper::mapUpdateLoop, this, mMapUpdateRate));

	map_cb_exec_end_pub = robotNode.advertise<std_msgs::Header>("exec_end_mapcb", 1, true);
	map_upd_exec_end_pub = robotNode.advertise<std_msgs::Header>("exec_end_mapupd", 1, true);
}

void MultiMapper::socket_recv()
{
	// initialize sockets for recv triggers [New:Jan]
	client_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	ROS_ERROR("MultiMapper:: SOCKET: client_sock_fd is %i", client_sock_fd);
	// if (client_sock_fd < 0) ROS_ERROR("MultiMapper:: SOCKET: client_sock_fd is NEGATIVE!!");

	struct sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(6327);

	int pton_ret = inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
	// if ( pton_ret <= 0 )
		ROS_ERROR("MultiMapper:: SOCKET:  inet_pton RET val: %i", pton_ret);

	int con_ret = connect(client_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
	// if ( con_ret < 0 )
		ROS_ERROR("MultiMapper:: SOCKET: In connect call!!! ret val: %i, errno: %i, str: %s", con_ret, errno, std::strerror(errno));

	std::string s = "mapcb\nmapupd\n";
        char smsg[1+s.length()];
	strcpy(smsg, s.c_str());
	send(client_sock_fd, smsg, strlen(smsg), 0);

	// priority=3.
	int ret = 7;
	struct sched_param sp = { .sched_priority = 3,};
        ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);
	ROS_ERROR("MultiMapper:: SOCKET tHREAD prio=3, retval: %i, tid: %i, pid: %i", ret, ::gettid(), ::getpid());

	// wait for msgs.
	int read_size;
	char msg [2048];

	while ( (read_size = recv(client_sock_fd, msg, 2048, 0) ) > 0 )
	{
		std::string s_msg = msg;
		memset(msg, 0, 2048);
		processTrigger(s_msg);
	}
}

MultiMapper::~MultiMapper()
{
	map_update_thread_shutdown_ = true;
	if (map_update_thread_ != NULL)
	{
		map_update_thread_->join();
    	delete map_update_thread_;
	}
	
	map_scan_cb_thread_shutdown_ = true;
        if (map_scan_cb_thread_ != NULL)
        {
                map_scan_cb_thread_->join();
        }
        delete map_scan_cb_thread_;
}

void MultiMapper::recv_trigger_exec(const std_msgs::Header::ConstPtr& msg)
{
	ROS_ERROR("Got a trigger msg! %s, TS: %f", msg->frame_id.c_str(), msg->stamp.toSec());
	processTrigger(msg->frame_id);
}

void MultiMapper::processTrigger(std::string msg)
{
	if (msg.find("cb") != std::string::npos)
	{
		// its the mapcb trigger.
		ROS_ERROR("Got a trigger for mapcb, %s curr count: %i", msg.c_str(), mapcb_trigger_count);
		boost::unique_lock<boost::mutex> lock(mapcb_trigger_mutex);
		/*
		if (msg.find("RESETCOUNT") != std::string::npos)
		{
			mapcb_trigger_count = 1;
			// ROS_ERROR("RESET counter for mapcb to 1.");
		}
		else
		*/
		mapcb_trigger_count += 1;
		cv_mapcb.notify_all();
	}
	else
	{
		// ROS_ERROR("Got a trigger for mapupd %s curr count: %i", msg.c_str(), mapupd_trigger_count);
		boost::unique_lock<boost::mutex> lock(mapupd_trigger_mutex);
		/*
		if (msg.find("RESETCOUNT") != std::string::npos)
		{
			mapupd_trigger_count = 1;
			// ROS_ERROR("RESET counter for mapupd to 1.");
		}
		else
		*/	
		mapupd_trigger_count += 1;
		cv_mapupd.notify_all();	
	}

}

void MultiMapper::mapUpdateLoop(double map_update_rate)
{
	if (map_update_rate == 0.0)
		return;
	ROS_ERROR("In MultiMapper::mapUpdateLoop STARTING! with period %fs", map_update_rate);
	ros::NodeHandle nh;
	ros::WallRate r(1.0/map_update_rate); // the rate param is actually in seconds.

	ROS_ERROR("Publishing node mapupd tid %i, pid %i to controller.", ::gettid(), ::getpid());
        std_msgs::Header hdr;
	
	std::stringstream ss_e;
	ss_e << ::getpid() << " mapupd " << ::gettid();
    	hdr.frame_id = ss_e.str();

	map_upd_exec_info_pub.publish(hdr);

	std::string use_td;
        nh.param<std::string>("/use_td", use_td, "");

        bool use_timer = ( use_td.find("yes") != std::string::npos );
	// use_timer = true;
	ROS_ERROR("In mapUpdateLoop, use_timer is %i, use_td param is %s",use_timer, use_td.c_str() );

	int map_upd_ct = 0;

	while (nh.ok() && !map_update_thread_shutdown_)
	{
		if (processed_scans)
			sendMap();
		
		map_upd_ct++;
		// r.sleep();
		// Nov5: To be triggered by Scheduler.
		// Nov23: if not timer then triggered by scheduler.
		if (use_timer)
			r.sleep();
		else
		{
			boost::unique_lock<boost::mutex> lock(mapupd_trigger_mutex);
			while (mapupd_trigger_count == 0)
			{
				cv_mapupd.wait(lock);
			}
			ROS_ERROR("About to run updateMap!! %i", mapupd_trigger_count);
			mapupd_trigger_count = 0; // flush on read
		}
	
		if (map_upd_ct<10)
		{
			if (map_upd_ct%2 == 0)
				publish_tid("mapupd_extra", mTransformListener.getTFCBTid(), &map_upd_exec_info_pub);	
			else
				publish_tid("mapupd", ::gettid(), &map_upd_exec_info_pub);
		}
	}
}

void MultiMapper::setScanSolver(karto::ScanSolver* scanSolver)
{
	mMapper->SetScanSolver(scanSolver);
}

void MultiMapper::setRobotPose(double x, double y, double yaw)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, 0));
	transform.setRotation(tf::createQuaternionFromYaw(yaw));
	transform = transform.inverse();
	
	tf::Stamped<tf::Pose> pose_in, pose_out;
	pose_in.setData(transform);
	pose_in.frame_id_ = mRobotFrame;
	pose_in.stamp_ = ros::Time(0);
	mTransformListener.transformPose(mOdometryFrame, pose_in, pose_out);
	
	transform = pose_out;
	mOdometryOffset = transform.inverse();

	if(mSelfLocalizer)
	{
		delete mSelfLocalizer;
		mSelfLocalizer = NULL;
	}
	
	// Publish the new pose (to inform other nodes, that we are localized now)
	geometry_msgs::PoseStamped locResult;
	locResult.header.stamp = ros::Time::now();
	locResult.header.frame_id = mMapFrame.c_str();
	locResult.pose.position.x = x;
	locResult.pose.position.y = y;
	locResult.pose.position.z = 0;
	locResult.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	mPosePublisher.publish(locResult);
	
	// Publish via tf
	mState = ST_MAPPING;
	ROS_ERROR("IN NAV2D::MAPPER setRobotPose func. mState IS NOW SET TO ST_MAPPING. ALso, used TF for transform %s and %s", mRobotFrame.c_str(), mOdometryFrame.c_str());
	publishTransform();
}

karto::LocalizedRangeScan* MultiMapper::createFromRosMessage(const sensor_msgs::LaserScan& scan, const karto::Identifier& robot)
{
	// Implementing REP 117: Informational Distance Measurements
	// http://www.ros.org/reps/rep-0117.html
	karto::RangeReadingsList readings;
	std::vector<float>::const_iterator it;
	for(it = scan.ranges.begin(); it != scan.ranges.end(); it++)
	{
		if(*it >= scan.range_min && *it <= scan.range_max)
		{
			// This is a valid measurement.
			readings.Add(*it);
		}else if( !std::isfinite(*it) && *it < 0)
		{
			// Object too close to measure.
			readings.Add(scan.range_max);
		}else if( !std::isfinite(*it) && *it > 0)
		{
			// No objects detected in range.
			readings.Add(scan.range_max);
		}else if( std::isnan(*it) )
		{
			// This is an erroneous, invalid, or missing measurement.
			ROS_WARN_THROTTLE(1,"Laser scan contains nan-values!");
			readings.Add(scan.range_max);
		}else
		{
			// The sensor reported these measurements as valid, but they are
			// discarded per the limits defined by minimum_range and maximum_range.
			ROS_WARN_THROTTLE(1, "Laser reading not between range_min and range_max!");
			readings.Add(scan.range_max);
		}
	}
	return new karto::LocalizedRangeScan(robot, readings);
}

void write_arrs_to_file(std::vector<double>& times, std::vector<double>& ts, std::string s, std::vector<int> sc = std::vector<int> ())
{
	std::string ename;
    ros::NodeHandle nh;
    nh.param<std::string>("/expt_name", ename, "");

	std::ofstream of;
	of.open( ("/home/ubuntu/robot_" + s + "_stats_" + ename + ".txt").c_str() , std::ios_base::app);
	of << "\n" << s << " times: ";
	int sz = times.size();
	for (int i = 0; i < sz; i++)
		of << times[i] << " ";
	of << "\n" << s << " ts: ";
	std::stringstream ss;	
	for (int i = 0; i < sz; i++)
		ss << ts[i] << " ";		
	of << ss.str();

	if (sc.size() > 0)
	{
		of << "\n" << s << " ScanCOunt: ";
		for (int i = 0; i < sc.size(); i++)
			of << sc[i] << " ";
	}

	times.clear();
	ts.clear();
}

void write_arr_to_file(std::vector<double>& tput, std::string s, std::string m)
{
	std::string ename;
    ros::NodeHandle nh;
    nh.param<std::string>("/expt_name", ename, "");
	int sz = tput.size();

	if (sz > 0)
	{
		std::ofstream of;
        	of.open( ("/home/ubuntu/robot_" + s + "_stats_" + ename + ".txt").c_str() , std::ios_base::app);

		of << "\n" << s << " " << m << ": ";
		for (int i = 0; i < sz; i++)
			of << tput[i] << " ";
	
		std::sort(tput.begin(), tput.end());
		ROS_ERROR("Nav2d Mapper %s, Median, 95ile TPUT %f %f", s.c_str(), tput[sz/2], tput[(95*sz)/100]);
		tput.clear();
	}
       
}

void MultiMapper::receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	struct timespec cb_start, cb_end;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_start);

	/*
	if (!processed_scans)
		publishTransform(); // publish for the first scanCB.
	*/

	// Ignore own readings until map has been received
	if(mState == ST_WAITING_FOR_MAP)
	{
		return;
	}

	{
		boost::mutex::scoped_lock lock(scan_lock);
		// Oct: for making mapCB TD:
		latest_scan_recv.header.frame_id = scan->header.frame_id;
		latest_scan_recv.header.seq = latest_scan_recv.header.seq;
		latest_scan_recv.header.stamp = scan->header.stamp;

		latest_scan_recv.angle_min = scan->angle_min;
		latest_scan_recv.angle_max = scan->angle_max;
		latest_scan_recv.angle_increment = scan->angle_increment;
		latest_scan_recv.time_increment = scan->time_increment;
		latest_scan_recv.scan_time = scan->scan_time;
		latest_scan_recv.range_min = scan->range_min;
		latest_scan_recv.range_max = scan->range_max;

		latest_scan_recv.ranges.clear();
		latest_scan_recv.intensities.clear();

		for (int i = 0; i < scan->ranges.size(); i++)
		    latest_scan_recv.ranges.push_back(scan->ranges[i]);

		for (int i = 0; i < scan->intensities.size(); i++)
		    latest_scan_recv.intensities.push_back(scan->intensities[i]);
	
		received_scans = true;
		// ROS_WARN("ROBOT_%i IN nav2d::Mapper receiveLaserScan with TS %f", mRobotID, scan->scan_time);
		publishTransform();
	}
	/*	

	if(!mLaser)
	{
		// Create a laser range finder device and copy in data from the first scan
		char name[10];
		sprintf(name, "robot_%d", mRobotID);

		// Add the laser to the mapper
		try
		{
			mLaser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);		
			mLaser->SetMinimumRange(scan->range_min);
			mLaser->SetMaximumRange(scan->range_max);
			mLaser->SetMinimumAngle(scan->angle_min);
			mLaser->SetMaximumAngle(scan->angle_max);
			mLaser->SetAngularResolution(scan->angle_increment);
			mLaser->SetRangeThreshold(mRangeThreshold);
			mMapper->Process(mLaser);
		
			ROS_ERROR("Created mLaser.");
		}
		catch(karto::Exception e)
		{
			ROS_ERROR("Could not add new Laser to Mapper: %s", e.GetErrorMessage().ToCString());
			return;
		}
	}
	
	if(mState == ST_LOCALIZING)
	{
		// ROS_WARN("IN nav2d_karto::MultiMapper receiveLaserScan -  In ST_LOCALIZING state.");
		mSelfLocalizer->process(scan);
		if(mSelfLocalizer->getCovariance() < mMaxCovariance)
		{
			// Localization finished, kill the localizer and start mapping
			ROS_WARN("Localization finished on robot %d, now starting to map.", mRobotID);
			tf::Transform p = mSelfLocalizer->getBestPose();
			setRobotPose(p.getOrigin().getX(), p.getOrigin().getY(), tf::getYaw(p.getRotation()));
		}
	}else 
	if(mState == ST_MAPPING)
	{
		// get the odometry pose from tf
		// ROS_WARN("IN nav2d::Mapper receiveLaserScan IN ST_MAPPING state.");
			// ROS_WARN("IN nav2d_karto::MultiMapper func receiveLaserScan. Transform between %s and %s", mOffsetFrame.c_str(), mLaserFrame.c_str());
		tf::StampedTransform tfPose;
		try
		{
			mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, scan->header.stamp, tfPose);
		}
		catch(tf::TransformException e)
		{
			try
			{
				mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, ros::Time(0), tfPose);
			}
			catch(tf::TransformException e)
			{
				ROS_WARN("ROBOT_%i Failed to compute odometry pose, skipping scan (%s)", mRobotID, e.what());
				return;
			}
		}
		// ROS_WARN("ROBOT_%i IN nav2d::Mapper receiveLaserScan GOT TF", mRobotID);
	
		karto::Pose2 kartoPose = karto::Pose2(tfPose.getOrigin().x(), tfPose.getOrigin().y(), tf::getYaw(tfPose.getRotation()));
		
		// create localized laser scan
		karto::LocalizedLaserScanPtr laserScan = createFromRosMessage(*scan, mLaser->GetIdentifier());
		laserScan->SetOdometricPose(kartoPose);
		laserScan->SetCorrectedPose(kartoPose);
		// laserScan->setScanTimeStamp(scan->header.stamp.toSec());
		// using real TS:
		laserScan->setScanTimeStamp(scan->scan_time);
		
		bool success;
		try
		{
			success = mMapper->Process(laserScan);
		}
		catch(karto::Exception e)
		{
			ROS_ERROR("%s", e.GetErrorMessage().ToCString());
			success = false;
		}
		
		// ROS_WARN("ROBOT_%i IN nav2d::Mapper receiveLaserScan mMapper->pROCESS : %i", mRobotID, success);	

		if(success)
		{
			// ROS_WARN("IN nav2d_karto::MultiMapper receiveLaserScan success=true. Now, need to update tf. WIll transform pose : %s and %s", mLaserFrame);
			// Compute the map->odom transform
			karto::Pose2 corrected_pose = laserScan->GetCorrectedPose();
			tf::Pose map_in_robot(tf::createQuaternionFromYaw(corrected_pose.GetHeading()), tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
			map_in_robot = map_in_robot.inverse();
			tf::Stamped<tf::Pose> map_in_odom;
			bool ok = true;
			try
			{
				mTransformListener.transformPose(mOffsetFrame, tf::Stamped<tf::Pose>(map_in_robot, ros::Time(0) , mLaserFrame), map_in_odom);
			}
			catch(tf::TransformException e)
			{
				ROS_ERROR("ROBOT_%i Transform from %s to %s failed! (%s)", mRobotID, mLaserFrame.c_str(), mOffsetFrame.c_str(), e.what());
				ok = false;
			}
			if(ok) 
			{
				mMapToOdometry = tf::Transform(tf::Quaternion( map_in_odom.getRotation() ), tf::Point(map_in_odom.getOrigin() ) ).inverse();
				tf::Vector3 v = mMapToOdometry.getOrigin();
				v.setZ(0);
				mMapToOdometry.setOrigin(v);

				// For measuring RT :
				// Latest scan used in the mapTOOdom TF:
				// last_scan_mapCB_tf_processed = scan->header.stamp.toSec();
				// using real TS:
				last_scan_mapCB_tf_processed = scan->scan_time;
			}
			mNodesAdded++;
			mMapChanged = true;

				ROS_ERROR("robot %d : IN nav2d_Mapper scanCB : SUCCESSFULLY processed scan rosTS %f  realTS%f, arr len %i, mNodesAdded: %i", mRobotID, scan->header.stamp.toSec(), scan->scan_time, scan_cb_times.size(), mNodesAdded);
				// Scan CB Times : Exclude map Update time.
				clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_end);
				double t = get_time_diff(cb_start, cb_end);

				scan_cb_times.push_back(t);
				scan_cb_ts.push_back(ros::Time::now().toSec());

			boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
			boost::chrono::system_clock::duration tse = now.time_since_epoch();
			if (scan_cb_times.size() > 1)
			{
				boost::chrono::duration<double> diff = now - last_map_cb_out;
				tput_map_cb.push_back( diff.count() );
			}
			// add to the tput array.
			// set last map cb out to current time.
			// last_map_cb_out = boost::chrono::duration_cast<boost::chrono::nanoseconds>(tse).count() * 1e-9;
			last_map_cb_out = now;
			
			// denotes that there's atleast 1scan in the list of processed scans.
			// Needed so that mapUpdate doesnt start running until there are scans. [Throws exception o.w.]
			received_scans = true; 
			// Moving mapUpdate to a separate thread.
			// ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
			// if(mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
			// {
			// 	sendMap();
			// }

			// Send the scan to the other robots via com-layer (DDS)
			ROS_DEBUG("Robot %d: Sending scan (uniqueID: %d, Sensor: %s, stateID: %d)", mRobotID, laserScan->GetUniqueId(), laserScan->GetSensorIdentifier().ToString().ToCString(), laserScan->GetStateId());
			sendLocalizedScan(scan, laserScan->GetOdometricPose());
			
			// Publish via extra topic
			nav2d_msgs::RobotPose other;
			other.header.stamp = ros::Time::now();
			other.header.frame_id = mMapFrame;
			other.robot_id = mRobotID;
			other.pose.x = laserScan->GetCorrectedPose().GetX();
			other.pose.y = laserScan->GetCorrectedPose().GetY();
			other.pose.theta = laserScan->GetCorrectedPose().GetHeading();
			mOtherRobotsPublisher.publish(other);
		}
	}
	*/
		
}

void MultiMapper::processLatestScan()
{
	struct timespec cb_start, cb_end;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_start);

	bool tf_changed = false;

	{
		boost::mutex::scoped_lock lock(scan_lock);
		// process latest_scan_recv.
		if(!mLaser)
		{
			// Create a laser range finder device and copy in data from the first scan
			char name[10];
			sprintf(name, "robot_%d", mRobotID);

			// Add the laser to the mapper
			try
			{
				mLaser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
				mLaser->SetMinimumRange(latest_scan_recv.range_min);
				mLaser->SetMaximumRange(latest_scan_recv.range_max);
				mLaser->SetMinimumAngle(latest_scan_recv.angle_min);
				mLaser->SetMaximumAngle(latest_scan_recv.angle_max);
				mLaser->SetAngularResolution(latest_scan_recv.angle_increment);
				mLaser->SetRangeThreshold(mRangeThreshold);
				mMapper->Process(mLaser);

				ROS_ERROR("Created mLaser.");
			}
			catch(karto::Exception e)
			{
				ROS_ERROR("Could not add new Laser to Mapper: %s", e.GetErrorMessage().ToCString());
				return;
			}
		}

		if(mState == ST_LOCALIZING)
		{
			// ROS_WARN("IN nav2d_karto::MultiMapper receiveLaserscan -  In ST_LOCALIZING state.");
			// oct: the shared_ptr throws an error double free. Would need to resolve this for multi robot scenario.
			boost::shared_ptr<sensor_msgs::LaserScan> cp (&latest_scan_recv);
			mSelfLocalizer->process(cp);
			if(mSelfLocalizer->getCovariance() < mMaxCovariance)
			{
				// Localization finished, kill the localizer and start mapping
				ROS_WARN("Localization finished on robot %d, now starting to map.", mRobotID);
				tf::Transform p = mSelfLocalizer->getBestPose();
				setRobotPose(p.getOrigin().getX(), p.getOrigin().getY(), tf::getYaw(p.getRotation()));
			}
		}else
		if(mState == ST_MAPPING)
		{
			tf::StampedTransform tfPose;
			try
			{
				mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, latest_scan_recv.header.stamp, tfPose);
			}
			catch(tf::TransformException e)
			{
				try
				{
					mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, ros::Time(0), tfPose);
				}
				catch(tf::TransformException e)
				{
					ROS_WARN("ROBOT_%i Failed to compute odometry pose, skipping scan (%s)", mRobotID, e.what());
					return;
				}
			}

			karto::Pose2 kartoPose = karto::Pose2(tfPose.getOrigin().x(), tfPose.getOrigin().y(), tf::getYaw(tfPose.getRotation()));

			// create localized laser scan
			karto::LocalizedLaserScanPtr laserScan = createFromRosMessage(latest_scan_recv, mLaser->GetIdentifier());
			laserScan->SetOdometricPose(kartoPose);
			laserScan->SetCorrectedPose(kartoPose);
			// laserScan->setScanTimeStamp(scan->header.stamp.toSec());
			// using real TS:
			laserScan->setScanTimeStamp(latest_scan_recv.scan_time);

			bool success;
			try
			{
				success = mMapper->Process(laserScan);
			}
			catch(karto::Exception e)
			{
				ROS_ERROR("%s", e.GetErrorMessage().ToCString());
				success = false;
			}
			
			if(success)
			{
				// ROS_WARN("IN nav2d_karto::MultiMapper receiveLaserScan success=true. Now, need to update tf. WIll transform pose : %s and %s", mLaserFrame);
				// Compute the map->odom transform
				karto::Pose2 corrected_pose = laserScan->GetCorrectedPose();
				tf::Pose map_in_robot(tf::createQuaternionFromYaw(corrected_pose.GetHeading()), tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
				map_in_robot = map_in_robot.inverse();
				tf::Stamped<tf::Pose> map_in_odom;
				bool ok = true;
				try
				{
					mTransformListener.transformPose(mOffsetFrame, tf::Stamped<tf::Pose>(map_in_robot, ros::Time(0) /* latest_scan_recv.header.stamp*/, mLaserFrame), map_in_odom);
				}
				catch(tf::TransformException e)
				{
					ROS_ERROR("ROBOT_%i Transform from %s to %s failed! (%s)", mRobotID, mLaserFrame.c_str(), mOffsetFrame.c_str(), e.what());
					ok = false;
				}

				if(ok)
				{
					mMapToOdometry = tf::Transform(tf::Quaternion( map_in_odom.getRotation() ), tf::Point(map_in_odom.getOrigin() ) ).inverse();
					tf::Vector3 v = mMapToOdometry.getOrigin();
					v.setZ(0);
					mMapToOdometry.setOrigin(v);

					tf_changed = true;

					// For measuring RT :
					// Latest scan used in the mapTOOdom TF:
					// last_scan_mapCB_tf_processed = scan->header.stamp.toSec();
					// using real TS:
					last_scan_mapCB_tf_processed = latest_scan_recv.scan_time;
					publishTransform();
				}
				mNodesAdded++;
				mMapChanged = true;

				ROS_ERROR("robot %d : IN nav2d_Mapper scanCB : SUCCESSFULLY processed scan rosTS %f  realTS%f, arr len %i, mNodesAdded: %i", mRobotID, latest_scan_recv.header.stamp.toSec(), latest_scan_recv.scan_time, scan_cb_times.size(), mNodesAdded);
					// Scan CB Times : Exclude map Update time.
					clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_end);
					double t = get_time_diff(cb_start, cb_end);

					total_mapcb_count += 1;
					double exec_rt_end = get_time_now();					

					scan_cb_times.push_back(t);
					scan_cb_ts.push_back(exec_rt_end);

					std_msgs::Header mcb_ee;
					std::stringstream ss;
					ss << t;
					mcb_ee.frame_id = ss.str() + " mapcb";
					map_cb_exec_end_pub.publish(mcb_ee);

					if (t > 2.0)
						ROS_ERROR("WEIRD!!!! MAP CB SLOW TPUT >2s!!! TPUT: %f", t);

				boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
				// boost::chrono::system_clock::duration tse = now.time_since_epoch();
				if (total_mapcb_count > 1)
				{       
					// boost::chrono::duration<double> diff = now - last_map_cb_out;
					// tput_map_cb.push_back( diff.count() );
					tput_map_cb.push_back(exec_rt_end - last_map_cb_out);			
	
				}

				// add to the tput array.
				// set last map cb out to current time.
				// last_map_cb_out = boost::chrono::duration_cast<boost::chrono::nanoseconds>(tse).count() * 1e-9;
				// last_map_cb_out = now;
				last_map_cb_out = exec_rt_end;

				// denotes that there's atleast 1scan in the list of processed scans.
				// Needed so that mapUpdate doesnt start running until there are scans. [Throws exception o.w.]
				processed_scans = true;
				// Moving mapUpdate to a separate thread.
				// ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
				// if(mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
				// {
				//      sendMap();
				// }

				// Send the scan to the other robots via com-layer (DDS)
				ROS_DEBUG("Robot %d: Sending scan (uniqueID: %d, Sensor: %s, stateID: %d)", mRobotID, laserScan->GetUniqueId(), laserScan->GetSensorIdentifier().ToString().ToCString(), laserScan->GetStateId());
				sendLocalizedScan(laserScan->GetOdometricPose());

				// Publish via extra topic
				nav2d_msgs::RobotPose other;
				other.header.stamp = ros::Time::now();
				other.header.frame_id = mMapFrame;
				other.robot_id = mRobotID;
				other.pose.x = laserScan->GetCorrectedPose().GetX();
				other.pose.y = laserScan->GetCorrectedPose().GetY();
				other.pose.theta = laserScan->GetCorrectedPose().GetHeading();
				mOtherRobotsPublisher.publish(other);
			}
			else
			{
				double sdrop_rt = get_time_now();
				scan_drop_ts.push_back(sdrop_rt);
			
				clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cb_end);
                                double dropt = get_time_diff(cb_start, cb_end);
				scan_drop_exec_time.push_back(dropt);
			
			}

		}
	}
	
	if (scan_cb_times.size()%20 == 3)
	{
		// std::ofstream of;
		std::string ss = "nav2d_mapper_scanCB";
        // of.open("/home/aditi/robot_" + ss + "_stats_r1.txt", std::ios_base::app);
        	write_arrs_to_file(scan_cb_times, scan_cb_ts, ss);
		write_arr_to_file(tput_map_cb, ss, "tput");
		write_arr_to_file(scan_drop_ts, ss, "scanDrop");
		write_arr_to_file(scan_drop_exec_time, ss, "scanDropExecTimes");
		write_arr_to_file(trig_tput_map_cb, ss, "trigTput");
	}


	// For converting navigation2d to ED:
	// Publishing transform from within scanCB
	// THis has to be at the end of receiveLaserScan so it spublished right after its updated in the func.

}

void MultiMapper::mapScanCBLoop(double per)
{
	if (per == 0.0)
                return;
        ROS_ERROR("In MultiMapper::mapScanCBLoop STARTING! with period %fs", per);
        ros::NodeHandle nh;

        ros::WallRate r(1.0/per); 

        ROS_ERROR("Publishing node mapcb tid %i, pid %i to controller.", ::gettid(), ::getpid());
        std_msgs::Header hdr;

        std::stringstream ss_e;
        ss_e << ::getpid() << " mapcb " << ::gettid();
        hdr.frame_id = ss_e.str();

        map_cb_exec_info_pub.publish(hdr);

	std::string use_td;
    	nh.param<std::string>("/use_td", use_td, "");

	bool use_timer = ( use_td.find("yes") != std::string::npos );
	// use_timer = true;
	ROS_ERROR("In mapScanCBLoop, use_timer is %i, use_td param is %s",use_timer, use_td.c_str() );

        while (nh.ok() && !map_scan_cb_thread_shutdown_)
	{
		// process stored scan.
		if (received_scans)
		{
			processLatestScan();
		}
		
		total_map_cb_trig_count += 1;
		double time_now = get_time_now();
		if (total_map_cb_trig_count > 1)
			trig_tput_map_cb.push_back( time_now - trig_last_map_cb_out);
		trig_last_map_cb_out = time_now;

		// r.sleep();
		// Nov5: replace sleep with while-wait on cv_mapcb, so that DAGController triggers this.
		// This does mapcb_trigger_count -=1 if it is +ve.

		if (use_timer)
		{
			r.sleep();
		}
		else
		{	
			boost::unique_lock<boost::mutex> lock(mapcb_trigger_mutex);
			while (mapcb_trigger_count == 0)
			{
				cv_mapcb.wait(lock);
			}
			ROS_ERROR("About to processLatestScan %i", mapcb_trigger_count);
			mapcb_trigger_count = 0; // flush on read.
		}

		if (total_map_cb_trig_count<9)
		{
			if (total_map_cb_trig_count%2 == 1)
				publish_tid("mapcb", ::gettid(), &map_cb_exec_info_pub);
			else
				publish_tid("mapcb_extra", mTransformListener.getTFCBTid(), &map_cb_exec_info_pub);
		}
	
	}
}

bool MultiMapper::getMap(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
	// ROS_WARN("IN nav2d::MultiMapper getMap service call!!!!");
	if(mState == ST_WAITING_FOR_MAP && mNodesAdded < mMinMapSize)
	{
		ROS_WARN("Still waiting for map from robot 1.");
		return false;
	}
	
	
	// if(sendMap())
	// {
		res.map = mGridMap;
		return true;
		
	// 	return true;
	// }
	// else
	// {
	// 	ROS_WARN("Serving map request failed!");
	// 	return false;
	// }
}

bool MultiMapper::sendMap()
{
	ROS_ERROR("MultiMapper::sendMap CALLED!! with mLastMapUpdate %f", mLastMapUpdate);
	if(!updateMap()) return false;
	// updateMap returns true if 1. map wasnt changed, i.e. no new scans or 2. map was updated successfully.
	// need to distinguish bw these two.
	
	// Publish the map
	mMapPublisher.publish(mGridMap);

	
	mLastMapUpdate = ros::WallTime::now();

	// Publish the pose-graph
	if(mPublishPoseGraph)
	{
		// Publish the vertices
		karto::MapperGraph::VertexList vertices = mMapper->GetGraph()->GetVertices();
		visualization_msgs::Marker marker;
		marker.header.frame_id = mMapFrame;
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.points.resize(vertices.Size());
		
		for(int i = 0; i < vertices.Size(); i++)
		{
			marker.points[i].x = vertices[i]->GetVertexObject()->GetCorrectedPose().GetX();
			marker.points[i].y = vertices[i]->GetVertexObject()->GetCorrectedPose().GetY();
			marker.points[i].z = 0;
		}
		mVerticesPublisher.publish(marker);
		
		// Publish the edges
		karto::MapperGraph::EdgeList edges = mMapper->GetGraph()->GetEdges();
		marker.header.frame_id = mMapFrame;
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.scale.x = 0.01;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.points.resize(edges.Size() * 2);
		
		for(int i = 0; i < edges.Size(); i++)
		{
			marker.points[2*i].x = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetX();
			marker.points[2*i].y = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetY();
			marker.points[2*i].z = 0;

			marker.points[2*i+1].x = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetX();
			marker.points[2*i+1].y = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetY();
			marker.points[2*i+1].z = 0;
		}
		mEdgesPublisher.publish(marker);
	}
	return true;
}

bool MultiMapper::updateMap()
{
	if(!mMapChanged) return true;
	ROS_ERROR("IN MultiMapper::updateMap, mMapChanged is TRUE!!!");

	// to measure updateMap time ALSO #scans.
	struct timespec map_update_start, map_update_end;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &map_update_start);

	const karto::LocalizedLaserScanList allScans = mMapper->GetAllProcessedScans();

	// can we get some TS from allScans itself?
	double using_scan_mapCB_ts = allScans[allScans.Size()-1]->getScanTimeStamp();
	ros::Time ts_rost = ros::Time(using_scan_mapCB_ts);
	ROS_ERROR("In MultiMapper::updateMap.... Using scans with latest TS : %f, converting to ros::Time %f", using_scan_mapCB_ts, ts_rost.toSec());
	// set the latest scan used TS in the map to be published.

	karto::OccupancyGridPtr kartoGrid = karto::OccupancyGrid::CreateFromScans(allScans, mMapResolution);
	int scan_count = allScans.Size();

	if(!kartoGrid)
	{
		ROS_ERROR("Failed to get occupancy map from Karto-Mapper.");
		return false; 
	}

	// Translate to ROS format
	unsigned int width = kartoGrid->GetWidth();
	unsigned int height = kartoGrid->GetHeight();
	karto::Vector2<kt_double> offset = kartoGrid->GetCoordinateConverter()->GetOffset();

	if(	mGridMap.info.width != width || 
		mGridMap.info.height != height || 
		mGridMap.info.origin.position.x != offset.GetX() || 
		mGridMap.info.origin.position.y != offset.GetY())
	{
		mGridMap.info.resolution = mMapResolution;
		mGridMap.info.origin.position.x = offset.GetX();
		mGridMap.info.origin.position.y = offset.GetY();
		mGridMap.info.width = width;
		mGridMap.info.height = height;
		mGridMap.data.resize(mGridMap.info.width * mGridMap.info.height);
	}

	// For measuring amt of map explored:
	int total_area = height*width;
	int unknown_area = 0;
	
	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++) 
		{
			// Getting the value at position x,y
			kt_int8u value = kartoGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

			switch (value)
			{
			case karto::GridStates_Unknown:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = -1;
				unknown_area += 1;
				break;
			case karto::GridStates_Occupied:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = 100;
				break;
			case karto::GridStates_Free:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = 0;
				break;
			default:
				ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
				break;
			}
		}
	}

	double exec_rt_end = get_time_now();
	ROS_ERROR("Curr_time: %f In MultiMapper::updateMap Current ratio of unknown/total area : %i %i #", exec_rt_end, unknown_area, total_area);

	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &map_update_end);
	double t = get_time_diff(map_update_start, map_update_end);


	total_mapupdate_count += 1;
	map_update_times.push_back(t);
	map_update_ts.push_back(exec_rt_end);
	map_update_scan_count.push_back(scan_count);

	std_msgs::Header mu_ee_pub;
	std::stringstream ss;
	ss << t;
	mu_ee_pub.frame_id = ss.str() + " mapupd";
	map_upd_exec_end_pub.publish(mu_ee_pub);

	// Measure tput of map Update:
	boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
	if (total_mapupdate_count > 1)
	{
		// boost::chrono::duration<double> diff = now - last_map_upd_out;
                ROS_ERROR("NAV2D::MAPPER NEW MAP TPUT!! %f", exec_rt_end - last_map_upd_out);
		// tput_map_update.push_back( diff.count() );	
		tput_map_update.push_back(exec_rt_end - last_map_upd_out);

		if ( (exec_rt_end - last_map_upd_out) > 3.0 )
			ROS_ERROR("WEIRD!!!! MAPPER MAPUPD TPUT TOO HIGH!!");
	}

	last_map_upd_out = exec_rt_end;

	if (map_update_ts.size() % 3 == 2)
	{
		// std::ofstream of;
		std::string mus = "nav2d_mapper_mapUpdate";
		// of.open("/home/aditi/robot_" + mus + "_stats_r1.txt",  std::ios_base::app);
		write_arrs_to_file(map_update_times, map_update_ts, mus, map_update_scan_count);
		write_arr_to_file(tput_map_update, mus, "tput");
		// of << "\n" << mus << " ScanCount: ";
		// for (int i = 0; i < map_update_scan_count.size(); i++)
		// 	of << map_update_scan_count[i] << " ";
		map_update_scan_count.clear();
	}

	// Set the header information on the map
	// For measuring RT along Scan-MapCB-MapUPdate-NavPlan-NavCmd-LP chain:
	mGridMap.header.stamp = ts_rost;
	mGridMap.header.frame_id = mMapFrame.c_str();
	mMapChanged = false;
	return true;
}

void MultiMapper::receiveLocalizedScan(const nav2d_msgs::LocalizedScan::ConstPtr& scan)
{
	// Ignore my own scans
	if(scan->robot_id == mRobotID) return;
	
	ROS_WARN("ROBOT %i In nav2d::MAPPER : receiveLocalizedScan", mRobotID);

	// Get the robot id
	char robot[10];
	sprintf(robot, "robot_%d", scan->robot_id);
	
	// Get the scan pose
	karto::Pose2 scanPose(scan->x, scan->y, scan->yaw);
	
	// create localized laser scan
	karto::LocalizedLaserScanPtr localizedScan = createFromRosMessage(scan->scan, robot);
	localizedScan->SetOdometricPose(scanPose);
	localizedScan->SetCorrectedPose(scanPose);
	
	// feed the localized scan to the Karto-Mapper
	bool added = false;
	try
	{
		added = mMapper->Process(localizedScan);
	}
	catch(karto::Exception e1)
	{
		if(mOtherLasers.find(scan->robot_id) == mOtherLasers.end())
		{
			try
			{
				karto::LaserRangeFinderPtr laser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, robot);
				laser->SetMinimumRange(scan->scan.range_min);
				laser->SetMaximumRange(scan->scan.range_max);
				laser->SetMinimumAngle(scan->scan.angle_min);
				laser->SetMaximumAngle(scan->scan.angle_max);
				laser->SetAngularResolution(scan->scan.angle_increment);
				laser->SetRangeThreshold(mRangeThreshold);
				mMapper->Process(laser);
				mOtherLasers.insert(std::pair<int,karto::LaserRangeFinderPtr>(scan->robot_id,laser));
			
				added = mMapper->Process(localizedScan);
			}
			catch(karto::Exception e2)
			{
				ROS_ERROR("%s", e2.GetErrorMessage().ToCString());
			}
		}else
		{
			ROS_ERROR("%s", e1.GetErrorMessage().ToCString());
		}
	}
	if(added)
	{
		mNodesAdded++;
		mMapChanged = true;
		ROS_WARN("Robot %d: Received scan (uniqueID: %d, Sensor: %s, stateID: %d), mNodesAdded: %i", mRobotID, localizedScan->GetUniqueId(), localizedScan->GetSensorIdentifier().ToString().ToCString(), localizedScan->GetStateId(), mNodesAdded);
		
		// Publish via extra topic
		nav2d_msgs::RobotPose other;
		other.header.stamp = ros::Time::now();
		other.header.frame_id = mMapFrame;
		other.robot_id = scan->robot_id;
		other.pose.x = localizedScan->GetCorrectedPose().GetX();
		other.pose.y = localizedScan->GetCorrectedPose().GetY();
		other.pose.theta = localizedScan->GetCorrectedPose().GetHeading();
		mOtherRobotsPublisher.publish(other);
		
		// Send the map via topic
		ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
		if(mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
		{
			sendMap();
			if(mState == ST_LOCALIZING)
			{
				mSelfLocalizer->convertMap(mGridMap);
			}
		}
	}else
	{
		ROS_DEBUG("Discarded Scan from Robot %d!", scan->robot_id);
	}

	if(mState == ST_WAITING_FOR_MAP && mNodesAdded >= mMinMapSize)
	{
		sendMap();
		mSelfLocalizer->convertMap(mGridMap);
		mSelfLocalizer->initialize();
		mState = ST_LOCALIZING;
		ROS_WARN("Received a map, now starting to localize. mState IS NOW ST_LOCALIZING.");
		mSelfLocalizer->publishParticleCloud();
	}
}

void MultiMapper::sendLocalizedScan(const karto::Pose2& pose)
{
	// Oct: Directly using the latest_scan_recv msg:
        nav2d_msgs::LocalizedScan rosScan;
        rosScan.robot_id = mRobotID;
        rosScan.laser_type = 0;
        rosScan.x = pose.GetX();
        rosScan.y = pose.GetY();
        rosScan.yaw = pose.GetHeading();

        rosScan.scan.angle_min = latest_scan_recv.angle_min;
        rosScan.scan.angle_max = latest_scan_recv.angle_max;
        rosScan.scan.range_min = latest_scan_recv.range_min;
        rosScan.scan.range_max = latest_scan_recv.range_max;
        rosScan.scan.angle_increment = latest_scan_recv.angle_increment;
        rosScan.scan.time_increment = latest_scan_recv.time_increment;
        rosScan.scan.scan_time = latest_scan_recv.scan_time;

        unsigned int nReadings = latest_scan_recv.ranges.size();
        rosScan.scan.ranges.resize(nReadings);
        for(unsigned int i = 0; i < nReadings; i++)
        {
                rosScan.scan.ranges[i] = latest_scan_recv.ranges[i];
        }

//      rosScan.scan = *scan;
        mScanPublisher.publish(rosScan);

}

void MultiMapper::receiveInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
	double x = pose->pose.pose.position.x;
	double y = pose->pose.pose.position.y;
	double yaw = tf::getYaw(pose->pose.pose.orientation);
	ROS_WARN("Received initial pose (%.2f, %.2f, %.2f) on robot %d, now starting to map.",x,y,yaw,mRobotID);
	try
	{
		setRobotPose(x,y,yaw);
	}
	catch(tf::TransformException e)
	{
		ROS_ERROR("Failed to set pose on robot %d! (%s)", mRobotID, e.what());
		return;
	}
}

void MultiMapper::onMessage(const void* sender, karto::MapperEventArguments& args)
{
	ROS_DEBUG("OpenMapper: %s\n", args.GetEventMessage().ToCString());
}

void MultiMapper::publishTransform()
{
	if(mState == ST_MAPPING)
	{
		// ROS_ERROR("IN MultiMapper::publishTransform About to publish!!");
		// For measuring RT:
		// Publish the latest scan TS on which these tf's are based :
		std_msgs::Header hdr;
		hdr.stamp = ros::Time(last_scan_mapCB_tf_processed);
		mScanTSPublisher.publish(hdr);
		// ROS_ERROR("From mapScanCB: Publishing transform.. with TS %f", last_scan_mapCB_tf_processed);

		mTransformBroadcaster.sendTransform(tf::StampedTransform (mOdometryOffset, ros::Time::now() , mOffsetFrame, mOdometryFrame));
		mTransformBroadcaster.sendTransform(tf::StampedTransform (mMapToOdometry, ros::Time::now() , mMapFrame, mOffsetFrame));
	}
}
