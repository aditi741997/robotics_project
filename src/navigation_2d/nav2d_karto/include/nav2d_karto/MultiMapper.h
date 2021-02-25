#ifndef MULTI_MAPPER_H
#define MULTI_MAPPER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav2d_msgs/LocalizedScan.h>
#include <nav2d_localizer/SelfLocalizer.h>

#include <OpenKarto/OpenKarto.h>

#include <string>
#include <map>
#include <list>
#include <boost/circular_buffer.hpp>
#include <atomic>

#include <fstream>
#include <boost/thread.hpp>

#define ST_WAITING_FOR_MAP  10
#define ST_LOCALIZING       20
#define ST_MAPPING          30

// for getting system clock time
#include <boost/chrono/system_clocks.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

// FOR sockets
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

void publish_tid(std::string name, int tid, ros::Publisher* pub)
{
        std_msgs::Header hdr;
        std::stringstream ss;
        ss << ::getpid() << " " << name << " " << tid;
        hdr.frame_id = ss.str();
        pub->publish(hdr);
}

class MultiMapper
{
public:
	// Constructor & Destructor
	MultiMapper();
	MultiMapper(ros::Publisher& mcb_pub);
	~MultiMapper();

	// Public methods
	void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
	void receiveLocalizedScan(const nav2d_msgs::LocalizedScan::ConstPtr& scan);
	void receiveInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
	void sendLocalizedScan(const karto::Pose2& pose);
	void onMessage(const void* sender, karto::MapperEventArguments& args);
	bool getMap(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);
	void publishLoop();
	void publishTransform();
	void setScanSolver(karto::ScanSolver* scanSolver);

	// For measuring RT:

	// TS of last scan processed by mapCB.
	double last_scan_mapCB_processed_ST;

	// TSS of last scan used in generating the mapToOdom TF.
	double last_scan_mapCB_tf_processed;

	// For measuring Tput of subchains MapCB and MapUpdate:
	std::vector<double> tput_map_cb, tput_map_update;
	double last_map_cb_out, last_map_upd_out;
	// boost::chrono::time_point<boost::chrono::system_clock> last_map_upd_out;

	std::vector<double> trig_tput_map_cb;
	double trig_last_map_cb_out;
	long int total_map_cb_trig_count;

	// For measuring how many scans does mapScanCB drop:
	std::vector<double> scan_drop_ts;
	std::vector<double> scan_drop_exec_time;

	std::vector<bool> scan_pose_ts;

	long int total_mapcb_count, total_mapupdate_count;

	ros::Publisher mScanTSPublisher;
	std::ofstream tf_publish_ts_log;

	// For converting the module into hybrid ED/TD
	// Making a separate thread for mapUpdates
	boost::thread* map_update_thread_;
	bool map_update_thread_shutdown_;
	bool received_scans, processed_scans;
	void mapUpdateLoop(double freq);

	// Making a thread for mapCB as well:
	boost::thread* map_scan_cb_thread_;
	bool map_scan_cb_thread_shutdown_;
	boost::circular_buffer<sensor_msgs::LaserScan> latest_scans_recv; // this is what'll be processed by the mapScanCBLoop.
	sensor_msgs::LaserScan latest_scan_recv;
	void mapScanCBLoop(double freq);
	void processLatestScans();
	double mMapScanUpdateRate; // Oct: period of scanCB thread.

	boost::mutex scan_lock; // to lock usage of latest_scan_recv.

	ros::Publisher map_cb_exec_info_pub, map_upd_exec_info_pub, map_cb_exec_end_pub, map_upd_exec_end_pub;

	// For fractional scheduling:
	int mDropFraction;

	// For receiving scheduler's trigger msgs:
	void recv_trigger_exec(const std_msgs::Header::ConstPtr& msg);
	boost::mutex mapcb_trigger_mutex, mapupd_trigger_mutex; // take this lock which reading/writing trigger_counts.
	int mapcb_trigger_count, mapupd_trigger_count;
	boost::condition_variable cv_mapcb, cv_mapupd; // threads will wait on these CVs.
	ros::Subscriber mapcb_trigger_sub, mapupd_trigger_sub;

	void processTrigger(std::string msg);

	// now using sockets for receiving scheduler's trigger msgs:
	int client_sock_fd;
	boost::thread sock_recv_thread; // to indefinitely listen on the socket fd.
	void socket_recv(); // does what recv_trigger_exec does when getting a trigger msg.
	tf::TransformListener* mTransformListener;

private:
	// Private methods
	bool updateMap();
	bool sendMap();
	void setRobotPose(double x, double y, double yaw);
	karto::LocalizedRangeScan* createFromRosMessage(const sensor_msgs::LaserScan& scan, const karto::Identifier& robot);

	// Particle filter to localize within received map
	SelfLocalizer* mSelfLocalizer;
	
	// Everything related to ROS
	tf::TransformBroadcaster mTransformBroadcaster;
	tf::Transform mMapToOdometry;
	tf::Transform mOdometryOffset;

	nav_msgs::OccupancyGrid mGridMap;

	ros::ServiceServer mMapServer;
	ros::Publisher mMapPublisher;
	ros::Publisher mScanPublisher;
	ros::Publisher mVerticesPublisher;
	ros::Publisher mEdgesPublisher;
	ros::Publisher mPosePublisher;
	ros::Publisher mOtherRobotsPublisher;
	ros::Subscriber mLaserSubscriber;
	ros::Subscriber mScanSubscriber;
	ros::Subscriber mInitialPoseSubscriber;

	// Everything related to KARTO
	karto::LaserRangeFinderPtr mLaser;
	karto::SmartPointer<karto::OpenMapper> mMapper;
	std::map<int, karto::LaserRangeFinderPtr> mOtherLasers;
	std::atomic<bool> mMapChanged;
	boost::mutex currentMapMutex;

	// Parameters and Variables
	int mRobotID;               // Who am I?
	double mMapResolution;      // Resolution of published grid map.
	double mRangeThreshold;     // Maximum range of laser sensor. (All robots MUST use the same due to Karto-Mapper!)
	double mMaxCovariance;      // When to accept the result of the particle filter?
	int mState;	                // What am I doing? (waiting, localizing, mapping)
	double mMapUpdateRate;	        // Publish the map every # received updates.
	bool mPublishPoseGraph;	    // Whether or not to publish the pose graph as marker-message.
	int mNodesAdded;            // Number of nodes added to the pose graph.
	int mMinMapSize;            // Minimum map size (# of nodes) needed for localization.
	ros::WallTime mLastMapUpdate;

	// Frames and Topics
	std::string mLaserFrame;
	std::string mRobotFrame;
	std::string mOdometryFrame;
	std::string mOffsetFrame;
	std::string mMapFrame;
	std::string mLaserTopic;
	std::string mMapTopic;
	std::string mMapService;
	std::string mScanInputTopic;
	std::string mScanOutputTopic;

	std::vector<double> scan_cb_times;
	std::vector<double> scan_cb_ts;

	std::vector<double> map_update_times;
	std::vector<double> map_update_ts;

	std::vector<int> map_update_scan_count;
};

#endif
