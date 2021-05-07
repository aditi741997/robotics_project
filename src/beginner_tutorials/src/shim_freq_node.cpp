#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <sys/sysinfo.h>
#include <vector>
#include <algorithm>
#include <time.h>
#include <bits/stdc++.h> 
#include <sys/time.h>
#include <stdlib.h>
#include <cstdlib>
#include <stdint.h>
#include <cerrno>

// For measuring system wide real time:
#include <boost/chrono/system_clocks.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

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
	
	//clock_gettime(CLOCK_REALTIME, &ts);
	// double time_now = ts.tv_sec + 1e-9*ts.tv_nsec - 1603000000.0;
	
	clock_gettime(CLOCK_MONOTONIC, &ts);
	double time_now = ts.tv_sec + 1e-9*ts.tv_nsec;

	return time_now;
}

class ShimFreqNode
{
public:
    // subscribe to and store latest sensor data.
    sensor_msgs::LaserScan latest_scan;
    ros::Subscriber sensor_sub;
    void scanDataCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    // can add a separate func. for each msg type....
    
    // freq at which to publish.
    double freq;

    // Publisher, thread that'll publish.
    ros::Publisher sensor_pub;
    void publishLatestDataMsg(const std_msgs::Header::ConstPtr& msg);
    void publishLatestDataTimer(const ros::WallTimerEvent& event);
    void publishLatestData();

    ros::Publisher thread_exec_info_pub, s_exec_end_pub;

    bool use_td;
    ros::Subscriber trigger_cc_sub; // publish on getting trigger from scheduler.
    ros::WallTimer sensor_data_publish_thread;

    ros::NodeHandle nh;

    double last_pub_ts;
    std::vector<double> scan_pub_tput;
    long int pub_count = 0;
    long int neg_lat_count = 0;

    double ros_last_recv_ts;
    clock_t real_last_recv_ts;
    struct timespec real_cg_last_recv_ts;
    std::vector<double> ros_recv_deltas, real_recv_deltas, real_cg_recv_deltas;
    std::vector<double> scan_cb_times;

    void printVecStats(std::vector<double> v, std::string s);

    // ShimFreqNode();
    ShimFreqNode(double f, std::string sub_topic, std::string pub_topic, std::string msg_type, std::string use_td_s);

	std::ofstream pub_scan_ts_log;

    // for locking the data fields used both by publisher & subscriber.
protected: 
    boost::mutex data_lock; 

    // We dont need the reconfigure since CC triggered by scheduler now:
    // dynamic_reconfigure::Server<beginner_tutorials::cc_freqConfig> dyn_server;
    // dynamic_reconfigure::Server<beginner_tutorials::cc_freqConfig>::CallbackType dyn_f;
    // void callback(beginner_tutorials::cc_freqConfig &config, uint32_t level);

	// for trigger comm: sockets
	int client_sock_fd;
	boost::thread sock_recv_thread; // to indefinitely listen on the socket fd.
	void socket_recv();
};

ShimFreqNode::ShimFreqNode(double f, std::string sub_topic, std::string pub_topic, std::string msg_type, std::string use_td_s)
{
	pub_scan_ts_log.open("shq_pubScan_log.csv");
    freq = f;

    // dyn_f = boost::bind(&ShimFreqNode::callback, this, _1, _2);
    // dyn_server.setCallback(dyn_f);

    if (msg_type.compare("scan") == 0)
    {
	// queue len : 1.
        sensor_sub = nh.subscribe(sub_topic, 1, &ShimFreqNode::scanDataCallback, this, ros::TransportHints().tcpNoDelay() );
        
        sensor_pub = nh.advertise<sensor_msgs::LaserScan> (pub_topic, 1);
        ROS_WARN("SHIM Node for type scan, Subscribed to topic %s, Will publish to topic %s", sub_topic.c_str(), pub_topic.c_str());
    }

    ros_last_recv_ts = 0.0;
    real_last_recv_ts = 0.0;

    thread_exec_info_pub = nh.advertise<std_msgs::Header>("/robot_0/exec_start_s", 100, true);
    s_exec_end_pub = nh.advertise<std_msgs::Header>("/robot_0/exec_end_s", 1, false);

    // start thread for publishing :
    // Dont need this thread if Scheduler triggers CC:
    
    // publishing based on DAGController's msgs:
    // trigger_cc_sub = nh.subscribe("/robot_0/trigger_exec_s" , 1, &ShimFreqNode::publishLatestData, this, ros::TransportHints().tcpNoDelay());


    use_td = (use_td_s.find("yes") != std::string::npos );
    ROS_ERROR("VALUE of use_td : %i, string : %s", use_td, use_td_s.c_str());

    if (use_td)
    	sensor_data_publish_thread = nh.createWallTimer(ros::WallDuration(1.0/freq), &ShimFreqNode::publishLatestDataTimer, this);
    else
    {
	    // for getting triggers via socket:
	errno = 0; // resetting before starting socket stuff.
	client_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	// if (client_sock_fd < 0)
		ROS_ERROR("SHIMFreqNode:: SOCKET: client_sock_fd is %i !", client_sock_fd);
	
	struct sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(5327);

	int pton_ret = inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
	// if ( pton_ret <= 0 )
		ROS_ERROR("SHIMFreqNode:: SOCKET: in inet_pton %i", pton_ret);

	int con_ret = connect(client_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
	// if ( connect(client_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0 )
	// if (con_ret < 0)
	{
		 ROS_ERROR("SHIMFreqNode:: SOCKET: Done with connecting, retval: %i, errno: %i, err str: %s !!!", con_ret, errno, std::strerror(errno) );
		std::cerr << std::strerror(errno) << std::endl;
	}

	sock_recv_thread = boost::thread(&ShimFreqNode::socket_recv, this);

    }

   }

void ShimFreqNode::socket_recv()
{
	std::string s = "s\n";
	char smsg[1+s.length()];
	strcpy(smsg, s.c_str());
	send(client_sock_fd, smsg, strlen(smsg), 0);

	// priority=3.
	int ret = 7;
	struct sched_param sp = { .sched_priority = 3,};
	ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);
	ROS_ERROR("ShimFreqNode Socket_recv thread set priority to 3, retval: %i, tid: %i", ret, ::gettid());

	int read_size;
	char msg [2048];


	while ( (read_size = recv(client_sock_fd, msg, 2048, 0) ) > 0 )
	{
		std::string s_msg = msg;
		memset(msg, 0, 2048);
		publishLatestData();
	}
	if(read_size == 0)
		ROS_ERROR("ShimFreqNode::socket_recv THREAD: Client disconnected!!");
	else if(read_size == -1)
		ROS_ERROR("ShimFreqNode::socket_recv THREAD: Client read error!!");
}

/*
void ShimFreqNode::callback(beginner_tutorials::cc_freqConfig &config, uint32_t level) {
	// config.cc_freq
	ROS_WARN("RECEIVED NEW config!! NEW freq: %f", config.cc_freq);
	// sensor_data_publish_thread = nh.createWallTimer(ros::WallDuration(1.0/config.cc_freq), &ShimFreqNode::publishLatestData, this);
}
*/

void ShimFreqNode::scanDataCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if (ros_recv_deltas.size() < 7)
	{
		std_msgs::Header hdr;
		std::stringstream ss_e;
		ss_e << ::getpid() << " s_extra " << ::gettid();
		hdr.frame_id = ss_e.str();
		thread_exec_info_pub.publish(hdr);
	
		// getting PMTid:
		std_msgs::Header hdre;
		std::stringstream ss_e2;
		ss_e2 << ::getpid() << " s_extra " << nh.getPMTId();
		hdre.frame_id = ss_e2.str();
	    	ROS_ERROR("At SHQ:: Getting PMT id via nodeHandle: %i, tid CBT: %i, Publishing msg-hdr with fid: [pmt]: %s, [cbt]: %s", nh.getPMTId(), ::gettid(), hdre.frame_id.c_str(), hdr.frame_id.c_str());
		thread_exec_info_pub.publish(hdre);

	}
	
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);

	struct timespec ts_mono;
	clock_gettime(CLOCK_MONOTONIC, &ts_mono);

	
	boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
	boost::chrono::system_clock::duration tse = now.time_since_epoch();
	std::time_t secs = boost::chrono::system_clock::to_time_t(now);
	if (ros_recv_deltas.size()%50 == 6)
	{
            ROS_WARN("Got scan with TS %f, current clockMono time %f, scan RT Stamp %f", msg->header.stamp.toSec(), ts_mono.tv_sec + ts_mono.tv_nsec * 1e-9, msg->scan_time );
	}

        
        if (ros_last_recv_ts > 0.0)
        {
            ros_recv_deltas.push_back(ros::Time::now().toSec() - ros_last_recv_ts);
            real_recv_deltas.push_back( double(clock() - real_last_recv_ts)/ double(CLOCKS_PER_SEC) );
            struct timespec end;
            clock_gettime(CLOCK_MONOTONIC, &end);
            real_cg_recv_deltas.push_back( (end.tv_sec - real_cg_last_recv_ts.tv_sec) + ((end.tv_nsec - real_cg_last_recv_ts.tv_nsec) * 1e-9) );
        }

        if (ros_recv_deltas.size()%100 == 57)
        {
            printVecStats(ros_recv_deltas, "ROS_Time_B/W_Recv_Scans");
            printVecStats(real_recv_deltas, "Real_Time_B/W_Recv_Scans");
            printVecStats(real_cg_recv_deltas, "Real_CG_Time_B/W_Recv_Scans");
	    std::cout << "msg sz: ranges len: " << latest_scan.ranges.size() << ", intensities len: " << latest_scan.intensities.size() << std::endl;
	}

        ros_last_recv_ts = ros::Time::now().toSec();
        real_last_recv_ts = clock();
        clock_gettime(CLOCK_MONOTONIC, &real_cg_last_recv_ts);
    
    {
    	boost::mutex::scoped_lock lock(data_lock);
        latest_scan.header.frame_id = msg->header.frame_id;
        latest_scan.header.seq = msg->header.seq;
        latest_scan.header.stamp = msg->header.stamp;

        latest_scan.angle_min = msg->angle_min;
        latest_scan.angle_max = msg->angle_max;
        latest_scan.angle_increment = msg->angle_increment;
        latest_scan.time_increment = msg->time_increment;
        latest_scan.scan_time = msg->scan_time;
        latest_scan.range_min = msg->range_min;
        latest_scan.range_max = msg->range_max;

        latest_scan.ranges.clear();
        latest_scan.intensities.clear();

        for (int i = 0; i < msg->ranges.size(); i++)
            latest_scan.ranges.push_back(msg->ranges[i]);

        for (int i = 0; i < msg->intensities.size(); i++)
            latest_scan.intensities.push_back(msg->intensities[i]);

    }
}

void ShimFreqNode::printVecStats(std::vector<double> v, std::string st)
{
    std::sort(v.begin(), v.end());
    if (v.size() > 0)
    {
        int s = v.size();
        ROS_WARN("%s : Mean %f, Median %f, 75ile %f, 95ile %f, 99ile %f", st.c_str(), std::accumulate(v.begin(), v.end(), 0.0)/s, v[s/2], v[(75*s)/100], v[(95*s)/100], v[(99*s)/100] );
    }
}

void ShimFreqNode::publishLatestData()
{
	struct timespec exec_start, exec_end;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &exec_start);

	double time_now = get_time_now();
	
	//publish thread id info.
	if (pub_count < 91)
	{
		ROS_INFO("SHQ:: PUBLISHING s TID: %i", ::gettid());
		std_msgs::Header hdr;
		std::stringstream ss_e;
        	ss_e << ::getpid() << " s " << ::gettid();
        	hdr.frame_id = ss_e.str();
		thread_exec_info_pub.publish(hdr);
	}

        pub_count += 1;
        
        if (scan_pub_tput.size()%500 == 111)
	{
		ROS_WARN("NEGATIVE LATENCY Ratio: %f", (float)neg_lat_count/pub_count);
		printVecStats(scan_pub_tput, "SHQ:ScanPubTput");
	}
	
	{
		boost::mutex::scoped_lock lock(data_lock);
		if ( (time_now - latest_scan.scan_time) < -0.004)
		{
			neg_lat_count += 1;
			ROS_ERROR("-VE LAT: ShimFreqNode: Publishing scan!!!! with realTS %f, time_now: %f", latest_scan.scan_time, time_now);
		}
		// else
			// ROS_WARN("ShimFreqNode: Publishing scan!!!! with realTS %f", latest_scan.scan_time);
		pub_scan_ts_log << latest_scan.scan_time << ", " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count() << "\n";

		// publish latest_scan...
		sensor_pub.publish(latest_scan);
		
		if (pub_count > 1)
                	scan_pub_tput.push_back(time_now - last_pub_ts);
        	last_pub_ts = time_now;

		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &exec_end);
		std_msgs::Header s_exec_time;
		s_exec_time.frame_id = std::to_string( (exec_end.tv_sec + 1e-9*exec_end.tv_nsec ) - ( exec_start.tv_sec + 1e-9*exec_start.tv_nsec ) ) + " s";
		s_exec_end_pub.publish(s_exec_time);
	}
}

void ShimFreqNode::publishLatestDataMsg(const std_msgs::Header::ConstPtr& msg ) //ros::WallTimerEvent& event
{
	publishLatestData();	
}

void ShimFreqNode::publishLatestDataTimer(const ros::WallTimerEvent& event)
{
	publishLatestData();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shim_freq_node");
    ShimFreqNode sfn( atof(argv[1]), argv[2], argv[3], argv[4], argv[5] );
    ros::spin();
    return 0;
}
