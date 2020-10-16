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

// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

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
    // TODO:Later : expose freq as a config parameter to be changed at runtime.

    // Publisher, thread that'll publish.
    ros::Publisher sensor_pub;
    ros::Timer sensor_data_publish_thread;
    void publishLatestData(const ros::TimerEvent& event);

    ros::NodeHandle nh;

    double ros_last_recv_ts;
    clock_t real_last_recv_ts;
    struct timespec real_cg_last_recv_ts;
    std::vector<double> ros_recv_deltas, real_recv_deltas, real_cg_recv_deltas;

    void printVecStats(std::vector<double> v, std::string s);

    // ShimFreqNode();
    ShimFreqNode(double f, std::string sub_topic, std::string pub_topic, std::string msg_type);

    // for locking the data fields used both by publisher & subscriber.
protected: 
    boost::mutex data_lock; 
};

ShimFreqNode::ShimFreqNode(double f, std::string sub_topic, std::string pub_topic, std::string msg_type)
{
    freq = f;

    if (msg_type.compare("scan") == 0)
    {
        sensor_sub = nh.subscribe(sub_topic, 1, &ShimFreqNode::scanDataCallback, this, ros::TransportHints().tcpNoDelay() );
        
        sensor_pub = nh.advertise<sensor_msgs::LaserScan> (pub_topic, 1);
        ROS_WARN("SHIM Node for type scan, Subscribed to topic %s, Will publish to topic %s", sub_topic.c_str(), pub_topic.c_str());
    }

    ros_last_recv_ts = 0.0;
    real_last_recv_ts = 0.0;

    // start thread for publishing :
    // sensor_data_publish_thread = nh.createTimer(ros::Duration(1.0/freq), &ShimFreqNode::publishLatestData, this);
}

void ShimFreqNode::scanDataCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    {
        boost::mutex::scoped_lock lock(data_lock);
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        if (ros_recv_deltas.size()%20 == 7)
            ROS_WARN("Got scan with TS %f, current clockMono time %f %f", msg->header.stamp.toSec(), ts.tv_sec, ts.tv_nsec * 1e-9);
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
        }

        
        ros_last_recv_ts = ros::Time::now().toSec();
        real_last_recv_ts = clock();
        clock_gettime(CLOCK_MONOTONIC, &real_cg_last_recv_ts);
    }
}

void ShimFreqNode::printVecStats(std::vector<double> v, std::string st)
{
    std::sort(v.begin(), v.end());
    if (v.size() > 0)
    {
        int s = v.size();
        ROS_WARN("%s : Mean %f, Median %f, 95ile %f, 99ile %f", st.c_str(), std::accumulate(v.begin(), v.end(), 0.0)/s, v[s/2], v[(95*s)/100], v[(99*s)/100] );
    }
}

void ShimFreqNode::publishLatestData(const ros::TimerEvent& event)
{
    {
        boost::mutex::scoped_lock lock(data_lock);

        // publish latest_scan...
        sensor_pub.publish(latest_scan);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shim_freq_node");
    ShimFreqNode sfn( atof(argv[1]), argv[2], argv[3], argv[4] );
    ros::spin();
    return 0;
}