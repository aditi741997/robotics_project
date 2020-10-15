#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
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

    // for locking the data fields used both by publisher & subscriber.
    protected: boost::mutex data_lock; 

    ros::NodeHandle nh;

    // ShimFreqNode();
    ShimFreqNode(double f, std::string topic);
};

void ShimFreqNode::ShimFreqNode(double f, std::string sub_topic, std::string pub_topic, std::string msg_type)
{
    freq = f;

    if (msg_type.compare("scan") == 0)
    {
        sensor_sub = nh.subscribe(sub_topic, 1, &ShimFreqNode::scanDataCallback, this, ros::TransportHints().tcpNoDelay() );
        
        sensor_pub = nh.advertise<sensor_msgs::LaserScan> (pub_topic, 1);
        ROS_WARN("SHIM Node for type scan, Subscribed to topic %s, Will publish to topic %s", topic, pub_topic);
    }

    // start thread for publishing :
    sensor_data_publish_thread = nh.createTimer(ros::Duration(1.0/freq), &ShimFreqNode::publishLatestData, this);
}

void ShimFreqNode::scanDataCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
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
        latest_scan.ranges = msg->ranges;
        latest_scan.intensities = msg->intensities;


        ROS_WARN("COPIED incoming msg into latest_scan. Verifying : 5th: %i %i, 111th: %i %i", latest_scan.ranges[5], msg->ranges[5], latest_scan.ranges[111], msg->ranges[111]);
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

int main(int argc, char const *argv[])
{
    ShimFreqNode sfn( atof(argv[1]), argv[2], argv[3], argv[4] );
    ros::spin();
    return 0;
}