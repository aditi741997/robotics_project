#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <sstream>
#include <time.h>
#include <bits/stdc++.h>
#include <sys/times.h>
#include <vector>
#include <fstream>

class TDGlobalCmp
{
    ros::Time last_scan_ts;

    double compute_rt_sum, compute_ros_sum;
    std::vector<double> compute_rt_arr, compute_ros_arr;

    double lat_scan_sum, last_scan_used_ts;
    std::vector<double> lat_scan_arr;

    int total_exec_count, total_scan_count;

    ros::NodeHandle nh;
    ros::Publisher gcmp_pub;
    ros::Subscriber scan_sub;
    ros::Timer exec_timer;

    float pub_rate;
    int climit;
    std_msgs::Header hdr_msg;

    int perc7 = 75;
    int percentile = 95;
    int perc0 = 90;
    int perc2 = 99;
    float perc3 = 99.9;
    float perc4 = 99.99;

public:

    TDGlobalCmp(int msgsz, float rate, int lim, std::string subt, std::string pubt)
    {
        initMessage(msgsz);
        pub_rate = rate;
        climit = lim;

        gcmp_pub = nh.advertise<std_msgs::Header>(pubt, 1);
        checkSubscriberCount();

	total_exec_count = 0;
	total_scan_count = 0;

        scan_sub = nh.subscribe(subt, 1, &TDGlobalCmp::scanCallback, this, ros::TransportHints().tcpNoDelay());
	ROS_INFO("Subscribed to topic %s", subt.c_str());
	last_scan_used_ts = -1.0;
        // when we have a subscriber, we're ready to start our timer.
        exec_timer = nh.createTimer(ros::Duration(1.0/pub_rate), &TDGlobalCmp::exec, this);
    }

    void scanCallback(const std_msgs::Header::ConstPtr& msg)
    {
        last_scan_ts = msg->stamp;
        total_scan_count += 1;
    }

    void print_smt(double m_sum, std::vector<double> m_arr, std::string m)
    {
        int l = m_arr.size();
        if (l > 0)
        {
            std::sort(m_arr.begin(), m_arr.end());
            double avg = m_sum/l;
            double med = m_arr[l/2];
            double perc = m_arr[(l*percentile)/100];
        
            ROS_INFO("Mean, median, tail of %s is %f %f %f , arr sz %i , %i : %f, %i : %f, %i : %f, %f : %f, %f : %f #", m.c_str(), avg, med, perc, l, perc7, m_arr[(l*perc7)/100], perc0, m_arr[(l*perc0)/100], perc2, m_arr[(l*perc2)/100], perc3, m_arr[(l*perc3)/100], perc4, m_arr[(l*perc4)/100]);            
        }
    }

    void exec(const ros::TimerEvent& event)
    {
        double using_scan_ts = last_scan_ts.toSec();
	double lat_scan = ros::Time::now().toSec() - using_scan_ts;
        if (using_scan_ts > 0.0)
        {
            double ros_start = ros::Time::now().toSec();
            clock_t rt_start = clock();
            calcPrimes();
            double ros_ci = ros::Time::now().toSec() - ros_start;
            double rt_ci = (double)(clock() - rt_start)/CLOCKS_PER_SEC;

	    total_exec_count += 1;

            compute_rt_arr.push_back(rt_ci);
            compute_rt_sum += rt_ci;

            compute_ros_arr.push_back(ros_ci);
            compute_ros_sum += ros_ci;

	    if (using_scan_ts > last_scan_used_ts)
	    {
		lat_scan_arr.push_back(lat_scan);
		lat_scan_sum += lat_scan;
		last_scan_used_ts = using_scan_ts;
	    } 

            if (total_exec_count%30 == 1)
            {
                print_smt(compute_ros_sum, compute_ros_arr, "Global Costmap Exec ROS time");
                print_smt(compute_rt_sum, compute_rt_arr, "Global Costmap Exec Real time");
            	print_smt(lat_scan_sum, lat_scan_arr, "GCmp : Lat of Scans");
	    }
            // now publish, with TS = using_scan_ts.
            hdr_msg.stamp = last_scan_ts;
            gcmp_pub.publish(hdr_msg);
        }
        else
            ROS_INFO("GCmp : Last scan TS is zero!!! Hence, not publishing!!!");
    }

    void checkSubscriberCount()
    {
        while (1 != gcmp_pub.getNumSubscribers())
        {
            ROS_INFO("Waiting for subscribers to connect %i", gcmp_pub.getNumSubscribers());
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1.0).sleep();
    }

    void initMessage(int msg_size)
    {
        std::stringstream ss;
        char* message;
        message = new char[msg_size - 4];
        memset(message, '0', msg_size - 4);
        message[msg_size - 4 - 1] = '\0';
        ss << message;
        hdr_msg.frame_id = ss.str();
        ROS_INFO("HDR msg initialized!");
    }

    void calcPrimes()
    {
        int i, num = 1, primes = 0;

        while (num <= climit) { 
            i = 2; 
            while (i <= num) { 
                if(num % i == 0)
                    break;
                i++; 
            }
            if (i == num)
                primes++;
            num++;
        }

        ROS_INFO("Found %i primes", primes);
    }
};

int main(int argc, char *argv[])
{
    int msg_sz = atoi(argv[1]);
    float gcmp_rate = atof(argv[2]); // GCmp timer rate
    int lim = atoi(argv[3]); // denotes compute load of Gcmp
    std::string node_name = argv[4];
    std::string sub_topic = argv[5];
    std::string pub_topic = argv[6];
    ROS_INFO("Init node %s, with msg sz %i, rate %f, limit %i", node_name.c_str(), msg_sz, gcmp_rate, lim);
    ros::init(argc, argv, node_name);
    TDGlobalCmp tdgc(msg_sz, gcmp_rate, lim, sub_topic, pub_topic);
    ros::spin();
    return 0;
}
