#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Header.h>

#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <vector>

/* inline std::string convert_to_str(double d, int len)
{
    std::string ans = std::to_string(d);
    return ans.append(len - ans.length(), ' ');
} */


void calc_primes(int64_t limit)
{
    int i, num = 1, primes = 0;

    while (num <= limit) { 
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
    ROS_INFO("FOUND %i primes", primes);
}

void execute(int64_t limit)
{
    // sieve(100);
    ROS_INFO("Limit : %i", limit);
    calc_primes(limit);
}

int main (int argc, char **argv)
{
    if (argc < 5)
    {
        std::cout << "Not enough params. Need msg size (in bytes), publisher queue length, loop rate, #msgs to send";
        return 0;
    }

    int msg_size = atoi(argv[1]);
    int pub_queue_len = atoi(argv[2]);
    int ros_rate = atoi(argv[3]);
    int num_msgs = atoi(argv[4]);
    int64_t limit = atoi(argv[5]);

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    // ros::Publisher chatter_pub = n.advertise<nw_experiments::StringTime>("chatter", pub_queue_len);
    ros::Publisher chatter_pub = n.advertise<std_msgs::Header>("chatter", pub_queue_len);

    ros::Rate loop_rate(ros_rate);

    // std_msgs::String msg;
    // nw_experiments::StringTime msgTime;
    std_msgs::Header hdr;

    std::stringstream ss;
    char* message;
    message = new char[msg_size - 4];
    memset(message, '0', msg_size - 4);
    message[msg_size - 4 - 1] = '\0';
    ss << message;

    // This showed that the method to convert to string takes almost 0.001 sec, which is of the order of our latency.
    // Hence, we can't use this.
    // ROS_INFO("B4");
    // msg.data = convert_to_str(11.45678, 256);
    // ROS_INFO("A8");
    // msgTime.data = ss.str();

    hdr.frame_id = ss.str();

    while (0 == chatter_pub.getNumSubscribers())
    {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
    }
    ros::Duration(0.9).sleep();
    // Code to check ros::Time format :
    //ros::Time x = ros::Time::now();
    // ROS_INFO("%f %f %f", x.sec, x.nsec, x.toSec());

    int i = 0;

    // if we decide to store the values in the future :
    // std::vector<double> sent_times (num_msgs);

    double total_c1 = 0.0;
    std::vector<double> c1_arr;

    while (ros::ok() && i < num_msgs)
    {
        // msgTime.time_stamp = ros::Time::now();
        // sent_times[i] = ros::Time::now().toSec();
        // chatter_pub.publish(msgTime);

        hdr.seq = i;
        hdr.stamp = ros::Time::now();
        
        // TODO : Add a fixed time compute here!!
        ros::Time c1_start = ros::Time::now();

	execute(limit);
	// ROS_INFO("Ended calc primes, Ans : %i", ans);
	
	ros::Time c1_end = ros::Time::now();
        double x = (c1_end - c1_start).toSec();
        total_c1 += x; 
	c1_arr.push_back(x);

        // std_msgs::String msg;
        // chatter_pub.publish(msg);
	chatter_pub.publish(hdr);
        ros::spinOnce();

        loop_rate.sleep();
        i += 1;
    }
    ROS_INFO("Total msgs sent %i, msg size %i, avg c1 : %f", i, msg_size, total_c1/i);
    std::sort(c1_arr.begin(), c1_arr.end());
    ROS_INFO("Mean_Median pub_c1 : median : %f, 99p : %f, mean : %f", c1_arr[c1_arr.size()/2], c1_arr[(99*(c1_arr.size()))/100], total_c1/i);
    // ROS_INFO(" %f %f %f", sent_times[0], sent_times[num_msgs/2], sent_times[num_msgs -1]);
    if (ros::ok())
    {
        ros::shutdown();
    }
    return 0;
}
