#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Header.h>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <fstream>

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
    calc_primes(limit);
}

class NewPublisher
{
public:
	NewPublisher(ros::NodeHandle *nh, int msg_sz, int qlen, int num, int64_t lim, int subc, std::string t)
	{
		sent_count = 0;
		msg_size = msg_sz;
		que_len = qlen;
		num_msgs = num;
		limit = lim;
		sub_count = subc;
		topic = t;

		chatter_pub = nh->advertise<std_msgs::Header>(topic, que_len);
		initMessage();
	}

	void checkSubscriberCount()
	{
		while (sub_count != chatter_pub.getNumSubscribers())
		{
			ROS_INFO("Waiting for subscribers to connect %i", chatter_pub.getNumSubscribers());
			ros::Duration(0.1).sleep();
		}
		ros::Duration(1.5).sleep();
	}

	void publishMsg(const ros::TimerEvent& event)
	{
		msg.seq = sent_count;
		msg.stamp = ros::Time::now();
		execute(limit);
		chatter_pub.publish(msg);
		sent_count += 1;

		if (sent_count == num_msgs)
		{
			ROS_INFO("All msgs published, Shutting down!");
			ros::shutdown();
		}
	}
private:
	int msg_size;
	int que_len;
	int num_msgs;
	int sent_count;
	int64_t limit;
	int sub_count;
	std::string topic;
	std_msgs::Header msg;
	ros::Publisher chatter_pub;

	void initMessage()
	{
		std::stringstream ss;
		char* message;
		message = new char[msg_size - 4];
		memset(message, '0', msg_size - 4);
		message[msg_size - 4 - 1] = '\0';
		ss << message;
		msg.frame_id = ss.str();
		ROS_INFO("HDR msg initialized!");
	}
};

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
    int sub_count = atoi(argv[7]);
    std::string node_name = argv[8];
    std::string topic = "/camera1/image_raw";
    ROS_INFO("Starting publisher with node name %s, topic %s", node_name.c_str(), topic.c_str());

    ros::init(argc, argv, node_name);

    ros::NodeHandle n;
    
    NewPublisher pub (&n, msg_size, pub_queue_len, num_msgs, limit, sub_count, topic);
    
    // ros::Publisher chatter_pub = n.advertise<std_msgs::Header>(topic, pub_queue_len);

    // ros::Rate loop_rate(ros_rate);

    // std_msgs::String msg;
    // nw_experiments::StringTime msgTime;
    
    /*
    This is done by the NewPub constructor
    std_msgs::Header hdr;

    std::stringstream ss;
    char* message;
    message = new char[msg_size - 4];
    memset(message, '0', msg_size - 4);
    message[msg_size - 4 - 1] = '\0';
    ss << message;
    hdr.frame_id = ss.str();
    */

    // This showed that the method to convert to string takes almost 0.001 sec, which is of the order of our latency.
    // Hence, we can't use this.
    // ROS_INFO("B4");
    // msg.data = convert_to_str(11.45678, 256);
    // ROS_INFO("A8");
    // msgTime.data = ss.str();

    /*
    while (sub_count != chatter_pub.getNumSubscribers())
    {
        ROS_INFO("Waiting for subscribers to connect %i", chatter_pub.getNumSubscribers());
        ros::Duration(0.1).sleep();
    }
    ros::Duration(1.5).sleep();
    */

    pub.checkSubscriberCount();

    ROS_INFO("Starting timer with duration %f", 1.0/ros_rate);
    ros::Timer publish_timer = n.createTimer(ros::Duration(1.0/ros_rate), &NewPublisher::publishMsg, &pub);
 
    ros::spin();
    // Code to check ros::Time format :
    //ros::Time x = ros::Time::now();
    // ROS_INFO("%f %f %f", x.sec, x.nsec, x.toSec());


    // if we decide to store the values in the future :
    // std::vector<double> sent_times (num_msgs);

/*
    int i = 0;
    double total_c1 = 0.0;
    std::vector<double> c1_arr;

    double pub_t = 0.0;
    std::vector<double> pub_times;

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

	ros::Time c1_end = ros::Time::now();
                // std_msgs::String msg;
        // chatter_pub.publish(msg);
	chatter_pub.publish(hdr);

	double pt = (ros::Time::now() - c1_end).toSec();
	pub_t += pt;
	pub_times.push_back(pt);

	double x = (c1_end - c1_start).toSec();
        total_c1 += x; 
	c1_arr.push_back(x);
	
        ros::spinOnce();

        loop_rate.sleep();
        i += 1;
    }
    ROS_INFO("Total msgs sent %i, msg size %i, avg c1 : %f", i, msg_size, total_c1/i);
    std::sort(c1_arr.begin(), c1_arr.end());
    std::sort(pub_times.begin(), pub_times.end());
    ROS_INFO("Mean_Median pub_c1 : median : %f, 99p : %f, mean : %f, freq : %i, ", c1_arr[c1_arr.size()/2], c1_arr[(99*(c1_arr.size()))/100], total_c1/i, ros_rate);

    std::ofstream outfile;
    outfile.open(argv[6], std::ios_base::app);
    outfile << msg_size << ", " << ros_rate << ", " << c1_arr[(99*(c1_arr.size()))/100] << ", " << c1_arr[c1_arr.size()/2] << ", " << total_c1/i << ", pub_t, " << pub_times[(99*(pub_times.size()))/100] << ", " << pub_times[pub_times.size()/2] << ", " << pub_t/i << ", \n";
    // ROS_INFO(" %f %f %f", sent_times[0], sent_times[num_msgs/2], sent_times[num_msgs -1]);
    if (ros::ok())
    {
        ros::shutdown();
    }
*/
    return 0;
}
