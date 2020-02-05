#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <sys/sysinfo.h>
#include <vector>
#include <algorithm>

// For adding a heavy thread :
// #include <thread>
#include <cstdlib>
#include <stdint.h>

double sum_latency = 0.0;
double sum_latency_sq = 0.0;
int msg_count = 0;

int msg_size = 0;
int pub_queue_len = 0;
int ros_rate = 0;
int num_msgs = 0;
int sub_queue_len = 0;
int transport_type = 0;

int sub_id = 0;
bool do_heavy = false;

const double megabyte = 1024*1024;
double used_ram = 0.0;

ros::Time last_recv_time;
double sum_recv_delta = 0.0;
std::vector<double> recv_delta_arr;

bool write_lat = false;
std::vector<double> latencies;

double sum_total_delay = 0.0;
std::vector<double> total_delay;

// for calculating tput : keeping array of times b/w outputs of subscriber [i.e. after heavy is over]
double last_sub_output = 0.0;
double sum_op_delta = 0.0;
std::vector<double> op_delta;

// for d+1/t : separate array
double sum_dt_sum = 0.0;
std::vector<double> dt_sum;

double avg_heavy_time = 0.0;
std::vector<double> heavy_times;

int percentile = 99;

int64_t limit;
bool stop_thread = false;

bool to_publish = false;
std::string expt;
std::string node_name;
std::string publish_topic = "";
ros::Publisher chatter_pub;
std_msgs::Header hdr;

double pub_time = 0.0;
std::vector<double> pub_times;

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
    ROS_INFO("Found %i primes", primes);
}

void chatterCallBack(const std_msgs::Header::ConstPtr& msg)
{
    ros::Time recv_time = ros::Time::now();

    double recv_delta = (msg_count == 0) ? 0.0 : (recv_time - last_recv_time).toSec();
    double lat = (recv_time - msg->stamp).toSec();
    recv_delta_arr.push_back(recv_delta);
    latencies.push_back(lat);
    ROS_INFO("sent time : [%f], recv_time : [%f], msg count : %i, msg id : %i", msg->stamp.toSec(), recv_time.toSec(), msg_count, msg->seq);

    msg_count += 1;
    sum_latency += lat;
    sum_latency_sq += lat * lat;
    sum_recv_delta += recv_delta;

    last_recv_time = recv_time;

    // struct sysinfo si;
    // sysinfo(&si);
    // used_ram += (si.totalram - si.freeram)/megabyte;

    if (do_heavy)
    {
        ros::Time heavy_start = ros::Time::now();
        // Single heavy function :
        // sieve(limit);
        calc_primes(limit);

        // Adding 2 threads :
        // std::thread t1(sieve, limit);
        // std::thread t2(sieve, limit);
        // t1.join();
        // t2.join();

        avg_heavy_time += (ros::Time::now() - heavy_start).toSec();
        heavy_times.push_back((ros::Time::now() - heavy_start).toSec());
    }
    
    double dt = 0.0;
    ros::Time to = ros::Time::now();

    double delay = (to - msg->stamp).toSec();
    total_delay.push_back(delay);
    sum_total_delay += delay;

    if (msg_count >= 2)
    {
        double op_del = (to).toSec() - last_sub_output;
        op_delta.push_back(op_del);
        sum_op_delta += op_del;

        // dt is the d+1/t for this cycle. Add to array, sum value.
        // dt += op_del + delay;
        // sum_dt_sum += dt;
        // dt_sum.push_back(dt);

	dt = op_del + total_delay[total_delay.size() - 2];
	sum_dt_sum += dt;
	dt_sum.push_back(dt);
    }
    last_sub_output = to.toSec();

    if (to_publish)
    {
	ros::Time pub_start = ros::Time::now();
	hdr.seq = msg->seq;
	hdr.stamp = msg->stamp;
	chatter_pub.publish(hdr);
	double pub_t = (ros::Time::now() - pub_start).toSec();
	pub_time += pub_t;
	pub_times.push_back(pub_t);
    }

    if (msg->seq >= ((num_msgs*98)/100))
    {
        int last_recv_msg = msg->seq;
        ROS_INFO("Last received msg Id %i", last_recv_msg);
        
        double std_dev = sqrt(sum_latency_sq - (sum_latency*sum_latency/msg_count))/msg_count;
        sum_latency = sum_latency/msg_count;
        
        sum_recv_delta /= (msg_count - 1);
        // sort the arr
        std::sort(recv_delta_arr.begin(), recv_delta_arr.end());
        double median_recv_delta = recv_delta_arr[msg_count/2];

        std::sort(latencies.begin(), latencies.end());
        int index = (percentile*(msg_count))/100;
        // std::cout << "index : " << index << "\n";
        double percentile_lat = latencies[index];
        double median_lat = latencies[latencies.size()/2];

        std::sort(total_delay.begin(), total_delay.end());
        double perc_total_delay = total_delay[index];
        double median_total_delay = total_delay[total_delay.size()/2];
        double mean_total_delay = sum_total_delay/(total_delay.size());

        std::sort(op_delta.begin(), op_delta.end());
        index = (percentile*(op_delta.size()))/100;
        double perc_op_delta = op_delta[index];
        double median_op_delta = op_delta[op_delta.size()/2];
        double mean_op_delta = sum_op_delta/(op_delta.size());

        std::sort(dt_sum.begin(), dt_sum.end());
        double perc_dt_sum = dt_sum[index];
        double median_dt_sum = dt_sum[dt_sum.size()/2];
        double mean_dt_sum = sum_dt_sum/(dt_sum.size());

        std::sort(heavy_times.begin(), heavy_times.end());
	double perc_heavy = heavy_times[index];
	double median_heavy = heavy_times[heavy_times.size()/2];

	if (to_publish)
	{
		std::sort(pub_times.begin(), pub_times.end());
	}

        // used_ram /= msg_count;std::to_string(sub_id) + 
        avg_heavy_time /= msg_count;

        std::ofstream outfile;
        outfile.open((expt + "_" + node_name + "_dec25.txt").c_str(), std::ios_base::app);
        outfile << msg_size << ", " << pub_queue_len << ", " << num_msgs << ", " << sub_queue_len << ", " << ros_rate << ", " << transport_type << ", " << do_heavy << ", " << perc_total_delay << ", " << median_total_delay << ", " << mean_total_delay << ", " << perc_op_delta << ", " << median_op_delta << ", " << mean_op_delta << ", " << perc_dt_sum << ", " << median_dt_sum << ", " << mean_dt_sum << ", " << avg_heavy_time << ", " << "lost_msgs, " << (msg->seq + 1 - msg_count) << ", " << (msg->seq + 1) << ", c1n_latency, " << percentile_lat << ", " << median_lat << ", " << sum_latency << ", c2, " << perc_heavy << ", " << median_heavy << ", " << avg_heavy_time << ", ";
	if (to_publish)
	{
		outfile << "pub_time, " << pub_times[index] << ", " << pub_times[pub_times.size()/2] << ", " << pub_time/msg_count << ", ";
	}
	outfile << "\n";
        // outfile << msg_size << ", " << pub_queue_len << ", " << num_msgs << ", " << sub_queue_len << ", " << ros_rate << ", " << transport_type << ", " << do_heavy << ", " << sum_latency << ", " << percentile_lat << ", " << std_dev << ", " << sum_recv_delta << ", " << median_recv_delta << ", " << avg_heavy_time  << ", " << msg_count << ", " << last_recv_msg << ", \n";

        if (write_lat)
        {
            std::ofstream outfile;
            outfile.open("latencies.txt", std::ios_base::app);
            outfile << msg_size << ", " << pub_queue_len << ", " << num_msgs << ", " << sub_queue_len << ", " << ros_rate << ", " << transport_type << ", " << do_heavy << "\n";
            for (double &d: latencies)
            {
                outfile << d << "\n";
            }
        }

        ROS_INFO("ALL PARAMS : %i, %i, %i, %i, %i, %i, %i, %i", msg_size, pub_queue_len, ros_rate, num_msgs, sub_queue_len, transport_type, sub_id, do_heavy);
        ROS_INFO("FINAL VALUE OF LATENCY : [%f], STD DEV : [%f], used ram : [%f], sum rec delta : [%f]", sum_latency, std_dev, used_ram, sum_recv_delta);
        ros::shutdown();
    }
}


int main (int argc, char **argv)
{
    if (argc < 13)
    {
        std::cout << "Not enough inputs. Need msg size, pub queue len, ros rate, num msgs, sub queue len, transport type (0 tcp, 1 tcpNoDelay, 2 udp), sub id, do heavy, limit for sieve";
        return 0;
    }

    msg_size = atoi(argv[1]);
    pub_queue_len = atoi(argv[2]);
    ros_rate = atoi(argv[3]);
    num_msgs = atoi(argv[4]);
    sub_queue_len = atoi(argv[5]);
    transport_type = atoi(argv[6]);
    sub_id = atoi(argv[7]);
    do_heavy = atoi(argv[8]);

    limit = atoi(argv[9]);
    node_name = argv[10];

    ROS_INFO("Init node %s, sub_topic %s, expt %s", node_name.c_str(), argv[11], argv[12]);
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    
    std::string sub_topic = argv[11];
    expt = argv[12];

    to_publish = atoi(argv[13]) == 1;
    if (to_publish)
    {
	publish_topic = argv[14];
	ROS_INFO("I will publish on %s", (publish_topic + "BLAH").c_str());
	chatter_pub = n.advertise<std_msgs::Header>(publish_topic, sub_queue_len);

	std::stringstream ss;
        char* message;
        message = new char[msg_size - 4];
        memset(message, '0', msg_size - 4);
        message[msg_size - 4 - 1] = '\0';
        ss << message;

	hdr.frame_id = ss.str();

	while (0 == chatter_pub.getNumSubscribers())
	{
            ROS_INFO("Waiting for subscribers to connect");
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1.5).sleep();
    }
    // ros::init(argc, argv, "listener");

    /* ros::TransportHints transport_type_ros;
    switch (transport_type)
    {
        case 0:
            transport_type_ros = ros::TransportHints().tcp();
            std::cout << "t 0\n";
            break;
        case 1:
            transport_type_ros = ros::TransportHints().tcpNoDelay();
            std::cout << "t 1\n";
            break;
        case 2:
            transport_type_ros = ros::TransportHints().udp();
            std::cout << "t 2\n";
            break;
    } */

    // creating a thread
    // std::thread t1(thread_func, limit);
    

    ros::Subscriber sub = n.subscribe(sub_topic, sub_queue_len, chatterCallBack, ros::TransportHints().tcpNoDelay());
    std::cout << "chatter subscribed, about to call spin \n";
    ros::spin();

    stop_thread = true;
    // t1.join();
    return 0;
}
